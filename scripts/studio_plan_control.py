#!/usr/bin/env python3
"""Control the Elements Studio jogging plan through the Studio gRPC bridge."""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GRPC_ADDRESS = "127.0.0.1:18001"
DEFAULT_PLAN_NAME = "isaac_ball_jog"
DEFAULT_STUDIO_ROOT = Path("/home/simate/workspace/elements_studio/FlexivElementsStudio")
DEFAULT_BUTTONS = ["MotorOn", "Confirm"]


def encode_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("varint value must be non-negative")
    out = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def encode_string_field(field_number: int, value: str) -> bytes:
    payload = value.encode("utf-8")
    return encode_varint((field_number << 3) | 2) + encode_varint(len(payload)) + payload


def encode_bool_field(field_number: int, value: bool) -> bytes:
    return encode_varint((field_number << 3) | 0) + encode_varint(1 if value else 0)


def encode_assign_plan_request(plan_name: str, *, is_internal_plan: bool = False) -> bytes:
    return encode_string_field(1, plan_name) + encode_bool_field(2, is_internal_plan)


def encode_plan_name_request(plan_name: str) -> bytes:
    return encode_string_field(1, plan_name)


def encode_click_button_request(button_name: str) -> bytes:
    return encode_string_field(1, button_name)


def decode_fvr_return_value(payload: bytes) -> str:
    if not payload:
        return "<empty>"
    fields: list[str] = []
    index = 0
    while index < len(payload):
        key = payload[index]
        index += 1
        field_number = key >> 3
        wire_type = key & 7
        if wire_type == 0:
            shift = 0
            value = 0
            while index < len(payload):
                byte = payload[index]
                index += 1
                value |= (byte & 0x7F) << shift
                if not byte & 0x80:
                    break
                shift += 7
            fields.append(f"f{field_number}={value}")
        elif wire_type == 2:
            shift = 0
            size = 0
            while index < len(payload):
                byte = payload[index]
                index += 1
                size |= (byte & 0x7F) << shift
                if not byte & 0x80:
                    break
                shift += 7
            data = payload[index : index + size]
            index += size
            try:
                fields.append(f"f{field_number}={data.decode('utf-8')!r}")
            except UnicodeDecodeError:
                fields.append(f"f{field_number}=0x{data.hex()}")
        else:
            fields.append(f"f{field_number}=wire{wire_type}")
            break
    return ", ".join(fields) if fields else f"0x{payload.hex()}"


def import_grpc_module():
    candidates = [
        REPO_ROOT / ".deps" / "grpc",
        Path(os.environ.get("GRPC_PYTHONPATH", "")),
    ]
    for candidate in candidates:
        if candidate and candidate.exists():
            sys.path.insert(0, str(candidate))
    import grpc  # type: ignore

    return grpc


class StudioClient:
    def __init__(self, address: str, timeout: float) -> None:
        grpc = import_grpc_module()
        self._channel = grpc.insecure_channel(address)
        self._timeout = timeout

    def call(self, method: str, request: bytes = b"") -> bytes:
        return self._channel.unary_unary(method)(request, timeout=self._timeout)

    def set_hack_mbar(self, enabled: bool) -> bytes:
        return self.call("/proto.robot.hack_mbar.HackMbarService/SetHackMbarState", encode_bool_field(1, enabled))

    def get_hack_mbar(self) -> bytes:
        return self.call("/proto.robot.hack_mbar.HackMbarService/GetHackMbarState")

    def clear_fault(self) -> bytes:
        return self.call("/proto.robot.workflow.WorkflowService/ClearFault")

    def switch_auto(self) -> bytes:
        return self.call("/proto.robot.workflow.WorkflowService/SwitchAutoMode")

    def assign_plan(self, plan_name: str) -> bytes:
        return self.call(
            "/proto.robot.workflow.WorkflowService/AssignPlan",
            encode_assign_plan_request(plan_name, is_internal_plan=False),
        )

    def stop_plan(self, plan_name: str) -> bytes:
        return self.call("/proto.robot.workflow.WorkflowService/StopPlan", encode_plan_name_request(plan_name))

    def click_button(self, button_name: str) -> bytes:
        return self.call("/proto.robot.hack_mbar.HackMbarService/ClickBtn", encode_click_button_request(button_name))


def latest_robot_log(studio_root: Path = DEFAULT_STUDIO_ROOT) -> Path | None:
    log_dir = studio_root / "log"
    if not log_dir.exists():
        return None
    logs = sorted(log_dir.glob("RobotControlApp_*.log"), key=lambda path: path.stat().st_mtime)
    return logs[-1] if logs else None


def read_log_tail(log_path: Path | None, *, offset: int = 0, max_bytes: int = 200_000) -> str:
    if log_path is None or not log_path.exists():
        return ""
    size = log_path.stat().st_size
    start = max(offset, size - max_bytes)
    with log_path.open("rb") as log_file:
        log_file.seek(start)
        return log_file.read().decode("utf-8", errors="replace")


def classify_log_status(text: str) -> tuple[str, list[str]]:
    lines = text.splitlines()
    evidence: list[str] = []
    status = "unknown"
    for line in lines:
        if "Switched to working state" in line or "WORKING_AUTO_EXECUTE" in line:
            status = "working"
            evidence.append(line)
        if "systemPlanData curPt = cart_jogging" in line or "cart_jogging" in line:
            status = "working"
            evidence.append(line)
        if (
            "safety check failed" in line
            or "failed to run the robot hardware" in line
            or "FAULT_" in line
            or "Servo cycle mismatch" in line
            or "JntTorqueExceed" in line
            or "exceeding limit" in line
        ):
            status = "fault"
            evidence.append(line)
        if "MotionBar button" in line and "not found" in line:
            status = "button-not-found"
            evidence.append(line)
    return status, evidence[-12:]


def print_response(label: str, payload: bytes) -> None:
    print(f"[StudioPlanControl] {label}: {decode_fvr_return_value(payload)}", flush=True)


def run_start(args: argparse.Namespace) -> int:
    log_path = Path(args.studio_log).expanduser() if args.studio_log else latest_robot_log(Path(args.studio_root))
    log_offset = log_path.stat().st_size if log_path and log_path.exists() else 0
    client = StudioClient(args.grpc_address, args.timeout)

    if args.hack_mbar:
        print_response("set_hack_mbar(true)", client.set_hack_mbar(True))
    if args.switch_auto:
        print_response("switch_auto", client.switch_auto())
        time.sleep(args.step_delay)
    if args.clear_fault:
        print_response("clear_fault", client.clear_fault())
        time.sleep(args.step_delay)
    print_response(f"assign_plan({args.plan_name})", client.assign_plan(args.plan_name))
    time.sleep(args.step_delay)

    for button in args.buttons:
        print_response(f"click_button({button})", client.click_button(button))
        time.sleep(args.button_delay)

    time.sleep(args.log_wait)
    text = read_log_tail(log_path, offset=log_offset)
    status, evidence = classify_log_status(text)
    if log_path:
        print(f"[StudioPlanControl] log={log_path}", flush=True)
    print(f"[StudioPlanControl] observed_status={status}", flush=True)
    for line in evidence:
        print(f"[StudioPlanControl] evidence: {line}", flush=True)
    return 0 if status in {"working", "unknown"} else 2


def run_stop(args: argparse.Namespace) -> int:
    client = StudioClient(args.grpc_address, args.timeout)
    print_response(f"stop_plan({args.plan_name})", client.stop_plan(args.plan_name))
    return 0


def run_status(args: argparse.Namespace) -> int:
    log_path = Path(args.studio_log).expanduser() if args.studio_log else latest_robot_log(Path(args.studio_root))
    text = read_log_tail(log_path)
    status, evidence = classify_log_status(text)
    if log_path:
        print(f"[StudioPlanControl] log={log_path}", flush=True)
    print(f"[StudioPlanControl] observed_status={status}", flush=True)
    for line in evidence:
        print(f"[StudioPlanControl] evidence: {line}", flush=True)
    return 0


def run_click(args: argparse.Namespace) -> int:
    client = StudioClient(args.grpc_address, args.timeout)
    for button in args.buttons:
        print_response(f"click_button({button})", client.click_button(button))
        time.sleep(args.button_delay)
    return 0


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--grpc-address", default=DEFAULT_GRPC_ADDRESS, help="Elements Studio gRPC address.")
    parser.add_argument("--timeout", type=float, default=2.0, help="Per-request gRPC timeout in seconds.")
    parser.add_argument("--studio-root", default=str(DEFAULT_STUDIO_ROOT), help="FlexivElementsStudio root directory.")
    parser.add_argument("--studio-log", default=None, help="RobotControlApp log path for status evidence.")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    start = subparsers.add_parser("start", help="Prepare Studio and click motion-bar buttons to start the plan.")
    add_common_args(start)
    start.add_argument("--plan-name", default=DEFAULT_PLAN_NAME)
    start.add_argument("--buttons", nargs="+", default=list(DEFAULT_BUTTONS))
    start.add_argument("--step-delay", type=float, default=0.8)
    start.add_argument("--button-delay", type=float, default=1.2)
    start.add_argument("--log-wait", type=float, default=1.0)
    start.add_argument("--no-hack-mbar", dest="hack_mbar", action="store_false")
    start.add_argument("--no-switch-auto", dest="switch_auto", action="store_false")
    start.add_argument("--no-clear-fault", dest="clear_fault", action="store_false")
    start.set_defaults(func=run_start, hack_mbar=True, switch_auto=True, clear_fault=True)

    stop = subparsers.add_parser("stop", help="Stop the assigned Studio plan.")
    add_common_args(stop)
    stop.add_argument("--plan-name", default=DEFAULT_PLAN_NAME)
    stop.set_defaults(func=run_stop)

    status = subparsers.add_parser("status", help="Summarize recent RobotControlApp plan status from logs.")
    add_common_args(status)
    status.set_defaults(func=run_status)

    click = subparsers.add_parser("click", help="Click one or more motion-bar buttons.")
    add_common_args(click)
    click.add_argument("buttons", nargs="+")
    click.add_argument("--button-delay", type=float, default=1.2)
    click.set_defaults(func=run_click)

    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
