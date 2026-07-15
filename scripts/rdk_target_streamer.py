#!/usr/bin/env python3
"""Stream Isaac target-pose UDP packets to the Flexiv runtime through RDK."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
UTILS_DIR = REPO_ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators"
RDK_COMPAT_PATH = REPO_ROOT / ".deps" / "flexivrdk_1_9_1"

for path in (str(RDK_COMPAT_PATH), str(UTILS_DIR)):
    if Path(path).exists() and path not in sys.path:
        sys.path.insert(0, path)

from elements_studio_utils import RdkRuntimeController, RdkRuntimeSettings
from control_helpers import format_pose_xyz_quat


DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 45678


def parse_target_pose_packet(
    packet: dict,
    *,
    serial_number: str,
    joint_group: str,
    max_age_sec: float,
    now: float | None = None,
) -> list[float] | None:
    if not isinstance(packet, dict):
        return None
    if packet.get("schema") != "flexiv_target_pose.v1":
        return None
    if str(packet.get("serial", "")) != str(serial_number):
        return None
    if str(packet.get("joint_group", "")) != str(joint_group):
        return None
    try:
        pose = [float(value) for value in packet["pose_base_tcp_des"]]
    except (KeyError, TypeError, ValueError):
        return None
    if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
        return None
    if max_age_sec > 0.0 and packet.get("monotonic_time") is not None:
        try:
            current_time = time.monotonic() if now is None else float(now)
            if current_time - float(packet["monotonic_time"]) > float(max_age_sec):
                return None
        except (TypeError, ValueError):
            return None
    return pose


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--joint-group", default=DEFAULT_JOINT_GROUP)
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--network-interface-whitelist", default="")
    parser.add_argument("--switch-mode", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clear-fault", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--strict-clear-fault", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--servo-on", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--enable-timeout-sec",
        type=float,
        default=15.0,
        help="Wait this long for Enable() to make the robot operational before reconnecting.",
    )
    parser.add_argument("--retry-last-pose-sec", type=float, default=1.0)
    parser.add_argument("--reconnect-delay-sec", type=float, default=1.0)
    parser.add_argument("--reconnect-on-error", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--log-hz", type=float, default=2.0)
    parser.add_argument("--status-host", default="")
    parser.add_argument("--status-port", type=int, default=0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    import flexivrdk

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((str(args.host), int(args.port)))
    sock.settimeout(0.5)
    status_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if int(args.status_port) > 0 else None
    status_address = (str(args.status_host or "127.0.0.1"), int(args.status_port))
    reference_pose: list[float] | None = None

    def publish_ready(ready: bool, current_pose: list[float] | None = None) -> None:
        if status_sock is None:
            return
        packet = {
            "schema": "flexiv_rdk_streamer_status.v1",
            "serial": str(args.serial_number),
            "ready": bool(ready),
            "monotonic_time": time.monotonic(),
        }
        if reference_pose is not None:
            packet["reference_pose_base_tcp"] = list(reference_pose)
        if current_pose is not None:
            packet["current_pose_base_tcp"] = list(current_pose)
        status_sock.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"), status_address)
    controller = RdkRuntimeController(
        RdkRuntimeSettings(
            serial_number=args.serial_number,
            joint_group=args.joint_group,
            network_interface_whitelist=args.network_interface_whitelist,
            switch_mode=bool(args.switch_mode),
            clear_fault=bool(args.clear_fault),
            strict_clear_fault=bool(args.strict_clear_fault),
            servo_on=bool(args.servo_on),
            verbose=True,
            enable_timeout_sec=float(args.enable_timeout_sec),
        ),
        flexivrdk=flexivrdk,
        log=lambda message: print(f"[RdkTargetStreamer] {message}", flush=True),
    )
    print(
        f"[RdkTargetStreamer] listening {args.host}:{args.port} serial={args.serial_number} joint_group={args.joint_group}",
        flush=True,
    )
    last_log_time = 0.0
    last_retry_time = 0.0
    next_connect_attempt_time = 0.0
    last_pose: list[float] | None = None
    last_control_active = True
    while True:
        try:
            data, _addr = sock.recvfrom(65536)
        except socket.timeout:
            now = time.monotonic()
            if last_pose is None or now - last_retry_time < max(0.0, float(args.retry_last_pose_sec)):
                continue
            pose = list(last_pose)
            last_retry_time = now
        else:
            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            pose = parse_target_pose_packet(
                packet,
                serial_number=args.serial_number,
                joint_group=args.joint_group,
                max_age_sec=float(args.max_age_sec),
            )
            if pose is None:
                continue
            last_pose = list(pose)
            last_control_active = bool(packet.get("control_active", True))
            last_retry_time = time.monotonic()
        try:
            if not controller.connected and time.monotonic() < next_connect_attempt_time:
                continue
            if not controller.connected:
                controller.connect()
                reference_pose = controller.current_tcp_pose()
                print(
                    f"[RdkTargetStreamer] latched current TCP reference {format_pose_xyz_quat(reference_pose)}",
                    flush=True,
                )
            command_pose = pose if last_control_active else reference_pose
            assert command_pose is not None
            controller.send_pose(command_pose)
            current_pose = controller.current_tcp_pose()
        except Exception as exc:
            print(f"[RdkTargetStreamer] send failed: {exc}", flush=True)
            publish_ready(False)
            if not args.reconnect_on_error:
                print(
                    "[RdkTargetStreamer] fault latched; exiting without clearing or reconnecting",
                    flush=True,
                )
                return 1
            next_connect_attempt_time = time.monotonic() + max(0.0, float(args.reconnect_delay_sec))
            controller = RdkRuntimeController(
                controller.settings,
                flexivrdk=flexivrdk,
                log=lambda message: print(f"[RdkTargetStreamer] {message}", flush=True),
            )
            continue
        publish_ready(True, current_pose)
        now = time.monotonic()
        if float(args.log_hz) > 0.0 and now - last_log_time >= 1.0 / float(args.log_hz):
            print(f"[RdkTargetStreamer] sent {format_pose_xyz_quat(command_pose)}", flush=True)
            last_log_time = now


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
