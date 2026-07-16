#!/usr/bin/env python3
"""Print live dual-arm joint and TCP states published by Isaac Sim."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from typing import Any


SCHEMA = "flexiv_dual_arm_state.v1"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 57684


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST, help="UDP address to bind")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to bind")
    parser.add_argument("--rate-hz", type=float, default=5.0, help="maximum terminal refresh rate")
    parser.add_argument("--timeout-sec", type=float, default=3.0, help="warning interval while no packets arrive")
    parser.add_argument("--once", action="store_true", help="print one valid packet and exit")
    parser.add_argument(
        "--clear",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="clear the terminal before each refresh",
    )
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be positive")
    if args.timeout_sec <= 0:
        parser.error("--timeout-sec must be positive")
    return args


def _float_list(value: Any, length: int, *, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain {length} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


def _optional_float_list(value: Any, length: int, *, name: str) -> list[float] | None:
    return None if value is None else _float_list(value, length, name=name)


def parse_state_packet(data: bytes) -> dict[str, Any]:
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict) or packet.get("schema") != SCHEMA:
        raise ValueError(f"unexpected state schema: {packet.get('schema') if isinstance(packet, dict) else None}")
    arms = packet.get("arms")
    if not isinstance(arms, dict):
        raise ValueError("state packet has no arms object")
    for side in ("left", "right"):
        arm = arms.get(side)
        if not isinstance(arm, dict):
            raise ValueError(f"state packet has no {side} arm")
        arm["q"] = _float_list(arm.get("q"), 7, name=f"{side}.q")
        arm["dq"] = _float_list(arm.get("dq"), 7, name=f"{side}.dq")
        arm["tcp_pose_base"] = _float_list(arm.get("tcp_pose_base"), 7, name=f"{side}.tcp_pose_base")
        arm["tcp_pose_world"] = _float_list(arm.get("tcp_pose_world"), 7, name=f"{side}.tcp_pose_world")
        quest = arm.get("quest")
        if quest is not None:
            if not isinstance(quest, dict):
                raise ValueError(f"{side}.quest must be an object")
            quest["controller_pose_openxr"] = _optional_float_list(
                quest.get("controller_pose_openxr"), 7, name=f"{side}.quest.controller_pose_openxr"
            )
            quest["controller_delta_base"] = _optional_float_list(
                quest.get("controller_delta_base"), 3, name=f"{side}.quest.controller_delta_base"
            )
            quest["target_packet_pose_base_tcp"] = _optional_float_list(
                quest.get("target_packet_pose_base_tcp"), 7, name=f"{side}.quest.target_packet_pose_base_tcp"
            )
            quest["mapped_goal_pose_base_tcp"] = _optional_float_list(
                quest.get("mapped_goal_pose_base_tcp"), 7, name=f"{side}.quest.mapped_goal_pose_base_tcp"
            )
            quest["tcp_rot_offset_wxyz"] = _optional_float_list(
                quest.get("tcp_rot_offset_wxyz"), 4, name=f"{side}.quest.tcp_rot_offset_wxyz"
            )
    return packet


def quaternion_wxyz_to_rpy_deg(quaternion: list[float]) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


def _numbers(values: list[float], precision: int = 4) -> str:
    return "[" + ", ".join(f"{value: .{precision}f}" for value in values) + "]"


def axis_map_formula(axis_map: str) -> str:
    components = []
    for raw_token in str(axis_map).split(","):
        token = raw_token.strip().lower()
        sign = "-" if token.startswith("-") else "+"
        axis = token[1:] if token.startswith(("-", "+")) else token
        if axis not in {"x", "y", "z"}:
            return f"axis_map({axis_map})"
        components.append(f"{sign}dOpenXR.{axis}")
    if len(components) != 3:
        return f"axis_map({axis_map})"
    return "[" + ", ".join(components) + "]"


def orientation_mapping_description(mode: str) -> str:
    if mode == "relative":
        return "q_goal=(q_packet * inverse(q_packet@engage)) * q_TCP@engage"
    if mode == "packet":
        return "q_goal=q_packet"
    if mode == "reference":
        return "q_goal=q_TCP@engage (orientation held)"
    if mode == "current":
        return "q_goal=q_TCP_current (orientation not commanded)"
    return f"q_goal uses unknown mode {mode!r}"


def format_state(packet: dict[str, Any], *, received_time: float | None = None) -> str:
    received_time = time.time() if received_time is None else received_time
    stamp_ns = int(packet.get("stamp_ns", 0))
    age_ms = max(0.0, (received_time - stamp_ns / 1e9) * 1000.0) if stamp_ns else math.nan
    lines = [
        f"Dual-arm state  cycle={int(packet.get('servo_cycle', 0))}  age={age_ms:.1f} ms",
        "q: rad / deg, dq: rad/s, quaternion: wxyz, RPY: deg",
    ]
    for side in ("left", "right"):
        arm = packet["arms"][side]
        q = arm["q"]
        dq = arm["dq"]
        base = arm["tcp_pose_base"]
        world = arm["tcp_pose_world"]
        quest = arm.get("quest")
        rpy = quaternion_wxyz_to_rpy_deg(base[3:])
        lines.extend(
            [
                "",
                f"{side.upper():5s} serial={arm.get('serial', '-')}  ready={bool(arm.get('ready', False))}",
                f"  q rad       {_numbers(q)}",
                f"  q deg       {_numbers([math.degrees(value) for value in q], 2)}",
                f"  dq rad/s    {_numbers(dq)}",
                f"  TCP base    xyz(m)={_numbers(base[:3])}  quat={_numbers(base[3:])}",
                f"              RPY(deg)={_numbers(list(rpy), 2)}",
                f"  TCP world   xyz(m)={_numbers(world[:3])}  quat={_numbers(world[3:])}",
            ]
        )
        if not isinstance(quest, dict) or not quest.get("available", False):
            lines.append("  Quest       unavailable")
            continue
        controller_pose = quest.get("controller_pose_openxr")
        quest_age_ms = 1000.0 * float(quest.get("age_sec") or 0.0)
        lines.append(
            f"  Quest       seq={quest.get('seq', '-')} age={quest_age_ms:.1f}ms "
            f"tracking_ready={bool(quest.get('motion_data_ready', False))}"
        )
        lines.append(
            f"    {quest.get('enable_button', 'enable')}={float(quest.get('enable_value', 0.0)):.3f} "
            f"enabled={bool(quest.get('enabled', False))}  "
            f"{quest.get('gripper_button', 'gripper')}={float(quest.get('gripper_value', 0.0)):.3f} "
            f"closed={bool(quest.get('gripper_closed', False))}"
        )
        if controller_pose is not None:
            controller_rpy = quaternion_wxyz_to_rpy_deg(controller_pose[3:])
            lines.append(
                f"    OpenXR     xyz(m)={_numbers(controller_pose[:3])} "
                f"quat={_numbers(controller_pose[3:])} RPY(deg)={_numbers(list(controller_rpy), 2)}"
            )
        if quest.get("controller_delta_base") is not None:
            lines.append(f"    mapped dxyz base(m)={_numbers(quest['controller_delta_base'])}")
        if quest.get("target_packet_pose_base_tcp") is not None:
            lines.append(f"    packet pose base TCP={_numbers(quest['target_packet_pose_base_tcp'])}")
        if quest.get("mapped_goal_pose_base_tcp") is not None:
            lines.append(f"    mapped goal base TCP={_numbers(quest['mapped_goal_pose_base_tcp'])}")
        axis_map = str(quest.get("axis_map", "x,y,z"))
        publisher_scale = float(quest.get("publisher_position_scale", 1.0))
        isaac_scale = float(quest.get("isaac_position_scale", 1.0))
        position_mode = str(quest.get("position_mode", "relative"))
        orientation_mode = str(quest.get("orientation_mode", "relative"))
        enable_button = str(quest.get("enable_button", "squeeze"))
        settle_sec = float(quest.get("engage_settle_sec", 0.0))
        lines.append("    Mapping:")
        lines.append(
            f"      XYZ axis_map={axis_map}: dbase={axis_map_formula(axis_map)} "
            f"* publisher_scale({publisher_scale:g}) * isaac_scale({isaac_scale:g})"
        )
        if position_mode == "relative":
            lines.append(
                f"      XYZ goal=TCP@{enable_button}-engage + dbase; "
                f"zero is latched after {settle_sec:g}s settle"
            )
        else:
            lines.append(f"      XYZ goal uses packet directly (position_mode={position_mode})")
        tcp_offset = quest.get("tcp_rot_offset_wxyz")
        offset_text = "unknown" if tcp_offset is None else _numbers(tcp_offset)
        lines.append(
            f"      QUAT q_packet=axis_map(q_OpenXR) * tcp_offset({offset_text}); "
            f"{orientation_mapping_description(orientation_mode)}"
        )
        lines.append(
            f"      release {enable_button}: stop following and hold the last mapped goal; "
            f"workspace_clip={bool(quest.get('workspace_clipping', False))}"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.host, args.port))
    listener.settimeout(min(args.timeout_sec, 0.5))
    min_period = 1.0 / args.rate_hz
    last_print = -math.inf
    last_packet = time.monotonic()
    last_warning = -math.inf
    print(f"[arm-state] listening on udp://{args.host}:{args.port}; Ctrl-C to exit", flush=True)
    try:
        while True:
            try:
                data, _address = listener.recvfrom(65535)
            except socket.timeout:
                current = time.monotonic()
                if current - last_packet >= args.timeout_sec and current - last_warning >= args.timeout_sec:
                    print(
                        f"[arm-state] no state received for {current - last_packet:.1f}s; "
                        "start/restart the dual-arm Isaac stack with state monitoring enabled",
                        file=sys.stderr,
                        flush=True,
                    )
                    last_warning = current
                continue
            try:
                packet = parse_state_packet(data)
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
                print(f"[arm-state] ignored invalid packet: {exc}", file=sys.stderr, flush=True)
                continue
            current = time.monotonic()
            last_packet = current
            if not args.once and current - last_print < min_period:
                continue
            if args.clear and not args.once and sys.stdout.isatty():
                sys.stdout.write("\033[2J\033[H")
            print(format_state(packet), flush=True)
            last_print = current
            if args.once:
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        listener.close()


if __name__ == "__main__":
    raise SystemExit(main())
