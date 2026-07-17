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
DEFAULT_FORWARD_HOST = "127.0.0.1"
TCP_FORWARD_LOCAL = [0.0, 0.0, 1.0]
DEFAULT_TCP_FORWARD_BASE = [1.0, 0.0, 0.0]
HAND_SEPARATION_M = 0.40
HAND_SEPARATION_TOLERANCE_M = 0.01
POSITION_ERROR_TOLERANCE_M = 0.03
ORIENTATION_ERROR_TOLERANCE_DEG = 10.0
FORWARD_DIRECTION_TOLERANCE_DEG = 15.0
ANSI_GREEN_BOLD = "\033[1;32m"
ANSI_RESET = "\033[0m"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST, help="UDP address to bind")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="UDP port to bind")
    parser.add_argument("--rate-hz", type=float, default=5.0, help="maximum terminal refresh rate")
    parser.add_argument("--timeout-sec", type=float, default=3.0, help="warning interval while no packets arrive")
    parser.add_argument(
        "--forward-host",
        default=DEFAULT_FORWARD_HOST,
        help="UDP address used to forward valid packets; active only when --forward-port is nonzero",
    )
    parser.add_argument(
        "--forward-port",
        type=int,
        default=0,
        help="forward every valid state packet to this UDP port (0 disables forwarding)",
    )
    parser.add_argument("--once", action="store_true", help="print one valid packet and exit")
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show raw joints, poses, quaternions, and mapping formulas instead of the concise collection gate",
    )
    parser.add_argument(
        "--clear",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="clear the terminal before each refresh",
    )
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    if not 0 <= args.forward_port <= 65535:
        parser.error("--forward-port must be between 0 and 65535")
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


def _optional_matrix3(value: Any, *, name: str) -> list[list[float]] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must contain 3 rows")
    return [_float_list(row, 3, name=f"{name}[{index}]") for index, row in enumerate(value)]


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
            quest["command_pose_base_tcp"] = _optional_float_list(
                quest.get("command_pose_base_tcp"), 7, name=f"{side}.quest.command_pose_base_tcp"
            )
            quest["tcp_rot_offset_wxyz"] = _optional_float_list(
                quest.get("tcp_rot_offset_wxyz"), 4, name=f"{side}.quest.tcp_rot_offset_wxyz"
            )
            quest["calibration_rotation_base_from_mapped"] = _optional_matrix3(
                quest.get("calibration_rotation_base_from_mapped"),
                name=f"{side}.quest.calibration_rotation_base_from_mapped",
            )
            relative_reference = quest.get("relative_reference")
            if relative_reference is not None:
                if not isinstance(relative_reference, dict):
                    raise ValueError(f"{side}.quest.relative_reference must be an object")
                relative_reference["controller_orientation_base"] = _float_list(
                    relative_reference.get("controller_orientation_base"),
                    4,
                    name=f"{side}.quest.relative_reference.controller_orientation_base",
                )
                relative_reference["tcp_pose_base"] = _float_list(
                    relative_reference.get("tcp_pose_base"),
                    7,
                    name=f"{side}.quest.relative_reference.tcp_pose_base",
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


def normalize_quaternion_wxyz(quaternion: list[float]) -> list[float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [float(value) / norm for value in quaternion]


def quaternion_inverse_wxyz(quaternion: list[float]) -> list[float]:
    w, x, y, z = normalize_quaternion_wxyz(quaternion)
    return [w, -x, -y, -z]


def quaternion_multiply_wxyz(left: list[float], right: list[float]) -> list[float]:
    lw, lx, ly, lz = normalize_quaternion_wxyz(left)
    rw, rx, ry, rz = normalize_quaternion_wxyz(right)
    return normalize_quaternion_wxyz(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def quaternion_error_deg(left: list[float], right: list[float]) -> float:
    left_q = normalize_quaternion_wxyz(left)
    right_q = normalize_quaternion_wxyz(right)
    dot = abs(sum(a * b for a, b in zip(left_q, right_q)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def position_error_m(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left[:3], right[:3])))


def rotate_vector_wxyz(quaternion: list[float], vector: list[float]) -> list[float]:
    pure_vector = [0.0, *[float(value) for value in vector]]
    rotated = quaternion_multiply_wxyz(
        quaternion_multiply_wxyz(quaternion, pure_vector),
        quaternion_inverse_wxyz(quaternion),
    )
    return rotated[1:4]


def vector_angle_deg(left: list[float], right: list[float]) -> float:
    left_norm = math.sqrt(sum(float(value) ** 2 for value in left))
    right_norm = math.sqrt(sum(float(value) ** 2 for value in right))
    if left_norm <= 0.0 or right_norm <= 0.0:
        return 180.0
    dot = sum(float(a) * float(b) for a, b in zip(left, right)) / (left_norm * right_norm)
    return math.degrees(math.acos(min(1.0, max(-1.0, dot))))


def horizontal_perpendicular_error_deg(direction: list[float], line: list[float]) -> float:
    direction_xy = [float(direction[0]), float(direction[1]), 0.0]
    line_xy = [float(line[0]), float(line[1]), 0.0]
    angle = vector_angle_deg(direction_xy, line_xy)
    return abs(angle - 90.0)


def horizontal_mutual_angle_deg(left: list[float], right: list[float]) -> float:
    return vector_angle_deg(
        [float(left[0]), float(left[1]), 0.0],
        [float(right[0]), float(right[1]), 0.0],
    )


def parse_axis_map_components(axis_map: str) -> list[tuple[int, float]]:
    components: list[tuple[int, float]] = []
    for raw_token in str(axis_map).split(","):
        token = raw_token.strip().lower()
        sign = -1.0 if token.startswith("-") else 1.0
        axis = token[1:] if token.startswith(("-", "+")) else token
        if axis not in {"x", "y", "z"}:
            raise ValueError(f"invalid Quest axis map: {axis_map!r}")
        components.append(({"x": 0, "y": 1, "z": 2}[axis], sign))
    if len(components) != 3 or len({index for index, _sign in components}) != 3:
        raise ValueError(f"Quest axis map must use x, y, z exactly once: {axis_map!r}")
    return components


def apply_axis_map_vector(vector: list[float], axis_map: str) -> list[float]:
    values = _float_list(vector, 3, name="axis-map vector")
    return [sign * values[index] for index, sign in parse_axis_map_components(axis_map)]


def invert_axis_map_vector(vector: list[float], axis_map: str) -> list[float]:
    mapped = _float_list(vector, 3, name="inverse-axis-map vector")
    original = [0.0, 0.0, 0.0]
    for output_index, (input_index, sign) in enumerate(parse_axis_map_components(axis_map)):
        original[input_index] = sign * mapped[output_index]
    return original


def mapped_controller_position(quest: dict[str, Any]) -> list[float]:
    pose = quest.get("controller_pose_openxr")
    if pose is None:
        raise ValueError("Quest controller pose unavailable")
    return apply_axis_map_vector(pose[:3], str(quest.get("axis_map", "x,y,z")))


def mapped_controller_forward(quest: dict[str, Any]) -> list[float]:
    """Map the raw controller pose and configured TCP offset without squeeze."""

    pose = quest.get("controller_pose_openxr")
    offset = quest.get("tcp_rot_offset_wxyz")
    if pose is None or offset is None:
        raise ValueError("Quest controller orientation unavailable")
    axis_map = str(quest.get("axis_map", "x,y,z"))
    # q_tcp = (A * q_openxr * A^-1) * q_offset. Apply that rotation to
    # local +Z using vectors so signed axis permutations stay explicit.
    offset_forward_base = rotate_vector_wxyz(offset, TCP_FORWARD_LOCAL)
    offset_forward_openxr = invert_axis_map_vector(offset_forward_base, axis_map)
    rotated_openxr = rotate_vector_wxyz(pose[3:7], offset_forward_openxr)
    return apply_axis_map_vector(rotated_openxr, axis_map)


def _quest_pose_for_match(packet: dict[str, Any], side: str) -> dict[str, Any] | None:
    arm = packet.get("arms", {}).get(side)
    if not isinstance(arm, dict):
        return None
    quest = arm.get("quest")
    if (
        not isinstance(quest, dict)
        or not quest.get("available", False)
        or not quest.get("motion_data_ready", False)
        or quest.get("controller_pose_openxr") is None
    ):
        return None
    return quest


def quest_hand_match_lines(packet: dict[str, Any], *, color: bool = False) -> tuple[list[str], bool]:
    """Return two live geometry checks plus dual-squeeze frame-lock state."""

    left = _quest_pose_for_match(packet, "left")
    right = _quest_pose_for_match(packet, "right")
    if left is None or right is None:
        return [
            "SPACING WAIT | both tracked controller poses are required",
            "DIRECTION WAIT | both tracked controller poses are required",
        ], False

    try:
        left_position = mapped_controller_position(left)
        right_position = mapped_controller_position(right)
        delta = [right_position[index] - left_position[index] for index in range(3)]
        separation = math.sqrt(sum(value * value for value in delta))
        spacing_ok = abs(separation - HAND_SEPARATION_M) <= HAND_SEPARATION_TOLERANCE_M
        left_forward = mapped_controller_forward(left)
        right_forward = mapped_controller_forward(right)
        left_angle = horizontal_perpendicular_error_deg(left_forward, delta)
        right_angle = horizontal_perpendicular_error_deg(right_forward, delta)
        mutual_angle = horizontal_mutual_angle_deg(left_forward, right_forward)
        direction_ok = (
            left_angle <= FORWARD_DIRECTION_TOLERANCE_DEG
            and right_angle <= FORWARD_DIRECTION_TOLERANCE_DEG
            and mutual_angle <= FORWARD_DIRECTION_TOLERANCE_DEG
        )
    except ValueError:
        return [
            "SPACING WAIT | invalid controller pose or axis map",
            "DIRECTION WAIT | invalid controller pose, axis map, or TCP offset",
        ], False

    calibration_confirmed = bool(
        left.get("calibration_confirmed", False) and right.get("calibration_confirmed", False)
    )
    both_squeeze = bool(left.get("both_squeeze", False) and right.get("both_squeeze", False))
    strict_geometry = bool(
        left.get("calibration_strict_geometry", False)
        or right.get("calibration_strict_geometry", False)
    )
    geometry_ok = spacing_ok and direction_ok
    if calibration_confirmed:
        frame_state = "LOCKED"
    elif both_squeeze and (geometry_ok or not strict_geometry):
        frame_state = "CONFIRMING"
    else:
        frame_state = "HOLD_BOTH_SQUEEZE" if geometry_ok or not strict_geometry else "ALIGN"
    gate_mode = "STRICT" if strict_geometry else "ADVISORY"
    lines = [
        f"SPACING {_status(spacing_ok)} | delta_base_xyz={_numbers(delta, 3)}m "
        f"distance={separation:.3f}m target={HAND_SEPARATION_M:.2f}±{HAND_SEPARATION_TOLERANCE_M:.2f}m "
        f"gate={gate_mode} frame={frame_state}",
        f"DIRECTION {_status(direction_ok)} | target=perpendicular-to-line-in-XY,same-way "
        f"left_perp_error={left_angle:.1f}deg right_perp_error={right_angle:.1f}deg "
        f"mutual_error={mutual_angle:.1f}deg"
        f"<={FORWARD_DIRECTION_TOLERANCE_DEG:.0f}deg",
    ]
    if color:
        lines = [
            f"{ANSI_GREEN_BOLD}{line}{ANSI_RESET}" if " PASS |" in line else line
            for line in lines
        ]
    return lines, calibration_confirmed and (geometry_ok or not strict_geometry)


def _axis_correction_text(delta: list[float], *, unit_scale: float = 100.0, unit: str = "cm") -> str:
    corrections = []
    for axis, value in zip(("X", "Y", "Z"), delta):
        if abs(float(value)) >= 1e-4:
            corrections.append(f"{'+' if value >= 0.0 else '-'}{axis} {abs(value) * unit_scale:.2f}{unit}")
    return ", ".join(corrections) if corrections else "no translation correction"


def direction_correction_axis(
    current: list[float],
    target: list[float],
) -> tuple[list[float], float]:
    angle_deg = vector_angle_deg(current, target)
    cross = [
        current[1] * target[2] - current[2] * target[1],
        current[2] * target[0] - current[0] * target[2],
        current[0] * target[1] - current[1] * target[0],
    ]
    norm = math.sqrt(sum(float(value) ** 2 for value in cross))
    if norm <= 1e-9:
        axis = [0.0, 1.0, 0.0] if angle_deg > 90.0 else [0.0, 0.0, 0.0]
    else:
        axis = [float(value) / norm for value in cross]
    return axis, angle_deg


def _status(passed: bool) -> str:
    return "PASS" if passed else "FAIL"


def quest_collection_assessment(arm: dict[str, Any]) -> tuple[list[str], bool]:
    """Assess whether live Quest-to-TCP mapping is ready for data collection."""

    quest = arm.get("quest")
    if not isinstance(quest, dict) or not quest.get("available", False):
        return [
            "    Assessment: WAIT (Quest input unavailable)",
            "      correction: start/connect the Quest publisher, keep both controllers tracked, then hold squeeze",
        ], False

    lines: list[str] = []
    checks = [
        bool(arm.get("ready", False)),
        bool(quest.get("motion_data_ready", False)),
        bool(quest.get("control_active", False)),
    ]
    lines.append(
        "    Assessment: "
        f"robot_ready={_status(checks[0])} "
        f"quest_tracking={_status(checks[1])} "
        f"quest_control={_status(checks[2])}"
    )
    if not checks[0]:
        lines.append("      correction: wait for both DRDK/SimPlugin sides to reach READY; reset faults if needed")
    if not checks[1]:
        lines.append("      correction: restore Quest controller tracking before moving the robot")
    elif not checks[2]:
        lines.append("      correction: hold squeeze for the settle interval to latch the relative pose origin")

    actual = arm["tcp_pose_base"]
    command = quest.get("command_pose_base_tcp")
    if command is None:
        lines.append("      relative pose: WAIT (no rate-limited command pose yet)")
        checks.append(False)
    else:
        pos_error = position_error_m(actual, command)
        ori_error = quaternion_error_deg(actual[3:7], command[3:7])
        tracking_ok = (
            pos_error <= POSITION_ERROR_TOLERANCE_M
            and ori_error <= ORIENTATION_ERROR_TOLERANCE_DEG
        )
        checks.append(tracking_ok)
        lines.append(
            f"      relative pose: {_status(tracking_ok)} "
            f"position_error={pos_error:.4f}m<={POSITION_ERROR_TOLERANCE_M:.3f}m "
            f"orientation_error={ori_error:.2f}deg<={ORIENTATION_ERROR_TOLERANCE_DEG:.1f}deg"
        )
        if not tracking_ok:
            translation_delta = [float(command[index]) - float(actual[index]) for index in range(3)]
            lines.append(
                "        correction: hold the controller still and let TCP move "
                f"{_axis_correction_text(translation_delta)} in base coordinates"
            )
            if ori_error > ORIENTATION_ERROR_TOLERANCE_DEG:
                lines.append(
                    f"        correction: keep orientation fixed until the robot closes the {ori_error:.2f}deg lag"
                )

    goal = quest.get("mapped_goal_pose_base_tcp")
    orientation = command[3:7] if command is not None else (None if goal is None else goal[3:7])
    if orientation is None:
        lines.append("      TCP forward +X: WAIT (no mapped orientation yet)")
        checks.append(False)
    else:
        forward = rotate_vector_wxyz(orientation, TCP_FORWARD_LOCAL)
        forward_error = vector_angle_deg(forward, DEFAULT_TCP_FORWARD_BASE)
        forward_ok = forward_error <= FORWARD_DIRECTION_TOLERANCE_DEG
        checks.append(forward_ok)
        lines.append(
            f"      TCP forward +X: {_status(forward_ok)} "
            f"forward_base={_numbers(forward)} target={_numbers(DEFAULT_TCP_FORWARD_BASE)} "
            f"error={forward_error:.2f}deg<={FORWARD_DIRECTION_TOLERANCE_DEG:.1f}deg"
        )
        if not forward_ok:
            correction_axis, correction_angle = direction_correction_axis(
                forward,
                DEFAULT_TCP_FORWARD_BASE,
            )
            lines.append(
                "        correction: rotate the mapped controller/TCP about base axis "
                f"{_numbers(correction_axis)} by +{correction_angle:.2f}deg toward +X"
            )

    reference = quest.get("relative_reference")
    packet_pose = quest.get("target_packet_pose_base_tcp")
    delta = quest.get("controller_delta_base")
    if (
        isinstance(reference, dict)
        and packet_pose is not None
        and delta is not None
        and goal is not None
        and str(quest.get("orientation_mode", "relative")) == "relative"
        and not bool(quest.get("workspace_clipping", False))
    ):
        scale = float(quest.get("isaac_position_scale", 1.0))
        deadband = float(quest.get("isaac_position_deadband_m", 0.0))
        mapped_delta = [float(value) * scale for value in delta]
        mapped_delta = [0.0 if abs(value) < deadband else value for value in mapped_delta]
        expected_xyz = [
            float(reference["tcp_pose_base"][index]) + mapped_delta[index]
            for index in range(3)
        ]
        expected_quat = quaternion_multiply_wxyz(
            quaternion_multiply_wxyz(
                packet_pose[3:7],
                quaternion_inverse_wxyz(reference["controller_orientation_base"]),
            ),
            reference["tcp_pose_base"][3:7],
        )
        mapping_pos_error = position_error_m(goal, expected_xyz)
        mapping_ori_error = quaternion_error_deg(goal[3:7], expected_quat)
        mapping_ok = mapping_pos_error <= 1e-3 and mapping_ori_error <= 0.5
        checks.append(mapping_ok)
        lines.append(
            f"      relative mapping: {_status(mapping_ok)} "
            f"position_error={mapping_pos_error:.5f}m orientation_error={mapping_ori_error:.3f}deg"
        )
    else:
        lines.append("      relative mapping: WAIT (hold squeeze to establish a fresh relative anchor)")
        lines.append("        correction: keep the controller still and hold squeeze through the settle interval")
        checks.append(False)

    return lines, all(checks)


def _assessment_status(lines: list[str], marker: str) -> str:
    line = next((item for item in lines if marker in item), "")
    if "PASS" in line:
        return "PASS"
    if "FAIL" in line:
        return "FAIL"
    return "WAIT"


def format_concise_state(packet: dict[str, Any], *, color: bool = False) -> str:
    """Render exactly two geometry-and-frame-confirmation lines."""

    lines, _ready = quest_hand_match_lines(packet, color=color)
    return "\n".join(lines)


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


def format_state(
    packet: dict[str, Any],
    *,
    received_time: float | None = None,
    color: bool = False,
) -> str:
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
        torque = arm.get("torque")
        if isinstance(torque, dict):
            for key, label in (
                ("tau", "tau Nm"),
                ("tau_ext", "tau_ext Nm"),
                ("tau_max", "tau_max Nm"),
                ("ratio", "torque ratio"),
            ):
                values = torque.get(key)
                if isinstance(values, (list, tuple)) and len(values) == 7:
                    lines.append(f"  {label:12s}{_numbers([float(value) for value in values])}")
            lines.append(f"  torque guard frozen={bool(torque.get('frozen', False))}")
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
        calibration_rotation = quest.get("calibration_rotation_base_from_mapped")
        lines.append(
            "    shared frame "
            f"confirmed={bool(quest.get('calibration_confirmed', False))} "
            f"both_squeeze={bool(quest.get('both_squeeze', False))}"
        )
        if calibration_rotation is not None:
            lines.append(
                "      C(base<-axis-mapped Quest)="
                + "["
                + ", ".join(_numbers(row) for row in calibration_rotation)
                + "]"
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
        if quest.get("command_pose_base_tcp") is not None:
            lines.append(f"    limited command base TCP={_numbers(quest['command_pose_base_tcp'])}")
        axis_map = str(quest.get("axis_map", "x,y,z"))
        publisher_scale = float(quest.get("publisher_position_scale", 1.0))
        isaac_scale = float(quest.get("isaac_position_scale", 1.0))
        position_mode = str(quest.get("position_mode", "relative"))
        orientation_mode = str(quest.get("orientation_mode", "relative"))
        enable_button = str(quest.get("enable_button", "squeeze"))
        settle_sec = float(quest.get("engage_settle_sec", 0.0))
        lines.append("    Mapping:")
        calibration_prefix = "C * " if calibration_rotation is not None else ""
        lines.append(
            f"      XYZ axis_map={axis_map}: dbase={calibration_prefix}{axis_map_formula(axis_map)} "
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
            f"      QUAT q_packet=C*axis_map(q_OpenXR)*C^-1 * tcp_offset({offset_text}); "
            f"{orientation_mapping_description(orientation_mode)}"
        )
        lines.append(
            f"      release {enable_button}: stop following and hold the last mapped goal; "
            f"workspace_clip={bool(quest.get('workspace_clipping', False))}"
        )
    lines.append("")
    hand_lines, _ready = quest_hand_match_lines(packet, color=color)
    lines.extend(hand_lines)
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.host, args.port))
    listener.settimeout(min(args.timeout_sec, 0.5))
    forwarder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.forward_port else None
    forward_address = (args.forward_host, args.forward_port)
    min_period = 1.0 / args.rate_hz
    last_print = -math.inf
    last_packet = time.monotonic()
    last_warning = -math.inf
    if args.verbose:
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
            if forwarder is not None:
                try:
                    forwarder.sendto(data, forward_address)
                except OSError as exc:
                    print(f"[arm-state] state forwarding failed: {exc}", file=sys.stderr, flush=True)
            current = time.monotonic()
            last_packet = current
            if not args.once and current - last_print < min_period:
                continue
            if args.clear and not args.once and sys.stdout.isatty():
                sys.stdout.write("\033[2J\033[H")
            if args.verbose:
                output = format_state(packet, color=sys.stdout.isatty())
            else:
                output = format_concise_state(packet, color=sys.stdout.isatty())
            print(output, flush=True)
            last_print = current
            if args.once:
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        listener.close()
        if forwarder is not None:
            forwarder.close()


if __name__ == "__main__":
    raise SystemExit(main())
