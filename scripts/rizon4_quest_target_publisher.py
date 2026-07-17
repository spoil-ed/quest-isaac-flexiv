#!/usr/bin/env python3
"""Publish Quest controller relative motion to the Rizon4 Isaac target UDP port."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path
from typing import Iterable


DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TELEVUER_ROOT = REPO_ROOT / "third_party" / "televuer" / "src"
DEFAULT_CERT_FILE = REPO_ROOT / "configs" / "xr_teleoperate" / "cert.pem"
DEFAULT_KEY_FILE = REPO_ROOT / "configs" / "xr_teleoperate" / "key.pem"
DEFAULT_HOST_IP = "192.168.32.10"
DEFAULT_UDP_HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 45679
DEFAULT_TCP_ROT_OFFSET_WXYZ = (0.0, 0.70710678, 0.0, 0.70710678)
DEFAULT_AXIS_MAP = "-z,-x,y"
DEFAULT_HAND_SEPARATION_M = 0.40
DEFAULT_HAND_SEPARATION_TOLERANCE_M = 0.01
DEFAULT_HAND_DIRECTION_TOLERANCE_DEG = 15.0


def _as_float_list(values: Iterable[float], expected_len: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def parse_csv_floats(value: str, expected_len: int, name: str) -> list[float]:
    return _as_float_list((item.strip() for item in value.split(",") if item.strip()), expected_len, name)


def parse_axis_map(value: str) -> list[tuple[int, float]]:
    axes = {"x": 0, "y": 1, "z": 2}
    result: list[tuple[int, float]] = []
    for item in value.split(","):
        token = item.strip().lower()
        sign = -1.0 if token.startswith("-") else 1.0
        axis = token[1:] if token.startswith(("-", "+")) else token
        if axis not in axes:
            raise ValueError(f"invalid axis map component: {item!r}")
        result.append((axes[axis], sign))
    if len(result) != 3:
        raise ValueError("axis map must contain three components")
    return result


def apply_axis_map(position: Iterable[float], axis_map: list[tuple[int, float]]) -> list[float]:
    values = _as_float_list(position, 3, "position")
    return [sign * values[index] for index, sign in axis_map]


def axis_map_matrix(axis_map: list[tuple[int, float]]) -> list[list[float]]:
    matrix = [[0.0, 0.0, 0.0] for _ in range(3)]
    for output_index, (input_index, sign) in enumerate(axis_map):
        matrix[output_index][input_index] = float(sign)
    return matrix


def multiply_matrix3(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(float(a[row][inner]) * float(b[inner][col]) for inner in range(3)) for col in range(3)]
        for row in range(3)
    ]


def transpose_matrix3(matrix: list[list[float]]) -> list[list[float]]:
    return [[float(matrix[row][col]) for row in range(3)] for col in range(3)]


def multiply_matrix3_vector(matrix: list[list[float]], vector: Iterable[float]) -> list[float]:
    values = _as_float_list(vector, 3, "vector")
    return [sum(float(matrix[row][col]) * values[col] for col in range(3)) for row in range(3)]


def dot3(left: Iterable[float], right: Iterable[float]) -> float:
    a = _as_float_list(left, 3, "left vector")
    b = _as_float_list(right, 3, "right vector")
    return sum(x * y for x, y in zip(a, b))


def norm3(vector: Iterable[float]) -> float:
    values = _as_float_list(vector, 3, "vector")
    return math.sqrt(sum(value * value for value in values))


def normalize3(vector: Iterable[float]) -> list[float]:
    values = _as_float_list(vector, 3, "vector")
    length = norm3(values)
    if length <= 1e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return [value / length for value in values]


def cross3(left: Iterable[float], right: Iterable[float]) -> list[float]:
    ax, ay, az = _as_float_list(left, 3, "left vector")
    bx, by, bz = _as_float_list(right, 3, "right vector")
    return [ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx]


def vector_angle_deg(left: Iterable[float], right: Iterable[float]) -> float:
    left_values = _as_float_list(left, 3, "left vector")
    right_values = _as_float_list(right, 3, "right vector")
    denominator = norm3(left_values) * norm3(right_values)
    if denominator <= 1e-9:
        return 180.0
    cosine = max(-1.0, min(1.0, dot3(left_values, right_values) / denominator))
    return math.degrees(math.acos(cosine))


def normalize_quat_wxyz(values: Iterable[float]) -> list[float]:
    quat = _as_float_list(values, 4, "quaternion")
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in quat]


def quat_inverse_wxyz(quat: Iterable[float]) -> list[float]:
    w, x, y, z = normalize_quat_wxyz(quat)
    return [w, -x, -y, -z]


def quat_multiply_wxyz(a: Iterable[float], b: Iterable[float]) -> list[float]:
    aw, ax, ay, az = normalize_quat_wxyz(a)
    bw, bx, by, bz = normalize_quat_wxyz(b)
    return normalize_quat_wxyz(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def quat_to_rotation_matrix(quaternion: Iterable[float]) -> list[list[float]]:
    w, x, y, z = normalize_quat_wxyz(quaternion)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]


def rotate_vector_wxyz(quaternion: Iterable[float], vector: Iterable[float]) -> list[float]:
    return multiply_matrix3_vector(quat_to_rotation_matrix(quaternion), vector)


def rotation_matrix_to_quat_wxyz(matrix: list[list[float]]) -> list[float]:
    m00, m01, m02 = matrix[0][:3]
    m10, m11, m12 = matrix[1][:3]
    m20, m21, m22 = matrix[2][:3]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return normalize_quat_wxyz([0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s])
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return normalize_quat_wxyz([(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s])
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return normalize_quat_wxyz([(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s])
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return normalize_quat_wxyz([(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s])


def pose_matrix_position(matrix: list[list[float]]) -> list[float]:
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("pose matrix must be 4x4")
    return [float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3])]


def pose_matrix_quat_wxyz(matrix: list[list[float]], axis_map: list[tuple[int, float]] | None = None) -> list[float]:
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise ValueError("pose matrix must be 4x4")
    rotation = [row[:3] for row in matrix[:3]]
    if axis_map is not None:
        transform = axis_map_matrix(axis_map)
        rotation = multiply_matrix3(multiply_matrix3(transform, rotation), transpose_matrix3(transform))
    return rotation_matrix_to_quat_wxyz(rotation)


class QuestSharedFrameCalibration:
    """Freeze a shared dual-controller frame after a valid two-squeeze confirmation."""

    def __init__(
        self,
        *,
        axis_map: str = DEFAULT_AXIS_MAP,
        tcp_rot_offset_wxyz: Iterable[float] = DEFAULT_TCP_ROT_OFFSET_WXYZ,
        settle_sec: float = 0.25,
        separation_m: float = DEFAULT_HAND_SEPARATION_M,
        separation_tolerance_m: float = DEFAULT_HAND_SEPARATION_TOLERANCE_M,
        direction_tolerance_deg: float = DEFAULT_HAND_DIRECTION_TOLERANCE_DEG,
    ) -> None:
        self.axis_map = parse_axis_map(axis_map)
        self.tcp_rot_offset_wxyz = normalize_quat_wxyz(tcp_rot_offset_wxyz)
        self.settle_sec = max(0.0, float(settle_sec))
        self.separation_m = float(separation_m)
        self.separation_tolerance_m = max(0.0, float(separation_tolerance_m))
        self.direction_tolerance_deg = max(0.0, float(direction_tolerance_deg))
        self.rotation_calibrated_from_mapped: list[list[float]] | None = None
        self._pending_since: float | None = None

    @property
    def confirmed(self) -> bool:
        return self.rotation_calibrated_from_mapped is not None

    def _mapped_position(self, pose: list[list[float]]) -> list[float]:
        return apply_axis_map(pose_matrix_position(pose), self.axis_map)

    def _mapped_forward(self, pose: list[list[float]]) -> list[float]:
        hand_quat = pose_matrix_quat_wxyz(pose, axis_map=self.axis_map)
        tcp_quat = quat_multiply_wxyz(hand_quat, self.tcp_rot_offset_wxyz)
        return rotate_vector_wxyz(tcp_quat, [0.0, 0.0, 1.0])

    def candidate_rotation(
        self,
        left_pose: list[list[float]],
        right_pose: list[list[float]],
    ) -> list[list[float]] | None:
        left_position = self._mapped_position(left_pose)
        right_position = self._mapped_position(right_pose)
        delta = [right_position[index] - left_position[index] for index in range(3)]
        if abs(norm3(delta) - self.separation_m) > self.separation_tolerance_m:
            return None

        left_forward = self._mapped_forward(left_pose)
        right_forward = self._mapped_forward(right_pose)
        line_xy = [delta[0], delta[1], 0.0]
        left_xy = [left_forward[0], left_forward[1], 0.0]
        right_xy = [right_forward[0], right_forward[1], 0.0]
        left_error = abs(vector_angle_deg(left_xy, line_xy) - 90.0)
        right_error = abs(vector_angle_deg(right_xy, line_xy) - 90.0)
        mutual_error = vector_angle_deg(left_xy, right_xy)
        if max(left_error, right_error, mutual_error) > self.direction_tolerance_deg:
            return None

        x_axis = normalize3([left_xy[0] + right_xy[0], left_xy[1] + right_xy[1], 0.0])
        # left-right defines positive lateral direction. Gram-Schmidt removes
        # the residual confirmation-angle error instead of carrying it into XYZ.
        left_from_right = [-line_xy[0], -line_xy[1], 0.0]
        lateral_projection = dot3(left_from_right, x_axis)
        y_axis = normalize3(
            [left_from_right[index] - lateral_projection * x_axis[index] for index in range(3)]
        )
        z_axis = normalize3(cross3(x_axis, y_axis))
        if z_axis[2] < 0.0:
            y_axis = [-value for value in y_axis]
            z_axis = [-value for value in z_axis]
        # Rows project mapped-base vectors onto the frozen Quest frame. This
        # frame is identified with Isaac/Studio base +X/+Y/+Z.
        return [x_axis, y_axis, z_axis]

    def update(
        self,
        left_pose: list[list[float]] | None,
        right_pose: list[list[float]] | None,
        *,
        both_squeeze: bool,
        now: float,
    ) -> bool:
        if self.confirmed:
            return True
        if not both_squeeze or left_pose is None or right_pose is None:
            self._pending_since = None
            return False
        candidate = self.candidate_rotation(left_pose, right_pose)
        if candidate is None:
            self._pending_since = None
            return False
        if self._pending_since is None:
            self._pending_since = float(now)
            if self.settle_sec > 0.0:
                return False
        if float(now) - self._pending_since < self.settle_sec:
            return False
        self.rotation_calibrated_from_mapped = candidate
        return True

    def transform_position(self, mapped_position: Iterable[float]) -> list[float]:
        if self.rotation_calibrated_from_mapped is None:
            return _as_float_list(mapped_position, 3, "mapped position")
        return multiply_matrix3_vector(self.rotation_calibrated_from_mapped, mapped_position)

    def transform_orientation(self, mapped_quaternion: Iterable[float]) -> list[float]:
        if self.rotation_calibrated_from_mapped is None:
            return normalize_quat_wxyz(mapped_quaternion)
        rotation = self.rotation_calibrated_from_mapped
        transformed = multiply_matrix3(
            multiply_matrix3(rotation, quat_to_rotation_matrix(mapped_quaternion)),
            transpose_matrix3(rotation),
        )
        return rotation_matrix_to_quat_wxyz(transformed)


def build_quest_packet(
    *,
    seq: int,
    side: str,
    pose_base_tcp_des: list[float],
    controller_position_openxr: list[float],
    controller_delta_base: list[float],
    now: float,
    reason: str,
    serial_number: str = DEFAULT_SERIAL_NUMBER,
    joint_group: str = DEFAULT_JOINT_GROUP,
) -> dict:
    return {
        "schema": "rizon4_quest_target.v1",
        "serial": str(serial_number),
        "joint_group": str(joint_group),
        "seq": int(seq),
        "side": str(side),
        "pose_base_tcp_des": _as_float_list(pose_base_tcp_des, 7, "pose_base_tcp_des"),
        "controller_position_openxr": _as_float_list(
            controller_position_openxr, 3, "controller_position_openxr"
        ),
        "controller_delta_base": _as_float_list(controller_delta_base, 3, "controller_delta_base"),
        "monotonic_time": float(now),
        "reason": str(reason),
    }


def build_gripper_packet(
    *,
    seq: int,
    side: str,
    closed: bool,
    now: float,
    serial_number: str = DEFAULT_SERIAL_NUMBER,
    joint_group: str = DEFAULT_JOINT_GROUP,
) -> dict:
    return {
        "schema": "rizon4_quest_gripper.v1",
        "serial": str(serial_number),
        "joint_group": str(joint_group),
        "seq": int(seq),
        "side": str(side),
        "closed": bool(closed),
        "monotonic_time": float(now),
    }


def build_quest_input_packet(
    *,
    seq: int,
    side: str,
    motion_data_ready: bool,
    controller_pose_openxr: list[float] | None,
    enable_button: str,
    enable_value: float,
    enabled: bool,
    gripper_button: str,
    gripper_value: float,
    gripper_closed: bool,
    now: float,
    axis_map: str = DEFAULT_AXIS_MAP,
    position_delta_scale: float = 1.0,
    position_deadband: float = 0.0,
    engage_settle_sec: float = 0.25,
    tcp_rot_offset_wxyz: str = "0.0,0.70710678,0.0,0.70710678",
    calibration_confirmed: bool = False,
    both_squeeze: bool = False,
    calibration_rotation_base_from_mapped: list[list[float]] | None = None,
    serial_number: str = DEFAULT_SERIAL_NUMBER,
    joint_group: str = DEFAULT_JOINT_GROUP,
) -> dict:
    return {
        "schema": "rizon4_quest_input.v1",
        "serial": str(serial_number),
        "joint_group": str(joint_group),
        "seq": int(seq),
        "side": str(side),
        "motion_data_ready": bool(motion_data_ready),
        "controller_pose_openxr": (
            None
            if controller_pose_openxr is None
            else _as_float_list(controller_pose_openxr, 7, "controller_pose_openxr")
        ),
        "enable_button": str(enable_button),
        "enable_value": float(enable_value),
        "enabled": bool(enabled),
        "gripper_button": str(gripper_button),
        "gripper_value": float(gripper_value),
        "gripper_closed": bool(gripper_closed),
        "axis_map": str(axis_map),
        "position_delta_scale": float(position_delta_scale),
        "position_deadband_m": float(position_deadband),
        "engage_settle_sec": float(engage_settle_sec),
        "tcp_rot_offset_wxyz": _as_float_list(
            (item.strip() for item in str(tcp_rot_offset_wxyz).split(",") if item.strip()),
            4,
            "tcp_rot_offset_wxyz",
        ),
        "calibration_confirmed": bool(calibration_confirmed),
        "both_squeeze": bool(both_squeeze),
        "calibration_rotation_base_from_mapped": calibration_rotation_base_from_mapped,
        "monotonic_time": float(now),
    }


class QuestRelativeMapper:
    def __init__(
        self,
        *,
        side: str = "right",
        serial_number: str = DEFAULT_SERIAL_NUMBER,
        joint_group: str = DEFAULT_JOINT_GROUP,
        axis_map: str = DEFAULT_AXIS_MAP,
        position_delta_scale: float = 1.0,
        tcp_rot_offset_wxyz: Iterable[float] = DEFAULT_TCP_ROT_OFFSET_WXYZ,
        engage_settle_sec: float = 0.25,
        position_deadband: float = 0.05,
    ) -> None:
        self.side = side
        self.serial_number = serial_number
        self.joint_group = joint_group
        self.axis_map = parse_axis_map(axis_map)
        self.position_delta_scale = float(position_delta_scale)
        self.tcp_rot_offset_wxyz = normalize_quat_wxyz(tcp_rot_offset_wxyz)
        self.engage_settle_sec = max(0.0, float(engage_settle_sec))
        self.position_deadband = max(0.0, float(position_deadband))
        self._position_zero: list[float] | None = None
        self._engage_time: float | None = None
        self._settled = False

    def update(
        self,
        pose_matrix: list[list[float]],
        *,
        enabled: bool,
        seq: int,
        now: float,
        calibration: QuestSharedFrameCalibration | None = None,
    ) -> dict | None:
        position_openxr = pose_matrix_position(pose_matrix)
        mapped_position = apply_axis_map(position_openxr, self.axis_map)
        hand_quat = pose_matrix_quat_wxyz(pose_matrix, axis_map=self.axis_map)
        if calibration is not None:
            mapped_position = calibration.transform_position(mapped_position)
            hand_quat = calibration.transform_orientation(hand_quat)
        if not enabled:
            self._position_zero = None
            self._engage_time = None
            self._settled = False
            return None

        if self._position_zero is None:
            self._position_zero = list(mapped_position)
            self._engage_time = float(now)
        tcp_quat = quat_multiply_wxyz(hand_quat, self.tcp_rot_offset_wxyz)

        if not self._settled:
            if self._engage_time is not None and float(now) - self._engage_time < self.engage_settle_sec:
                # Do not expose the button-press transient to Isaac. Keep
                # tracking the hand zero until squeeze has remained stable.
                self._position_zero = list(mapped_position)
                return None
            # The first packet after settling is exactly zero displacement.
            # Isaac uses this packet to latch the current RDK TCP and current
            # hand orientation as its two relative-motion origins.
            self._position_zero = list(mapped_position)
            self._settled = True

        delta = [
            (mapped_position[index] - self._position_zero[index]) * self.position_delta_scale
            for index in range(3)
        ]
        delta = [0.0 if abs(value) < self.position_deadband else value for value in delta]
        pose = delta + tcp_quat
        return build_quest_packet(
            seq=seq,
            side=self.side,
            pose_base_tcp_des=pose,
            controller_position_openxr=position_openxr,
            controller_delta_base=delta,
            now=now,
            reason="tracking",
            serial_number=self.serial_number,
            joint_group=self.joint_group,
        )


class UdpJsonPublisher:
    def __init__(self, host: str, port: int) -> None:
        self._address = (host, int(port))
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, packet: dict) -> None:
        self._socket.sendto(json.dumps(packet, separators=(",", ":"), sort_keys=True).encode("utf-8"), self._address)

    def close(self) -> None:
        self._socket.close()


def import_televuer(televuer_root: Path):
    root = Path(televuer_root).expanduser()
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from televuer import TeleVuer  # type: ignore

    return TeleVuer


def select_controller_pose(tv, side: str) -> list[list[float]]:
    matrix = tv.right_arm_pose if side == "right" else tv.left_arm_pose
    return [[float(matrix[row, col]) for col in range(4)] for row in range(4)]


def select_button_value(tv, side: str, button: str) -> float:
    return float(getattr(tv, f"{side}_ctrl_{button}Value", 0.0))


def select_enable(tv, side: str, button: str, *, threshold: float = 0.5) -> bool:
    return bool(getattr(tv, f"{side}_ctrl_{button}")) or select_button_value(tv, side, button) >= float(threshold)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-ip", default=DEFAULT_HOST_IP)
    parser.add_argument("--udp-host", default=DEFAULT_UDP_HOST)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--left-serial-number", default="Rizon4-qSaFLh")
    parser.add_argument("--right-serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--joint-group", default=DEFAULT_JOINT_GROUP)
    parser.add_argument("--side", choices=["left", "right", "both"], default="right")
    parser.add_argument("--enable-button", choices=["squeeze", "trigger", "thumbstick"], default="squeeze")
    parser.add_argument("--gripper-button", choices=["squeeze", "trigger", "thumbstick"], default="trigger")
    parser.add_argument("--axis-map", default=DEFAULT_AXIS_MAP)
    parser.add_argument("--position-delta-scale", type=float, default=1.0)
    parser.add_argument("--position-deadband", type=float, default=0.0)
    parser.add_argument("--engage-settle-sec", type=float, default=0.25)
    parser.add_argument("--right-tcp-rot-offset", default="0.0,0.70710678,0.0,0.70710678")
    parser.add_argument("--enable-threshold", type=float, default=0.5)
    parser.add_argument("--gripper-threshold", type=float, default=0.5)
    parser.add_argument("--televuer-root", type=Path, default=DEFAULT_TELEVUER_ROOT)
    parser.add_argument("--cert-file", default=str(DEFAULT_CERT_FILE))
    parser.add_argument("--key-file", default=str(DEFAULT_KEY_FILE))
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--log-hz", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    TeleVuer = import_televuer(args.televuer_root)
    tv = TeleVuer(
        use_hand_tracking=False,
        binocular=True,
        img_shape=(480, 1280),
        display_fps=30.0,
        display_mode="pass-through",
        zmq=False,
        webrtc=False,
        cert_file=args.cert_file,
        key_file=args.key_file,
    )
    sides = ("left", "right") if args.side == "both" else (args.side,)
    serial_numbers = {
        "left": args.left_serial_number if args.side == "both" else args.serial_number,
        "right": args.right_serial_number if args.side == "both" else args.serial_number,
    }
    mappers = {
        side: QuestRelativeMapper(
            side=side,
            serial_number=serial_numbers[side],
            joint_group=args.joint_group,
            axis_map=args.axis_map,
            position_delta_scale=args.position_delta_scale,
            tcp_rot_offset_wxyz=parse_csv_floats(
                args.right_tcp_rot_offset,
                4,
                "right-tcp-rot-offset",
            ),
            engage_settle_sec=args.engage_settle_sec,
            position_deadband=args.position_deadband,
        )
        for side in sides
    }
    shared_calibration = None
    if args.side == "both":
        shared_calibration = QuestSharedFrameCalibration(
            axis_map=args.axis_map,
            tcp_rot_offset_wxyz=parse_csv_floats(
                args.right_tcp_rot_offset,
                4,
                "right-tcp-rot-offset",
            ),
            settle_sec=args.engage_settle_sec,
        )
    publisher = UdpJsonPublisher(args.udp_host, args.udp_port)
    url = f"https://{args.host_ip}:8012/?ws=wss://{args.host_ip}:8012"
    print("[Rizon4QuestTargetPublisher] Open this URL in the Quest browser:", flush=True)
    print(url, flush=True)
    print(
        "[Rizon4QuestTargetPublisher] Enter VR and allow controller tracking. "
        + (
            "Match the two checks in print.sh, then hold squeeze on BOTH controllers "
            "to lock the shared Quest-to-Isaac frame. Release pauses; the next squeeze resumes relative motion."
            if args.side == "both"
            else f"Hold {args.enable_button} on the {args.side} controller, then move it."
        ),
        flush=True,
    )
    print(f"[Rizon4QuestTargetPublisher] UDP target: {args.udp_host}:{args.udp_port}", flush=True)
    seq = 0
    sent = 0
    last_log = 0.0
    period = 1.0 / max(float(args.rate_hz), 1.0)
    try:
        while True:
            now = time.monotonic()
            ready = bool(tv.motion_data_ready)
            button_values = {
                side: select_button_value(tv, side, args.enable_button) if ready else 0.0
                for side in sides
            }
            gripper_values = {
                side: select_button_value(tv, side, args.gripper_button) if ready else 0.0
                for side in sides
            }
            enabled_values = {
                side: bool(
                    ready
                    and select_enable(
                        tv,
                        side,
                        args.enable_button,
                        threshold=args.enable_threshold,
                    )
                )
                for side in sides
            }
            squeeze_values = {
                side: bool(
                    ready
                    and select_enable(
                        tv,
                        side,
                        "squeeze",
                        threshold=args.enable_threshold,
                    )
                )
                for side in sides
            }
            gripper_closed_values = {
                side: bool(
                    ready
                    and select_enable(
                        tv,
                        side,
                        args.gripper_button,
                        threshold=args.gripper_threshold,
                    )
                )
                for side in sides
            }
            controller_matrices = {
                side: select_controller_pose(tv, side) if ready else None for side in sides
            }
            both_squeeze = bool(
                ready
                and len(sides) == 2
                and all(squeeze_values.get(side, False) for side in ("left", "right"))
            )
            calibration_was_confirmed = bool(
                shared_calibration is not None and shared_calibration.confirmed
            )
            if shared_calibration is not None:
                shared_calibration.update(
                    controller_matrices.get("left"),
                    controller_matrices.get("right"),
                    both_squeeze=both_squeeze,
                    now=now,
                )
                if shared_calibration.confirmed and not calibration_was_confirmed:
                    print(
                        "[Rizon4QuestTargetPublisher] shared Quest-to-Isaac frame LOCKED; "
                        "position and orientation now use the frozen 3D rotation.",
                        flush=True,
                    )
            states = []
            for side in sides:
                button_value = button_values[side]
                gripper_value = gripper_values[side]
                enabled = enabled_values[side]
                gripper_closed = gripper_closed_values[side]
                control_enabled = bool(
                    enabled
                    and (shared_calibration is None or shared_calibration.confirmed)
                )
                packet = None
                controller_matrix = controller_matrices[side]
                controller_pose_openxr = (
                    None
                    if controller_matrix is None
                    else [
                        *pose_matrix_position(controller_matrix),
                        *pose_matrix_quat_wxyz(controller_matrix),
                    ]
                )
                reason = "not_ready" if not ready else ("settling" if enabled else "disabled")
                publisher.publish(
                    build_quest_input_packet(
                        seq=seq,
                        side=side,
                        motion_data_ready=ready,
                        controller_pose_openxr=controller_pose_openxr,
                        enable_button=args.enable_button,
                        enable_value=button_value,
                        enabled=enabled,
                        gripper_button=args.gripper_button,
                        gripper_value=gripper_value,
                        gripper_closed=gripper_closed,
                        calibration_confirmed=bool(
                            shared_calibration is not None and shared_calibration.confirmed
                        ),
                        both_squeeze=both_squeeze,
                        calibration_rotation_base_from_mapped=(
                            None
                            if shared_calibration is None
                            else shared_calibration.rotation_calibrated_from_mapped
                        ),
                        now=now,
                        axis_map=args.axis_map,
                        position_delta_scale=args.position_delta_scale,
                        position_deadband=args.position_deadband,
                        engage_settle_sec=args.engage_settle_sec,
                        tcp_rot_offset_wxyz=args.right_tcp_rot_offset,
                        serial_number=serial_numbers[side],
                        joint_group=args.joint_group,
                    )
                )
                sent += 1
                if ready:
                    publisher.publish(
                        build_gripper_packet(
                            seq=seq,
                            side=side,
                            closed=gripper_closed,
                            now=now,
                            serial_number=serial_numbers[side],
                            joint_group=args.joint_group,
                        )
                    )
                    sent += 1
                    packet = mappers[side].update(
                        controller_matrix,
                        enabled=control_enabled,
                        seq=seq,
                        now=now,
                        calibration=shared_calibration,
                    )
                    if packet is not None:
                        reason = str(packet.get("reason", "tracking"))
                        publisher.publish(packet)
                        sent += 1
                if shared_calibration is not None and not shared_calibration.confirmed:
                    reason = "confirming_frame" if both_squeeze else "awaiting_dual_confirmation"
                states.append(
                    (side, button_value, enabled, control_enabled, gripper_value, gripper_closed, reason, packet)
                )
            if args.log_hz > 0.0 and now - last_log >= 1.0 / float(args.log_hz):
                state_text = []
                for (
                    side,
                    button_value,
                    enabled,
                    control_enabled,
                    gripper_value,
                    gripper_closed,
                    reason,
                    packet,
                ) in states:
                    pose_text = ""
                    if packet is not None:
                        pose_text = f" pose={[round(value, 4) for value in packet['pose_base_tcp_des']]}"
                    state_text.append(
                        f"{side}[{args.enable_button}={enabled} control={control_enabled} "
                        f"{args.enable_button}Value={button_value:.3f} "
                        f"{args.gripper_button}Value={gripper_value:.3f} gripper_closed={gripper_closed} "
                        f"reason={reason}{pose_text}]"
                    )
                print(
                    f"[Rizon4QuestTargetPublisher] seq={seq} ready={ready} "
                    f"{' '.join(state_text)} sent={sent} udp={args.udp_host}:{args.udp_port}",
                    flush=True,
                )
                last_log = now
            seq += 1
            time.sleep(period)
    except KeyboardInterrupt:
        return 130
    finally:
        publisher.close()
        tv.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
