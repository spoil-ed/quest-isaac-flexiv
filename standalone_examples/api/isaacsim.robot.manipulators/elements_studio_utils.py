#!/usr/bin/env python3
"""Small helpers for Elements Studio IK, RDK pose commands, and SimPlugin drives.

The module is intentionally Isaac-free. It can be imported by standalone scripts
before Isaac Sim starts, and only imports grpc/flexivrdk at the call sites that
need those packages.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, NamedTuple


STUDIO_CAL_REACHABILITY_METHOD = "/proto.robot.motion.MotionService/CalReachability"
RDK_CARTESIAN_MAX_LINEAR_VEL = 1.5
RDK_CARTESIAN_MAX_ANGULAR_VEL = 3.0
RDK_CARTESIAN_MAX_LINEAR_ACC = 5.0
RDK_CARTESIAN_MAX_ANGULAR_ACC = 10.0


@dataclass(frozen=True)
class RdkCartesianCommand:
    """Transport-neutral representation of flexivrdk.RtCartesianCmd data."""

    pose_d: list[float]
    twist_d: list[float]
    wrench_d: list[float]
    acc_d: list[float]

    @classmethod
    def from_pose(
        cls,
        pose_d: Iterable[float],
        *,
        twist_d: Iterable[float] | None = None,
        wrench_d: Iterable[float] | None = None,
        acc_d: Iterable[float] | None = None,
    ) -> "RdkCartesianCommand":
        pose = _float_list(pose_d, expected_len=7, name="pose_d")
        return cls(
            pose_d=pose,
            twist_d=_float_list(twist_d or [0.0] * 6, expected_len=6, name="twist_d"),
            wrench_d=_float_list(wrench_d or [0.0] * 6, expected_len=6, name="wrench_d"),
            acc_d=_float_list(acc_d or [0.0] * 6, expected_len=6, name="acc_d"),
        )


@dataclass(frozen=True)
class RdkRuntimeSettings:
    """Connection settings for controlling the Flexiv runtime through RDK."""

    serial_number: str
    joint_group: str = "ARM_1"
    network_interface_whitelist: str | Iterable[str] | None = ""
    switch_mode: bool = True
    clear_fault: bool = False
    strict_clear_fault: bool = True
    servo_on: bool = False
    verbose: bool = False


def _float_list(values: Iterable[float], *, expected_len: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != int(expected_len):
        raise ValueError(f"{name} must contain {expected_len} values")
    return result


def parse_csv(value: str | Iterable[str]) -> list[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    return [str(item).strip() for item in value if str(item).strip()]


def _fmt_float(value: float) -> str:
    return f"{float(value):.10g}"


def normalize_quat_xyzw(values: Iterable[float]) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def normalize_quat_wxyz(values: Iterable[float]) -> tuple[float, float, float, float]:
    w, x, y, z = (float(value) for value in values)
    x, y, z, w = normalize_quat_xyzw((x, y, z, w))
    return w, x, y, z


def rdk_pose_from_position_quat_xyzw(
    position: Iterable[float], orientation_xyzw: Iterable[float]
) -> list[float]:
    """Return the Flexiv RDK pose vector [x, y, z, qw, qx, qy, qz]."""

    x, y, z = _float_list(position, expected_len=3, name="position")
    qx, qy, qz, qw = normalize_quat_xyzw(orientation_xyzw)
    return [x, y, z, qw, qx, qy, qz]


def rdk_pose_to_position_quat_xyzw(
    pose_d: Iterable[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Split an RDK [x, y, z, qw, qx, qy, qz] pose into position and xyzw quat."""

    x, y, z, qw, qx, qy, qz = _float_list(pose_d, expected_len=7, name="pose_d")
    qw, qx, qy, qz = normalize_quat_wxyz((qw, qx, qy, qz))
    return (x, y, z), (qx, qy, qz, qw)


def quat_wxyz_to_euler_xyz_deg(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    qw, qx, qy, qz = normalize_quat_wxyz((qw, qx, qy, qz))

    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def studio_target_pose_from_rdk_pose(pose_d: Iterable[float]) -> list[str]:
    """Convert RDK [x, y, z, qw, qx, qy, qz] into Studio IK m/deg strings."""

    x, y, z, qw, qx, qy, qz = _float_list(pose_d, expected_len=7, name="pose_d")
    roll, pitch, yaw = quat_wxyz_to_euler_xyz_deg(qw, qx, qy, qz)
    return [_fmt_float(value) for value in (x, y, z, roll, pitch, yaw)]


def joint_positions_rad_to_studio_seed(joint_positions_rad: Iterable[float]) -> list[str]:
    values = _float_list(joint_positions_rad, expected_len=7, name="joint_positions_rad")
    return [_fmt_float(math.degrees(value)) for value in values]


def encode_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("varint value must be non-negative")
    chunks = bytearray()
    while value > 0x7F:
        chunks.append((value & 0x7F) | 0x80)
        value >>= 7
    chunks.append(value)
    return bytes(chunks)


def encode_length_delimited(field_number: int, payload: bytes) -> bytes:
    return encode_varint((int(field_number) << 3) | 2) + encode_varint(len(payload)) + payload


def encode_int32(field_number: int, value: int) -> bytes:
    return encode_varint((int(field_number) << 3) | 0) + encode_varint(int(value) & 0xFFFFFFFFFFFFFFFF)


def encode_double(field_number: int, value: float) -> bytes:
    return encode_varint((int(field_number) << 3) | 1) + struct.pack("<d", float(value))


def encode_cal_reachability_request(
    *,
    target_pose: list[str],
    seed_jnt_pos: list[str],
    ext_axis_pos: list[str] | None = None,
    tcp_pose: list[str] | None = None,
    ref_frame_base_flag: bool = True,
) -> bytes:
    if len(target_pose) != 6:
        raise ValueError("target_pose must contain 6 Studio m-deg values")
    if len(seed_jnt_pos) != 7:
        raise ValueError("seed_jnt_pos must contain 7 joint values")
    message = bytearray()
    for value in target_pose:
        message.extend(encode_length_delimited(1, str(value).encode("utf-8")))
    for value in seed_jnt_pos:
        message.extend(encode_length_delimited(2, str(value).encode("utf-8")))
    for value in ext_axis_pos or ():
        message.extend(encode_length_delimited(3, str(value).encode("utf-8")))
    for value in tcp_pose or ():
        message.extend(encode_length_delimited(5, str(value).encode("utf-8")))
    if ref_frame_base_flag:
        message.extend(encode_varint((7 << 3) | 0))
        message.extend(b"\x01")
    return bytes(message)


def parse_cal_reachability_response(response: bytes) -> list[float]:
    solved: list[float] = []
    index = 0
    while index < len(response):
        key = response[index]
        index += 1
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type == 0:
            while index < len(response):
                byte = response[index]
                index += 1
                if not byte & 0x80:
                    break
        elif wire_type == 1:
            index += 8
        elif wire_type == 2:
            length = 0
            shift = 0
            while index < len(response):
                byte = response[index]
                index += 1
                length |= (byte & 0x7F) << shift
                if not byte & 0x80:
                    break
                shift += 7
            payload = response[index : index + length]
            index += length
            if field_number == 2:
                solved.append(float(payload.decode("utf-8")))
        elif wire_type == 5:
            index += 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire_type}")
    if len(solved) != 7:
        raise ValueError(f"CalReachability returned {len(solved)} joint values, expected 7")
    return solved


def grpc_dependency_path() -> Path:
    return Path(__file__).absolute().parents[3] / ".deps" / "grpc"


def import_grpc_module():
    try:
        import grpc  # type: ignore

        return grpc
    except ImportError:
        import sys

        dependency_path = grpc_dependency_path()
        if dependency_path.exists() and str(dependency_path) not in sys.path:
            sys.path.insert(0, str(dependency_path))
        import grpc  # type: ignore

        return grpc


class StudioReachabilityClient:
    def __init__(self, address: str, timeout: float) -> None:
        try:
            grpc = import_grpc_module()
        except ImportError as exc:
            raise RuntimeError("Studio IK requires grpcio in this Python environment") from exc
        self._timeout = float(timeout)
        self._channel = grpc.insecure_channel(address)
        self._call = self._channel.unary_unary(
            STUDIO_CAL_REACHABILITY_METHOD,
            request_serializer=lambda value: value,
            response_deserializer=lambda value: value,
        )

    def solve(self, *, target_pose: list[str], seed_jnt_pos: list[str]) -> list[float]:
        request = encode_cal_reachability_request(target_pose=target_pose, seed_jnt_pos=seed_jnt_pos)
        return parse_cal_reachability_response(self._call(request, timeout=self._timeout))

    def solve_rdk_pose(self, *, pose_d: Iterable[float], seed_jnt_pos_rad: Iterable[float]) -> list[float]:
        return self.solve(
            target_pose=studio_target_pose_from_rdk_pose(pose_d),
            seed_jnt_pos=joint_positions_rad_to_studio_seed(seed_jnt_pos_rad),
        )

    def close(self) -> None:
        self._channel.close()


def make_rdk_cartesian_command(flexivrdk, command: RdkCartesianCommand):
    """Create a flexivrdk.RtCartesianCmd from a transport-neutral command."""

    cmd = flexivrdk.RtCartesianCmd()
    cmd.pose_d = list(command.pose_d)
    cmd.twist_d = list(command.twist_d)
    cmd.wrench_d = list(command.wrench_d)
    cmd.acc_d = list(command.acc_d)
    return cmd


class RdkCartesianStreamer:
    """Synchronous Flexiv RDK Cartesian target sender.

    The caller owns connection/reconnection. This class only converts the
    transport-neutral command into flexivrdk's command object and sends it to
    the requested joint group.
    """

    def __init__(self, flexivrdk, robot) -> None:
        self._flexivrdk = flexivrdk
        self._robot = robot
        self._uses_stream_api = hasattr(robot, "StreamCartesianMotionForce") and hasattr(flexivrdk, "RtCartesianCmd")

    def send(self, joint_group: str, command: RdkCartesianCommand) -> None:
        if self._uses_stream_api:
            group = getattr(self._flexivrdk, str(joint_group))
            rdk_command = make_rdk_cartesian_command(self._flexivrdk, command)
            self._robot.StreamCartesianMotionForce({group: rdk_command})
            return
        self._robot.SendCartesianMotionForce(
            list(command.pose_d),
            list(command.wrench_d),
            list(command.twist_d),
            RDK_CARTESIAN_MAX_LINEAR_VEL,
            RDK_CARTESIAN_MAX_ANGULAR_VEL,
            RDK_CARTESIAN_MAX_LINEAR_ACC,
            RDK_CARTESIAN_MAX_ANGULAR_ACC,
        )


def create_rdk_robot(
    flexivrdk,
    serial_number: str,
    verbose: bool,
    network_interface_whitelist: str | Iterable[str] | None = None,
):
    if network_interface_whitelist is not None:
        whitelist = parse_csv(network_interface_whitelist)
        try:
            return flexivrdk.Robot(serial_number, whitelist, verbose)
        except TypeError:
            pass
    try:
        return flexivrdk.Robot(serial_number, verbose)
    except TypeError:
        return flexivrdk.Robot(serial_number, [], verbose)


def rdk_cartesian_mode(flexivrdk):
    mode_name = "RT_CARTESIAN_MOTION_FORCE"
    mode_enum = getattr(flexivrdk, "Mode", None)
    if mode_enum is not None and hasattr(mode_enum, mode_name):
        return getattr(mode_enum, mode_name)
    if hasattr(flexivrdk, mode_name):
        return getattr(flexivrdk, mode_name)
    fallback = "NRT_CARTESIAN_MOTION_FORCE"
    if mode_enum is not None and hasattr(mode_enum, fallback):
        return getattr(mode_enum, fallback)
    return getattr(flexivrdk, fallback)


def connect_rdk_cartesian_streamer(
    serial_number: str,
    *,
    flexivrdk=None,
    network_interface_whitelist: str | Iterable[str] | None = None,
    switch_mode: bool = True,
    clear_fault: bool = False,
    strict_clear_fault: bool = True,
    servo_on: bool = False,
    verbose: bool = False,
    log=None,
) -> RdkCartesianStreamer:
    if flexivrdk is None:
        import flexivrdk as flexivrdk_module

        flexivrdk = flexivrdk_module
    logger = log or (lambda _message: None)
    robot = create_rdk_robot(
        flexivrdk,
        serial_number,
        bool(verbose),
        network_interface_whitelist=network_interface_whitelist,
    )
    if bool(clear_fault) and robot.fault():
        logger(f"[FlexivRDK] clearing fault on {serial_number}")
        try:
            ok = robot.ClearFault(30)
            if ok is False:
                message = f"failed to clear fault on {serial_number}"
                if strict_clear_fault:
                    raise RuntimeError(message)
                logger(f"[FlexivRDK] {message}; continuing")
        except TypeError:
            robot.ClearFault()
    if bool(servo_on) and hasattr(robot, "ServoOn"):
        logger(f"[FlexivRDK] servo on {serial_number}")
        robot.ServoOn()
    if not robot.operational():
        if hasattr(robot, "Enable"):
            logger(f"[FlexivRDK] enabling {serial_number}")
            robot.Enable()
        if not robot.operational():
            logger(f"[FlexivRDK] {serial_number} is not operational immediately after Enable(); continuing to SwitchMode")
    mode = rdk_cartesian_mode(flexivrdk)
    if bool(switch_mode) and robot.mode() != mode:
        logger(f"[FlexivRDK] switching {serial_number} to Cartesian motion force")
        robot.SwitchMode(mode)
    return RdkCartesianStreamer(flexivrdk, robot)


class RdkRuntimeController:
    """High-level RDK entry point for sending TCP targets into Flexiv runtime.

    The controller keeps RDK connection/setup details local to this module. The
    caller provides target TCP poses in Flexiv/RDK order
    [x, y, z, qw, qx, qy, qz]; runtime-side IK/control remains inside Flexiv.
    """

    def __init__(self, settings: RdkRuntimeSettings, *, flexivrdk=None, log=None) -> None:
        self.settings = settings
        self._flexivrdk = flexivrdk
        self._log = log
        self._streamer: RdkCartesianStreamer | None = None

    @property
    def connected(self) -> bool:
        return self._streamer is not None

    def connect(self) -> None:
        if self._streamer is not None:
            return
        self._streamer = connect_rdk_cartesian_streamer(
            self.settings.serial_number,
            flexivrdk=self._flexivrdk,
            network_interface_whitelist=self.settings.network_interface_whitelist,
            switch_mode=self.settings.switch_mode,
            clear_fault=self.settings.clear_fault,
            strict_clear_fault=self.settings.strict_clear_fault,
            servo_on=self.settings.servo_on,
            verbose=self.settings.verbose,
            log=self._log,
        )

    def send(self, command: RdkCartesianCommand) -> None:
        self.connect()
        assert self._streamer is not None
        self._streamer.send(self.settings.joint_group, command)

    def send_pose(
        self,
        pose_d: Iterable[float],
        *,
        twist_d: Iterable[float] | None = None,
        wrench_d: Iterable[float] | None = None,
        acc_d: Iterable[float] | None = None,
    ) -> None:
        self.send(
            RdkCartesianCommand.from_pose(
                pose_d,
                twist_d=twist_d,
                wrench_d=wrench_d,
                acc_d=acc_d,
            )
        )

    def disconnect(self) -> None:
        self._streamer = None


def valid_target_drives_or_none(target_drives, *, max_norm: float, max_abs: float = float("inf")):
    values = [float(value) for value in target_drives]
    if not values or not all(math.isfinite(value) for value in values):
        return None, float("nan")
    norm = math.sqrt(sum(value * value for value in values))
    if norm > float(max_norm) or any(abs(value) > float(max_abs) for value in values):
        return None, norm
    return values, norm


def joint_speed_limit_exceeded(joint_velocities, *, max_abs_rad_s: float) -> bool:
    limit = float(max_abs_rad_s)
    if limit <= 0.0:
        return False
    values = [float(value) for value in joint_velocities]
    return (not all(math.isfinite(value) for value in values)) or any(abs(value) > limit for value in values)
