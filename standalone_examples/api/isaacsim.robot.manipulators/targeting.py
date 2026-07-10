"""Target pose, Quest packet, and coordinate conversion helpers."""

from __future__ import annotations

import json
import math
import socket
import time
from typing import NamedTuple


class TargetPose(NamedTuple):
    position: tuple[float, float, float]
    euler_deg: tuple[float, float, float]


class QuestTargetPacket(NamedTuple):
    seq: int
    side: str
    pose_base_tcp_des: list[float]
    controller_position_openxr: list[float] | None
    gripper_open_ratio: float | None
    monotonic_time: float


class QuestAxisMapEntry(NamedTuple):
    source_index: int
    sign: float


class QuestRelativeReference(NamedTuple):
    controller_position_openxr: list[float]
    tcp_pose_base: list[float]


def triple(values) -> tuple[float, float, float]:
    return (float(values[0]), float(values[1]), float(values[2]))


def parse_float_list(value, *, expected: int, name: str) -> list[float]:
    if isinstance(value, str):
        values = [float(item.strip()) for item in value.split(",") if item.strip()]
    else:
        values = [float(item) for item in value]
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} values")
    return values


def parse_quest_axis_map(value: str) -> tuple[QuestAxisMapEntry, QuestAxisMapEntry, QuestAxisMapEntry]:
    axis_to_index = {"x": 0, "y": 1, "z": 2}
    entries = []
    for raw_token in str(value).split(","):
        token = raw_token.strip().lower()
        if not token:
            continue
        sign = -1.0 if token.startswith("-") else 1.0
        token = token[1:] if token[0] in "+-" else token
        if token not in axis_to_index:
            raise ValueError("--quest-axis-map tokens must be x, y, z with optional +/- prefix")
        entries.append(QuestAxisMapEntry(source_index=axis_to_index[token], sign=sign))
    if len(entries) != 3:
        raise ValueError("--quest-axis-map must contain exactly 3 comma-separated axes")
    return tuple(entries)


def map_openxr_delta_to_base(
    delta_openxr,
    *,
    axis_map: tuple[QuestAxisMapEntry, QuestAxisMapEntry, QuestAxisMapEntry],
    scale: float,
) -> list[float]:
    values = [float(value) for value in delta_openxr]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("delta_openxr must contain 3 finite values")
    return [entry.sign * values[entry.source_index] * float(scale) for entry in axis_map]


class QuestRelativeTargetMapper:
    def __init__(
        self,
        *,
        axis_map: tuple[QuestAxisMapEntry, QuestAxisMapEntry, QuestAxisMapEntry],
        scale: float,
        workspace_min,
        workspace_max,
    ) -> None:
        self.axis_map = axis_map
        self.scale = float(scale)
        self.workspace_min = parse_float_list(workspace_min, expected=3, name="workspace_min")
        self.workspace_max = parse_float_list(workspace_max, expected=3, name="workspace_max")
        self.reference: QuestRelativeReference | None = None

    def reset(self) -> None:
        self.reference = None

    def update(self, quest_target: QuestTargetPacket, current_pose_base_tcp: list[float]) -> list[float]:
        current_pose = [float(value) for value in current_pose_base_tcp]
        if len(current_pose) != 7:
            raise ValueError("current_pose_base_tcp must contain 7 values")
        if quest_target.controller_position_openxr is None:
            return current_pose
        controller_position = [float(value) for value in quest_target.controller_position_openxr]
        if self.reference is None:
            self.reference = QuestRelativeReference(
                controller_position_openxr=controller_position,
                tcp_pose_base=list(current_pose),
            )
            return list(current_pose)
        delta_openxr = [
            controller_position[index] - self.reference.controller_position_openxr[index]
            for index in range(3)
        ]
        delta_base = map_openxr_delta_to_base(delta_openxr, axis_map=self.axis_map, scale=self.scale)
        xyz = [self.reference.tcp_pose_base[index] + delta_base[index] for index in range(3)]
        return xyz + list(self.reference.tcp_pose_base[3:7])


def target_pose_from_world_pose(world_position, world_orientation_wxyz) -> TargetPose:
    qw, qx, qy, qz = (float(value) for value in world_orientation_wxyz)
    return TargetPose(
        position=triple(world_position),
        euler_deg=quat_wxyz_to_euler_xyz_deg(qw, qx, qy, qz),
    )


def euler_xyz_deg_to_quat_wxyz(euler_deg: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = (math.radians(value) for value in euler_deg)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_wxyz_to_euler_xyz_deg(qw: float, qx: float, qy: float, qz: float) -> tuple[float, float, float]:
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm <= 0.0:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        qw, qx, qy, qz = qw / norm, qx / norm, qy / norm, qz / norm

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


def _quat_xyzw(values) -> tuple[float, float, float, float]:
    x, y, z, w = (float(v) for v in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def _wxyz_to_xyzw(values) -> tuple[float, float, float, float]:
    w, x, y, z = (float(v) for v in values)
    return _quat_xyzw((x, y, z, w))


def _xyzw_to_wxyz(values) -> tuple[float, float, float, float]:
    x, y, z, w = _quat_xyzw(values)
    return w, x, y, z


def _quat_conjugate_xyzw(q) -> tuple[float, float, float, float]:
    x, y, z, w = _quat_xyzw(q)
    return -x, -y, -z, w


def _quat_mul_xyzw(a, b) -> tuple[float, float, float, float]:
    ax, ay, az, aw = _quat_xyzw(a)
    bx, by, bz, bw = _quat_xyzw(b)
    return _quat_xyzw(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def _rotate_vector_xyzw(q, v) -> tuple[float, float, float]:
    qx, qy, qz, qw = _quat_xyzw(q)
    vx, vy, vz = triple(v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def world_target_to_flexiv_pose(
    *,
    world_position,
    world_orientation_wxyz,
    base_position,
    base_orientation_wxyz,
) -> list[float]:
    wp = triple(world_position)
    bp = triple(base_position)
    wo = _wxyz_to_xyzw(world_orientation_wxyz)
    bo = _wxyz_to_xyzw(base_orientation_wxyz)
    inv_base = _quat_conjugate_xyzw(bo)
    rel_world = (wp[0] - bp[0], wp[1] - bp[1], wp[2] - bp[2])
    rel_pos = _rotate_vector_xyzw(inv_base, rel_world)
    rel_ori_xyzw = _quat_mul_xyzw(inv_base, wo)
    qw, qx, qy, qz = _xyzw_to_wxyz(rel_ori_xyzw)
    return [rel_pos[0], rel_pos[1], rel_pos[2], qw, qx, qy, qz]


def select_pose_base_tcp_des(
    *,
    quest_target: QuestTargetPacket | None,
    world_position,
    world_orientation_wxyz,
    base_position,
    base_orientation_wxyz,
) -> list[float]:
    if quest_target is not None:
        return list(quest_target.pose_base_tcp_des)
    return world_target_to_flexiv_pose(
        world_position=world_position,
        world_orientation_wxyz=world_orientation_wxyz,
        base_position=base_position,
        base_orientation_wxyz=base_orientation_wxyz,
    )


def quest_target_is_fresh(
    quest_target: QuestTargetPacket | None,
    *,
    now: float | None = None,
    max_age_sec: float,
) -> bool:
    if quest_target is None:
        return False
    if max_age_sec <= 0.0:
        return True
    current = time.monotonic() if now is None else float(now)
    age = current - float(quest_target.monotonic_time)
    return -1.0 <= age <= float(max_age_sec)


def flexiv_pose_to_world_target(
    *,
    pose_base_tcp_des,
    base_position,
    base_orientation_wxyz,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    values = [float(value) for value in pose_base_tcp_des]
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError("pose_base_tcp_des must contain 7 finite values")
    bp = triple(base_position)
    bo = _wxyz_to_xyzw(base_orientation_wxyz)
    rel_pos = (values[0], values[1], values[2])
    rel_ori_xyzw = _wxyz_to_xyzw(values[3:7])
    world_rel = _rotate_vector_xyzw(bo, rel_pos)
    world_pos = (bp[0] + world_rel[0], bp[1] + world_rel[1], bp[2] + world_rel[2])
    world_ori_xyzw = _quat_mul_xyzw(bo, rel_ori_xyzw)
    return world_pos, _xyzw_to_wxyz(world_ori_xyzw)


def sync_target_to_base_tcp_pose(
    target,
    *,
    pose_base_tcp_des,
    base_position,
    base_orientation_wxyz,
) -> TargetPose:
    import numpy as np

    world_position, world_orientation_wxyz = flexiv_pose_to_world_target(
        pose_base_tcp_des=pose_base_tcp_des,
        base_position=base_position,
        base_orientation_wxyz=base_orientation_wxyz,
    )
    target.set_world_pose(
        position=np.array(world_position),
        orientation=np.array(world_orientation_wxyz),
    )
    return target_pose_from_world_pose(world_position, world_orientation_wxyz)


sync_target_ball_to_base_tcp_pose = sync_target_to_base_tcp_pose


def build_target_pose_packet(
    *,
    serial_number: str,
    joint_group: str,
    servo_cycle: int,
    pose_base_tcp_des: list[float],
    monotonic_time: float | None = None,
) -> dict:
    if len(pose_base_tcp_des) != 7:
        raise ValueError("pose_base_tcp_des must contain 7 values")
    return {
        "schema": "flexiv_target_pose.v1",
        "serial": serial_number,
        "joint_group": joint_group,
        "servo_cycle": int(servo_cycle),
        "monotonic_time": time.monotonic() if monotonic_time is None else float(monotonic_time),
        "pose_base_tcp_des": [float(value) for value in pose_base_tcp_des],
    }


def build_coordinate_observation_packet(
    *,
    serial_number: str,
    joint_group: str,
    servo_cycle: int,
    quest_target: QuestTargetPacket | None,
    active: bool,
    current_pose_base_tcp: list[float] | None,
    target_pose_base_tcp: list[float] | None = None,
    monotonic_time: float | None = None,
) -> dict:
    current_time = time.monotonic() if monotonic_time is None else float(monotonic_time)
    if target_pose_base_tcp is not None:
        target_pose = [float(value) for value in target_pose_base_tcp]
    else:
        target_pose = None if quest_target is None else [float(value) for value in quest_target.pose_base_tcp_des]
    current_pose = None if current_pose_base_tcp is None else [float(value) for value in current_pose_base_tcp]
    position_error = None
    if target_pose is not None and current_pose is not None:
        position_error = [target_pose[index] - current_pose[index] for index in range(3)]
    return {
        "schema": "rizon4_quest_coordinate_observation.v1",
        "serial": str(serial_number),
        "joint_group": str(joint_group),
        "servo_cycle": int(servo_cycle),
        "active": bool(active),
        "quest_seq": None if quest_target is None else int(quest_target.seq),
        "side": None if quest_target is None else str(quest_target.side),
        "age_sec": None if quest_target is None else current_time - float(quest_target.monotonic_time),
        "target_pose_base_tcp": target_pose,
        "current_pose_base_tcp": current_pose,
        "position_error": position_error,
        "monotonic_time": current_time,
    }


class TargetPoseUdpPublisher:
    def __init__(self, host: str, port: int) -> None:
        self._address = (host, int(port))
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, packet: dict) -> None:
        data = json.dumps(packet, separators=(",", ":"), sort_keys=True).encode("utf-8")
        self._socket.sendto(data, self._address)

    def close(self) -> None:
        self._socket.close()


def parse_quest_target_packet(
    packet: dict,
    *,
    serial_number: str,
    joint_group: str,
    now: float | None = None,
    max_age_sec: float = 0.5,
) -> QuestTargetPacket | None:
    if not isinstance(packet, dict):
        return None
    if packet.get("schema") != "rizon4_quest_target.v1":
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
    controller_position_openxr = None
    if packet.get("controller_position_openxr") is not None:
        try:
            controller_position_openxr = [float(value) for value in packet["controller_position_openxr"]]
        except (TypeError, ValueError):
            return None
        if len(controller_position_openxr) != 3 or not all(
            math.isfinite(value) for value in controller_position_openxr
        ):
            return None
    if max_age_sec > 0.0 and packet.get("monotonic_time") is not None:
        current = time.monotonic() if now is None else float(now)
        try:
            age = current - float(packet["monotonic_time"])
        except (TypeError, ValueError):
            return None
        if age < -1.0 or age > float(max_age_sec):
            return None
    try:
        monotonic_time = float(packet.get("monotonic_time", time.monotonic()))
    except (TypeError, ValueError):
        return None
    gripper_open_ratio = packet.get("gripper_open_ratio")
    if gripper_open_ratio is not None:
        try:
            gripper_open_ratio = float(gripper_open_ratio)
        except (TypeError, ValueError):
            gripper_open_ratio = None
    return QuestTargetPacket(
        seq=int(packet.get("seq", -1)),
        side=str(packet.get("side", "")),
        pose_base_tcp_des=pose,
        controller_position_openxr=controller_position_openxr,
        gripper_open_ratio=gripper_open_ratio,
        monotonic_time=monotonic_time,
    )


class QuestTargetUdpReceiver:
    def __init__(
        self,
        host: str,
        port: int,
        *,
        serial_number: str,
        joint_group: str,
        max_age_sec: float,
    ) -> None:
        self._address = (str(host), int(port))
        self._serial_number = str(serial_number)
        self._joint_group = str(joint_group)
        self._max_age_sec = float(max_age_sec)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(self._address)
        self._socket.setblocking(False)

    @property
    def address(self) -> tuple[str, int]:
        return self._address

    def poll_latest(self) -> QuestTargetPacket | None:
        latest = None
        while True:
            try:
                data, _addr = self._socket.recvfrom(65536)
            except BlockingIOError:
                return latest
            except OSError:
                return latest
            try:
                packet = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            parsed = parse_quest_target_packet(
                packet,
                serial_number=self._serial_number,
                joint_group=self._joint_group,
                max_age_sec=self._max_age_sec,
            )
            if parsed is not None:
                latest = parsed

    def close(self) -> None:
        self._socket.close()
