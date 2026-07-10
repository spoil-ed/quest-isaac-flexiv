#!/usr/bin/env python3
"""Pure Isaac Sim scene driven through the documented Flexiv-Isaac bridge."""

from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path
from typing import NamedTuple


DEFAULT_RIZON4_USD = Path(
    "/home/simate/workspace/isaacsim-flexiv/isaac_sim_ws/exts/"
    "isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd"
)
DEFAULT_EXAMPLES_EXT = Path(
    "/home/simate/workspace/isaacsim-flexiv/isaac_sim_ws/exts/isaacsim.robot.manipulators.examples"
)
DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"
DEFAULT_TARGET_PRIM_PATH = "/World/TargetBall"
DEFAULT_TARGET_NAME = "target_ball"
DEFAULT_TARGET_POSITION = (0.45, 0.0, 0.35)
DEFAULT_TARGET_EULER_DEG = (0.0, 0.0, 0.0)
DEFAULT_INITIAL_Q = (0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0)
DEFAULT_STUDIO_INITIAL_Q = DEFAULT_INITIAL_Q
DEFAULT_TARGET_POSE_UDP_HOST = "127.0.0.1"
DEFAULT_TARGET_POSE_UDP_PORT = 45678
DEFAULT_QUEST_TARGET_UDP_HOST = "127.0.0.1"
DEFAULT_QUEST_TARGET_UDP_PORT = 45679
DEFAULT_QUEST_COORD_OBSERVE_UDP_PORT = 45680
DEFAULT_QUEST_AXIS_MAP = "-z,-x,y"
DEFAULT_QUEST_WORKSPACE_MIN = (0.15, -0.70, 0.20)
DEFAULT_QUEST_WORKSPACE_MAX = (1.00, 0.70, 1.35)
DEFAULT_STUDIO_GRPC_ADDRESS = "127.0.0.1:18001"
PHYSICS_FREQ = 2000.0
RENDER_FREQ = 60.0
DEFAULT_PHYSICS_HZ = PHYSICS_FREQ
DEFAULT_STUDIO_IK_HZ = 30.0
DEFAULT_STUDIO_JOG_HZ = 30.0
COMPATIBLE_SIM_PLUGIN_VER = "1.2.0"


class TargetPose(NamedTuple):
    position: tuple[float, float, float]
    euler_deg: tuple[float, float, float]


class CartJogCommand(NamedTuple):
    jog_index: int
    jog_dir: int
    step_size: float
    jog_axis_type: int
    vel_scale: float


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


KEYBOARD_DELTAS = {
    "W": ((0.0, 1.0, 0.0), (0.0, 0.0, 0.0)),
    "S": ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
    "D": ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "A": ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "E": ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
    "Q": ((0.0, 0.0, -1.0), (0.0, 0.0, 0.0)),
    "O": ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
    "U": ((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
    "I": ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
    "K": ((0.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
    "J": ((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    "L": ((0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--params-json",
        type=Path,
        default=None,
        help="Optional JSON file containing parameter overrides by argument name.",
    )
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER, help="Elements Studio simulated robot serial.")
    parser.add_argument("--joint-group", default=DEFAULT_JOINT_GROUP, help="Studio joint group label, normally ARM_1.")
    parser.add_argument("--usd", type=Path, default=DEFAULT_RIZON4_USD, help="Rizon4 USD asset.")
    parser.add_argument("--examples-ext", type=Path, default=DEFAULT_EXAMPLES_EXT, help="Flexiv examples extension path.")
    parser.add_argument("--headless", action="store_true", help="Run without a GUI.")
    parser.add_argument("--smoke-test", action="store_true", help="Run headless and exit after --max-frames.")
    parser.add_argument("--max-frames", type=int, default=0, help="Frame limit for smoke tests. 0 means unlimited.")
    parser.add_argument("--manual-play", action="store_true", help="Do not auto-start the timeline.")
    parser.add_argument(
        "--physics-hz",
        type=float,
        default=None,
        help="Isaac physics callback frequency. Default is 30 Hz; use params JSON or CLI to override.",
    )
    parser.add_argument("--render-hz", type=float, default=RENDER_FREQ, help="Isaac rendering frequency.")
    parser.add_argument(
        "--control-source",
        choices=("studio-jog", "studio-ik", "studio-bridge"),
        default="studio-jog",
        help=(
            "studio-jog sends CartesianJogging commands to Elements Studio and applies Studio target_drives; "
            "studio-ik uses Studio CalReachability directly; studio-bridge only applies target_drives."
        ),
    )
    parser.add_argument("--studio-grpc-address", default=DEFAULT_STUDIO_GRPC_ADDRESS, help="Elements Studio RobotGrpc address.")
    parser.add_argument("--studio-grpc-timeout", type=float, default=0.02, help="Studio motion gRPC timeout in seconds.")
    parser.add_argument(
        "--studio-ik-hz",
        type=float,
        default=DEFAULT_STUDIO_IK_HZ,
        help="Studio IK query frequency. Increase only after checking gRPC latency.",
    )
    parser.add_argument(
        "--studio-jog-hz",
        type=float,
        default=DEFAULT_STUDIO_JOG_HZ,
        help="Studio CartesianJogging command frequency.",
    )
    parser.add_argument(
        "--studio-jog-position-deadband",
        type=float,
        default=0.005,
        help="Position error deadband in meters before sending a stop jog command.",
    )
    parser.add_argument(
        "--studio-jog-max-step-size",
        type=float,
        default=0.01,
        help="Maximum CartesianJogging step size in meters per command.",
    )
    parser.add_argument(
        "--studio-jog-vel-scale",
        type=float,
        default=0.4,
        help="CartesianJogging velocity scale sent to Elements Studio.",
    )
    parser.add_argument("--target-prim-path", default=DEFAULT_TARGET_PRIM_PATH, help="USD prim path for the ball target.")
    parser.add_argument("--target-name", default=DEFAULT_TARGET_NAME, help="Scene object name for the ball target.")
    parser.add_argument("--ball-radius", type=float, default=0.035, help="Visual ball radius in meters.")
    parser.add_argument(
        "--initial-q",
        type=float,
        nargs=7,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="Initial robot joint positions in radians. Defaults to Studio home for bridge/jog modes.",
    )
    parser.add_argument("--position-step", type=float, default=0.01, help="Keyboard translation step in meters.")
    parser.add_argument("--rotation-step-deg", type=float, default=5.0, help="Keyboard rotation step in degrees.")
    parser.add_argument("--command-timeout-ms", type=int, default=100, help="SimPlugin command wait timeout per physics tick.")
    parser.add_argument(
        "--target-drive-warmup-cycles",
        type=int,
        default=30,
        help="Connected SimPlugin cycles to skip before applying Studio target_drives.",
    )
    parser.add_argument(
        "--max-target-drive-norm",
        type=float,
        default=200.0,
        help="Reject Studio target_drives whose vector norm exceeds this value.",
    )
    parser.add_argument(
        "--target-drive-required-valid-cycles",
        type=int,
        default=5,
        help="Require this many consecutive valid target_drives before entering effort control.",
    )
    parser.add_argument(
        "--disable-simplugin-target-drives",
        action="store_true",
        help="In studio-jog mode, send Studio CartesianJogging only and skip applying SimPlugin target_drives to Isaac.",
    )
    parser.add_argument(
        "--studio-jog-feedback-source",
        choices=("isaac", "virtual"),
        default="isaac",
        help="Feedback pose for studio-jog error control. virtual integrates sent jog commands for Studio-only following.",
    )
    parser.add_argument(
        "--state-torque-log-hz",
        type=float,
        default=2.0,
        help="Log Isaac q/dq input state and SimPlugin target_drives at this frequency. Use <=0 to disable.",
    )
    parser.add_argument("--target-pose-udp-host", default=DEFAULT_TARGET_POSE_UDP_HOST, help="Target pose UDP host.")
    parser.add_argument("--target-pose-udp-port", type=int, default=DEFAULT_TARGET_POSE_UDP_PORT, help="Target pose UDP port.")
    parser.add_argument(
        "--target-pose-publish-hz",
        type=float,
        default=PHYSICS_FREQ,
        help="Target pose UDP publish frequency. Default follows the Isaac physics step frequency.",
    )
    parser.add_argument("--disable-target-pose-udp", action="store_true", help="Disable target pose UDP publishing.")
    parser.add_argument(
        "--enable-quest-target-udp",
        action="store_true",
        help="Listen for Quest target pose UDP packets and move the visual target ball.",
    )
    parser.add_argument(
        "--quest-target-udp-host",
        default=DEFAULT_QUEST_TARGET_UDP_HOST,
        help="Local host/IP to bind for Quest target pose UDP packets.",
    )
    parser.add_argument(
        "--quest-target-udp-port",
        type=int,
        default=DEFAULT_QUEST_TARGET_UDP_PORT,
        help="Local UDP port for Quest target pose packets.",
    )
    parser.add_argument(
        "--quest-target-max-age-sec",
        type=float,
        default=0.5,
        help="Drop Quest target packets older than this many seconds. Use <=0 to disable age checks.",
    )
    parser.add_argument(
        "--quest-coordinate-observe-udp-host",
        default=DEFAULT_QUEST_TARGET_UDP_HOST,
        help="UDP host for coordinate observation packets.",
    )
    parser.add_argument(
        "--quest-coordinate-observe-udp-port",
        type=int,
        default=DEFAULT_QUEST_COORD_OBSERVE_UDP_PORT,
        help="UDP port for coordinate observation packets. Use <=0 to disable.",
    )
    parser.add_argument(
        "--quest-target-mode",
        choices=("absolute", "relative"),
        default="relative",
        help="absolute uses incoming pose_base_tcp_des; relative anchors Quest motion to current TCP on press.",
    )
    parser.add_argument(
        "--quest-axis-map",
        default=DEFAULT_QUEST_AXIS_MAP,
        help="Relative mode axis map from OpenXR delta to Rizon4 base xyz, e.g. '-z,-x,y'.",
    )
    parser.add_argument(
        "--quest-position-scale",
        type=float,
        default=0.5,
        help="Relative mode scale from Quest meters to Rizon4 base meters.",
    )
    parser.add_argument(
        "--quest-workspace-min",
        default=",".join(str(value) for value in DEFAULT_QUEST_WORKSPACE_MIN),
        help="Relative mode Rizon4 base xyz lower bounds.",
    )
    parser.add_argument(
        "--quest-workspace-max",
        default=",".join(str(value) for value in DEFAULT_QUEST_WORKSPACE_MAX),
        help="Relative mode Rizon4 base xyz upper bounds.",
    )
    parser.add_argument(
        "--target-position",
        type=float,
        nargs=3,
        default=DEFAULT_TARGET_POSITION,
        metavar=("X", "Y", "Z"),
        help="Initial ball position in world frame.",
    )
    parser.add_argument(
        "--target-euler-deg",
        type=float,
        nargs=3,
        default=DEFAULT_TARGET_EULER_DEG,
        metavar=("ROLL", "PITCH", "YAW"),
        help="Initial target orientation as XYZ Euler angles in degrees.",
    )
    parser.add_argument(
        "--attach-target-to-ee-on-start",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Initialize the visual target ball at the current end-effector pose.",
    )
    args = parser.parse_args(argv)
    apply_param_overrides(args)
    if args.smoke_test:
        args.headless = True
    return args


PARAM_OVERRIDE_KEYS = {
    "control_source",
    "physics_hz",
    "render_hz",
    "studio_jog_hz",
    "studio_jog_position_deadband",
    "studio_jog_max_step_size",
    "studio_jog_vel_scale",
    "studio_grpc_address",
    "studio_grpc_timeout",
    "command_timeout_ms",
    "target_drive_warmup_cycles",
    "max_target_drive_norm",
    "target_drive_required_valid_cycles",
    "disable_simplugin_target_drives",
    "studio_jog_feedback_source",
    "state_torque_log_hz",
    "initial_q",
    "target_position",
    "target_euler_deg",
    "attach_target_to_ee_on_start",
    "position_step",
    "rotation_step_deg",
    "manual_play",
    "headless",
    "disable_target_pose_udp",
    "enable_quest_target_udp",
    "quest_target_udp_host",
    "quest_target_udp_port",
    "quest_target_max_age_sec",
    "quest_coordinate_observe_udp_host",
    "quest_coordinate_observe_udp_port",
    "quest_target_mode",
    "quest_axis_map",
    "quest_position_scale",
    "quest_workspace_min",
    "quest_workspace_max",
}


def apply_param_overrides(args: argparse.Namespace) -> None:
    if args.params_json is None:
        return
    path = Path(args.params_json).expanduser()
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("--params-json must contain a JSON object")
    for raw_key, value in data.items():
        key = str(raw_key).replace("-", "_")
        if key not in PARAM_OVERRIDE_KEYS:
            raise ValueError(f"unsupported params-json key: {raw_key}")
        setattr(args, key, value)
    if args.initial_q is not None and len(args.initial_q) != 7:
        raise ValueError("initial_q in params-json must contain 7 joint values")
    if len(args.target_position) != 3:
        raise ValueError("target_position in params-json must contain 3 values")
    if len(args.target_euler_deg) != 3:
        raise ValueError("target_euler_deg in params-json must contain 3 values")


def _triple(values) -> tuple[float, float, float]:
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


def _clip_xyz(values, workspace_min, workspace_max) -> list[float]:
    mins = parse_float_list(workspace_min, expected=3, name="workspace_min")
    maxs = parse_float_list(workspace_max, expected=3, name="workspace_max")
    return [min(max(float(values[index]), mins[index]), maxs[index]) for index in range(3)]


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


def initial_q_for_args(args: argparse.Namespace) -> list[float]:
    if args.initial_q is not None:
        return [float(value) for value in args.initial_q]
    if args.control_source in {"studio-jog", "studio-bridge"}:
        return list(DEFAULT_STUDIO_INITIAL_Q)
    return list(DEFAULT_INITIAL_Q)


def target_pose_from_world_pose(world_position, world_orientation_wxyz) -> TargetPose:
    qw, qx, qy, qz = (float(value) for value in world_orientation_wxyz)
    return TargetPose(
        position=_triple(world_position),
        euler_deg=quat_wxyz_to_euler_xyz_deg(qw, qx, qy, qz),
    )


def configured_initial_target_pose(args: argparse.Namespace) -> TargetPose:
    return TargetPose(position=_triple(args.target_position), euler_deg=_triple(args.target_euler_deg))


def initial_target_pose_for_args(
    args: argparse.Namespace,
    *,
    end_effector_position,
    end_effector_orientation_wxyz,
) -> TargetPose:
    if bool(args.attach_target_to_ee_on_start):
        return target_pose_from_world_pose(end_effector_position, end_effector_orientation_wxyz)
    return configured_initial_target_pose(args)


def physics_hz_for_args(args: argparse.Namespace) -> float:
    if args.physics_hz is not None:
        return float(args.physics_hz)
    return DEFAULT_PHYSICS_HZ


def _fmt_float(value: float) -> str:
    return f"{float(value):.10g}"


def grpc_dependency_path() -> Path:
    return Path(__file__).absolute().parents[4] / ".deps" / "grpc"


def import_grpc_module():
    try:
        import grpc  # type: ignore

        return grpc
    except ImportError:
        dependency_path = grpc_dependency_path()
        if dependency_path.exists() and str(dependency_path) not in sys.path:
            sys.path.insert(0, str(dependency_path))
        import grpc  # type: ignore

        return grpc


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
    vx, vy, vz = _triple(v)
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
    wp = _triple(world_position)
    bp = _triple(base_position)
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
    bp = _triple(base_position)
    bo = _wxyz_to_xyzw(base_orientation_wxyz)
    rel_pos = (values[0], values[1], values[2])
    rel_ori_xyzw = _wxyz_to_xyzw(values[3:7])
    world_rel = _rotate_vector_xyzw(bo, rel_pos)
    world_pos = (bp[0] + world_rel[0], bp[1] + world_rel[1], bp[2] + world_rel[2])
    world_ori_xyzw = _quat_mul_xyzw(bo, rel_ori_xyzw)
    return world_pos, _xyzw_to_wxyz(world_ori_xyzw)


def sync_target_ball_to_base_tcp_pose(
    ball,
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
    ball.set_world_pose(
        position=np.array(world_position),
        orientation=np.array(world_orientation_wxyz),
    )
    return target_pose_from_world_pose(world_position, world_orientation_wxyz)


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


class TargetPosePublishGate(NamedTuple):
    period_cycles: int

    @classmethod
    def from_hz(cls, publish_hz: float, *, physics_freq: float) -> "TargetPosePublishGate":
        if publish_hz <= 0.0:
            return cls(period_cycles=1)
        return cls(period_cycles=max(1, int(round(float(physics_freq) / float(publish_hz)))))

    def should_publish(self, servo_cycle: int) -> bool:
        return int(servo_cycle) % self.period_cycles == 0


class StepRateLimiter:
    def __init__(self, step_hz: float, *, time_fn=time.perf_counter, sleep_fn=time.sleep) -> None:
        self._period = 0.0 if step_hz <= 0.0 else 1.0 / float(step_hz)
        self._time_fn = time_fn
        self._sleep_fn = sleep_fn
        self._next_time = self._time_fn()

    def sleep(self) -> None:
        if self._period <= 0.0:
            return
        self._next_time += self._period
        now = self._time_fn()
        delay = self._next_time - now
        if delay > 0.0:
            self._sleep_fn(delay)
        elif delay < -self._period:
            self._next_time = now


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


def pose_base_tcp_des_to_studio_target_pose(pose_base_tcp_des: list[float]) -> list[str]:
    if len(pose_base_tcp_des) != 7:
        raise ValueError("pose_base_tcp_des must contain 7 values")
    x, y, z, qw, qx, qy, qz = (float(value) for value in pose_base_tcp_des)
    roll, pitch, yaw = quat_wxyz_to_euler_xyz_deg(qw, qx, qy, qz)
    return [_fmt_float(value) for value in (x, y, z, roll, pitch, yaw)]


def joint_positions_rad_to_studio_seed(joint_positions_rad) -> list[str]:
    values = [float(value) for value in joint_positions_rad]
    if len(values) != 7:
        raise ValueError("joint_positions_rad must contain 7 values")
    return [_fmt_float(math.degrees(value)) for value in values]


def _encode_varint(value: int) -> bytes:
    if value < 0:
        raise ValueError("varint value must be non-negative")
    chunks = bytearray()
    while value > 0x7F:
        chunks.append((value & 0x7F) | 0x80)
        value >>= 7
    chunks.append(value)
    return bytes(chunks)


def _encode_length_delimited(field_number: int, payload: bytes) -> bytes:
    return _encode_varint((int(field_number) << 3) | 2) + _encode_varint(len(payload)) + payload


def _encode_int32(field_number: int, value: int) -> bytes:
    return _encode_varint((int(field_number) << 3) | 0) + _encode_varint(int(value) & 0xFFFFFFFFFFFFFFFF)


def _encode_double(field_number: int, value: float) -> bytes:
    return _encode_varint((int(field_number) << 3) | 1) + struct.pack("<d", float(value))


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
        message.extend(_encode_length_delimited(1, str(value).encode("utf-8")))
    for value in seed_jnt_pos:
        message.extend(_encode_length_delimited(2, str(value).encode("utf-8")))
    for value in ext_axis_pos or ():
        message.extend(_encode_length_delimited(3, str(value).encode("utf-8")))
    for value in tcp_pose or ():
        message.extend(_encode_length_delimited(5, str(value).encode("utf-8")))
    if ref_frame_base_flag:
        message.extend(_encode_varint((7 << 3) | 0))
        message.extend(b"\x01")
    return bytes(message)


def encode_set_cart_jogging_cmd_request(
    *,
    coord_type: int,
    coord_name: str,
    jog_index: int,
    jog_dir: int,
    step_size: float,
    jog_axis_type: int,
    vel_scale: float,
) -> bytes:
    jog_cmd = b"".join(
        (
            _encode_int32(1, int(jog_index)),
            _encode_int32(2, int(jog_dir)),
            _encode_double(3, float(step_size)),
            _encode_int32(4, int(jog_axis_type)),
            _encode_double(5, float(vel_scale)),
        )
    )
    cart_jogging_cmd = b"".join(
        (
            _encode_int32(1, int(coord_type)),
            _encode_length_delimited(2, str(coord_name).encode("utf-8")),
            _encode_length_delimited(3, jog_cmd),
        )
    )
    return _encode_length_delimited(1, cart_jogging_cmd)


def pose_error_to_cart_jog_cmd(
    pose_error_base_tcp: list[float],
    *,
    position_deadband: float,
    max_step_size: float,
    vel_scale: float,
) -> CartJogCommand:
    if len(pose_error_base_tcp) != 7:
        raise ValueError("pose_error_base_tcp must contain 7 values")
    position_error = [float(value) for value in pose_error_base_tcp[:3]]
    axis = max(range(3), key=lambda index: abs(position_error[index]))
    error = position_error[axis]
    if abs(error) <= float(position_deadband):
        return CartJogCommand(jog_index=0, jog_dir=0, step_size=0.0, jog_axis_type=0, vel_scale=0.0)
    return CartJogCommand(
        jog_index=axis,
        jog_dir=1 if error > 0.0 else 2,
        step_size=min(abs(error), float(max_step_size)),
        jog_axis_type=0,
        vel_scale=float(vel_scale),
    )


def select_cart_jog_command(
    *,
    control_pose_base_tcp: list[float],
    current_pose_base_tcp: list[float],
    can_jog: bool,
    position_deadband: float,
    max_step_size: float,
    vel_scale: float,
) -> tuple[CartJogCommand, list[float]]:
    pose_error_base_tcp = [
        float(control_pose_base_tcp[0]) - float(current_pose_base_tcp[0]),
        float(control_pose_base_tcp[1]) - float(current_pose_base_tcp[1]),
        float(control_pose_base_tcp[2]) - float(current_pose_base_tcp[2]),
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    if not can_jog:
        return (
            CartJogCommand(jog_index=0, jog_dir=0, step_size=0.0, jog_axis_type=0, vel_scale=0.0),
            pose_error_base_tcp,
        )
    return (
        pose_error_to_cart_jog_cmd(
            pose_error_base_tcp,
            position_deadband=position_deadband,
            max_step_size=max_step_size,
            vel_scale=vel_scale,
        ),
        pose_error_base_tcp,
    )


class VirtualJogFeedback:
    def __init__(self) -> None:
        self._pose_base_tcp: list[float] | None = None

    def reset(self) -> None:
        self._pose_base_tcp = None

    def current_pose(self, measured_pose_base_tcp: list[float]) -> list[float]:
        measured_pose = [float(value) for value in measured_pose_base_tcp]
        if len(measured_pose) != 7:
            raise ValueError("measured_pose_base_tcp must contain 7 values")
        if self._pose_base_tcp is None:
            self._pose_base_tcp = list(measured_pose)
        return list(self._pose_base_tcp)

    def advance(self, command: CartJogCommand) -> None:
        if self._pose_base_tcp is None:
            return
        if command.jog_dir not in {1, 2} or command.step_size <= 0.0:
            return
        if command.jog_index not in {0, 1, 2}:
            return
        sign = 1.0 if command.jog_dir == 1 else -1.0
        self._pose_base_tcp[command.jog_index] += sign * float(command.step_size)


def valid_target_drives_or_none(target_drives, *, max_norm: float):
    values = [float(value) for value in target_drives]
    if not values or not all(math.isfinite(value) for value in values):
        return None, float("nan")
    norm = math.sqrt(sum(value * value for value in values))
    if norm > float(max_norm):
        return None, norm
    return values, norm


def _format_float_list(values, *, precision: int = 4) -> str:
    return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"


def format_state_torque_telemetry(
    *,
    servo_cycle: int,
    q,
    dq,
    target_drives,
    torque_norm: float,
    current_pose_base_tcp,
    control_pose_base_tcp,
) -> str:
    current_pose = [float(value) for value in current_pose_base_tcp]
    target_pose = [float(value) for value in control_pose_base_tcp]
    pos_error = [target_pose[index] - current_pose[index] for index in range(3)]
    return (
        "[FlexivStudioBall] state_torque "
        f"cycle={int(servo_cycle)} "
        f"q={_format_float_list(q)} "
        f"dq={_format_float_list(dq)} "
        f"tau={_format_float_list(target_drives)} "
        f"tau_norm={float(torque_norm):.4f} "
        f"tcp_xyz={_format_float_list(current_pose[:3])} "
        f"target_xyz={_format_float_list(target_pose[:3])} "
        f"pos_err={_format_float_list(pos_error)}"
    )


def format_jog_command_telemetry(
    *,
    servo_cycle: int,
    command: CartJogCommand,
    pose_error_base_tcp,
    can_jog: bool,
) -> str:
    return (
        "[FlexivStudioBall] jog_cmd "
        f"cycle={int(servo_cycle)} "
        f"can_jog={bool(can_jog)} "
        f"jog_index={int(command.jog_index)} "
        f"jog_dir={int(command.jog_dir)} "
        f"step_size={float(command.step_size):.4f} "
        f"axis_type={int(command.jog_axis_type)} "
        f"vel_scale={float(command.vel_scale):.4f} "
        f"pos_err={_format_float_list([float(value) for value in pose_error_base_tcp[:3]])}"
    )


def should_poll_simplugin_target_drives(
    *,
    connected: bool,
    disable_simplugin_target_drives: bool,
    quest_target_active: bool,
) -> bool:
    _ = quest_target_active
    return bool(connected) and not bool(disable_simplugin_target_drives)


def target_pose_control_is_active(
    *,
    quest_target_receiver_enabled: bool,
    latest_quest_target: QuestTargetPacket | None,
) -> bool:
    _ = quest_target_receiver_enabled
    _ = latest_quest_target
    return True


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


class StudioReachabilityClient:
    def __init__(self, address: str, timeout: float) -> None:
        try:
            grpc = import_grpc_module()
        except ImportError as exc:
            raise RuntimeError("Studio IK requires grpcio in this Python environment") from exc
        self._timeout = float(timeout)
        self._channel = grpc.insecure_channel(address)
        self._call = self._channel.unary_unary(
            "/proto.robot.motion.MotionService/CalReachability",
            request_serializer=lambda value: value,
            response_deserializer=lambda value: value,
        )

    def solve(self, *, target_pose: list[str], seed_jnt_pos: list[str]) -> list[float]:
        request = encode_cal_reachability_request(target_pose=target_pose, seed_jnt_pos=seed_jnt_pos)
        return parse_cal_reachability_response(self._call(request, timeout=self._timeout))

    def close(self) -> None:
        self._channel.close()


class StudioJoggingClient:
    def __init__(self, address: str, timeout: float, *, coord_type: int = 0, coord_name: str = "WORLD") -> None:
        try:
            grpc = import_grpc_module()
        except ImportError as exc:
            raise RuntimeError("Studio jogging requires grpcio in this Python environment") from exc
        self._timeout = float(timeout)
        self._coord_type = int(coord_type)
        self._coord_name = str(coord_name)
        self._channel = grpc.insecure_channel(address)
        self._call = self._channel.unary_unary(
            "/proto.robot.motion.MotionService/SetCartJoggingCmd",
            request_serializer=lambda value: value,
            response_deserializer=lambda value: value,
        )

    def send(self, command: CartJogCommand) -> bytes:
        request = encode_set_cart_jogging_cmd_request(
            coord_type=self._coord_type,
            coord_name=self._coord_name,
            jog_index=command.jog_index,
            jog_dir=command.jog_dir,
            step_size=command.step_size,
            jog_axis_type=command.jog_axis_type,
            vel_scale=command.vel_scale,
        )
        return self._call(request, timeout=self._timeout)

    def stop(self) -> None:
        self.send(CartJogCommand(jog_index=0, jog_dir=0, step_size=0.0, jog_axis_type=0, vel_scale=0.0))

    def close(self) -> None:
        self._channel.close()


def apply_key_nudge(
    pose: TargetPose,
    key_name: str,
    *,
    position_step: float,
    rotation_step_deg: float,
) -> TargetPose:
    delta = KEYBOARD_DELTAS.get(key_name.upper())
    if delta is None:
        return pose
    dpos, deuler = delta
    position = tuple(round(value + scale * position_step, 10) for value, scale in zip(pose.position, dpos))
    euler = tuple(round(value + scale * rotation_step_deg, 10) for value, scale in zip(pose.euler_deg, deuler))
    return TargetPose(position=position, euler_deg=euler)


class KeyboardBallDriver:
    def __init__(self, ball, initial_pose: TargetPose, position_step: float, rotation_step_deg: float) -> None:
        import carb.input
        import omni.appwindow

        self._carb_input = carb.input
        self._ball = ball
        self._pose = initial_pose
        self._position_step = position_step
        self._rotation_step_deg = rotation_step_deg
        self._input = carb.input.acquire_input_interface()
        self._keyboard = omni.appwindow.get_default_app_window().get_keyboard()
        self._subscription = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)

    def close(self) -> None:
        if self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._subscription)
            self._subscription = None

    def apply_pose(self) -> None:
        import numpy as np

        self._ball.set_world_pose(
            position=np.array(self._pose.position),
            orientation=np.array(euler_xyz_deg_to_quat_wxyz(self._pose.euler_deg)),
        )

    def reset_pose(self, pose: TargetPose) -> None:
        self._pose = pose
        self.apply_pose()

    def update_pose_reference(self, pose: TargetPose) -> None:
        self._pose = pose

    def _on_keyboard_event(self, event, *_args, **_kwargs) -> bool:
        if event.type not in {
            self._carb_input.KeyboardEventType.KEY_PRESS,
            self._carb_input.KeyboardEventType.KEY_REPEAT,
        }:
            return False
        next_pose = apply_key_nudge(
            self._pose,
            event.input.name,
            position_step=self._position_step,
            rotation_step_deg=self._rotation_step_deg,
        )
        if next_pose == self._pose:
            return False
        self._pose = next_pose
        self.apply_pose()
        print(
            "[FlexivStudioBall] ball "
            f"pos={tuple(round(v, 4) for v in self._pose.position)} "
            f"euler_deg={tuple(round(v, 1) for v in self._pose.euler_deg)}",
            flush=True,
        )
        return False


def _select_target(prim_path: str) -> None:
    try:
        import omni.usd

        omni.usd.get_context().get_selection().set_selected_prim_paths([prim_path], True)
    except Exception:
        pass


def _add_default_lighting() -> None:
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Gf, UsdLux

    stage = get_current_stage()
    dome = UsdLux.DomeLight.Define(stage, "/World/defaultDomeLight")
    dome.CreateIntensityAttr(500.0)
    dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
    distant = UsdLux.DistantLight.Define(stage, "/World/defaultDistantLight")
    distant.CreateIntensityAttr(3000.0)
    distant.CreateAngleAttr(0.53)
    distant.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))


def _is_robot_ready(robot) -> bool:
    articulation_view = getattr(robot, "_articulation_view", None)
    if articulation_view is None:
        return False
    try:
        return bool(articulation_view.is_physics_handle_valid())
    except Exception:
        return False


def run(args: argparse.Namespace) -> int:
    examples_ext = Path(args.examples_ext).expanduser().resolve()
    if str(examples_ext) not in sys.path:
        sys.path.insert(0, str(examples_ext))
    physics_hz = physics_hz_for_args(args)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "extra_args": [
                "--enable",
                "isaacsim.robot.manipulators.examples",
                "--/crashreporter/skipOldDumpUpload=1",
                "--/crashreporter/gatherUserStory=0",
            ],
        }
    )

    import flexivsimplugin
    import numpy as np
    import omni.timeline
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import VisualSphere
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.robot.manipulators.examples.flexiv import FlexivSerial

    if flexivsimplugin.__version__ != COMPATIBLE_SIM_PLUGIN_VER:
        raise ImportError(
            f"flexivsimplugin=={COMPATIBLE_SIM_PLUGIN_VER} is required, but found {flexivsimplugin.__version__}"
        )

    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / physics_hz,
        rendering_dt=1.0 / float(args.render_hz),
        set_defaults=False,
    )
    world.scene.add_default_ground_plane()
    _add_default_lighting()

    usd = str(Path(args.usd).expanduser().resolve())
    add_reference_to_stage(usd_path=usd, prim_path="/World/Flexiv/Rizon4")
    robot = world.scene.add(
        FlexivSerial(
            prim_path="/World/Flexiv/Rizon4",
            name="Rizon4",
            end_effector_prim_name="flange",
        )
    )
    initial_pose = configured_initial_target_pose(args)
    initial_q = initial_q_for_args(args)
    ball = world.scene.add(
        VisualSphere(
            prim_path=args.target_prim_path,
            name=args.target_name,
            position=np.array(initial_pose.position),
            orientation=np.array(euler_xyz_deg_to_quat_wxyz(initial_pose.euler_deg)),
            radius=float(args.ball_radius),
            color=np.array([0.0, 0.45, 1.0]),
        )
    )

    sim_node = flexivsimplugin.UserNode(args.serial_number)
    keyboard = None
    studio_ik = (
        StudioReachabilityClient(args.studio_grpc_address, args.studio_grpc_timeout)
        if args.control_source == "studio-ik"
        else None
    )
    studio_jog = (
        StudioJoggingClient(args.studio_grpc_address, args.studio_grpc_timeout)
        if args.control_source == "studio-jog"
        else None
    )
    target_pose_publisher = None if args.disable_target_pose_udp else TargetPoseUdpPublisher(
        args.target_pose_udp_host,
        args.target_pose_udp_port,
    )
    quest_target_receiver = (
        QuestTargetUdpReceiver(
            args.quest_target_udp_host,
            args.quest_target_udp_port,
            serial_number=args.serial_number,
            joint_group=args.joint_group,
            max_age_sec=float(args.quest_target_max_age_sec),
        )
        if args.enable_quest_target_udp
        else None
    )
    if quest_target_receiver is not None:
        print(
            "[FlexivStudioBall] Quest target UDP listening on "
            f"{quest_target_receiver.address[0]}:{quest_target_receiver.address[1]}",
            flush=True,
        )
    quest_coordinate_observer = (
        TargetPoseUdpPublisher(
            args.quest_coordinate_observe_udp_host,
            args.quest_coordinate_observe_udp_port,
        )
        if args.enable_quest_target_udp and int(args.quest_coordinate_observe_udp_port) > 0
        else None
    )
    if quest_coordinate_observer is not None:
        print(
            "[FlexivStudioBall] Quest coordinate observation UDP publishing to "
            f"{args.quest_coordinate_observe_udp_host}:{args.quest_coordinate_observe_udp_port}",
            flush=True,
        )
    quest_relative_mapper = QuestRelativeTargetMapper(
        axis_map=parse_quest_axis_map(args.quest_axis_map),
        scale=float(args.quest_position_scale),
        workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
        workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
    )
    virtual_jog_feedback = VirtualJogFeedback()
    target_pose_publish_gate = TargetPosePublishGate.from_hz(
        float(args.target_pose_publish_hz),
        physics_freq=physics_hz,
    )
    studio_ik_gate = TargetPosePublishGate.from_hz(float(args.studio_ik_hz), physics_freq=physics_hz)
    studio_jog_gate = TargetPosePublishGate.from_hz(float(args.studio_jog_hz), physics_freq=physics_hz)
    state_torque_log_gate = TargetPosePublishGate.from_hz(float(args.state_torque_log_hz), physics_freq=physics_hz)
    servo_cycle = 0
    last_connected = False
    effort_control_enabled = False
    target_drive_warmup_remaining = 0
    valid_target_drive_streak = 0
    last_studio_ik_q = None
    last_studio_ik_error_cycle = -10_000
    last_studio_jog_error_cycle = -10_000
    last_invalid_target_drive_cycle = -10_000
    last_target_drive_log_cycle = -10_000
    last_jog_command_log_cycle = -10_000
    last_quest_target_log_cycle = -10_000
    latest_quest_target = None

    def on_physics_step(_dt):
        nonlocal servo_cycle, last_connected, effort_control_enabled, target_drive_warmup_remaining
        nonlocal valid_target_drive_streak
        nonlocal last_studio_ik_q, last_studio_ik_error_cycle, last_studio_jog_error_cycle
        nonlocal last_invalid_target_drive_cycle, last_target_drive_log_cycle, last_jog_command_log_cycle
        nonlocal last_quest_target_log_cycle
        nonlocal latest_quest_target
        servo_cycle += 1
        sim_node.SendRobotStates(flexivsimplugin.SimRobotStates(servo_cycle, robot.q, robot.dq))

        base_position, base_orientation = robot.get_world_pose()
        if quest_target_receiver is not None:
            quest_target = quest_target_receiver.poll_latest()
            if quest_target is not None:
                latest_quest_target = quest_target
                if servo_cycle - last_quest_target_log_cycle >= int(physics_hz):
                    rounded_pose = tuple(round(value, 4) for value in quest_target.pose_base_tcp_des[:3])
                    print(
                        "[FlexivStudioBall] "
                        f"direct_quest_target seq={quest_target.seq} side={quest_target.side} "
                        f"pose_xyz={rounded_pose}",
                        flush=True,
                    )
                    last_quest_target_log_cycle = servo_cycle
            if not quest_target_is_fresh(
                latest_quest_target,
                max_age_sec=float(args.quest_target_max_age_sec),
            ):
                latest_quest_target = None
                quest_relative_mapper.reset()
                virtual_jog_feedback.reset()
        target_position, target_orientation = ball.get_world_pose()
        pose_base_tcp_des = select_pose_base_tcp_des(
            quest_target=latest_quest_target,
            world_position=target_position,
            world_orientation_wxyz=target_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        if latest_quest_target is not None and args.quest_target_mode == "absolute":
            synced_target_pose = sync_target_ball_to_base_tcp_pose(
                ball,
                pose_base_tcp_des=pose_base_tcp_des,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            if keyboard is not None:
                keyboard.update_pose_reference(synced_target_pose)

        if target_pose_publisher is not None and target_pose_publish_gate.should_publish(servo_cycle):
            target_pose_publisher.publish(
                build_target_pose_packet(
                    serial_number=args.serial_number,
                    joint_group=args.joint_group,
                    servo_cycle=servo_cycle,
                    pose_base_tcp_des=pose_base_tcp_des,
                    monotonic_time=time.monotonic(),
                )
            )

        if args.control_source == "studio-ik":
            if not _is_robot_ready(robot):
                return
            if studio_ik is not None and studio_ik_gate.should_publish(servo_cycle):
                try:
                    last_studio_ik_q = studio_ik.solve(
                        target_pose=pose_base_tcp_des_to_studio_target_pose(pose_base_tcp_des),
                        seed_jnt_pos=joint_positions_rad_to_studio_seed(robot.q),
                    )
                except Exception as exc:
                    if servo_cycle - last_studio_ik_error_cycle >= int(physics_hz):
                        print(f"[FlexivStudioBall] Studio IK failed: {exc}", flush=True)
                        last_studio_ik_error_cycle = servo_cycle
            if last_studio_ik_q is not None:
                robot.apply_action(
                    ArticulationAction(
                        joint_positions=np.array(last_studio_ik_q),
                        joint_indices=np.arange(0, 7),
                    )
            )
            return

        if args.control_source == "studio-jog":
            if not _is_robot_ready(robot):
                last_connected = False
                effort_control_enabled = False
                return
            connected = sim_node.connected()
            quest_target_active = target_pose_control_is_active(
                quest_target_receiver_enabled=quest_target_receiver is not None,
                latest_quest_target=latest_quest_target,
            )
            current_pose_base_tcp = None
            tcp_position, tcp_orientation = robot.end_effector.get_world_pose()
            measured_pose_base_tcp = world_target_to_flexiv_pose(
                world_position=tcp_position,
                world_orientation_wxyz=tcp_orientation,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            current_pose_base_tcp = (
                virtual_jog_feedback.current_pose(measured_pose_base_tcp)
                if args.studio_jog_feedback_source == "virtual"
                else measured_pose_base_tcp
            )
            control_pose_base_tcp = pose_base_tcp_des
            if latest_quest_target is not None and args.quest_target_mode == "relative":
                control_pose_base_tcp = quest_relative_mapper.update(latest_quest_target, current_pose_base_tcp)
            if latest_quest_target is not None and args.quest_target_mode == "relative":
                synced_target_pose = sync_target_ball_to_base_tcp_pose(
                    ball,
                    pose_base_tcp_des=control_pose_base_tcp,
                    base_position=base_position,
                    base_orientation_wxyz=base_orientation,
                )
                if keyboard is not None:
                    keyboard.update_pose_reference(synced_target_pose)
            if quest_coordinate_observer is not None and target_pose_publish_gate.should_publish(servo_cycle):
                quest_coordinate_observer.publish(
                    build_coordinate_observation_packet(
                        serial_number=args.serial_number,
                        joint_group=args.joint_group,
                        servo_cycle=servo_cycle,
                        quest_target=latest_quest_target,
                        active=quest_target_active,
                        current_pose_base_tcp=current_pose_base_tcp,
                        target_pose_base_tcp=control_pose_base_tcp if quest_target_active else None,
                        monotonic_time=time.monotonic(),
                    )
                )
            if connected and not last_connected:
                print(
                    f"[FlexivStudioBall] SimPlugin connected {args.serial_number}; "
                    f"warming up {max(0, int(args.target_drive_warmup_cycles))} cycles",
                    flush=True,
                )
                target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
                effort_control_enabled = False
                valid_target_drive_streak = 0
                robot.switch_control_mode("position")
                robot.teleport_to(robot.q)

            if not quest_target_active:
                virtual_jog_feedback.reset()
                if studio_jog is not None and studio_jog_gate.should_publish(servo_cycle):
                    try:
                        studio_jog.stop()
                    except Exception as exc:
                        if servo_cycle - last_studio_jog_error_cycle >= int(physics_hz):
                            print(f"[FlexivStudioBall] Studio jogging failed: {exc}", flush=True)
                            last_studio_jog_error_cycle = servo_cycle

            can_jog = bool(args.disable_simplugin_target_drives) or (
                connected and target_drive_warmup_remaining <= 0
            )
            if quest_target_active and studio_jog is not None and studio_jog_gate.should_publish(servo_cycle):
                try:
                    command, pose_error_base_tcp = select_cart_jog_command(
                        control_pose_base_tcp=control_pose_base_tcp,
                        current_pose_base_tcp=current_pose_base_tcp,
                        can_jog=can_jog,
                        position_deadband=float(args.studio_jog_position_deadband),
                        max_step_size=float(args.studio_jog_max_step_size),
                        vel_scale=float(args.studio_jog_vel_scale),
                    )
                    jog_log_period_cycles = (
                        int(physics_hz / float(args.state_torque_log_hz))
                        if float(args.state_torque_log_hz) > 0.0
                        else 0
                    )
                    if jog_log_period_cycles > 0 and servo_cycle - last_jog_command_log_cycle >= jog_log_period_cycles:
                        print(
                            format_jog_command_telemetry(
                                servo_cycle=servo_cycle,
                                command=command,
                                pose_error_base_tcp=pose_error_base_tcp,
                                can_jog=can_jog,
                            ),
                            flush=True,
                        )
                        last_jog_command_log_cycle = servo_cycle
                    studio_jog.send(command)
                    if args.studio_jog_feedback_source == "virtual":
                        virtual_jog_feedback.advance(command)
                except Exception as exc:
                    if servo_cycle - last_studio_jog_error_cycle >= int(physics_hz):
                        print(f"[FlexivStudioBall] Studio jogging failed: {exc}", flush=True)
                        last_studio_jog_error_cycle = servo_cycle

            if args.disable_simplugin_target_drives:
                if effort_control_enabled:
                    robot.switch_control_mode("position")
                    robot.teleport_to(robot.q)
                    effort_control_enabled = False
                valid_target_drive_streak = 0
                last_connected = connected
                return

            if should_poll_simplugin_target_drives(
                connected=connected,
                disable_simplugin_target_drives=bool(args.disable_simplugin_target_drives),
                quest_target_active=quest_target_active,
            ):
                if sim_node.WaitForRobotCommands(max(0, int(args.command_timeout_ms))):
                    if target_drive_warmup_remaining > 0:
                        target_drive_warmup_remaining -= 1
                        last_connected = True
                        return
                    target_drives, torque_norm = valid_target_drives_or_none(
                        sim_node.robot_commands().target_drives,
                        max_norm=float(args.max_target_drive_norm),
                    )
                    if target_drives is None:
                        if effort_control_enabled:
                            robot.switch_control_mode("position")
                            robot.teleport_to(robot.q)
                            effort_control_enabled = False
                        valid_target_drive_streak = 0
                        if servo_cycle - last_invalid_target_drive_cycle >= int(physics_hz):
                            print(
                                f"[FlexivStudioBall] rejected target_drives norm={torque_norm:.3g}; "
                                "waiting for Studio/SimPlugin to settle",
                                flush=True,
                            )
                            last_invalid_target_drive_cycle = servo_cycle
                        last_connected = True
                        return
                    valid_target_drive_streak += 1
                    if valid_target_drive_streak < max(1, int(args.target_drive_required_valid_cycles)):
                        last_connected = True
                        return
                    if not effort_control_enabled:
                        robot.switch_control_mode("effort")
                        effort_control_enabled = True
                    if state_torque_log_gate.should_publish(servo_cycle):
                        print(
                            format_state_torque_telemetry(
                                servo_cycle=servo_cycle,
                                q=robot.q,
                                dq=robot.dq,
                                target_drives=target_drives,
                                torque_norm=torque_norm,
                                current_pose_base_tcp=current_pose_base_tcp,
                                control_pose_base_tcp=control_pose_base_tcp,
                            ),
                            flush=True,
                        )
                    robot.apply_torques(target_drives)
                    if servo_cycle - last_target_drive_log_cycle >= int(physics_hz):
                        print(f"[FlexivStudioBall] target_drives norm={torque_norm:.3f}", flush=True)
                        last_target_drive_log_cycle = servo_cycle
                last_connected = True
            else:
                if last_connected:
                    print(f"[FlexivStudioBall] SimPlugin disconnected {args.serial_number}", flush=True)
                    robot.switch_control_mode("position")
                    robot.teleport_to(robot.q)
                effort_control_enabled = False
                valid_target_drive_streak = 0
                last_connected = False
            return

        if args.control_source == "studio-bridge" and sim_node.connected():
            if not _is_robot_ready(robot):
                last_connected = False
                effort_control_enabled = False
                return
            if not last_connected:
                print(
                    f"[FlexivStudioBall] SimPlugin connected {args.serial_number}; "
                    f"warming up {max(0, int(args.target_drive_warmup_cycles))} cycles",
                    flush=True,
                )
                target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
                effort_control_enabled = False
                valid_target_drive_streak = 0
                robot.switch_control_mode("position")
                robot.teleport_to(robot.q)
            if sim_node.WaitForRobotCommands(max(0, int(args.command_timeout_ms))):
                if target_drive_warmup_remaining > 0:
                    target_drive_warmup_remaining -= 1
                    last_connected = True
                    return
                target_drives, torque_norm = valid_target_drives_or_none(
                    sim_node.robot_commands().target_drives,
                    max_norm=float(args.max_target_drive_norm),
                )
                if target_drives is None:
                    if effort_control_enabled:
                        robot.switch_control_mode("position")
                        robot.teleport_to(robot.q)
                        effort_control_enabled = False
                    valid_target_drive_streak = 0
                    if servo_cycle - last_invalid_target_drive_cycle >= int(physics_hz):
                        print(
                            f"[FlexivStudioBall] rejected target_drives norm={torque_norm:.3g}; "
                            "waiting for Studio/SimPlugin to settle",
                            flush=True,
                        )
                        last_invalid_target_drive_cycle = servo_cycle
                    last_connected = True
                    return
                valid_target_drive_streak += 1
                if valid_target_drive_streak < max(1, int(args.target_drive_required_valid_cycles)):
                    last_connected = True
                    return
                if not effort_control_enabled:
                    robot.switch_control_mode("effort")
                    effort_control_enabled = True
                robot.apply_torques(target_drives)
            last_connected = True
        else:
            if last_connected:
                print(f"[FlexivStudioBall] SimPlugin disconnected {args.serial_number}", flush=True)
                robot.switch_control_mode("position")
                robot.teleport_to(robot.q)
            effort_control_enabled = False
            valid_target_drive_streak = 0
            last_connected = False

    def reset_target_to_start_pose() -> TargetPose:
        nonlocal initial_pose
        if bool(args.attach_target_to_ee_on_start):
            ee_position, ee_orientation = robot.end_effector.get_world_pose()
            initial_pose = initial_target_pose_for_args(
                args,
                end_effector_position=ee_position,
                end_effector_orientation_wxyz=ee_orientation,
            )
            ball.set_world_pose(
                position=np.array(initial_pose.position),
                orientation=np.array(ee_orientation),
            )
        else:
            initial_pose = configured_initial_target_pose(args)
            ball.set_world_pose(
                position=np.array(initial_pose.position),
                orientation=np.array(euler_xyz_deg_to_quat_wxyz(initial_pose.euler_deg)),
            )
        return initial_pose

    world.add_physics_callback("studio_ball_step", callback_fn=on_physics_step)
    world.reset()
    robot.teleport_to(initial_q)
    robot.switch_control_mode("position")
    reset_target_to_start_pose()
    if not args.smoke_test:
        keyboard = KeyboardBallDriver(
            ball,
            initial_pose,
            position_step=args.position_step,
            rotation_step_deg=args.rotation_step_deg,
        )
    _select_target(args.target_prim_path)

    if not args.manual_play:
        omni.timeline.get_timeline_interface().play()

    print(
        f"[FlexivStudioBall] Ready. control_source={args.control_source}. "
        f"physics_hz={physics_hz:g}. Drag /World/TargetBall as the visual task target.",
        flush=True,
    )

    reset_needed = False
    frame_count = 0
    rate_limiter = StepRateLimiter(physics_hz)
    render_gate = TargetPosePublishGate.from_hz(float(args.render_hz), physics_freq=physics_hz)
    try:
        while simulation_app.is_running():
            frame_count += 1
            world.step(render=render_gate.should_publish(frame_count))
            if world.is_stopped() and not reset_needed:
                reset_needed = True
            if world.is_playing():
                if reset_needed:
                    world.reset()
                    robot.teleport_to(initial_q)
                    robot.switch_control_mode("position")
                    reset_needed = False
                    reset_target_to_start_pose()
                    if keyboard is not None:
                        keyboard.reset_pose(initial_pose)

            if args.smoke_test and args.max_frames > 0 and frame_count >= args.max_frames:
                break
            rate_limiter.sleep()
    finally:
        try:
            world.remove_physics_callback("studio_ball_step")
        except Exception:
            pass
        if keyboard is not None:
            keyboard.close()
        if target_pose_publisher is not None:
            target_pose_publisher.close()
        if quest_coordinate_observer is not None:
            quest_coordinate_observer.close()
        if quest_target_receiver is not None:
            quest_target_receiver.close()
        if studio_ik is not None:
            studio_ik.close()
        if studio_jog is not None:
            try:
                studio_jog.stop()
            except Exception:
                pass
            studio_jog.close()
        if args.smoke_test:
            print("FLEXIV_STUDIO_BALL_SMOKE_TEST_OK", flush=True)
        simulation_app.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
