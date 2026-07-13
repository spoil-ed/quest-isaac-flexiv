#!/usr/bin/env python3
"""Pure Isaac Sim scene driven through the documented Flexiv-Isaac bridge."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

UTILS_DIR = Path(__file__).resolve().parents[1]
if str(UTILS_DIR) not in sys.path:
    sys.path.insert(0, str(UTILS_DIR))
REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from elements_studio_utils import (
    RdkRuntimeController,
    RdkRuntimeSettings,
    StudioReachabilityClient,
    joint_speed_limit_exceeded,
    joint_positions_rad_to_studio_seed,
    studio_target_pose_from_rdk_pose as pose_base_tcp_des_to_studio_target_pose,
    valid_target_drives_or_none,
)
from control_helpers import (
    StepRateLimiter,
    TargetPosePublishGate,
    format_float_list as _format_float_list,
    format_pose_xyz_quat,
    format_state_torque_telemetry,
    should_poll_simplugin_target_drives,
    target_pose_control_is_active,
)
from targeting import (
    CartesianTargetLimiter,
    QuestRelativeTargetMapper,
    QuestTargetPacket,
    QuestTargetUdpReceiver,
    TargetPose,
    TargetPoseUdpPublisher,
    build_coordinate_observation_packet,
    build_target_pose_packet,
    camera_look_at_quat_wxyz,
    euler_xyz_deg_to_quat_wxyz,
    flexiv_pose_to_world_target,
    map_openxr_delta_to_base,
    parse_float_list,
    parse_quest_axis_map,
    parse_quest_target_packet,
    quest_target_is_fresh,
    select_pose_base_tcp_des,
    sync_target_to_base_tcp_pose,
    target_pose_from_world_pose,
    triple as _triple,
    world_target_to_flexiv_pose,
)
from flexiv_data_collection.protocol import BRIDGE_SAMPLE_TYPE, JsonLinePushClient, encode_image_bgr, now_ns
from flexiv_data_collection.schema import unitree_parts_from_single_arm


DEFAULT_RIZON4_USD = Path(os.environ["FLEXIV_RIZON4_USD"]) if os.environ.get("FLEXIV_RIZON4_USD") else None
DEFAULT_EXAMPLES_EXT = Path(os.environ["FLEXIV_EXAMPLES_EXT"]) if os.environ.get("FLEXIV_EXAMPLES_EXT") else None
DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"
DEFAULT_ROBOT_PRIM_PATH = "/World/Flexiv/Rizon4"
DEFAULT_ROBOT_NAME = "Rizon4"
DEFAULT_END_EFFECTOR_PRIM_NAME = "flange"
DEFAULT_TARGET_PRIM_PATH = "/World/TargetFrame"
DEFAULT_TARGET_NAME = "target_frame"
DEFAULT_TARGET_POSITION = (0.45, 0.0, 0.35)
DEFAULT_TARGET_EULER_DEG = (0.0, 0.0, 0.0)
DEFAULT_TARGET_AXIS_LENGTH = 0.14
DEFAULT_TARGET_AXIS_RADIUS = 0.006
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
DEFAULT_STAGE1_GATEWAY_FPS = 30.0
DEFAULT_STAGE1_GATEWAY_JPEG_QUALITY = 90
PHYSICS_FREQ = 2000.0
RENDER_FREQ = 60.0
DEFAULT_PHYSICS_HZ = PHYSICS_FREQ
DEFAULT_STUDIO_IK_HZ = 30.0
COMPATIBLE_SIM_PLUGIN_VER = "1.2.0"


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


def target_arrow_specs(axis_length: float = DEFAULT_TARGET_AXIS_LENGTH, axis_radius: float = DEFAULT_TARGET_AXIS_RADIUS):
    shaft_length = float(axis_length) * 0.72
    head_length = float(axis_length) - shaft_length
    shaft_center = shaft_length / 2.0
    head_center = shaft_length + head_length / 2.0
    head_radius = float(axis_radius) * 2.8
    q_45 = math.sqrt(0.5)
    axis_data = {
        "x": {
            "direction": (1.0, 0.0, 0.0),
            "orientation": (q_45, 0.0, q_45, 0.0),
            "color": (1.0, 0.05, 0.05),
        },
        "y": {
            "direction": (0.0, 1.0, 0.0),
            "orientation": (q_45, -q_45, 0.0, 0.0),
            "color": (0.05, 0.75, 0.15),
        },
        "z": {
            "direction": (0.0, 0.0, 1.0),
            "orientation": (1.0, 0.0, 0.0, 0.0),
            "color": (0.1, 0.35, 1.0),
        },
    }
    specs = []
    for axis in ("x", "y", "z"):
        data = axis_data[axis]
        direction = data["direction"]
        specs.append(
            {
                "axis": axis,
                "kind": "shaft",
                "translation": tuple(component * shaft_center for component in direction),
                "orientation": data["orientation"],
                "color": data["color"],
                "radius": float(axis_radius),
                "height": shaft_length,
            }
        )
        specs.append(
            {
                "axis": axis,
                "kind": "head",
                "translation": tuple(component * head_center for component in direction),
                "orientation": data["orientation"],
                "color": data["color"],
                "radius": head_radius,
                "height": head_length,
            }
        )
    return specs


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--params-json",
        type=Path,
        default=None,
        help="Optional JSON file containing parameter overrides by argument name.",
    )
    parser.add_argument("--scene-config", type=Path, default=None, help="Optional Stage1 scene YAML containing robot and camera definitions.")
    parser.add_argument("--serial-number", default=None, help="Elements Studio simulated robot serial.")
    parser.add_argument("--joint-group", default=None, help="Studio joint group label, normally ARM_1.")
    parser.add_argument("--robot-prim-path", default=None, help="USD prim path for the Rizon4 robot.")
    parser.add_argument("--robot-name", default=None, help="Isaac scene object name for the Rizon4 robot.")
    parser.add_argument("--end-effector-prim-name", default=None, help="End-effector prim name used by the Flexiv articulation.")
    parser.add_argument("--usd", type=Path, default=None, help="Rizon4 USD asset. Defaults to scene config or FLEXIV_RIZON4_USD.")
    parser.add_argument("--examples-ext", type=Path, default=None, help="Flexiv examples extension path. Defaults to scene config or FLEXIV_EXAMPLES_EXT.")
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
        choices=("rdk-cartesian", "studio-ik", "studio-bridge"),
        default="rdk-cartesian",
        help=(
            "rdk-cartesian streams target TCP poses through flexivrdk and applies Studio target_drives; "
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
        "--rdk-target-hz",
        type=float,
        default=30.0,
        help="RDK Cartesian target update frequency.",
    )
    parser.add_argument(
        "--rdk-network-interface-whitelist",
        default="",
        help="Comma-separated local IPv4 whitelist for Flexiv RDK discovery. Empty tries all interfaces.",
    )
    parser.add_argument("--rdk-switch-mode", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--rdk-serial-number",
        default=None,
        help="Optional Flexiv RDK robot serial. Defaults to --serial-number.",
    )
    parser.add_argument("--rdk-clear-fault", action="store_true")
    parser.add_argument("--rdk-servo-on", action="store_true")
    parser.add_argument("--rdk-verbose", action="store_true")
    parser.add_argument("--target-prim-path", default=DEFAULT_TARGET_PRIM_PATH, help="USD prim path for the XYZ target frame.")
    parser.add_argument("--target-name", default=DEFAULT_TARGET_NAME, help="Scene object name for the XYZ target frame.")
    parser.add_argument("--target-axis-length", type=float, default=DEFAULT_TARGET_AXIS_LENGTH, help="XYZ target frame axis length in meters.")
    parser.add_argument("--target-axis-radius", type=float, default=DEFAULT_TARGET_AXIS_RADIUS, help="XYZ target frame axis radius in meters.")
    parser.add_argument(
        "--initial-q",
        type=float,
        nargs=7,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="Initial robot joint positions in radians. Defaults to Studio home for bridge mode.",
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
        "--max-target-drive-abs",
        type=float,
        default=100.0,
        help="Reject Studio target_drives when any joint magnitude exceeds this value.",
    )
    parser.add_argument(
        "--max-joint-speed-rad-s",
        type=float,
        default=1.5,
        help="Leave effort mode if any simulated joint exceeds this absolute speed; <=0 disables.",
    )
    parser.add_argument(
        "--target-drive-required-valid-cycles",
        type=int,
        default=5,
        help="Require this many consecutive valid target_drives before entering effort control.",
    )
    parser.add_argument(
        "--target-drive-scale",
        type=float,
        default=1.0,
        help="Scale valid Studio target_drives before applying them in Isaac. Default 1 preserves legacy behavior.",
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
        help="Listen for Quest target pose UDP packets and move the visual target frame.",
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
        "--quest-relative-orientation-mode",
        choices=("packet", "reference", "current"),
        default="packet",
        help=(
            "Relative mode orientation source. packet preserves legacy Quest packet behavior; "
            "reference keeps the press-time TCP orientation; current keeps the live TCP orientation."
        ),
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
        "--quest-position-deadband-m",
        type=float,
        default=0.01,
        help="Ignore mapped Quest translation components smaller than this value in meters.",
    )
    parser.add_argument(
        "--max-linear-speed-m-s",
        type=float,
        default=0.10,
        help="Maximum commanded TCP translation speed after all input mapping; <=0 disables.",
    )
    parser.add_argument(
        "--max-angular-speed-rad-s",
        type=float,
        default=0.75,
        help="Maximum commanded TCP angular speed after all input mapping; <=0 disables.",
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
        help="Initial target frame position in world frame.",
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
        help="Initialize the visual target frame at the current end-effector pose.",
    )
    parser.add_argument(
        "--gateway-endpoint",
        default="",
        help="Optional Stage1 data gateway bridge endpoint, e.g. tcp://127.0.0.1:5591.",
    )
    parser.add_argument("--gateway-fps", type=float, default=DEFAULT_STAGE1_GATEWAY_FPS)
    parser.add_argument("--gateway-jpeg-quality", type=int, default=DEFAULT_STAGE1_GATEWAY_JPEG_QUALITY)
    parser.add_argument(
        "--coordinated-reset",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Consume gateway reset commands and reinitialize Isaac/Studio at the configured home pose.",
    )
    parser.add_argument(
        "--reset-settle-sec",
        type=float,
        default=2.0,
        help="Publish and hold the home TCP target for this long after reset.",
    )
    parser.add_argument(
        "--camera-config",
        type=Path,
        default=None,
        help="Legacy alias for --scene-config when only camera definitions are needed.",
    )
    args = parser.parse_args(argv)
    apply_param_overrides(args)
    apply_scene_config(args)
    args.serial_number = args.serial_number or DEFAULT_SERIAL_NUMBER
    args.joint_group = args.joint_group or DEFAULT_JOINT_GROUP
    args.robot_prim_path = args.robot_prim_path or DEFAULT_ROBOT_PRIM_PATH
    args.robot_name = args.robot_name or DEFAULT_ROBOT_NAME
    args.end_effector_prim_name = args.end_effector_prim_name or DEFAULT_END_EFFECTOR_PRIM_NAME
    args.usd = args.usd or DEFAULT_RIZON4_USD
    args.examples_ext = args.examples_ext or DEFAULT_EXAMPLES_EXT
    if args.smoke_test:
        args.headless = True
    return args


PARAM_OVERRIDE_KEYS = {
    "control_source",
    "physics_hz",
    "render_hz",
    "rdk_target_hz",
    "rdk_network_interface_whitelist",
    "rdk_serial_number",
    "robot_prim_path",
    "robot_name",
    "end_effector_prim_name",
    "rdk_switch_mode",
    "rdk_clear_fault",
    "rdk_servo_on",
    "rdk_verbose",
    "studio_grpc_address",
    "studio_grpc_timeout",
    "command_timeout_ms",
    "target_drive_warmup_cycles",
    "max_target_drive_norm",
    "max_target_drive_abs",
    "max_joint_speed_rad_s",
    "target_drive_required_valid_cycles",
    "target_drive_scale",
    "state_torque_log_hz",
    "initial_q",
    "target_position",
    "target_euler_deg",
    "target_axis_length",
    "target_axis_radius",
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
    "quest_relative_orientation_mode",
    "quest_axis_map",
    "quest_position_scale",
    "quest_position_deadband_m",
    "max_linear_speed_m_s",
    "max_angular_speed_rad_s",
    "quest_workspace_min",
    "quest_workspace_max",
    "gateway_endpoint",
    "gateway_fps",
    "gateway_jpeg_quality",
    "coordinated_reset",
    "reset_settle_sec",
    "scene_config",
    "camera_config",
}


def _read_structured_config(path: Path) -> dict | list:
    config_path = Path(path).expanduser().resolve()
    raw = config_path.read_text(encoding="utf-8")
    if config_path.suffix.lower() == ".json":
        return json.loads(raw)
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required for YAML scene/camera config files") from exc
    return yaml.safe_load(raw) or {}


def apply_scene_config(args: argparse.Namespace) -> None:
    if args.scene_config is None:
        return
    args.scene_config = args.scene_config.expanduser().resolve()
    data = _read_structured_config(args.scene_config)
    scene_base = args.scene_config.parent
    if not isinstance(data, dict):
        raise ValueError("--scene-config must contain a YAML/JSON object")
    robot = data.get("robot") or {}
    if not isinstance(robot, dict):
        raise ValueError("--scene-config robot must be an object")
    if args.serial_number is None and robot.get("serial_number") is not None:
        args.serial_number = str(robot["serial_number"])
    if args.joint_group is None and robot.get("joint_group") is not None:
        args.joint_group = str(robot["joint_group"])
    if args.robot_prim_path is None and robot.get("prim_path") is not None:
        args.robot_prim_path = str(robot["prim_path"])
    if args.robot_name is None and robot.get("name") is not None:
        args.robot_name = str(robot["name"])
    if args.end_effector_prim_name is None and robot.get("end_effector_prim_name") is not None:
        args.end_effector_prim_name = str(robot["end_effector_prim_name"])
    if args.usd is None and robot.get("usd") is not None:
        args.usd = Path(str(robot["usd"])).expanduser()
        if not args.usd.is_absolute():
            args.usd = (scene_base / args.usd).resolve()
    if args.examples_ext is None and robot.get("examples_ext") is not None:
        args.examples_ext = Path(str(robot["examples_ext"])).expanduser()
        if not args.examples_ext.is_absolute():
            args.examples_ext = (scene_base / args.examples_ext).resolve()
    if args.camera_config is None and data.get("cameras") is not None:
        args.camera_config = args.scene_config


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


def initial_q_for_args(args: argparse.Namespace) -> list[float]:
    if args.initial_q is not None:
        return [float(value) for value in args.initial_q]
    if args.control_source == "studio-bridge":
        return list(DEFAULT_STUDIO_INITIAL_Q)
    return list(DEFAULT_INITIAL_Q)


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


class KeyboardTargetDriver:
    def __init__(self, target, initial_pose: TargetPose, position_step: float, rotation_step_deg: float) -> None:
        import carb.input
        import omni.appwindow

        self._carb_input = carb.input
        self._target = target
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

        self._target.set_world_pose(
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
            "[FlexivTargetFrame] target_frame "
            f"pos={tuple(round(v, 4) for v in self._pose.position)} "
            f"euler_deg={tuple(round(v, 1) for v in self._pose.euler_deg)}",
            flush=True,
        )
        return False


def create_xyz_target_frame(world, *, prim_path: str, name: str, initial_pose: TargetPose, axis_length: float, axis_radius: float):
    import numpy as np
    from isaacsim.core.api.objects import VisualCone, VisualCylinder
    from isaacsim.core.prims import SingleXFormPrim

    target = world.scene.add(
        SingleXFormPrim(
            prim_path=prim_path,
            name=name,
            position=np.array(initial_pose.position),
            orientation=np.array(euler_xyz_deg_to_quat_wxyz(initial_pose.euler_deg)),
        )
    )
    for spec in target_arrow_specs(axis_length=axis_length, axis_radius=axis_radius):
        prim_cls = VisualCylinder if spec["kind"] == "shaft" else VisualCone
        prim_cls(
            prim_path=f"{prim_path}/{spec['axis']}_{spec['kind']}",
            name=f"{name}_{spec['axis']}_{spec['kind']}",
            translation=np.array(spec["translation"]),
            orientation=np.array(spec["orientation"]),
            radius=float(spec["radius"]),
            height=float(spec["height"]),
            color=np.array(spec["color"]),
        )
    return target


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


def _load_camera_config(path: Path | None):
    if path is None:
        raise ValueError("Stage1 gateway requires --scene-config or legacy --camera-config with a non-empty cameras list")
    data = _read_structured_config(path)
    if isinstance(data, dict):
        cameras = data.get("cameras", [])
    else:
        cameras = data
    if not isinstance(cameras, list) or not cameras:
        raise ValueError("--scene-config/--camera-config must define a non-empty cameras list")
    return cameras


def _xyz_mapping(values, *, name: str) -> list[float]:
    if not isinstance(values, dict):
        raise ValueError(f"{name} must be an object with x, y, z fields")
    return [float(values[key]) for key in ("x", "y", "z")]


def camera_pose_from_config(camera_cfg: dict) -> tuple[list[float], list[float]]:
    """Resolve either position+look_at or the legacy position+quaternion camera form."""
    position = _xyz_mapping(camera_cfg["position"], name="camera position")
    if camera_cfg.get("look_at") is not None:
        look_at = _xyz_mapping(camera_cfg["look_at"], name="camera look_at")
        up = _xyz_mapping(camera_cfg.get("up", {"x": 0.0, "y": 0.0, "z": 1.0}), name="camera up")
        orientation = camera_look_at_quat_wxyz(position, look_at, up)
    else:
        orientation_cfg = camera_cfg.get("orientation")
        if not isinstance(orientation_cfg, dict):
            raise ValueError("camera requires either look_at or orientation")
        orientation = [float(orientation_cfg[key]) for key in ("w", "x", "y", "z")]
    return position, orientation


def _camera_rgba_to_bgr(image):
    import cv2
    import numpy as np

    arr = np.asarray(image)
    if arr.dtype != np.uint8:
        arr = np.clip(arr * 255.0, 0, 255).astype(np.uint8)
    if arr.ndim != 3:
        raise ValueError(f"Unexpected camera image shape: {arr.shape}")
    if arr.shape[2] == 4:
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if arr.shape[2] == 3:
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    raise ValueError(f"Unexpected camera channels: {arr.shape}")


def _is_robot_ready(robot) -> bool:
    articulation_view = getattr(robot, "_articulation_view", None)
    if articulation_view is None:
        return False
    try:
        return bool(articulation_view.is_physics_handle_valid())
    except Exception:
        return False


def run(args: argparse.Namespace) -> int:
    if args.examples_ext is None:
        raise RuntimeError("Flexiv examples extension is not configured; pass --scene-config, --examples-ext, or set FLEXIV_EXAMPLES_EXT")
    if args.usd is None:
        raise RuntimeError("Rizon4 USD is not configured; pass --scene-config, --usd, or set FLEXIV_RIZON4_USD")
    examples_ext = Path(args.examples_ext).expanduser().resolve()
    physics_hz = physics_hz_for_args(args)

    from isaacsim import SimulationApp

    if str(examples_ext) not in sys.path:
        sys.path.append(str(examples_ext))

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
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.robot.manipulators.examples.flexiv import FlexivSerial
    from isaacsim.sensors.camera import Camera

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

    stage1_cameras = []
    if args.gateway_endpoint:
        for idx, camera_cfg in enumerate(_load_camera_config(args.camera_config)):
            name = str(camera_cfg.get("name", f"cam_{idx}"))
            pos, ori = camera_pose_from_config(camera_cfg)
            camera = Camera(
                prim_path="/World/" + name,
                frequency=float(camera_cfg.get("fps", args.gateway_fps)),
                resolution=tuple(camera_cfg.get("resolution", [640, 480])),
                position=pos,
                orientation=ori,
            )
            camera.set_focal_length(float(camera_cfg.get("focal_length", 2.5)))
            camera.set_world_pose(position=pos, orientation=ori, camera_axes="usd")
            stage1_cameras.append(camera)
        print(f"[FlexivTargetFrame] Stage1 gateway cameras enabled: {len(stage1_cameras)}", flush=True)

    usd = str(Path(args.usd).expanduser().resolve())
    add_reference_to_stage(usd_path=usd, prim_path=args.robot_prim_path)
    robot = world.scene.add(
        FlexivSerial(
            prim_path=args.robot_prim_path,
            name=args.robot_name,
            end_effector_prim_name=args.end_effector_prim_name,
        )
    )
    initial_pose = configured_initial_target_pose(args)
    initial_q = initial_q_for_args(args)
    target_frame = create_xyz_target_frame(
        world,
        prim_path=args.target_prim_path,
        name=args.target_name,
        initial_pose=initial_pose,
        axis_length=float(args.target_axis_length),
        axis_radius=float(args.target_axis_radius),
    )

    sim_node = flexivsimplugin.UserNode(args.serial_number)
    keyboard = None
    studio_ik = (
        StudioReachabilityClient(args.studio_grpc_address, args.studio_grpc_timeout)
        if args.control_source == "studio-ik"
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
            "[FlexivTargetFrame] Quest target UDP listening on "
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
            "[FlexivTargetFrame] Quest coordinate observation UDP publishing to "
            f"{args.quest_coordinate_observe_udp_host}:{args.quest_coordinate_observe_udp_port}",
            flush=True,
        )
    quest_relative_mapper = QuestRelativeTargetMapper(
        axis_map=parse_quest_axis_map(args.quest_axis_map),
        scale=float(args.quest_position_scale),
        workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
        workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
        position_deadband_m=float(args.quest_position_deadband_m),
        orientation_mode=str(args.quest_relative_orientation_mode),
    )
    cartesian_target_limiter = CartesianTargetLimiter(
        workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
        workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
        max_linear_speed_m_s=float(args.max_linear_speed_m_s),
        max_angular_speed_rad_s=float(args.max_angular_speed_rad_s),
    )
    target_pose_publish_gate = TargetPosePublishGate.from_hz(
        float(args.target_pose_publish_hz),
        physics_freq=physics_hz,
    )
    rdk_target_gate = TargetPosePublishGate.from_hz(float(args.rdk_target_hz), physics_freq=physics_hz)
    studio_ik_gate = TargetPosePublishGate.from_hz(float(args.studio_ik_hz), physics_freq=physics_hz)
    state_torque_log_gate = TargetPosePublishGate.from_hz(float(args.state_torque_log_hz), physics_freq=physics_hz)
    servo_cycle = 0
    last_connected = False
    effort_control_enabled = False
    target_drive_warmup_remaining = 0
    valid_target_drive_streak = 0
    last_studio_ik_q = None
    last_studio_ik_error_cycle = -10_000
    last_invalid_target_drive_cycle = -10_000
    last_target_drive_log_cycle = -10_000
    last_quest_target_log_cycle = -10_000
    last_rdk_error_cycle = -10_000
    last_rdk_command_log_cycle = -10_000
    latest_quest_target = None
    rdk_runtime = None
    rdk_runtime_target_active = False
    latest_target_drives = [0.0] * 7
    gateway_client = None
    gateway_last_connect_attempt = 0.0
    gateway_last_publish = 0.0
    pending_reset_control = None
    reset_hold_pose_base_tcp = None
    reset_hold_cycles_remaining = 0
    last_reset_seq = 0
    latest_control_pose_base_tcp = None

    def on_physics_step(_dt):
        nonlocal servo_cycle, last_connected, effort_control_enabled, target_drive_warmup_remaining
        nonlocal valid_target_drive_streak
        nonlocal last_studio_ik_q, last_studio_ik_error_cycle
        nonlocal last_invalid_target_drive_cycle, last_target_drive_log_cycle
        nonlocal last_quest_target_log_cycle, last_rdk_error_cycle, last_rdk_command_log_cycle
        nonlocal latest_quest_target, rdk_runtime, rdk_runtime_target_active, latest_target_drives
        nonlocal reset_hold_pose_base_tcp, reset_hold_cycles_remaining, latest_control_pose_base_tcp
        servo_cycle += 1
        sim_node.SendRobotStates(flexivsimplugin.SimRobotStates(servo_cycle, robot.q, robot.dq))

        base_position, base_orientation = robot.get_world_pose()
        if quest_target_receiver is not None:
            reset_hold_active = reset_hold_pose_base_tcp is not None and reset_hold_cycles_remaining > 0
            if reset_hold_active:
                quest_target_receiver.clear()
                quest_target = None
                latest_quest_target = None
                quest_relative_mapper.reset()
            else:
                quest_target = quest_target_receiver.poll_latest()
            if quest_target is not None:
                latest_quest_target = quest_target
                if servo_cycle - last_quest_target_log_cycle >= int(physics_hz):
                    print(
                        "[FlexivTargetFrame] "
                        f"direct_quest_target seq={quest_target.seq} side={quest_target.side} "
                        f"{format_pose_xyz_quat(quest_target.pose_base_tcp_des)}",
                        flush=True,
                    )
                    last_quest_target_log_cycle = servo_cycle
            if not quest_target_is_fresh(
                latest_quest_target,
                max_age_sec=float(args.quest_target_max_age_sec),
            ):
                latest_quest_target = None
                quest_relative_mapper.reset()
        target_position, target_orientation = target_frame.get_world_pose()
        pose_base_tcp_des = select_pose_base_tcp_des(
            quest_target=latest_quest_target,
            world_position=target_position,
            world_orientation_wxyz=target_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        current_pose_base_tcp = None
        control_pose_base_tcp = pose_base_tcp_des
        reset_hold_active = reset_hold_pose_base_tcp is not None and reset_hold_cycles_remaining > 0
        if reset_hold_active:
            control_pose_base_tcp = list(reset_hold_pose_base_tcp)
            synced_target_pose = sync_target_to_base_tcp_pose(
                target_frame,
                pose_base_tcp_des=control_pose_base_tcp,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            if keyboard is not None:
                keyboard.update_pose_reference(synced_target_pose)
            cartesian_target_limiter.reset(control_pose_base_tcp)
        elif latest_quest_target is not None and args.quest_target_mode == "relative":
            tcp_position, tcp_orientation = robot.end_effector.get_world_pose()
            current_pose_base_tcp = world_target_to_flexiv_pose(
                world_position=tcp_position,
                world_orientation_wxyz=tcp_orientation,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            control_pose_base_tcp = quest_relative_mapper.update(latest_quest_target, current_pose_base_tcp)
        if latest_quest_target is not None and not reset_hold_active:
            if current_pose_base_tcp is None:
                tcp_position, tcp_orientation = robot.end_effector.get_world_pose()
                current_pose_base_tcp = world_target_to_flexiv_pose(
                    world_position=tcp_position,
                    world_orientation_wxyz=tcp_orientation,
                    base_position=base_position,
                    base_orientation_wxyz=base_orientation,
                )
            if cartesian_target_limiter.last_pose is None:
                cartesian_target_limiter.reset(current_pose_base_tcp)
            control_pose_base_tcp = cartesian_target_limiter.limit(control_pose_base_tcp, dt=float(_dt))
            synced_target_pose = sync_target_to_base_tcp_pose(
                target_frame,
                pose_base_tcp_des=control_pose_base_tcp,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            if keyboard is not None:
                keyboard.update_pose_reference(synced_target_pose)
        elif not reset_hold_active:
            cartesian_target_limiter.reset()

        latest_control_pose_base_tcp = [float(value) for value in control_pose_base_tcp]

        quest_target_active = target_pose_control_is_active(
            quest_target_receiver_enabled=quest_target_receiver is not None,
            latest_quest_target=latest_quest_target,
        ) or reset_hold_active

        if (
            quest_target_active
            and target_pose_publisher is not None
            and target_pose_publish_gate.should_publish(servo_cycle)
        ):
            target_pose_publisher.publish(
                build_target_pose_packet(
                    serial_number=args.serial_number,
                    joint_group=args.joint_group,
                    servo_cycle=servo_cycle,
                    pose_base_tcp_des=control_pose_base_tcp,
                    monotonic_time=time.monotonic(),
                )
            )

        if reset_hold_active:
            reset_hold_cycles_remaining -= 1
            if reset_hold_cycles_remaining <= 0:
                reset_hold_pose_base_tcp = None
                print("[FlexivTargetFrame] startup/reset Studio hold completed", flush=True)

        if args.control_source == "studio-ik":
            if not _is_robot_ready(robot):
                return
            if studio_ik is not None and studio_ik_gate.should_publish(servo_cycle):
                try:
                    last_studio_ik_q = studio_ik.solve(
                        target_pose=pose_base_tcp_des_to_studio_target_pose(control_pose_base_tcp),
                        seed_jnt_pos=joint_positions_rad_to_studio_seed(robot.q),
                    )
                except Exception as exc:
                    if servo_cycle - last_studio_ik_error_cycle >= int(physics_hz):
                        print(f"[FlexivTargetFrame] Studio IK failed: {exc}", flush=True)
                        last_studio_ik_error_cycle = servo_cycle
            if last_studio_ik_q is not None:
                robot.apply_action(
                    ArticulationAction(
                        joint_positions=np.array(last_studio_ik_q),
                        joint_indices=np.arange(0, 7),
                    )
            )
            return

        if args.control_source == "rdk-cartesian":
            if not _is_robot_ready(robot):
                last_connected = False
                effort_control_enabled = False
                return
            connected = sim_node.connected()
            if current_pose_base_tcp is None:
                tcp_position, tcp_orientation = robot.end_effector.get_world_pose()
                current_pose_base_tcp = world_target_to_flexiv_pose(
                    world_position=tcp_position,
                    world_orientation_wxyz=tcp_orientation,
                    base_position=base_position,
                    base_orientation_wxyz=base_orientation,
                )
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

            if quest_target_active and rdk_target_gate.should_publish(servo_cycle):
                try:
                    if rdk_runtime is None:
                        rdk_serial_number = args.rdk_serial_number or args.serial_number
                        rdk_runtime = RdkRuntimeController(
                            RdkRuntimeSettings(
                                serial_number=rdk_serial_number,
                                joint_group=args.joint_group,
                                network_interface_whitelist=args.rdk_network_interface_whitelist,
                                switch_mode=bool(args.rdk_switch_mode),
                                clear_fault=bool(args.rdk_clear_fault),
                                servo_on=bool(args.rdk_servo_on),
                                verbose=bool(args.rdk_verbose),
                            ),
                            log=lambda message: print(f"[FlexivTargetFrame] {message}", flush=True),
                        )
                        rdk_runtime.connect()
                        print(f"[FlexivTargetFrame] RDK connected {rdk_serial_number}", flush=True)
                    rdk_runtime.send_pose(control_pose_base_tcp)
                    rdk_runtime_target_active = True
                    rdk_log_period_cycles = (
                        int(physics_hz / float(args.state_torque_log_hz))
                        if float(args.state_torque_log_hz) > 0.0
                        else 0
                    )
                    if rdk_log_period_cycles > 0 and servo_cycle - last_rdk_command_log_cycle >= rdk_log_period_cycles:
                        print(
                            "[FlexivTargetFrame] rdk_target "
                            f"cycle={servo_cycle} "
                            f"{format_pose_xyz_quat(control_pose_base_tcp)}",
                            flush=True,
                        )
                        last_rdk_command_log_cycle = servo_cycle
                except Exception as exc:
                    rdk_runtime = None
                    rdk_runtime_target_active = False
                    if servo_cycle - last_rdk_error_cycle >= int(physics_hz):
                        print(f"[FlexivTargetFrame] RDK Cartesian streaming failed: {exc}", flush=True)
                        last_rdk_error_cycle = servo_cycle

            if connected and not last_connected:
                print(
                    f"[FlexivTargetFrame] SimPlugin connected {args.serial_number}; "
                    f"warming up {max(0, int(args.target_drive_warmup_cycles))} cycles",
                    flush=True,
                )
                target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
                effort_control_enabled = False
                valid_target_drive_streak = 0
                robot.switch_control_mode("position")
                robot.teleport_to(robot.q)

            if should_poll_simplugin_target_drives(
                connected=connected,
                runtime_target_active=rdk_runtime_target_active,
            ):
                if joint_speed_limit_exceeded(robot.dq, max_abs_rad_s=float(args.max_joint_speed_rad_s)):
                    if effort_control_enabled:
                        robot.switch_control_mode("position")
                        robot.teleport_to(robot.q)
                        effort_control_enabled = False
                    valid_target_drive_streak = 0
                    if servo_cycle - last_invalid_target_drive_cycle >= int(physics_hz):
                        print(
                            "[FlexivTargetFrame] joint speed limit exceeded; leaving effort control",
                            flush=True,
                        )
                        last_invalid_target_drive_cycle = servo_cycle
                    last_connected = True
                    return
                if sim_node.WaitForRobotCommands(max(0, int(args.command_timeout_ms))):
                    if target_drive_warmup_remaining > 0:
                        target_drive_warmup_remaining -= 1
                        last_connected = True
                        return
                    target_drives, torque_norm = valid_target_drives_or_none(
                        sim_node.robot_commands().target_drives,
                        max_norm=float(args.max_target_drive_norm),
                        max_abs=float(args.max_target_drive_abs),
                    )
                    if target_drives is None:
                        if effort_control_enabled:
                            robot.switch_control_mode("position")
                            robot.teleport_to(robot.q)
                            effort_control_enabled = False
                        valid_target_drive_streak = 0
                        if servo_cycle - last_invalid_target_drive_cycle >= int(physics_hz):
                            print(
                                f"[FlexivTargetFrame] rejected target_drives norm={torque_norm:.3g}; "
                                "waiting for Studio/SimPlugin to settle",
                                flush=True,
                            )
                            last_invalid_target_drive_cycle = servo_cycle
                        last_connected = True
                        return
                    target_drive_scale = float(args.target_drive_scale)
                    if target_drive_scale != 1.0:
                        target_drives = [float(value) * target_drive_scale for value in target_drives]
                        torque_norm = math.sqrt(sum(float(value) * float(value) for value in target_drives))
                    if reset_hold_active:
                        latest_target_drives = list(target_drives[:7])
                        valid_target_drive_streak = 0
                        last_connected = True
                        return
                    valid_target_drive_streak += 1
                    if valid_target_drive_streak < max(1, int(args.target_drive_required_valid_cycles)):
                        last_connected = True
                        return
                    if not effort_control_enabled:
                        robot.switch_control_mode("effort")
                        effort_control_enabled = True
                    latest_target_drives = list(target_drives[:7])
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
                        print(f"[FlexivTargetFrame] target_drives norm={torque_norm:.3f}", flush=True)
                        last_target_drive_log_cycle = servo_cycle
                last_connected = True
            else:
                if connected:
                    if effort_control_enabled:
                        robot.switch_control_mode("position")
                        robot.teleport_to(robot.q)
                        effort_control_enabled = False
                    valid_target_drive_streak = 0
                    last_connected = True
                    return
                if last_connected:
                    print(f"[FlexivTargetFrame] SimPlugin disconnected {args.serial_number}", flush=True)
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
                    f"[FlexivTargetFrame] SimPlugin connected {args.serial_number}; "
                    f"warming up {max(0, int(args.target_drive_warmup_cycles))} cycles",
                    flush=True,
                )
                target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
                effort_control_enabled = False
                valid_target_drive_streak = 0
                robot.switch_control_mode("position")
                robot.teleport_to(robot.q)
            if joint_speed_limit_exceeded(robot.dq, max_abs_rad_s=float(args.max_joint_speed_rad_s)):
                if effort_control_enabled:
                    robot.switch_control_mode("position")
                    robot.teleport_to(robot.q)
                    effort_control_enabled = False
                valid_target_drive_streak = 0
                if servo_cycle - last_invalid_target_drive_cycle >= int(physics_hz):
                    print(
                        "[FlexivTargetFrame] joint speed limit exceeded; leaving effort control",
                        flush=True,
                    )
                    last_invalid_target_drive_cycle = servo_cycle
                last_connected = True
                return
            if sim_node.WaitForRobotCommands(max(0, int(args.command_timeout_ms))):
                if target_drive_warmup_remaining > 0:
                    target_drive_warmup_remaining -= 1
                    last_connected = True
                    return
                target_drives, torque_norm = valid_target_drives_or_none(
                    sim_node.robot_commands().target_drives,
                    max_norm=float(args.max_target_drive_norm),
                    max_abs=float(args.max_target_drive_abs),
                )
                if target_drives is None:
                    if effort_control_enabled:
                        robot.switch_control_mode("position")
                        robot.teleport_to(robot.q)
                        effort_control_enabled = False
                    valid_target_drive_streak = 0
                    if servo_cycle - last_invalid_target_drive_cycle >= int(physics_hz):
                        print(
                            f"[FlexivTargetFrame] rejected target_drives norm={torque_norm:.3g}; "
                            "waiting for Studio/SimPlugin to settle",
                            flush=True,
                        )
                        last_invalid_target_drive_cycle = servo_cycle
                    last_connected = True
                    return
                target_drive_scale = float(args.target_drive_scale)
                if target_drive_scale != 1.0:
                    target_drives = [float(value) * target_drive_scale for value in target_drives]
                    torque_norm = math.sqrt(sum(float(value) * float(value) for value in target_drives))
                if reset_hold_active:
                    latest_target_drives = list(target_drives[:7])
                    valid_target_drive_streak = 0
                    last_connected = True
                    return
                valid_target_drive_streak += 1
                if valid_target_drive_streak < max(1, int(args.target_drive_required_valid_cycles)):
                    last_connected = True
                    return
                if not effort_control_enabled:
                    robot.switch_control_mode("effort")
                    effort_control_enabled = True
                latest_target_drives = list(target_drives[:7])
                robot.apply_torques(target_drives)
            last_connected = True
        else:
            if last_connected:
                print(f"[FlexivTargetFrame] SimPlugin disconnected {args.serial_number}", flush=True)
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
            target_frame.set_world_pose(
                position=np.array(initial_pose.position),
                orientation=np.array(ee_orientation),
            )
        else:
            initial_pose = configured_initial_target_pose(args)
            target_frame.set_world_pose(
                position=np.array(initial_pose.position),
                orientation=np.array(euler_xyz_deg_to_quat_wxyz(initial_pose.euler_deg)),
            )
        return initial_pose

    def initialize_like_startup(reason: str, *, reset_world: bool) -> None:
        """Use exactly the startup sequence for both first launch and later resets."""
        nonlocal last_connected, effort_control_enabled, target_drive_warmup_remaining
        nonlocal valid_target_drive_streak, latest_quest_target, latest_target_drives
        nonlocal rdk_runtime_target_active, reset_hold_pose_base_tcp, reset_hold_cycles_remaining
        if reset_world:
            world.reset()
        robot.teleport_to(initial_q)
        robot.switch_control_mode("position")
        pose = reset_target_to_start_pose()
        base_position, base_orientation = robot.get_world_pose()
        ee_position, ee_orientation = robot.end_effector.get_world_pose()
        reset_hold_pose_base_tcp = world_target_to_flexiv_pose(
            world_position=ee_position,
            world_orientation_wxyz=ee_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        reset_hold_cycles_remaining = max(1, int(max(0.0, float(args.reset_settle_sec)) * physics_hz))
        latest_quest_target = None
        latest_target_drives = [0.0] * 7
        quest_relative_mapper.reset()
        cartesian_target_limiter.reset(reset_hold_pose_base_tcp)
        if quest_target_receiver is not None:
            quest_target_receiver.clear()
        last_connected = False
        effort_control_enabled = False
        target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
        valid_target_drive_streak = 0
        rdk_runtime_target_active = False
        if keyboard is not None:
            keyboard.reset_pose(pose)
        print(
            f"[FlexivTargetFrame] startup initialization applied reason={reason}; "
            f"Studio home hold={max(0.0, float(args.reset_settle_sec)):.3f}s",
            flush=True,
        )

    world.add_physics_callback("target_frame_step", callback_fn=on_physics_step)
    world.reset()
    for camera in stage1_cameras:
        camera.initialize()
    initialize_like_startup("startup", reset_world=False)
    if not args.smoke_test:
        keyboard = KeyboardTargetDriver(
            target_frame,
            initial_pose,
            position_step=args.position_step,
            rotation_step_deg=args.rotation_step_deg,
        )
    _select_target(args.target_prim_path)

    if not args.manual_play:
        omni.timeline.get_timeline_interface().play()

    print(
        f"[FlexivTargetFrame] Ready. control_source={args.control_source}. "
        f"physics_hz={physics_hz:g}. Drag {args.target_prim_path} as the visual task target.",
        flush=True,
    )

    reset_needed = False
    frame_count = 0
    rate_limiter = StepRateLimiter(physics_hz)
    render_gate = TargetPosePublishGate.from_hz(float(args.render_hz), physics_freq=physics_hz)

    def publish_gateway_sample() -> None:
        nonlocal gateway_client, gateway_last_connect_attempt, gateway_last_publish, pending_reset_control
        if not args.gateway_endpoint:
            return
        now = time.monotonic()
        if now - gateway_last_publish < 1.0 / max(float(args.gateway_fps), 1e-6):
            return
        gateway_last_publish = now
        if gateway_client is None:
            if now - gateway_last_connect_attempt < 1.0:
                return
            gateway_last_connect_attempt = now
            try:
                gateway_client = JsonLinePushClient(args.gateway_endpoint, timeout=0.2, retry=False)
                print(f"[FlexivTargetFrame] Stage1 gateway connected {args.gateway_endpoint}", flush=True)
            except Exception as exc:
                print(f"[FlexivTargetFrame] Stage1 gateway connect failed: {exc}", flush=True)
                gateway_client = None
                return

        colors = {}
        for idx, camera in enumerate(stage1_cameras):
            try:
                rgba = camera.get_rgba()
                if rgba is None:
                    continue
                colors[f"color_{idx}"] = encode_image_bgr(
                    _camera_rgba_to_bgr(rgba),
                    quality=int(args.gateway_jpeg_quality),
                )
            except Exception as exc:
                print(f"[FlexivTargetFrame] Stage1 camera {idx} publish failed: {exc}", flush=True)
        if stage1_cameras and len(colors) != len(stage1_cameras):
            return

        torque = list(latest_target_drives[:7])
        if len(torque) < 7:
            torque = [*torque, *([0.0] * (7 - len(torque)))]
        base_position, base_orientation = robot.get_world_pose()
        target_position, target_orientation = target_frame.get_world_pose()
        target_frame_base_tcp = world_target_to_flexiv_pose(
            world_position=target_position,
            world_orientation_wxyz=target_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        latest_quest = None
        if latest_quest_target is not None:
            latest_quest = {
                "seq": int(latest_quest_target.seq),
                "side": str(latest_quest_target.side),
                "controller_delta_base": (
                    None
                    if latest_quest_target.controller_delta_base is None
                    else [float(value) for value in latest_quest_target.controller_delta_base]
                ),
                "pose_base_tcp_des": [float(value) for value in latest_quest_target.pose_base_tcp_des],
            }
        states = unitree_parts_from_single_arm(robot.q, qvel=robot.dq, torque=torque, arm="left")
        actions = unitree_parts_from_single_arm(robot.q, qvel=robot.dq, torque=torque, arm="left")
        sample = {
            "type": BRIDGE_SAMPLE_TYPE,
            "version": 1,
            "seq": int(servo_cycle),
            "stamp_ns": now_ns(),
            "colors": colors,
            "states": states,
            "actions": actions,
            "sim_state": {
                "backend": "quest_isaac_flexiv_stage1",
                "servo_cycle": int(servo_cycle),
                "serial": str(args.serial_number),
                "target_drives": torque,
                "robot_q": [float(value) for value in robot.q],
                "robot_dq": [float(value) for value in robot.dq],
                "target_frame": {
                    "world_position": [float(value) for value in target_position],
                    "world_orientation_wxyz": [float(value) for value in target_orientation],
                    "base_tcp_pose": [float(value) for value in target_frame_base_tcp],
                    "control_pose_base_tcp_des": (
                        None
                        if latest_control_pose_base_tcp is None
                        else [float(value) for value in latest_control_pose_base_tcp]
                    ),
                    "latest_quest": latest_quest,
                },
                "reset": {
                    "last_seq": int(last_reset_seq),
                    "holding_start_pose": bool(reset_hold_cycles_remaining > 0),
                },
            },
        }
        try:
            gateway_client.send_json(sample)
            control = gateway_client.recv_json_if_available()
            if (
                bool(args.coordinated_reset)
                and isinstance(control, dict)
                and control.get("type") == "flexiv_bridge_control"
                and control.get("command") == "reset"
            ):
                pending_reset_control = control
        except Exception as exc:
            print(f"[FlexivTargetFrame] Stage1 gateway send failed: {exc}", flush=True)
            try:
                gateway_client.close()
            except Exception:
                pass
            gateway_client = None

    try:
        while simulation_app.is_running():
            frame_count += 1
            world.step(render=render_gate.should_publish(frame_count))
            publish_gateway_sample()
            if pending_reset_control is not None:
                control = pending_reset_control
                pending_reset_control = None
                reset_seq = int(control.get("seq", 0))
                last_reset_seq = reset_seq
                initialize_like_startup(
                    f"gateway:{control.get('reason', 'unspecified')} seq={reset_seq}",
                    reset_world=True,
                )
                reset_needed = False
            if world.is_stopped() and not reset_needed:
                reset_needed = True
            if world.is_playing():
                if reset_needed:
                    initialize_like_startup("timeline-resume", reset_world=True)
                    reset_needed = False

            if args.smoke_test and args.max_frames > 0 and frame_count >= args.max_frames:
                break
            rate_limiter.sleep()
    finally:
        try:
            world.remove_physics_callback("target_frame_step")
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
        if gateway_client is not None:
            gateway_client.close()
        if args.smoke_test:
            print("FLEXIV_TARGET_FRAME_SMOKE_TEST_OK", flush=True)
        simulation_app.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
