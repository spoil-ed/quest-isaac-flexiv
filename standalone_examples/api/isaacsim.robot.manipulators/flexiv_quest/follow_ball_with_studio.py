#!/usr/bin/env python3
"""Pure Isaac Sim scene driven through the documented Flexiv-Isaac bridge."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

UTILS_DIR = Path(__file__).resolve().parents[1]
if str(UTILS_DIR) not in sys.path:
    sys.path.insert(0, str(UTILS_DIR))

from elements_studio_utils import (
    RdkRuntimeController,
    RdkRuntimeSettings,
    StudioReachabilityClient,
    joint_positions_rad_to_studio_seed,
    studio_target_pose_from_rdk_pose as pose_base_tcp_des_to_studio_target_pose,
    valid_target_drives_or_none,
)
from control_helpers import (
    StepRateLimiter,
    TargetPosePublishGate,
    format_float_list as _format_float_list,
    format_state_torque_telemetry,
    should_poll_simplugin_target_drives,
    target_pose_control_is_active,
)
from targeting import (
    QuestRelativeTargetMapper,
    QuestTargetPacket,
    QuestTargetUdpReceiver,
    TargetPose,
    TargetPoseUdpPublisher,
    build_coordinate_observation_packet,
    build_target_pose_packet,
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


DEFAULT_RIZON4_USD = Path(
    "/home/simate/workspace/isaacsim-flexiv/isaac_sim_ws/exts/"
    "isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd"
)
DEFAULT_EXAMPLES_EXT = Path(
    "/home/simate/workspace/isaacsim-flexiv/isaac_sim_ws/exts/isaacsim.robot.manipulators.examples"
)
DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"
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
        "--target-drive-required-valid-cycles",
        type=int,
        default=5,
        help="Require this many consecutive valid target_drives before entering effort control.",
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
    args = parser.parse_args(argv)
    apply_param_overrides(args)
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
    "rdk_switch_mode",
    "rdk_clear_fault",
    "rdk_servo_on",
    "rdk_verbose",
    "studio_grpc_address",
    "studio_grpc_timeout",
    "command_timeout_ms",
    "target_drive_warmup_cycles",
    "max_target_drive_norm",
    "target_drive_required_valid_cycles",
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

    def on_physics_step(_dt):
        nonlocal servo_cycle, last_connected, effort_control_enabled, target_drive_warmup_remaining
        nonlocal valid_target_drive_streak
        nonlocal last_studio_ik_q, last_studio_ik_error_cycle
        nonlocal last_invalid_target_drive_cycle, last_target_drive_log_cycle
        nonlocal last_quest_target_log_cycle, last_rdk_error_cycle, last_rdk_command_log_cycle
        nonlocal latest_quest_target, rdk_runtime, rdk_runtime_target_active
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
                        "[FlexivTargetFrame] "
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
        target_position, target_orientation = target_frame.get_world_pose()
        pose_base_tcp_des = select_pose_base_tcp_des(
            quest_target=latest_quest_target,
            world_position=target_position,
            world_orientation_wxyz=target_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        if latest_quest_target is not None and args.quest_target_mode == "absolute":
            synced_target_pose = sync_target_to_base_tcp_pose(
                target_frame,
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
            quest_target_active = target_pose_control_is_active(
                quest_target_receiver_enabled=quest_target_receiver is not None,
                latest_quest_target=latest_quest_target,
            )
            tcp_position, tcp_orientation = robot.end_effector.get_world_pose()
            current_pose_base_tcp = world_target_to_flexiv_pose(
                world_position=tcp_position,
                world_orientation_wxyz=tcp_orientation,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            control_pose_base_tcp = pose_base_tcp_des
            if latest_quest_target is not None and args.quest_target_mode == "relative":
                control_pose_base_tcp = quest_relative_mapper.update(latest_quest_target, current_pose_base_tcp)
                synced_target_pose = sync_target_to_base_tcp_pose(
                    target_frame,
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

            if rdk_target_gate.should_publish(servo_cycle):
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
                            f"pose={_format_float_list(control_pose_base_tcp)}",
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
                                f"[FlexivTargetFrame] rejected target_drives norm={torque_norm:.3g}; "
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
                            f"[FlexivTargetFrame] rejected target_drives norm={torque_norm:.3g}; "
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

    world.add_physics_callback("target_frame_step", callback_fn=on_physics_step)
    world.reset()
    robot.teleport_to(initial_q)
    robot.switch_control_mode("position")
    reset_target_to_start_pose()
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
        if args.smoke_test:
            print("FLEXIV_TARGET_FRAME_SMOKE_TEST_OK", flush=True)
        simulation_app.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
