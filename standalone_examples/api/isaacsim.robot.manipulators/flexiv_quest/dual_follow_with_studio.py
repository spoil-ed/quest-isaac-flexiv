#!/usr/bin/env python3
"""Dual Rizon4 Isaac scene driven by Quest/fake targets and Studio bridge."""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


APP_DIR = Path(__file__).resolve().parent
UTILS_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]
for path in (APP_DIR, UTILS_DIR, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from control_helpers import StepRateLimiter, TargetPosePublishGate  # noqa: E402
from elements_studio_utils import joint_speed_limit_exceeded, valid_target_drives_or_none  # noqa: E402
from follow_ball_with_studio import (  # noqa: E402
    COMPATIBLE_SIM_PLUGIN_VER,
    DEFAULT_END_EFFECTOR_PRIM_NAME,
    DEFAULT_INITIAL_Q,
    DEFAULT_JOINT_GROUP,
    DEFAULT_QUEST_AXIS_MAP,
    DEFAULT_QUEST_TARGET_UDP_HOST,
    DEFAULT_QUEST_WORKSPACE_MAX,
    DEFAULT_QUEST_WORKSPACE_MIN,
    DEFAULT_STAGE1_GATEWAY_FPS,
    DEFAULT_STAGE1_GATEWAY_JPEG_QUALITY,
    PHYSICS_FREQ,
    RENDER_FREQ,
    _add_default_lighting,
    _camera_rgba_to_bgr,
    _is_robot_ready,
    _load_camera_config,
    camera_pose_from_config,
    create_xyz_target_frame,
)
from targeting import (  # noqa: E402
    CartesianTargetLimiter,
    QuestRelativeTargetMapper,
    QuestTargetPacket,
    TargetPose,
    TargetPoseUdpPublisher,
    build_target_pose_packet,
    euler_xyz_deg_to_quat_wxyz,
    parse_float_list,
    parse_quest_axis_map,
    parse_quest_target_packet,
    quest_target_is_fresh,
    select_pose_base_tcp_des,
    sync_target_to_base_tcp_pose,
    target_pose_from_world_pose,
    triple,
    world_target_to_flexiv_pose,
)
from flexiv_data_collection.dual_validation import EXPECTED_STAGE2_BACKEND  # noqa: E402
from flexiv_data_collection.protocol import BRIDGE_SAMPLE_TYPE, JsonLinePushClient, encode_image_bgr, now_ns  # noqa: E402
from flexiv_data_collection.schema import unitree_parts_from_dual_arms  # noqa: E402
from flexiv_sim_scenes.config import scene_task_metadata  # noqa: E402


DEFAULT_LEFT_SERIAL_NUMBER = "Rizon4-VIHhZM"
DEFAULT_RIGHT_SERIAL_NUMBER = "Rizon4-WE7ssd"
DEFAULT_LEFT_ROBOT_PRIM_PATH = "/World/Flexiv/LeftRizon4"
DEFAULT_RIGHT_ROBOT_PRIM_PATH = "/World/Flexiv/RightRizon4"
DEFAULT_LEFT_ROBOT_NAME = "LeftRizon4"
DEFAULT_RIGHT_ROBOT_NAME = "RightRizon4"
DEFAULT_LEFT_TARGET_PRIM_PATH = "/World/TargetFrameLeft"
DEFAULT_RIGHT_TARGET_PRIM_PATH = "/World/TargetFrameRight"
DEFAULT_LEFT_TARGET_NAME = "target_frame_left"
DEFAULT_RIGHT_TARGET_NAME = "target_frame_right"
DEFAULT_LEFT_TARGET_POSE_UDP_PORT = 57680
DEFAULT_RIGHT_TARGET_POSE_UDP_PORT = 57681
DEFAULT_QUEST_TARGET_UDP_PORT = 57679


def _read_structured_config(path: Path) -> dict[str, Any] | list[Any]:
    config_path = Path(path).expanduser().resolve()
    raw = config_path.read_text(encoding="utf-8")
    if config_path.suffix.lower() == ".json":
        return json.loads(raw)
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required for YAML scene config files") from exc
    return yaml.safe_load(raw) or {}


def _resolve_path(value: Any, *, base: Path) -> Path | None:
    if value in (None, ""):
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def _xyz(value: Any, *, name: str) -> tuple[float, float, float]:
    if isinstance(value, dict):
        return (float(value["x"]), float(value["y"]), float(value["z"]))
    return triple(value)


def _quat(value: Any | None) -> tuple[float, float, float, float]:
    if value is None:
        return (1.0, 0.0, 0.0, 0.0)
    if isinstance(value, dict):
        return (float(value["w"]), float(value["x"]), float(value["y"]), float(value["z"]))
    values = [float(item) for item in value]
    if len(values) != 4:
        raise ValueError("orientation must contain w,x,y,z")
    return (values[0], values[1], values[2], values[3])


def _euler_deg(value: Any | None) -> tuple[float, float, float]:
    if value is None:
        return (0.0, 0.0, 0.0)
    if isinstance(value, dict):
        return (float(value["x"]), float(value["y"]), float(value["z"]))
    return triple(value)


def _robot_by_side(scene: dict[str, Any], side: str) -> dict[str, Any]:
    for robot in scene.get("robots") or []:
        if isinstance(robot, dict) and str(robot.get("side", "")).lower() == side:
            return robot
    return {}


def apply_scene_config(args: argparse.Namespace) -> None:
    if args.scene_config is None:
        return
    args.scene_config = args.scene_config.expanduser().resolve()
    data = _read_structured_config(args.scene_config)
    if not isinstance(data, dict):
        raise ValueError("--scene-config must contain a YAML/JSON object")
    scene_base = args.scene_config.parent
    left = _robot_by_side(data, "left")
    right = _robot_by_side(data, "right")
    if args.left_serial_number is None and left.get("serial_number") is not None:
        args.left_serial_number = str(left["serial_number"])
    if args.right_serial_number is None and right.get("serial_number") is not None:
        args.right_serial_number = str(right["serial_number"])
    if args.joint_group is None:
        args.joint_group = str(left.get("joint_group") or right.get("joint_group") or DEFAULT_JOINT_GROUP)
    if args.usd is None:
        args.usd = _resolve_path(left.get("usd") or right.get("usd"), base=scene_base)
    if args.examples_ext is None:
        args.examples_ext = _resolve_path(left.get("examples_ext") or right.get("examples_ext"), base=scene_base)
    args.scene_data = data


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--left-serial-number", default=None)
    parser.add_argument("--right-serial-number", default=None)
    parser.add_argument("--joint-group", default=None)
    parser.add_argument("--usd", type=Path, default=None)
    parser.add_argument("--examples-ext", type=Path, default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--smoke-test", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--manual-play", action="store_true")
    parser.add_argument("--physics-hz", type=float, default=PHYSICS_FREQ)
    parser.add_argument("--render-hz", type=float, default=RENDER_FREQ)
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    parser.add_argument("--quest-target-udp-host", default=DEFAULT_QUEST_TARGET_UDP_HOST)
    parser.add_argument("--quest-target-udp-port", type=int, default=DEFAULT_QUEST_TARGET_UDP_PORT)
    parser.add_argument("--quest-target-max-age-sec", type=float, default=0.5)
    parser.add_argument("--quest-target-mode", choices=("absolute", "relative"), default="relative")
    parser.add_argument(
        "--quest-relative-orientation-mode",
        choices=("packet", "reference", "current"),
        default="packet",
    )
    parser.add_argument("--quest-axis-map", default=DEFAULT_QUEST_AXIS_MAP)
    parser.add_argument("--quest-position-scale", type=float, default=0.5)
    parser.add_argument("--quest-position-deadband-m", type=float, default=0.01)
    parser.add_argument("--quest-workspace-min", default=",".join(str(value) for value in DEFAULT_QUEST_WORKSPACE_MIN))
    parser.add_argument("--quest-workspace-max", default=",".join(str(value) for value in DEFAULT_QUEST_WORKSPACE_MAX))
    parser.add_argument("--max-linear-speed-m-s", type=float, default=0.10)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=0.75)
    parser.add_argument("--left-target-pose-udp-host", default="127.0.0.1")
    parser.add_argument("--left-target-pose-udp-port", type=int, default=DEFAULT_LEFT_TARGET_POSE_UDP_PORT)
    parser.add_argument("--right-target-pose-udp-host", default="127.0.0.1")
    parser.add_argument("--right-target-pose-udp-port", type=int, default=DEFAULT_RIGHT_TARGET_POSE_UDP_PORT)
    parser.add_argument("--target-pose-publish-hz", type=float, default=30.0)
    parser.add_argument("--command-timeout-ms", type=int, default=1)
    parser.add_argument("--target-drive-warmup-cycles", type=int, default=2)
    parser.add_argument("--target-drive-required-valid-cycles", type=int, default=1)
    parser.add_argument("--target-drive-scale", type=float, default=1.0)
    parser.add_argument("--max-target-drive-norm", type=float, default=200.0)
    parser.add_argument("--max-target-drive-abs", type=float, default=100.0)
    parser.add_argument("--max-joint-speed-rad-s", type=float, default=1.5)
    parser.add_argument("--target-axis-length", type=float, default=0.14)
    parser.add_argument("--target-axis-radius", type=float, default=0.006)
    parser.add_argument("--gateway-endpoint", default="")
    parser.add_argument("--gateway-fps", type=float, default=DEFAULT_STAGE1_GATEWAY_FPS)
    parser.add_argument("--gateway-jpeg-quality", type=int, default=DEFAULT_STAGE1_GATEWAY_JPEG_QUALITY)
    parser.add_argument("--coordinated-reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    args = parser.parse_args(argv)
    args.scene_data = {}
    apply_scene_config(args)
    args.left_serial_number = args.left_serial_number or DEFAULT_LEFT_SERIAL_NUMBER
    args.right_serial_number = args.right_serial_number or DEFAULT_RIGHT_SERIAL_NUMBER
    args.joint_group = args.joint_group or DEFAULT_JOINT_GROUP
    if args.smoke_test:
        args.headless = True
    return args


class DualQuestTargetUdpReceiver:
    def __init__(
        self,
        host: str,
        port: int,
        *,
        serials: dict[str, str],
        joint_group: str,
        max_age_sec: float,
    ) -> None:
        self._address = (str(host), int(port))
        self._serials = dict(serials)
        self._joint_group = str(joint_group)
        self._max_age_sec = float(max_age_sec)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind(self._address)
        self._socket.setblocking(False)

    @property
    def address(self) -> tuple[str, int]:
        return self._address

    def poll_latest(self) -> dict[str, QuestTargetPacket]:
        latest: dict[str, QuestTargetPacket] = {}
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
            side = str(packet.get("side", "")).lower()
            if side not in self._serials:
                continue
            parsed = parse_quest_target_packet(
                packet,
                serial_number=self._serials[side],
                joint_group=self._joint_group,
                max_age_sec=self._max_age_sec,
            )
            if parsed is not None:
                latest[side] = parsed

    def clear(self) -> None:
        self.poll_latest()

    def close(self) -> None:
        self._socket.close()


@dataclass
class ArmRuntime:
    side: str
    serial_number: str
    joint_group: str
    robot: Any
    target_frame: Any
    sim_node: Any
    target_pose_publisher: TargetPoseUdpPublisher
    mapper: QuestRelativeTargetMapper
    limiter: CartesianTargetLimiter
    target_pose_gate: TargetPosePublishGate
    configured_initial_pose: TargetPose
    initial_q: list[float]
    base_position: tuple[float, float, float]
    base_orientation: tuple[float, float, float, float]
    latest_quest_target: QuestTargetPacket | None = None
    latest_target_drives: list[float] | None = None
    last_connected: bool = False
    effort_control_enabled: bool = False
    target_drive_warmup_remaining: int = 0
    valid_target_drive_streak: int = 0
    last_invalid_target_drive_cycle: int = -10_000
    reset_hold_pose_base_tcp: list[float] | None = None
    reset_hold_cycles_remaining: int = 0
    latest_control_pose_base_tcp: list[float] | None = None


def _scene_robot_config(args: argparse.Namespace, side: str) -> dict[str, Any]:
    return _robot_by_side(args.scene_data, side) if isinstance(args.scene_data, dict) else {}


def _robot_position_orientation(robot_cfg: dict[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    return _xyz(robot_cfg.get("position", (0.0, 0.0, 0.0)), name="robot position"), _quat(robot_cfg.get("orientation"))


def _target_pose_config(robot_cfg: dict[str, Any], *, default_position: tuple[float, float, float]) -> tuple[str, str, TargetPose]:
    target = robot_cfg.get("target") or {}
    prim_path = str(target.get("prim_path") or "")
    name = str(target.get("name") or "")
    position = _xyz(target.get("position", default_position), name="target position")
    euler_deg = _euler_deg(target.get("euler_deg"))
    return prim_path, name, TargetPose(position=position, euler_deg=euler_deg)


def _initial_q_config(robot_cfg: dict[str, Any]) -> list[float]:
    raw = robot_cfg.get("initial_q")
    if raw is None:
        return list(DEFAULT_INITIAL_Q)
    values = [float(value) for value in raw]
    if len(values) != 7:
        raise ValueError("robot initial_q must contain 7 values")
    return values


def _set_robot_base_pose(robot, position, orientation) -> None:
    try:
        import numpy as np

        robot.set_world_pose(position=np.array(position), orientation=np.array(orientation))
    except Exception:
        pass


def _padded(values, length: int) -> list[float]:
    result = [float(value) for value in (values or [])][:length]
    if len(result) < length:
        result.extend([0.0] * (length - len(result)))
    return result


def run(args: argparse.Namespace) -> int:
    if args.examples_ext is None:
        raise RuntimeError("Flexiv examples extension is not configured; pass --scene-config, --examples-ext, or set FLEXIV_EXAMPLES_EXT")
    if args.usd is None:
        raise RuntimeError("Rizon4 USD is not configured; pass --scene-config, --usd, or set FLEXIV_RIZON4_USD")
    if args.left_serial_number == args.right_serial_number:
        raise RuntimeError("left and right serials must be different")
    examples_ext = Path(args.examples_ext).expanduser().resolve()
    physics_hz = float(args.physics_hz)

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
    from isaacsim.sensors.camera import Camera
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
    scene_object_summary: list[dict[str, Any]] = []
    stage3_task = scene_task_metadata(args.scene_data) if isinstance(args.scene_data, dict) else {}
    if isinstance(args.scene_data, dict) and args.scene_data.get("scene_objects"):
        try:
            from flexiv_sim_scenes.isaac import build_scene_objects

            scene_object_summary = build_scene_objects(
                world,
                args.scene_data,
                config_path=args.scene_config,
            )
            print(
                "[FlexivDualTargetFrame] Stage3 scene objects loaded: "
                f"{[item.get('name') for item in scene_object_summary]}",
                flush=True,
            )
        except Exception as exc:
            raise RuntimeError(f"Failed to load Stage3 scene objects from {args.scene_config}: {exc}") from exc

    stage2_cameras = []
    if args.gateway_endpoint:
        for idx, camera_cfg in enumerate(_load_camera_config(args.scene_config)):
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
            stage2_cameras.append(camera)
        print(f"[FlexivDualTargetFrame] Stage2 gateway cameras enabled: {len(stage2_cameras)}", flush=True)

    usd = str(Path(args.usd).expanduser().resolve())
    serials = {"left": args.left_serial_number, "right": args.right_serial_number}
    robot_defaults = {
        "left": (DEFAULT_LEFT_ROBOT_PRIM_PATH, DEFAULT_LEFT_ROBOT_NAME, DEFAULT_LEFT_TARGET_PRIM_PATH, DEFAULT_LEFT_TARGET_NAME, (0.45, 0.35, 0.35)),
        "right": (DEFAULT_RIGHT_ROBOT_PRIM_PATH, DEFAULT_RIGHT_ROBOT_NAME, DEFAULT_RIGHT_TARGET_PRIM_PATH, DEFAULT_RIGHT_TARGET_NAME, (0.45, -0.35, 0.35)),
    }
    arms: dict[str, ArmRuntime] = {}
    for side in ("left", "right"):
        robot_cfg = _scene_robot_config(args, side)
        default_prim, default_name, default_target_prim, default_target_name, default_target_position = robot_defaults[side]
        prim_path = str(robot_cfg.get("prim_path") or default_prim)
        robot_name = str(robot_cfg.get("name") or default_name)
        end_effector = str(robot_cfg.get("end_effector_prim_name") or DEFAULT_END_EFFECTOR_PRIM_NAME)
        base_position, base_orientation = _robot_position_orientation(robot_cfg)
        target_prim, target_name, initial_pose = _target_pose_config(robot_cfg, default_position=default_target_position)
        target_prim = target_prim or default_target_prim
        target_name = target_name or default_target_name

        add_reference_to_stage(usd_path=usd, prim_path=prim_path)
        robot = world.scene.add(
            FlexivSerial(
                prim_path=prim_path,
                name=robot_name,
                end_effector_prim_name=end_effector,
            )
        )
        _set_robot_base_pose(robot, base_position, base_orientation)
        target_frame = create_xyz_target_frame(
            world,
            prim_path=target_prim,
            name=target_name,
            initial_pose=initial_pose,
            axis_length=float(args.target_axis_length),
            axis_radius=float(args.target_axis_radius),
        )
        target_pose_publisher = TargetPoseUdpPublisher(
            getattr(args, f"{side}_target_pose_udp_host"),
            getattr(args, f"{side}_target_pose_udp_port"),
        )
        arms[side] = ArmRuntime(
            side=side,
            serial_number=serials[side],
            joint_group=args.joint_group,
            robot=robot,
            target_frame=target_frame,
            sim_node=flexivsimplugin.UserNode(serials[side]),
            target_pose_publisher=target_pose_publisher,
            mapper=QuestRelativeTargetMapper(
                axis_map=parse_quest_axis_map(args.quest_axis_map),
                scale=float(args.quest_position_scale),
                workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
                workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
                position_deadband_m=float(args.quest_position_deadband_m),
                orientation_mode=str(args.quest_relative_orientation_mode),
            ),
            limiter=CartesianTargetLimiter(
                workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
                workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
                max_linear_speed_m_s=float(args.max_linear_speed_m_s),
                max_angular_speed_rad_s=float(args.max_angular_speed_rad_s),
            ),
            target_pose_gate=TargetPosePublishGate.from_hz(float(args.target_pose_publish_hz), physics_freq=physics_hz),
            configured_initial_pose=initial_pose,
            initial_q=_initial_q_config(robot_cfg),
            base_position=base_position,
            base_orientation=base_orientation,
            latest_target_drives=[0.0] * 7,
        )

    quest_target_receiver = (
        DualQuestTargetUdpReceiver(
            args.quest_target_udp_host,
            args.quest_target_udp_port,
            serials=serials,
            joint_group=args.joint_group,
            max_age_sec=float(args.quest_target_max_age_sec),
        )
        if args.enable_quest_target_udp
        else None
    )
    if quest_target_receiver is not None:
        print(
            "[FlexivDualTargetFrame] Dual Quest target UDP listening on "
            f"{quest_target_receiver.address[0]}:{quest_target_receiver.address[1]}",
            flush=True,
        )

    servo_cycle = 0
    gateway_client = None
    gateway_last_connect_attempt = 0.0
    gateway_last_publish = 0.0
    pending_reset_control = None
    last_reset_seq = 0

    def _current_pose_base_tcp(arm: ArmRuntime) -> list[float]:
        base_position, base_orientation = arm.robot.get_world_pose()
        tcp_position, tcp_orientation = arm.robot.end_effector.get_world_pose()
        return world_target_to_flexiv_pose(
            world_position=tcp_position,
            world_orientation_wxyz=tcp_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )

    def _reset_arm_to_start_pose(arm: ArmRuntime, *, reset_world: bool) -> None:
        if reset_world:
            world.reset()
        _set_robot_base_pose(arm.robot, arm.base_position, arm.base_orientation)
        arm.robot.teleport_to(arm.initial_q)
        arm.robot.switch_control_mode("position")
        arm.target_frame.set_world_pose(
            position=np.array(arm.configured_initial_pose.position),
            orientation=np.array(euler_xyz_deg_to_quat_wxyz(arm.configured_initial_pose.euler_deg)),
        )
        arm.reset_hold_pose_base_tcp = _current_pose_base_tcp(arm)
        arm.reset_hold_cycles_remaining = max(1, int(max(0.0, float(args.reset_settle_sec)) * physics_hz))
        arm.latest_quest_target = None
        arm.latest_target_drives = [0.0] * 7
        arm.mapper.reset()
        arm.limiter.reset(arm.reset_hold_pose_base_tcp)
        arm.last_connected = False
        arm.effort_control_enabled = False
        arm.target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
        arm.valid_target_drive_streak = 0

    def initialize_like_startup(reason: str, *, reset_world: bool) -> None:
        for idx, side in enumerate(("left", "right")):
            _reset_arm_to_start_pose(arms[side], reset_world=reset_world and idx == 0)
        if quest_target_receiver is not None:
            quest_target_receiver.clear()
        print(
            f"[FlexivDualTargetFrame] startup initialization applied reason={reason}; "
            f"Studio home hold={max(0.0, float(args.reset_settle_sec)):.3f}s",
            flush=True,
        )

    def _update_quest_targets() -> None:
        if quest_target_receiver is None:
            return
        latest = quest_target_receiver.poll_latest()
        for side, target in latest.items():
            arms[side].latest_quest_target = target
        for arm in arms.values():
            if not quest_target_is_fresh(arm.latest_quest_target, max_age_sec=float(args.quest_target_max_age_sec)):
                arm.latest_quest_target = None
                arm.mapper.reset()

    def _step_arm(arm: ArmRuntime, dt: float) -> None:
        nonlocal servo_cycle
        arm.sim_node.SendRobotStates(flexivsimplugin.SimRobotStates(servo_cycle, arm.robot.q, arm.robot.dq))
        base_position, base_orientation = arm.robot.get_world_pose()
        target_position, target_orientation = arm.target_frame.get_world_pose()
        pose_base_tcp_des = select_pose_base_tcp_des(
            quest_target=arm.latest_quest_target,
            world_position=target_position,
            world_orientation_wxyz=target_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        control_pose_base_tcp = pose_base_tcp_des
        current_pose_base_tcp = None
        reset_hold_active = arm.reset_hold_pose_base_tcp is not None and arm.reset_hold_cycles_remaining > 0
        if reset_hold_active:
            control_pose_base_tcp = list(arm.reset_hold_pose_base_tcp)
            sync_target_to_base_tcp_pose(
                arm.target_frame,
                pose_base_tcp_des=control_pose_base_tcp,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            arm.mapper.reset()
            arm.limiter.reset(control_pose_base_tcp)
        elif arm.latest_quest_target is not None and args.quest_target_mode == "relative":
            current_pose_base_tcp = _current_pose_base_tcp(arm)
            control_pose_base_tcp = arm.mapper.update(arm.latest_quest_target, current_pose_base_tcp)

        if arm.latest_quest_target is not None and not reset_hold_active:
            if current_pose_base_tcp is None:
                current_pose_base_tcp = _current_pose_base_tcp(arm)
            if arm.limiter.last_pose is None:
                arm.limiter.reset(current_pose_base_tcp)
            control_pose_base_tcp = arm.limiter.limit(control_pose_base_tcp, dt=float(dt))
            sync_target_to_base_tcp_pose(
                arm.target_frame,
                pose_base_tcp_des=control_pose_base_tcp,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
        elif not reset_hold_active:
            arm.limiter.reset()

        arm.latest_control_pose_base_tcp = [float(value) for value in control_pose_base_tcp]

        target_active = arm.latest_quest_target is not None or reset_hold_active
        if target_active and arm.target_pose_gate.should_publish(servo_cycle):
            arm.target_pose_publisher.publish(
                build_target_pose_packet(
                    serial_number=arm.serial_number,
                    joint_group=arm.joint_group,
                    servo_cycle=servo_cycle,
                    pose_base_tcp_des=control_pose_base_tcp,
                    monotonic_time=time.monotonic(),
                )
            )

        if reset_hold_active:
            arm.reset_hold_cycles_remaining -= 1
            if arm.reset_hold_cycles_remaining <= 0:
                arm.reset_hold_pose_base_tcp = None
                print(f"[FlexivDualTargetFrame] {arm.side} startup/reset Studio hold completed", flush=True)

        connected = arm.sim_node.connected()
        if connected and not arm.last_connected:
            print(
                f"[FlexivDualTargetFrame] SimPlugin connected {arm.side} {arm.serial_number}; "
                f"warming up {max(0, int(args.target_drive_warmup_cycles))} cycles",
                flush=True,
            )
            arm.target_drive_warmup_remaining = max(0, int(args.target_drive_warmup_cycles))
            arm.effort_control_enabled = False
            arm.valid_target_drive_streak = 0
            arm.robot.switch_control_mode("position")
            arm.robot.teleport_to(arm.robot.q)
        if not connected:
            if arm.last_connected:
                print(f"[FlexivDualTargetFrame] SimPlugin disconnected {arm.side} {arm.serial_number}", flush=True)
                arm.robot.switch_control_mode("position")
                arm.robot.teleport_to(arm.robot.q)
            arm.last_connected = False
            arm.effort_control_enabled = False
            arm.valid_target_drive_streak = 0
            return
        arm.last_connected = True
        if not _is_robot_ready(arm.robot):
            return
        if joint_speed_limit_exceeded(arm.robot.dq, max_abs_rad_s=float(args.max_joint_speed_rad_s)):
            if arm.effort_control_enabled:
                arm.robot.switch_control_mode("position")
                arm.robot.teleport_to(arm.robot.q)
                arm.effort_control_enabled = False
            arm.valid_target_drive_streak = 0
            if servo_cycle - arm.last_invalid_target_drive_cycle >= int(physics_hz):
                print(f"[FlexivDualTargetFrame] {arm.side} joint speed limit exceeded; leaving effort control", flush=True)
                arm.last_invalid_target_drive_cycle = servo_cycle
            return
        if not arm.sim_node.WaitForRobotCommands(max(0, int(args.command_timeout_ms))):
            return
        if arm.target_drive_warmup_remaining > 0:
            arm.target_drive_warmup_remaining -= 1
            return
        target_drives, torque_norm = valid_target_drives_or_none(
            arm.sim_node.robot_commands().target_drives,
            max_norm=float(args.max_target_drive_norm),
            max_abs=float(args.max_target_drive_abs),
        )
        if target_drives is None:
            if arm.effort_control_enabled:
                arm.robot.switch_control_mode("position")
                arm.robot.teleport_to(arm.robot.q)
                arm.effort_control_enabled = False
            arm.valid_target_drive_streak = 0
            if servo_cycle - arm.last_invalid_target_drive_cycle >= int(physics_hz):
                print(
                    f"[FlexivDualTargetFrame] {arm.side} rejected target_drives norm={torque_norm:.3g}; "
                    "waiting for Studio/SimPlugin to settle",
                    flush=True,
                )
                arm.last_invalid_target_drive_cycle = servo_cycle
            return
        target_drive_scale = float(args.target_drive_scale)
        if target_drive_scale != 1.0:
            target_drives = [float(value) * target_drive_scale for value in target_drives]
            torque_norm = math.sqrt(sum(float(value) * float(value) for value in target_drives))
        arm.latest_target_drives = list(target_drives[:7])
        if reset_hold_active:
            arm.valid_target_drive_streak = 0
            return
        arm.valid_target_drive_streak += 1
        if arm.valid_target_drive_streak < max(1, int(args.target_drive_required_valid_cycles)):
            return
        if not arm.effort_control_enabled:
            arm.robot.switch_control_mode("effort")
            arm.effort_control_enabled = True
        arm.robot.apply_torques(target_drives)

    def on_physics_step(dt):
        nonlocal servo_cycle
        servo_cycle += 1
        _update_quest_targets()
        _step_arm(arms["left"], dt)
        _step_arm(arms["right"], dt)

    def publish_gateway_sample() -> None:
        nonlocal gateway_client, gateway_last_connect_attempt, gateway_last_publish, pending_reset_control
        if not args.gateway_endpoint:
            return
        current_time = time.monotonic()
        if current_time - gateway_last_publish < 1.0 / max(float(args.gateway_fps), 1e-6):
            return
        gateway_last_publish = current_time
        if gateway_client is None:
            if current_time - gateway_last_connect_attempt < 1.0:
                return
            gateway_last_connect_attempt = current_time
            try:
                gateway_client = JsonLinePushClient(args.gateway_endpoint, timeout=0.2, retry=False)
                print(f"[FlexivDualTargetFrame] Stage2 gateway connected {args.gateway_endpoint}", flush=True)
            except Exception as exc:
                print(f"[FlexivDualTargetFrame] Stage2 gateway connect failed: {exc}", flush=True)
                gateway_client = None
                return

        colors = {}
        for idx, camera in enumerate(stage2_cameras):
            try:
                rgba = camera.get_rgba()
                if rgba is None:
                    continue
                colors[f"color_{idx}"] = encode_image_bgr(
                    _camera_rgba_to_bgr(rgba),
                    quality=int(args.gateway_jpeg_quality),
                )
            except Exception as exc:
                print(f"[FlexivDualTargetFrame] Stage2 camera {idx} publish failed: {exc}", flush=True)
        if stage2_cameras and len(colors) != len(stage2_cameras):
            return

        left = arms["left"]
        right = arms["right"]
        left_torque = _padded(left.latest_target_drives, 7)
        right_torque = _padded(right.latest_target_drives, 7)

        def _target_frame_state(arm: ArmRuntime) -> dict[str, Any]:
            base_position, base_orientation = arm.robot.get_world_pose()
            target_position, target_orientation = arm.target_frame.get_world_pose()
            base_tcp_pose = world_target_to_flexiv_pose(
                world_position=target_position,
                world_orientation_wxyz=target_orientation,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            latest_quest = None
            if arm.latest_quest_target is not None:
                latest_quest = {
                    "seq": int(arm.latest_quest_target.seq),
                    "side": str(arm.latest_quest_target.side),
                    "controller_delta_base": (
                        None
                        if arm.latest_quest_target.controller_delta_base is None
                        else [float(value) for value in arm.latest_quest_target.controller_delta_base]
                    ),
                    "pose_base_tcp_des": [float(value) for value in arm.latest_quest_target.pose_base_tcp_des],
                }
            return {
                "world_position": [float(value) for value in target_position],
                "world_orientation_wxyz": [float(value) for value in target_orientation],
                "base_tcp_pose": [float(value) for value in base_tcp_pose],
                "control_pose_base_tcp_des": (
                    None
                    if arm.latest_control_pose_base_tcp is None
                    else [float(value) for value in arm.latest_control_pose_base_tcp]
                ),
                "latest_quest": latest_quest,
            }

        states = unitree_parts_from_dual_arms(
            left.robot.q,
            right.robot.q,
            left_qvel=left.robot.dq,
            right_qvel=right.robot.dq,
            left_torque=left_torque,
            right_torque=right_torque,
        )
        actions = unitree_parts_from_dual_arms(
            left.robot.q,
            right.robot.q,
            left_qvel=left.robot.dq,
            right_qvel=right.robot.dq,
            left_torque=left_torque,
            right_torque=right_torque,
        )
        sample = {
            "type": BRIDGE_SAMPLE_TYPE,
            "version": 1,
            "seq": int(servo_cycle),
            "stamp_ns": now_ns(),
            "colors": colors,
            "states": states,
            "actions": actions,
            "sim_state": {
                "backend": EXPECTED_STAGE2_BACKEND,
                "servo_cycle": int(servo_cycle),
                "servo_cycles": {"left": int(servo_cycle), "right": int(servo_cycle)},
                "serials": {"left": left.serial_number, "right": right.serial_number},
                "target_drives": {"left": left_torque, "right": right_torque},
                "robot_q": {"left": [float(value) for value in left.robot.q], "right": [float(value) for value in right.robot.q]},
                "robot_dq": {"left": [float(value) for value in left.robot.dq], "right": [float(value) for value in right.robot.dq]},
                "target_frames": {
                    "left": _target_frame_state(left),
                    "right": _target_frame_state(right),
                },
                "stage3_task": stage3_task,
                "scene_config": str(args.scene_config) if args.scene_config else None,
                "scene_objects": scene_object_summary,
                "reset": {
                    "last_seq": int(last_reset_seq),
                    "holding_start_pose": bool(left.reset_hold_cycles_remaining > 0 or right.reset_hold_cycles_remaining > 0),
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
            print(f"[FlexivDualTargetFrame] Stage2 gateway send failed: {exc}", flush=True)
            try:
                gateway_client.close()
            except Exception:
                pass
            gateway_client = None

    world.add_physics_callback("dual_target_frame_step", callback_fn=on_physics_step)
    world.reset()
    for camera in stage2_cameras:
        camera.initialize()
    initialize_like_startup("startup", reset_world=False)
    if not args.manual_play:
        omni.timeline.get_timeline_interface().play()

    print(
        f"[FlexivDualTargetFrame] Ready. control_source=studio-bridge. physics_hz={physics_hz:g}. "
        f"left={args.left_serial_number} right={args.right_serial_number}",
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
            publish_gateway_sample()
            if pending_reset_control is not None:
                control = pending_reset_control
                pending_reset_control = None
                last_reset_seq = int(control.get("seq", 0))
                initialize_like_startup(
                    f"gateway:{control.get('reason', 'unspecified')} seq={last_reset_seq}",
                    reset_world=True,
                )
                reset_needed = False
            if world.is_stopped() and not reset_needed:
                reset_needed = True
            if world.is_playing() and reset_needed:
                initialize_like_startup("timeline-resume", reset_world=True)
                reset_needed = False
            if args.smoke_test and args.max_frames > 0 and frame_count >= args.max_frames:
                break
            rate_limiter.sleep()
    finally:
        try:
            world.remove_physics_callback("dual_target_frame_step")
        except Exception:
            pass
        for arm in arms.values():
            arm.target_pose_publisher.close()
        if quest_target_receiver is not None:
            quest_target_receiver.close()
        if gateway_client is not None:
            gateway_client.close()
        if args.smoke_test:
            print("FLEXIV_DUAL_TARGET_FRAME_SMOKE_TEST_OK", flush=True)
        simulation_app.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
