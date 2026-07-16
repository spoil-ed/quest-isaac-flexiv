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

from control_helpers import (  # noqa: E402
    TargetPosePublishGate,
    cartesian_pose_changed,
    rdk_streamer_status_is_ready,
)
from elements_studio_utils import hold_robot_joint_positions  # noqa: E402
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
    _select_target,
    camera_pose_from_config,
    create_xyz_target_frame,
)
from targeting import (  # noqa: E402
    CartesianTargetLimiter,
    QuestRelativeTargetMapper,
    QuestTargetPacket,
    RdkWorldFrameCalibration,
    TargetPose,
    TargetPoseUdpPublisher,
    build_target_pose_packet,
    parse_float_list,
    parse_quest_axis_map,
    parse_quest_gripper_packet,
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
DEFAULT_LEFT_RDK_STATUS_UDP_PORT = 57682
DEFAULT_RIGHT_RDK_STATUS_UDP_PORT = 57683
DEFAULT_QUEST_TARGET_UDP_PORT = 57679
DEFAULT_STATE_MONITOR_UDP_PORT = 57684


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
    parser.add_argument("--gpu-dynamics", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--physics-hz", type=float, default=PHYSICS_FREQ)
    parser.add_argument("--render-hz", type=float, default=RENDER_FREQ)
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    parser.add_argument("--quest-target-udp-host", default=DEFAULT_QUEST_TARGET_UDP_HOST)
    parser.add_argument("--quest-target-udp-port", type=int, default=DEFAULT_QUEST_TARGET_UDP_PORT)
    parser.add_argument("--quest-target-max-age-sec", type=float, default=0.5)
    parser.add_argument("--quest-target-mode", choices=("absolute", "relative"), default="relative")
    parser.add_argument(
        "--quest-relative-orientation-mode",
        choices=("packet", "relative", "reference", "current"),
        default="relative",
    )
    parser.add_argument("--quest-axis-map", default=DEFAULT_QUEST_AXIS_MAP)
    parser.add_argument("--quest-position-scale", type=float, default=0.5)
    parser.add_argument("--quest-position-deadband-m", type=float, default=0.01)
    parser.add_argument("--quest-workspace-min", default=",".join(str(value) for value in DEFAULT_QUEST_WORKSPACE_MIN))
    parser.add_argument("--quest-workspace-max", default=",".join(str(value) for value in DEFAULT_QUEST_WORKSPACE_MAX))
    parser.add_argument(
        "--quest-workspace-clipping",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Clamp Quest TCP goals to --quest-workspace-min/max (disabled for wall-mounted dual arms).",
    )
    parser.add_argument("--max-linear-speed-m-s", type=float, default=0.10)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=0.75)
    parser.add_argument("--left-target-pose-udp-host", default="127.0.0.1")
    parser.add_argument("--left-target-pose-udp-port", type=int, default=DEFAULT_LEFT_TARGET_POSE_UDP_PORT)
    parser.add_argument("--right-target-pose-udp-host", default="127.0.0.1")
    parser.add_argument("--right-target-pose-udp-port", type=int, default=DEFAULT_RIGHT_TARGET_POSE_UDP_PORT)
    parser.add_argument("--left-rdk-status-udp-host", default="127.0.0.1")
    parser.add_argument("--left-rdk-status-udp-port", type=int, default=DEFAULT_LEFT_RDK_STATUS_UDP_PORT)
    parser.add_argument("--right-rdk-status-udp-host", default="127.0.0.1")
    parser.add_argument("--right-rdk-status-udp-port", type=int, default=DEFAULT_RIGHT_RDK_STATUS_UDP_PORT)
    parser.add_argument("--rdk-status-max-age-sec", type=float, default=1.0)
    parser.add_argument("--target-pose-publish-hz", type=float, default=30.0)
    parser.add_argument("--target-activation-position-tolerance-m", type=float, default=1e-3)
    parser.add_argument("--target-activation-orientation-tolerance-rad", type=float, default=8.726646e-3)
    parser.add_argument("--command-timeout-ms", type=int, default=1)
    parser.add_argument("--target-axis-length", type=float, default=0.14)
    parser.add_argument("--target-axis-radius", type=float, default=0.006)
    parser.add_argument("--gateway-endpoint", default="")
    parser.add_argument("--gateway-fps", type=float, default=DEFAULT_STAGE1_GATEWAY_FPS)
    parser.add_argument("--gateway-jpeg-quality", type=int, default=DEFAULT_STAGE1_GATEWAY_JPEG_QUALITY)
    parser.add_argument("--state-monitor-udp-host", default="127.0.0.1")
    parser.add_argument("--state-monitor-udp-port", type=int, default=DEFAULT_STATE_MONITOR_UDP_PORT)
    parser.add_argument("--state-monitor-hz", type=float, default=10.0)
    parser.add_argument(
        "--capture-initial-frame",
        type=Path,
        default=None,
        help="Write one rendered camera frame after startup initialization and exit.",
    )
    parser.add_argument("--capture-camera-name", default="cam_front")
    parser.add_argument("--capture-after-frames", type=int, default=45)
    parser.add_argument("--coordinated-reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=90.0)
    parser.add_argument("--startup-joint-tolerance-rad", type=float, default=0.03)
    args = parser.parse_args(argv)
    args.scene_data = {}
    apply_scene_config(args)
    args.left_serial_number = args.left_serial_number or DEFAULT_LEFT_SERIAL_NUMBER
    args.right_serial_number = args.right_serial_number or DEFAULT_RIGHT_SERIAL_NUMBER
    args.joint_group = args.joint_group or DEFAULT_JOINT_GROUP
    if args.capture_initial_frame is not None:
        args.headless = True
        if args.max_frames <= 0:
            args.max_frames = max(int(args.capture_after_frames) + 120, 180)
    if args.smoke_test:
        args.headless = True
    if float(args.reset_timeout_sec) <= 0.0:
        parser.error("--reset-timeout-sec must be positive")
    if not 0 <= int(args.state_monitor_udp_port) <= 65535:
        parser.error("--state-monitor-udp-port must be between 0 and 65535; use 0 to disable")
    if float(args.state_monitor_hz) <= 0.0:
        parser.error("--state-monitor-hz must be positive")
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
        self._pending_grippers: dict[str, bool] = {}
        self._latest_inputs: dict[str, dict[str, Any] | None] = {
            side: None for side in self._serials
        }

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
            if packet.get("schema") == "rizon4_quest_input.v1":
                if (
                    str(packet.get("serial", "")) != self._serials[side]
                    or str(packet.get("joint_group", "")) != self._joint_group
                ):
                    continue
                try:
                    packet["seq"] = int(packet.get("seq", -1))
                    packet["monotonic_time"] = float(packet["monotonic_time"])
                    packet["enable_value"] = float(packet.get("enable_value", 0.0))
                    packet["gripper_value"] = float(packet.get("gripper_value", 0.0))
                    pose = packet.get("controller_pose_openxr")
                    if pose is not None:
                        pose = [float(value) for value in pose]
                        if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
                            continue
                        packet["controller_pose_openxr"] = pose
                except (KeyError, TypeError, ValueError):
                    continue
                self._latest_inputs[side] = packet
                continue
            gripper = parse_quest_gripper_packet(
                packet,
                serial_number=self._serials[side],
                joint_group=self._joint_group,
                max_age_sec=self._max_age_sec,
            )
            if gripper is not None:
                self._pending_grippers[side] = bool(gripper.closed)
                continue
            parsed = parse_quest_target_packet(
                packet,
                serial_number=self._serials[side],
                joint_group=self._joint_group,
                max_age_sec=self._max_age_sec,
            )
            if parsed is not None:
                latest[side] = parsed

    def take_latest_grippers(self) -> dict[str, bool]:
        latest = dict(self._pending_grippers)
        self._pending_grippers.clear()
        return latest

    def latest_input(self, side: str) -> dict[str, Any] | None:
        packet = self._latest_inputs.get(str(side))
        if packet is None:
            return None
        age_sec = time.monotonic() - float(packet["monotonic_time"])
        if age_sec < -1.0 or age_sec > self._max_age_sec:
            return None
        return packet

    def clear(self) -> None:
        self.poll_latest()
        self.take_latest_grippers()
        for side in self._latest_inputs:
            self._latest_inputs[side] = None

    def close(self) -> None:
        self._socket.close()


class DualRdkStatusReceiver:
    """Receive readiness/fault state from the two external RDK streamers."""

    def __init__(
        self,
        addresses: dict[str, tuple[str, int]],
        *,
        serials: dict[str, str],
        max_age_sec: float,
    ) -> None:
        self._serials = dict(serials)
        self._max_age_sec = float(max_age_sec)
        self._sockets: dict[str, socket.socket] = {}
        self._latest: dict[str, dict[str, Any] | None] = {side: None for side in addresses}
        for side, address in addresses.items():
            status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            status_socket.bind((str(address[0]), int(address[1])))
            status_socket.setblocking(False)
            self._sockets[side] = status_socket

    def poll(self) -> dict[str, bool]:
        for side, status_socket in self._sockets.items():
            while True:
                try:
                    data, _addr = status_socket.recvfrom(65536)
                except BlockingIOError:
                    break
                except OSError:
                    break
                try:
                    packet = json.loads(data.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue
                if isinstance(packet, dict) and str(packet.get("serial", "")) == self._serials[side]:
                    self._latest[side] = packet
        now = time.monotonic()
        return {
            side: rdk_streamer_status_is_ready(
                self._latest[side],
                serial_number=self._serials[side],
                max_age_sec=self._max_age_sec,
                now=now,
            )
            for side in self._sockets
        }

    def clear(self) -> None:
        for side in self._latest:
            self._latest[side] = None
        self.poll()

    def latest_packet(self, side: str) -> dict[str, Any] | None:
        packet = self._latest.get(str(side))
        return packet if isinstance(packet, dict) else None

    def close(self) -> None:
        for status_socket in self._sockets.values():
            status_socket.close()


def _finite_status_pose(packet: dict[str, Any] | None, key: str) -> list[float] | None:
    if packet is None:
        return None
    try:
        pose = [float(value) for value in packet[key]]
    except (KeyError, TypeError, ValueError):
        return None
    return pose if len(pose) == 7 and all(math.isfinite(value) for value in pose) else None


@dataclass
class ArmRuntime:
    side: str
    serial_number: str
    joint_group: str
    robot: Any
    target_frame: Any
    target_prim_path: str
    sim_node: Any
    target_pose_publisher: TargetPoseUdpPublisher
    mapper: QuestRelativeTargetMapper
    limiter: CartesianTargetLimiter
    target_pose_gate: TargetPosePublishGate
    configured_initial_pose: TargetPose
    initial_q: list[float]
    bootstrap_q: list[float]
    base_position: tuple[float, float, float]
    base_orientation: tuple[float, float, float, float]
    latest_quest_target: QuestTargetPacket | None = None
    latest_target_drives: list[float] | None = None
    last_connected: bool = False
    effort_control_enabled: bool = False
    articulation_ready: bool = False
    reset_hold_pose_base_tcp: list[float] | None = None
    reset_hold_cycles_remaining: int = 0
    latest_control_pose_base_tcp: list[float] | None = None
    idle_hold_pose_base_tcp: list[float] | None = None
    idle_target_world_pose: list[float] | None = None
    target_control_requested: bool = False
    target_control_source: str | None = None
    rdk_ready: bool = False
    rdk_reference_pose_base_tcp: list[float] | None = None
    rdk_current_pose_base_tcp: list[float] | None = None
    rdk_current_q: list[float] | None = None
    rdk_phase: str = "disconnected"
    rdk_reset_seq: int = 0
    rdk_world_calibration: RdkWorldFrameCalibration | None = None
    rdk_reference_world_pose: list[float] | None = None
    startup_trajectory_complete: bool = False
    quest_goal_pose_base_tcp: list[float] | None = None
    pending_gripper_closed: bool | None = None
    applied_gripper_closed: bool | None = None


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


def _bootstrap_q_config(robot_cfg: dict[str, Any], *, initial_q: list[float]) -> list[float]:
    """Return the safe SimPlugin/Studio handshake posture.

    Existing scenes keep their former behavior when ``bootstrap_q`` is absent.
    A task with a non-home ``initial_q`` can explicitly bootstrap from Studio's
    safe home and reach the task pose later through Cartesian target control.
    """

    raw = robot_cfg.get("bootstrap_q")
    if raw is None:
        return list(initial_q)
    values = [float(value) for value in raw]
    if len(values) != 7:
        raise ValueError("robot bootstrap_q must contain 7 values")
    return values


def _max_wrapped_joint_error(current_q, target_q) -> float:
    current = [float(value) for value in current_q]
    target = [float(value) for value in target_q]
    if len(current) != len(target):
        raise ValueError("current_q and target_q must have equal length")
    if not current:
        return 0.0
    return max(
        abs(math.atan2(math.sin(actual - desired), math.cos(actual - desired)))
        for actual, desired in zip(current, target)
    )


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
            # Isaac Sim defaults to multi-GPU rendering. On workstations that
            # expose both an NVIDIA GPU and an integrated Intel GPU, Vulkan can
            # fail while initializing the unsupported secondary adapter. Keep
            # rendering and PhysX on the primary CUDA device.
            "active_gpu": 0,
            "physics_gpu": 0,
            "multi_gpu": False,
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
    from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper

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
    if args.gpu_dynamics:
        physics_context = world.get_physics_context()
        physics_context.enable_gpu_dynamics(True)
        physics_context.set_broadphase_type("GPU")
        if not physics_context.is_gpu_dynamics_enabled():
            raise RuntimeError("PhysX refused to enable GPU dynamics")
        print(
            "[FlexivDualTargetFrame] PhysX GPU dynamics enabled on CUDA device 0 "
            "with GPU broadphase; CPU readback remains enabled for SimPlugin",
            flush=True,
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
    if args.gateway_endpoint or args.capture_initial_frame is not None:
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
        mode = "gateway/capture" if args.gateway_endpoint else "capture"
        print(f"[FlexivDualTargetFrame] Stage2 {mode} cameras enabled: {len(stage2_cameras)}", flush=True)

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
        gripper = ParallelGripper(
            end_effector_prim_path=prim_path + "/Grav_gripper/right_finger_tip",
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([45.0, 0.0]),
            joint_closed_positions=np.array([-8.88, 0.0]),
        )
        robot = world.scene.add(
            FlexivSerial(
                prim_path=prim_path,
                name=robot_name,
                end_effector_prim_name=end_effector,
                gripper=gripper,
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
        initial_q = _initial_q_config(robot_cfg)
        arms[side] = ArmRuntime(
            side=side,
            serial_number=serials[side],
            joint_group=args.joint_group,
            robot=robot,
            target_frame=target_frame,
            target_prim_path=target_prim,
            sim_node=flexivsimplugin.UserNode(serials[side]),
            target_pose_publisher=target_pose_publisher,
            mapper=QuestRelativeTargetMapper(
                axis_map=parse_quest_axis_map(args.quest_axis_map),
                scale=float(args.quest_position_scale),
                workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
                workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
                position_deadband_m=float(args.quest_position_deadband_m),
                orientation_mode=str(args.quest_relative_orientation_mode),
                clamp_workspace=bool(args.quest_workspace_clipping),
            ),
            limiter=CartesianTargetLimiter(
                workspace_min=parse_float_list(args.quest_workspace_min, expected=3, name="quest_workspace_min"),
                workspace_max=parse_float_list(args.quest_workspace_max, expected=3, name="quest_workspace_max"),
                max_linear_speed_m_s=float(args.max_linear_speed_m_s),
                max_angular_speed_rad_s=float(args.max_angular_speed_rad_s),
                clamp_workspace=bool(args.quest_workspace_clipping),
            ),
            target_pose_gate=TargetPosePublishGate.from_hz(float(args.target_pose_publish_hz), physics_freq=physics_hz),
            configured_initial_pose=initial_pose,
            initial_q=initial_q,
            bootstrap_q=_bootstrap_q_config(robot_cfg, initial_q=initial_q),
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
    rdk_status_receiver = DualRdkStatusReceiver(
        {
            "left": (args.left_rdk_status_udp_host, args.left_rdk_status_udp_port),
            "right": (args.right_rdk_status_udp_host, args.right_rdk_status_udp_port),
        },
        serials=serials,
        max_age_sec=float(args.rdk_status_max_age_sec),
    )
    print(
        "[FlexivDualTargetFrame] RDK status UDP listening on "
        f"left={args.left_rdk_status_udp_host}:{args.left_rdk_status_udp_port} "
        f"right={args.right_rdk_status_udp_host}:{args.right_rdk_status_udp_port}",
        flush=True,
    )

    servo_cycle = 0
    gateway_client = None
    gateway_last_connect_attempt = 0.0
    gateway_last_publish = 0.0
    state_monitor_publisher = (
        TargetPoseUdpPublisher(args.state_monitor_udp_host, args.state_monitor_udp_port)
        if int(args.state_monitor_udp_port) > 0
        else None
    )
    state_monitor_last_publish = 0.0
    pending_reset_control = None
    last_reset_seq = 0
    reset_state = "idle"
    reset_error = None
    reset_started_time = 0.0
    reset_assets_restore_time = 0.0
    reset_reason = None
    reset_scene_collision_states: dict[str, bool] = {}
    reset_scene_kinematic_states: dict[str, bool] = {}
    control_loop_enabled = False
    dual_task_ready_announced = False

    def _current_pose_base_tcp(arm: ArmRuntime) -> list[float]:
        base_position, base_orientation = arm.robot.get_world_pose()
        tcp_position, tcp_orientation = arm.robot.end_effector.get_world_pose()
        return world_target_to_flexiv_pose(
            world_position=tcp_position,
            world_orientation_wxyz=tcp_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )

    def _hold_arm_position(arm: ArmRuntime, joint_positions=None, *, switch_mode: bool = True) -> int:
        return hold_robot_joint_positions(
            arm.robot,
            joint_positions,
            switch_mode=switch_mode,
            zeros_factory=lambda count: np.zeros(count),
        )

    def _reset_arm_to_start_pose(arm: ArmRuntime, *, reset_world: bool) -> None:
        if reset_world:
            world.reset()
        _set_robot_base_pose(arm.robot, arm.base_position, arm.base_orientation)
        # SimPlugin and both runtimes must first agree on Studio's safe home.
        # DRDK moves from this posture to task initial_q using the runtime's
        # official NRT joint-position trajectory generator.
        _hold_arm_position(arm, arm.bootstrap_q)
        if arm.robot.gripper is not None:
            arm.robot.gripper.open()
            arm.pending_gripper_closed = False
            arm.applied_gripper_closed = False
        print(
            f"[FlexivDualTargetFrame] {arm.side} bootstrap joint hold applied "
            f"q={[round(float(value), 6) for value in arm.robot.q]}",
            flush=True,
        )
        arm.reset_hold_pose_base_tcp = _current_pose_base_tcp(arm)
        arm.idle_hold_pose_base_tcp = list(arm.reset_hold_pose_base_tcp)
        synced_target_pose = sync_target_to_base_tcp_pose(
            arm.target_frame,
            pose_base_tcp_des=arm.reset_hold_pose_base_tcp,
            base_position=arm.base_position,
            base_orientation_wxyz=arm.base_orientation,
        )
        print(
            f"[FlexivDualTargetFrame] {arm.side} TargetFrame aligned to end effector "
            f"world_position={[round(float(value), 6) for value in synced_target_pose.position]} "
            f"world_euler_deg={[round(float(value), 3) for value in synced_target_pose.euler_deg]}",
            flush=True,
        )
        target_position, target_orientation = arm.target_frame.get_world_pose()
        arm.idle_target_world_pose = [
            *[float(value) for value in target_position],
            *[float(value) for value in target_orientation],
        ]
        arm.reset_hold_cycles_remaining = max(1, int(max(0.0, float(args.reset_settle_sec)) * physics_hz))
        arm.latest_quest_target = None
        arm.latest_target_drives = [0.0] * 7
        arm.mapper.reset()
        arm.limiter.reset(arm.reset_hold_pose_base_tcp)
        arm.last_connected = False
        arm.effort_control_enabled = False
        arm.articulation_ready = True
        arm.target_control_requested = False
        arm.target_control_source = None
        arm.rdk_ready = False
        arm.rdk_reference_pose_base_tcp = None
        arm.rdk_current_pose_base_tcp = None
        arm.rdk_current_q = None
        arm.rdk_phase = "disconnected"
        arm.rdk_reset_seq = 0
        arm.rdk_world_calibration = None
        arm.rdk_reference_world_pose = None
        arm.startup_trajectory_complete = False
        arm.quest_goal_pose_base_tcp = None

    def initialize_like_startup(reason: str, *, reset_world: bool) -> None:
        nonlocal control_loop_enabled, dual_task_ready_announced
        # world.reset() can invoke physics callbacks before articulation views
        # and configured joint positions are ready. Never expose those
        # transient states to an already-operational Studio controller.
        control_loop_enabled = False
        for idx, side in enumerate(("left", "right")):
            _reset_arm_to_start_pose(arms[side], reset_world=reset_world and idx == 0)
        if quest_target_receiver is not None:
            quest_target_receiver.clear()
        rdk_status_receiver.clear()
        dual_task_ready_announced = False
        control_loop_enabled = True
        print(
            f"[FlexivDualTargetFrame] startup initialization applied reason={reason}; "
            f"Studio home hold={max(0.0, float(args.reset_settle_sec)):.3f}s",
            flush=True,
        )

    def set_reset_scene_collisions_suppressed(suppressed: bool) -> None:
        """Temporarily remove task-object contacts so joint recovery can leave a collision."""

        from pxr import UsdPhysics

        if suppressed:
            if reset_scene_collision_states:
                return
            for item in scene_object_summary:
                has_collision = bool(item.get("collision"))
                is_rigid = bool(item.get("rigid_body"))
                if not has_collision and not is_rigid:
                    continue
                path = str(item.get("prim_path") or "")
                prim = world.stage.GetPrimAtPath(path)
                if not prim.IsValid():
                    raise RuntimeError(f"missing scene physics prim during reset: {path}")
                if is_rigid:
                    rigid_api = UsdPhysics.RigidBodyAPI(prim)
                    kinematic_attr = rigid_api.GetKinematicEnabledAttr()
                    if not kinematic_attr or not kinematic_attr.IsValid():
                        kinematic_attr = rigid_api.CreateKinematicEnabledAttr(False)
                    previous_kinematic = kinematic_attr.Get()
                    reset_scene_kinematic_states[path] = (
                        False if previous_kinematic is None else bool(previous_kinematic)
                    )
                    kinematic_attr.Set(True)
                if has_collision:
                    collision_api = UsdPhysics.CollisionAPI(prim)
                    collision_attr = collision_api.GetCollisionEnabledAttr()
                    if not collision_attr or not collision_attr.IsValid():
                        collision_attr = collision_api.CreateCollisionEnabledAttr(True)
                    previous_collision = collision_attr.Get()
                    reset_scene_collision_states[path] = (
                        True if previous_collision is None else bool(previous_collision)
                    )
                    collision_attr.Set(False)
            if not reset_scene_collision_states and not reset_scene_kinematic_states:
                raise RuntimeError("no physical scene objects found for coordinated reset")
            print(
                "[FlexivDualTargetFrame] coordinated reset temporarily disabled collisions for "
                f"{len(reset_scene_collision_states)} scene objects and froze "
                f"{len(reset_scene_kinematic_states)} rigid bodies",
                flush=True,
            )
            return

        for path, enabled in reset_scene_collision_states.items():
            prim = world.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"scene collision prim disappeared during reset: {path}")
            collision_api = UsdPhysics.CollisionAPI(prim)
            attr = collision_api.GetCollisionEnabledAttr()
            if not attr or not attr.IsValid():
                attr = collision_api.CreateCollisionEnabledAttr(bool(enabled))
            attr.Set(bool(enabled))
        for path, enabled in reset_scene_kinematic_states.items():
            prim = world.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"scene rigid-body prim disappeared during reset: {path}")
            rigid_api = UsdPhysics.RigidBodyAPI(prim)
            attr = rigid_api.GetKinematicEnabledAttr()
            if not attr or not attr.IsValid():
                attr = rigid_api.CreateKinematicEnabledAttr(bool(enabled))
            attr.Set(bool(enabled))
        restored = len(reset_scene_collision_states)
        reset_scene_collision_states.clear()
        reset_scene_kinematic_states.clear()
        print(
            f"[FlexivDualTargetFrame] coordinated reset restored collisions for {restored} scene objects",
            flush=True,
        )

    def reset_configured_scene_assets() -> None:
        """Return all configured task assets to their scene-config initial state."""

        from flexiv_sim_scenes.isaac import reset_scene_objects

        results = reset_scene_objects(
            world,
            args.scene_data,
            config_path=args.scene_config,
        )
        rigid_count = sum(bool(item.get("rigid_body")) for item in results)
        print(
            "[FlexivDualTargetFrame] coordinated reset restored configured state for "
            f"{len(results)} scene assets ({rigid_count} rigid bodies)",
            flush=True,
        )

    def begin_coordinated_reset(control: dict[str, Any]) -> None:
        """Disarm user targets and ask DRDK to recover both robots to init_q."""

        nonlocal last_reset_seq, reset_state, reset_error, reset_started_time, reset_reason
        nonlocal reset_assets_restore_time
        nonlocal dual_task_ready_announced
        last_reset_seq = int(control.get("seq", 0))
        reset_state = "moving"
        reset_error = None
        reset_started_time = time.monotonic()
        reset_assets_restore_time = 0.0
        reset_reason = str(control.get("reason", "unspecified"))
        dual_task_ready_announced = False
        try:
            set_reset_scene_collisions_suppressed(True)
        except Exception as exc:
            reset_state = "failed"
            reset_error = f"failed to suppress scene collisions for reset: {exc}"
            print(f"[FlexivDualTargetFrame] {reset_error}", flush=True)
            return
        if quest_target_receiver is not None:
            quest_target_receiver.clear()
        for arm in arms.values():
            arm.latest_quest_target = None
            arm.target_control_requested = False
            arm.target_control_source = None
            arm.quest_goal_pose_base_tcp = None
            arm.latest_control_pose_base_tcp = None
            arm.mapper.reset()
            arm.limiter.reset()
            arm.startup_trajectory_complete = False
            arm.rdk_reference_pose_base_tcp = None
            arm.rdk_reference_world_pose = None
            arm.rdk_world_calibration = None
            arm.idle_target_world_pose = None
            arm.reset_hold_pose_base_tcp = None
            arm.reset_hold_cycles_remaining = 0
            if arm.robot.gripper is not None:
                arm.robot.gripper.open()
                arm.pending_gripper_closed = False
                arm.applied_gripper_closed = False
        print(
            f"[FlexivDualTargetFrame] coordinated reset requested seq={last_reset_seq} "
            f"reason={reset_reason}; waiting for DRDK Stop/ClearFault/SendJointPosition(init_q)",
            flush=True,
        )

    def _update_quest_targets() -> None:
        if quest_target_receiver is None:
            return
        latest = quest_target_receiver.poll_latest()
        for side, target in latest.items():
            arms[side].latest_quest_target = target
        for side, closed in quest_target_receiver.take_latest_grippers().items():
            arms[side].pending_gripper_closed = bool(closed)
        for arm in arms.values():
            if not quest_target_is_fresh(arm.latest_quest_target, max_age_sec=float(args.quest_target_max_age_sec)):
                arm.latest_quest_target = None
                arm.mapper.reset()

    def _apply_quest_gripper(arm: ArmRuntime) -> None:
        if reset_state not in {"idle", "succeeded"}:
            return
        closed = arm.pending_gripper_closed
        if closed is None or closed == arm.applied_gripper_closed:
            return
        if arm.robot.gripper is None:
            return
        if closed:
            arm.robot.gripper.close()
        else:
            arm.robot.gripper.open()
        arm.applied_gripper_closed = closed
        print(
            f"[FlexivDualTargetFrame] {arm.side} Quest gripper "
            f"{'closed' if closed else 'opened'} (direct Isaac control)",
            flush=True,
        )

    def _update_arm_target(arm: ArmRuntime, target_dt: float) -> None:
        """Sample and publish one target pose at the low-rate target clock."""

        base_position, base_orientation = arm.robot.get_world_pose()
        target_position, target_orientation = arm.target_frame.get_world_pose()
        pose_base_tcp_des = select_pose_base_tcp_des(
            quest_target=arm.latest_quest_target,
            world_position=target_position,
            world_orientation_wxyz=target_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )
        if (
            arm.latest_quest_target is None
            and arm.rdk_world_calibration is not None
        ):
            pose_base_tcp_des = arm.rdk_world_calibration.world_pose_to_rdk(
                world_position=target_position,
                world_orientation_wxyz=target_orientation,
            )
        control_pose_base_tcp = pose_base_tcp_des
        if (
            arm.latest_quest_target is None
            and arm.target_control_source == "quest"
            and arm.quest_goal_pose_base_tcp is not None
        ):
            # Releasing squeeze stops Quest packets. Keep the last raw Quest
            # goal instead of falling back to the startup TargetFrame pose.
            # The limiter below may continue moving toward this fixed goal.
            control_pose_base_tcp = list(arm.quest_goal_pose_base_tcp)
        current_pose_base_tcp = None
        reset_hold_active = arm.reset_hold_pose_base_tcp is not None and arm.reset_hold_cycles_remaining > 0
        if reset_hold_active:
            control_pose_base_tcp = list(arm.reset_hold_pose_base_tcp)
            # Keep the visible target exactly on the initialized flange while
            # the two Studio controllers finish starting.
            sync_target_to_base_tcp_pose(
                arm.target_frame,
                pose_base_tcp_des=control_pose_base_tcp,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            )
            arm.mapper.reset()
            arm.limiter.reset(control_pose_base_tcp)
        elif arm.latest_quest_target is not None:
            if arm.startup_trajectory_complete:
                # Quest deltas are commands in Flexiv's RDK base frame. The
                # wall-mounted Isaac articulation has a different base-axis
                # convention, so use the TCP measured by the RDK streamer as
                # the position anchor instead of deriving it from the USD root.
                rdk_anchor_pose = arm.rdk_current_pose_base_tcp or arm.rdk_reference_pose_base_tcp
                if rdk_anchor_pose is not None:
                    current_pose_base_tcp = list(rdk_anchor_pose)
                    if args.quest_target_mode == "relative":
                        control_pose_base_tcp = arm.mapper.update(
                            arm.latest_quest_target,
                            current_pose_base_tcp,
                        )
                    arm.quest_goal_pose_base_tcp = [float(value) for value in control_pose_base_tcp]
                else:
                    arm.mapper.reset()

        if not reset_hold_active and not arm.target_control_requested:
            activation_source = None
            if arm.latest_quest_target is not None and arm.startup_trajectory_complete:
                activation_source = "quest"
            current_target_world_pose = [
                *[float(value) for value in target_position],
                *[float(value) for value in target_orientation],
            ]
            if (
                activation_source is None
                and arm.startup_trajectory_complete
                and not args.enable_quest_target_udp
                and arm.idle_target_world_pose is not None
                and cartesian_pose_changed(
                    arm.idle_target_world_pose,
                    current_target_world_pose,
                    position_tolerance_m=float(args.target_activation_position_tolerance_m),
                    orientation_tolerance_rad=float(args.target_activation_orientation_tolerance_rad),
                )
            ):
                activation_source = "target-frame"
            if activation_source is not None:
                arm.target_control_requested = True
                arm.target_control_source = activation_source
                print(
                    f"[FlexivDualTargetFrame] {arm.side} target control armed by {activation_source}; "
                    "startup hold released",
                    flush=True,
                )

        quest_goal_active = bool(
            arm.target_control_source == "quest"
            and arm.quest_goal_pose_base_tcp is not None
        )
        if quest_goal_active and not reset_hold_active:
            if arm.limiter.last_pose is None:
                limiter_seed = (
                    current_pose_base_tcp
                    or arm.latest_control_pose_base_tcp
                    or arm.rdk_current_pose_base_tcp
                    or arm.rdk_reference_pose_base_tcp
                )
                if limiter_seed is not None:
                    arm.limiter.reset(limiter_seed)
            control_pose_base_tcp = arm.limiter.limit(
                arm.quest_goal_pose_base_tcp,
                dt=float(target_dt),
            )
        elif not reset_hold_active:
            arm.limiter.reset()

        arm.latest_control_pose_base_tcp = [float(value) for value in control_pose_base_tcp]

        if not reset_hold_active:
            user_target_active = bool(
                arm.target_control_requested
                and arm.startup_trajectory_complete
                and arm.rdk_ready
                and arm.effort_control_enabled
                and arm.rdk_reference_pose_base_tcp is not None
            )
            publish_pose_base_tcp = control_pose_base_tcp
            if not user_target_active:
                publish_pose_base_tcp = (
                    list(arm.rdk_reference_pose_base_tcp)
                    if arm.rdk_reference_pose_base_tcp is not None
                    else _current_pose_base_tcp(arm)
                )
            packet = build_target_pose_packet(
                serial_number=arm.serial_number,
                joint_group=arm.joint_group,
                servo_cycle=servo_cycle,
                pose_base_tcp_des=publish_pose_base_tcp,
                monotonic_time=time.monotonic(),
            )
            # The streamer ignores the uncalibrated packet pose until it has
            # latched Studio's actual TCP and Isaac has entered effort mode.
            packet["control_active"] = user_target_active
            if reset_state == "moving" and last_reset_seq > 0:
                packet["reset_seq"] = int(last_reset_seq)
                packet["reset_reason"] = str(reset_reason or "coordinated")
            arm.target_pose_publisher.publish(packet)

        if reset_hold_active:
            arm.reset_hold_cycles_remaining = max(
                0,
                arm.reset_hold_cycles_remaining - arm.target_pose_gate.period_cycles,
            )
            if arm.reset_hold_cycles_remaining <= 0:
                arm.reset_hold_pose_base_tcp = None
                idle_position, idle_orientation = arm.target_frame.get_world_pose()
                arm.idle_target_world_pose = [
                    *[float(value) for value in idle_position],
                    *[float(value) for value in idle_orientation],
                ]
                print(
                    f"[FlexivDualTargetFrame] {arm.side} startup/reset hold completed; "
                    "starting RDK current-TCP coordinate handoff",
                    flush=True,
                )

    def _update_quest_target_frames() -> None:
        """Render the two limited Quest targets without touching the 2 kHz loop."""

        if not args.enable_quest_target_udp:
            return
        for arm in arms.values():
            pose_base_tcp = arm.quest_goal_pose_base_tcp
            calibration = arm.rdk_world_calibration
            if pose_base_tcp is None or calibration is None:
                continue
            world_position, world_orientation = calibration.rdk_pose_to_world(pose_base_tcp)
            arm.target_frame.set_world_pose(
                position=np.array(world_position),
                orientation=np.array(world_orientation),
            )

    def _update_rdk_statuses() -> None:
        nonlocal dual_task_ready_announced, reset_state, reset_error, reset_assets_restore_time
        readiness = rdk_status_receiver.poll()
        for side, ready in readiness.items():
            arm = arms[side]
            packet = rdk_status_receiver.latest_packet(side)
            if packet is not None:
                phase = str(packet.get("phase") or "ready")
                try:
                    arm.rdk_reset_seq = max(arm.rdk_reset_seq, int(packet.get("reset_seq", 0)))
                except (TypeError, ValueError):
                    pass
                reference_pose = _finite_status_pose(packet, "reference_pose_base_tcp")
                current_pose = _finite_status_pose(packet, "current_pose_base_tcp")
                current_q = _finite_status_pose(packet, "current_q")
                if phase != arm.rdk_phase:
                    print(
                        f"[FlexivDualTargetFrame] RDK phase {side} {arm.rdk_phase} -> {phase}",
                        flush=True,
                    )
                arm.rdk_phase = phase
                if current_q is not None:
                    arm.rdk_current_q = current_q
                if current_pose is not None:
                    arm.rdk_current_pose_base_tcp = current_pose
                if phase == "reset_failed" and arm.rdk_reset_seq >= last_reset_seq and reset_state == "moving":
                    reset_state = "failed"
                    reset_error = str(packet.get("error") or "DRDK coordinated reset failed")
                    print(
                        f"[FlexivDualTargetFrame] coordinated reset failed seq={last_reset_seq}: {reset_error}",
                        flush=True,
                    )
                if ready and phase == "ready" and reference_pose is not None:
                    if arm.rdk_reference_pose_base_tcp is None:
                        print(
                            f"[FlexivDualTargetFrame] calibrated {side} RDK TCP reference "
                            f"{[round(value, 6) for value in reference_pose]}",
                            flush=True,
                        )
                    arm.rdk_reference_pose_base_tcp = reference_pose
                    if not arm.startup_trajectory_complete:
                        tcp_position, tcp_orientation = arm.robot.end_effector.get_world_pose()
                        arm.rdk_reference_world_pose = [
                            *[float(value) for value in tcp_position],
                            *[float(value) for value in tcp_orientation],
                        ]
                        arm.target_frame.set_world_pose(
                            position=np.array(tcp_position),
                            orientation=np.array(tcp_orientation),
                        )
                        arm.idle_target_world_pose = list(arm.rdk_reference_world_pose)
                    if arm.rdk_reference_world_pose is not None:
                        arm.rdk_world_calibration = RdkWorldFrameCalibration.from_reference_pair(
                            reference_world_pose=arm.rdk_reference_world_pose,
                            reference_pose_base_tcp=reference_pose,
                        )
                    if not arm.startup_trajectory_complete:
                        isaac_joint_error = _max_wrapped_joint_error(arm.robot.q, arm.initial_q)
                        rdk_joint_error = (
                            math.inf
                            if arm.rdk_current_q is None
                            else _max_wrapped_joint_error(arm.rdk_current_q, arm.initial_q)
                        )
                        if max(isaac_joint_error, rdk_joint_error) <= float(args.startup_joint_tolerance_rad):
                            arm.startup_trajectory_complete = True
                            arm.limiter.reset(reference_pose)
                            print(
                                f"[FlexivDualTargetFrame] {side} task initial_q reached "
                                f"isaac_max_error_rad={isaac_joint_error:.6f} "
                                f"rdk_max_error_rad={rdk_joint_error:.6f}",
                                flush=True,
                            )
            if ready and not arm.rdk_ready:
                print(f"[FlexivDualTargetFrame] RDK transport operational {side} {arm.serial_number}", flush=True)
            elif not ready and arm.rdk_ready:
                print(
                    f"[FlexivDualTargetFrame] RDK not ready {side} {arm.serial_number}; pausing user target handoff",
                    flush=True,
                )
            arm.rdk_ready = bool(ready)
        task_ready = all(
            arm.startup_trajectory_complete
            and arm.rdk_ready
            and arm.rdk_phase == "ready"
            and arm.effort_control_enabled
            for arm in arms.values()
        )
        if task_ready and not dual_task_ready_announced:
            dual_task_ready_announced = True
            print(
                "[FlexivDualTargetFrame] READY: both task initial poses reached; user control enabled",
                flush=True,
            )
        if reset_state == "moving":
            reset_acknowledged = all(arm.rdk_reset_seq >= last_reset_seq for arm in arms.values())
            if task_ready and reset_acknowledged:
                try:
                    reset_configured_scene_assets()
                except Exception as exc:
                    reset_state = "failed"
                    reset_error = f"failed to restore configured scene assets after reset: {exc}"
                    print(f"[FlexivDualTargetFrame] {reset_error}", flush=True)
                else:
                    reset_state = "restoring_assets"
                    reset_assets_restore_time = time.monotonic()
                    print(
                        f"[FlexivDualTargetFrame] coordinated reset seq={last_reset_seq} "
                        "scene assets restored; waiting one target cycle before re-enabling physics",
                        flush=True,
                    )
            elif time.monotonic() - reset_started_time >= float(args.reset_timeout_sec):
                reset_state = "failed"
                reset_error = (
                    f"coordinated reset seq={last_reset_seq} timed out after "
                    f"{float(args.reset_timeout_sec):.3f}s"
                )
                print(f"[FlexivDualTargetFrame] {reset_error}", flush=True)
        elif reset_state == "restoring_assets":
            asset_settle_sec = max(1.0 / max(float(args.target_pose_publish_hz), 1.0), 2.0 / physics_hz)
            if time.monotonic() - reset_assets_restore_time >= asset_settle_sec:
                try:
                    set_reset_scene_collisions_suppressed(False)
                except Exception as exc:
                    reset_state = "failed"
                    reset_error = f"failed to restore scene physics after asset reset: {exc}"
                    print(f"[FlexivDualTargetFrame] {reset_error}", flush=True)
                else:
                    reset_state = "succeeded"
                    reset_error = None
                    print(
                        f"[FlexivDualTargetFrame] coordinated reset succeeded seq={last_reset_seq}; "
                        "both arms are READY at init_q and scene assets are at configured initial state",
                        flush=True,
                    )

    def _apply_arm_studio_torque(arm: ArmRuntime) -> None:
        """Receive and apply Studio's unmodified torque command for one 2 kHz cycle."""

        connected = arm.sim_node.connected()
        if connected and not arm.last_connected:
            print(
                f"[FlexivDualTargetFrame] SimPlugin connected {arm.side} {arm.serial_number}",
                flush=True,
            )
            # Match the official multi-robot bridge: SimPlugin connectivity is
            # sufficient to close the 2 kHz Studio torque loop.  Waiting for
            # RDK/DRDK readiness here creates a dependency cycle: the runtime
            # needs its torque applied to remain healthy while DRDK is still
            # discovering the pair.
            arm.robot.switch_control_mode("effort")
            arm.effort_control_enabled = True
        if not connected:
            if arm.last_connected:
                print(f"[FlexivDualTargetFrame] SimPlugin disconnected {arm.side} {arm.serial_number}", flush=True)
                if arm.articulation_ready:
                    _hold_arm_position(arm)
            arm.last_connected = False
            arm.effort_control_enabled = False
            return
        arm.last_connected = True
        if not arm.articulation_ready:
            arm.articulation_ready = _is_robot_ready(arm.robot)
            if not arm.articulation_ready:
                return
        if not arm.sim_node.WaitForRobotCommands(max(0, int(args.command_timeout_ms))):
            return
        target_drives = arm.sim_node.robot_commands().target_drives
        arm.latest_target_drives = list(target_drives[:7])
        if not arm.effort_control_enabled:
            arm.robot.switch_control_mode("effort")
            arm.effort_control_enabled = True
        arm.robot.apply_torques(target_drives)

    target_update_gate = arms["left"].target_pose_gate
    target_update_dt = target_update_gate.period_cycles / physics_hz

    def on_physics_step(_dt):
        nonlocal servo_cycle
        if not control_loop_enabled:
            return
        for arm in arms.values():
            if not arm.articulation_ready:
                arm.articulation_ready = _is_robot_ready(arm.robot)
            if not arm.articulation_ready:
                return
        servo_cycle += 1
        # The official multi-robot bridge sends every robot state before it
        # waits for any command. This keeps the two Studio controllers on the
        # same servo cycle even when one transport is Docker-backed.
        for arm in arms.values():
            arm.sim_node.SendRobotStates(
                flexivsimplugin.SimRobotStates(servo_cycle, arm.robot.q, arm.robot.dq)
            )
        if target_update_gate.should_publish(servo_cycle):
            _update_quest_targets()
            _apply_quest_gripper(arms["left"])
            _apply_quest_gripper(arms["right"])
            _update_arm_target(arms["left"], target_update_dt)
            _update_arm_target(arms["right"], target_update_dt)
            _update_rdk_statuses()
        for arm in arms.values():
            _apply_arm_studio_torque(arm)

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
            if arm.rdk_world_calibration is not None:
                base_tcp_pose = arm.rdk_world_calibration.world_pose_to_rdk(
                    world_position=target_position,
                    world_orientation_wxyz=target_orientation,
                )
            else:
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
                "control_state": {
                    "left": {
                        "target_control_requested": bool(left.target_control_requested),
                        "target_control_source": left.target_control_source,
                        "rdk_ready": bool(left.rdk_ready),
                        "effort_control_enabled": bool(left.effort_control_enabled),
                        "startup_trajectory_complete": bool(left.startup_trajectory_complete),
                        "task_ready": bool(
                            left.startup_trajectory_complete
                            and left.rdk_ready
                            and left.effort_control_enabled
                        ),
                    },
                    "right": {
                        "target_control_requested": bool(right.target_control_requested),
                        "target_control_source": right.target_control_source,
                        "rdk_ready": bool(right.rdk_ready),
                        "effort_control_enabled": bool(right.effort_control_enabled),
                        "startup_trajectory_complete": bool(right.startup_trajectory_complete),
                        "task_ready": bool(
                            right.startup_trajectory_complete
                            and right.rdk_ready
                            and right.effort_control_enabled
                        ),
                    },
                },
                "stage3_task": stage3_task,
                "scene_config": str(args.scene_config) if args.scene_config else None,
                "scene_objects": scene_object_summary,
                "reset": {
                    "last_seq": int(last_reset_seq),
                    "state": str(reset_state),
                    "ready": bool(
                        reset_state in {"idle", "succeeded"}
                        and left.startup_trajectory_complete
                        and right.startup_trajectory_complete
                        and left.rdk_ready
                        and right.rdk_ready
                    ),
                    "error": reset_error,
                    "reason": reset_reason,
                    "rdk_reset_seq": {
                        "left": int(left.rdk_reset_seq),
                        "right": int(right.rdk_reset_seq),
                    },
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

    def publish_state_monitor() -> None:
        """Publish read-only actual arm state independently of the recorder gateway."""

        nonlocal state_monitor_last_publish
        if state_monitor_publisher is None:
            return
        current_time = time.monotonic()
        if current_time - state_monitor_last_publish < 1.0 / float(args.state_monitor_hz):
            return
        state_monitor_last_publish = current_time

        arm_states = {}
        for side, arm in arms.items():
            tcp_position, tcp_orientation = arm.robot.end_effector.get_world_pose()
            tcp_pose_base = arm.rdk_current_pose_base_tcp
            if tcp_pose_base is None:
                tcp_pose_base = _current_pose_base_tcp(arm)
            quest_input = (
                None
                if quest_target_receiver is None
                else quest_target_receiver.latest_input(side)
            )
            quest_target = arm.latest_quest_target
            arm_states[side] = {
                "serial": arm.serial_number,
                "q": [float(value) for value in arm.robot.q],
                "dq": [float(value) for value in arm.robot.dq],
                "tcp_pose_base": [float(value) for value in tcp_pose_base],
                "tcp_pose_world": [
                    *[float(value) for value in tcp_position],
                    *[float(value) for value in tcp_orientation],
                ],
                "ready": bool(
                    arm.startup_trajectory_complete
                    and arm.rdk_ready
                    and arm.rdk_phase == "ready"
                    and arm.effort_control_enabled
                ),
                "quest": {
                    "available": quest_input is not None,
                    "seq": None if quest_input is None else int(quest_input["seq"]),
                    "age_sec": (
                        None
                        if quest_input is None
                        else max(0.0, current_time - float(quest_input["monotonic_time"]))
                    ),
                    "motion_data_ready": bool(
                        quest_input is not None and quest_input.get("motion_data_ready", False)
                    ),
                    "enable_button": (
                        "squeeze" if quest_input is None else str(quest_input.get("enable_button", "squeeze"))
                    ),
                    "enable_value": (
                        0.0 if quest_input is None else float(quest_input.get("enable_value", 0.0))
                    ),
                    "enabled": bool(quest_input is not None and quest_input.get("enabled", False)),
                    "gripper_button": (
                        "trigger" if quest_input is None else str(quest_input.get("gripper_button", "trigger"))
                    ),
                    "gripper_value": (
                        0.0 if quest_input is None else float(quest_input.get("gripper_value", 0.0))
                    ),
                    "gripper_closed": bool(
                        quest_input is not None and quest_input.get("gripper_closed", False)
                    ),
                    "controller_pose_openxr": (
                        None
                        if quest_input is None or quest_input.get("controller_pose_openxr") is None
                        else [float(value) for value in quest_input["controller_pose_openxr"]]
                    ),
                    "controller_delta_base": (
                        None
                        if quest_target is None or quest_target.controller_delta_base is None
                        else [float(value) for value in quest_target.controller_delta_base]
                    ),
                    "target_packet_pose_base_tcp": (
                        None
                        if quest_target is None
                        else [float(value) for value in quest_target.pose_base_tcp_des]
                    ),
                    "mapped_goal_pose_base_tcp": (
                        None
                        if arm.quest_goal_pose_base_tcp is None
                        else [float(value) for value in arm.quest_goal_pose_base_tcp]
                    ),
                },
            }
        state_monitor_publisher.publish(
            {
                "schema": "flexiv_dual_arm_state.v1",
                "servo_cycle": int(servo_cycle),
                "stamp_ns": now_ns(),
                "monotonic_time": current_time,
                "arms": arm_states,
            }
        )

    def capture_initial_frame_if_ready(frame_count: int) -> bool:
        if args.capture_initial_frame is None or frame_count < max(1, int(args.capture_after_frames)):
            return False
        if not stage2_cameras:
            raise RuntimeError("--capture-initial-frame requires at least one camera in --scene-config")
        selected = stage2_cameras[0]
        for camera in stage2_cameras:
            if getattr(camera, "name", "") == args.capture_camera_name or camera.prim_path.endswith(
                "/" + args.capture_camera_name
            ):
                selected = camera
                break
        rgba = selected.get_rgba()
        if rgba is None:
            return False
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("opencv-python is required for --capture-initial-frame") from exc
        output_path = Path(args.capture_initial_frame).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(output_path), _camera_rgba_to_bgr(rgba)):
            raise RuntimeError(f"Failed to write initial frame: {output_path}")
        print(
            f"[FlexivDualTargetFrame] initial frame captured camera={args.capture_camera_name} "
            f"path={output_path}",
            flush=True,
        )
        return True

    world.add_physics_callback("dual_target_frame_step", callback_fn=on_physics_step)
    world.reset()
    for camera in stage2_cameras:
        camera.initialize()
    initialize_like_startup("startup", reset_world=False)
    _select_target(arms["left"].target_prim_path)
    if not args.manual_play:
        omni.timeline.get_timeline_interface().play()

    print(
        f"[FlexivDualTargetFrame] Started; waiting for task-pose READY. "
        f"control_source=studio-bridge. physics_hz={physics_hz:g}. "
        f"target_pose_hz={physics_hz / max(1, arms['left'].target_pose_gate.period_cycles):.3f}. "
        f"left={args.left_serial_number} right={args.right_serial_number}. "
        f"Drag {arms['left'].target_prim_path}; select {arms['right'].target_prim_path} for the right arm.",
        flush=True,
    )

    reset_needed = False
    frame_count = 0
    try:
        while simulation_app.is_running():
            frame_count += 1
            # World batches physics_dt substeps internally until rendering_dt.
            # At 2000/30 Hz this invokes the callback about 66-67 times per
            # rendered frame without crossing Python's world.step boundary for
            # every 0.5 ms physics tick.
            world.step(render=True)
            _update_quest_target_frames()
            publish_state_monitor()
            publish_gateway_sample()
            if capture_initial_frame_if_ready(frame_count):
                break
            if pending_reset_control is not None:
                control = pending_reset_control
                pending_reset_control = None
                begin_coordinated_reset(control)
                reset_needed = False
            if world.is_stopped() and not reset_needed:
                reset_needed = True
            if world.is_playing() and reset_needed:
                initialize_like_startup("timeline-resume", reset_world=True)
                reset_needed = False
            if args.smoke_test and args.max_frames > 0 and frame_count >= args.max_frames:
                break
    finally:
        try:
            world.remove_physics_callback("dual_target_frame_step")
        except Exception:
            pass
        for arm in arms.values():
            arm.target_pose_publisher.close()
        if quest_target_receiver is not None:
            quest_target_receiver.close()
        rdk_status_receiver.close()
        if state_monitor_publisher is not None:
            state_monitor_publisher.close()
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
