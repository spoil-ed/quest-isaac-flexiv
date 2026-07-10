"""Flexiv-Isaac bridge runtime for Kit extension mode."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .config import FlexivTeleopConfig, RobotConfig


APP_VERSION = "0.1.0"
COMPATIBLE_SIM_PLUGIN_VER = "1.2.0"
PHYSICS_FREQ = 2000.0
RENDER_FREQ = 60.0


def add_examples_python_path(path: str | Path | None) -> None:
    if not path:
        return
    value = str(Path(path).expanduser().resolve())
    if value not in sys.path:
        sys.path.insert(0, value)


@dataclass
class _RobotRuntime:
    config: RobotConfig
    instance: Any
    sim_plugin: Any
    last_connected: bool = False
    initial_pose_applied: bool = False
    warned_not_ready: bool = False


class FlexivBridgeRuntime:
    """Owns the Isaac World and flexivsimplugin state/torque bridge."""

    ROBOT_DOF = 7

    def __init__(self, config: FlexivTeleopConfig, *, log=None, examples_path: str | Path | None = None) -> None:
        self.config = config
        self._log = log or (lambda _msg: None)
        self._examples_path = examples_path
        self._world = None
        self._robots: list[_RobotRuntime] = []
        self._cameras: list[Any] = []
        self._servo_cycle = 0
        self._reset_needed = False

    @property
    def world(self):
        return self._world

    @property
    def robots(self) -> list[_RobotRuntime]:
        return list(self._robots)

    def setup(self) -> None:
        add_examples_python_path(self._examples_path)

        import flexivsimplugin
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
        from isaacsim.robot.manipulators.examples.flexiv import FlexivSerial
        from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, UsdLux

        if flexivsimplugin.__version__ != COMPATIBLE_SIM_PLUGIN_VER:
            raise ImportError(
                f"flexivsimplugin=={COMPATIBLE_SIM_PLUGIN_VER} is required, "
                f"but found {flexivsimplugin.__version__}"
            )

        self._log(f"[FlexivBridge] starting v{APP_VERSION}")
        self._world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / PHYSICS_FREQ,
            rendering_dt=1.0 / RENDER_FREQ,
            set_defaults=False,
        )
        if self.config.gpu_dynamics:
            self._world.get_physics_context().enable_gpu_dynamics(True)

        if self.config.env_usd:
            add_reference_to_stage(usd_path=self.config.env_usd, prim_path="/World")
        else:
            self._world.scene.add_default_ground_plane()
            stage = get_current_stage()
            dome = UsdLux.DomeLight.Define(stage, "/World/defaultDomeLight")
            dome.CreateIntensityAttr(500.0)
            dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            distant = UsdLux.DistantLight.Define(stage, "/World/defaultDistantLight")
            distant.CreateIntensityAttr(3000.0)
            distant.CreateAngleAttr(0.53)
            distant.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 35.0))

        self._world.reset()

        for camera_cfg in self.config.cameras:
            name = camera_cfg["name"]
            pos = [float(camera_cfg["position"][key]) for key in ("x", "y", "z")]
            ori = [float(camera_cfg["orientation"][key]) for key in ("w", "x", "y", "z")]
            camera = Camera(
                prim_path="/World/" + name,
                frequency=camera_cfg["fps"],
                resolution=tuple(camera_cfg["resolution"]),
                position=pos,
                orientation=ori,
            )
            camera.set_focal_length(camera_cfg["focal_length"])
            camera.set_world_pose(position=pos, orientation=ori, camera_axes="usd")
            self._cameras.append(camera)

        for robot_cfg in self.config.robots:
            self._log(
                f"[FlexivBridge] adding {robot_cfg.serial_number} at {robot_cfg.prim_path} "
                f"from {robot_cfg.usd}"
            )
            add_reference_to_stage(usd_path=robot_cfg.usd, prim_path=robot_cfg.prim_path)

            gripper = None
            end_effector_prim_name = "flange"
            if "Grav" in robot_cfg.usd:
                end_effector_prim_name = "Grav_gripper/right_finger_tip"
                gripper = ParallelGripper(
                    end_effector_prim_path=robot_cfg.prim_path + "/" + end_effector_prim_name,
                    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
                    joint_opened_positions=np.array([45.0, 0.0]),
                    joint_closed_positions=np.array([-8.88, 0.0]),
                )
            elif "Robotiq" in robot_cfg.usd:
                end_effector_prim_name = "Robotiq_2F_85_flattened/Robotiq_2F_85/right_inner_finger"
                gripper = ParallelGripper(
                    end_effector_prim_path=robot_cfg.prim_path + "/" + end_effector_prim_name,
                    joint_prim_names=["finger_joint", "right_inner_finger_joint"],
                    joint_opened_positions=np.array([0.0, 0.0]),
                    joint_closed_positions=np.array([45.0, 0.0]),
                )

            qx, qy, qz, qw = robot_cfg.orientation_xyzw
            robot = self._world.scene.add(
                FlexivSerial(
                    prim_path=robot_cfg.prim_path,
                    name=robot_cfg.prim_name,
                    end_effector_prim_name=end_effector_prim_name,
                    arm_dof=self.ROBOT_DOF,
                    pos_in_world=list(robot_cfg.position),
                    ori_in_world=[qw, qx, qy, qz],
                    gripper=gripper,
                )
            )
            self._robots.append(
                _RobotRuntime(
                    config=robot_cfg,
                    instance=robot,
                    sim_plugin=flexivsimplugin.UserNode(robot_cfg.serial_number),
                )
            )

        self._world.add_physics_callback("flexiv_studio_teleop_step", callback_fn=self.on_physics_step)
        self._world.reset()

        for camera in self._cameras:
            camera.initialize()

    def on_physics_step(self, _dt: float) -> None:
        import flexivsimplugin

        current_servo_cycle = self._servo_cycle + 1

        for robot in self._robots:
            self._apply_initial_pose_if_ready(robot)
            robot.sim_plugin.SendRobotStates(
                flexivsimplugin.SimRobotStates(
                    current_servo_cycle,
                    robot.instance.q,
                    robot.instance.dq,
                )
            )

        for robot in self._robots:
            robot_ready = self._is_robot_ready(robot)
            if robot.sim_plugin.connected():
                if not robot_ready:
                    if not robot.warned_not_ready:
                        self._log(
                            f"[FlexivBridge] waiting for Isaac physics handle before controlling "
                            f"{robot.config.serial_number}"
                        )
                        robot.warned_not_ready = True
                    robot.last_connected = False
                    continue
                if not robot.last_connected:
                    self._log(f"[FlexivBridge] connected {robot.config.serial_number}")
                    robot.instance.switch_control_mode("effort")
                if robot.sim_plugin.WaitForRobotCommands(100):
                    robot.instance.apply_torques(robot.sim_plugin.robot_commands().target_drives)
                else:
                    self._log(f"[FlexivBridge] missed command from {robot.config.serial_number}")
                robot.last_connected = True
            else:
                if robot.last_connected:
                    self._log(f"[FlexivBridge] disconnected {robot.config.serial_number}")
                    if robot_ready:
                        robot.instance.switch_control_mode("position")
                        robot.instance.teleport_to(robot.instance.q)
                robot.last_connected = False

        self._servo_cycle = current_servo_cycle

    def _is_robot_ready(self, robot: _RobotRuntime) -> bool:
        articulation_view = getattr(robot.instance, "_articulation_view", None)
        if articulation_view is None:
            return False
        try:
            return bool(articulation_view.is_physics_handle_valid())
        except Exception:
            return False

    def _apply_initial_pose_if_ready(self, robot: _RobotRuntime) -> bool:
        if robot.initial_pose_applied:
            return True
        if not self._is_robot_ready(robot):
            return False
        robot.instance.teleport_to(robot.config.initial_q)
        robot.initial_pose_applied = True
        robot.warned_not_ready = False
        self._log(f"[FlexivBridge] initial pose applied for {robot.config.serial_number}")
        return True

    def shutdown(self) -> None:
        if self._world is not None:
            try:
                self._world.remove_physics_callback("flexiv_studio_teleop_step")
            except Exception:
                pass
        self._robots.clear()
        self._cameras.clear()
        self._world = None
