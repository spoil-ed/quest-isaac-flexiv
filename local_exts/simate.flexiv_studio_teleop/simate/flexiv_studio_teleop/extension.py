"""Kit extension entry point for Flexiv Studio teleop."""

from __future__ import annotations

import asyncio
import builtins
import os
import traceback
from pathlib import Path

import carb
import omni.ext
import omni.kit.app
import omni.timeline

from .bridge import FlexivBridgeRuntime
from .config import load_config
from .ik_adapter import FlexivStudioIKController, SideBinding
from .rdk_sink import FlexivRdkCartesianSink, RdkSinkSettings


FLEXIV_WORKFLOWS = {"flexiv-studio", "flexiv-studio-teleop", "flexiv_studio", "flexiv_studio_teleop"}


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


class Extension(omni.ext.IExt):
    def on_startup(self, _ext_id: str) -> None:
        workflow = os.environ.get("SIMATE_TELEOP_WORKFLOW", "").strip().lower()
        carb.log_warn(f"[FlexivTeleop] on_startup workflow={workflow!r}")
        self._task = None
        self._bridge = None
        self._sink = None
        self._windows = ()
        if workflow in FLEXIV_WORKFLOWS:
            self._task = asyncio.ensure_future(self._run())
            self._task.add_done_callback(self._on_task_done)

    def _on_task_done(self, task) -> None:
        if task.cancelled():
            carb.log_warn("[FlexivTeleop] startup task cancelled")
            return
        exc = task.exception()
        if exc is not None:
            carb.log_error(f"[FlexivTeleop] startup task failed: {exc}")

    def on_shutdown(self) -> None:
        if self._sink is not None:
            self._sink.stop()
        if self._bridge is not None:
            self._bridge.shutdown()
        self._task = None
        self._bridge = None
        self._sink = None
        self._windows = ()

    def _log(self, message: str) -> None:
        carb.log_warn(message)

    async def _run(self) -> None:
        carb.log_warn("[FlexivTeleop] startup task running")
        app = omni.kit.app.get_app()
        for _ in range(30):
            await app.next_update_async()

        config_path = os.environ.get("SIMATE_FLEXIV_TELEOP_CONFIG", "").strip()
        if not config_path:
            carb.log_error("[FlexivTeleop] SIMATE_FLEXIV_TELEOP_CONFIG is required.")
            return

        try:
            config = load_config(config_path)
        except Exception as exc:
            carb.log_error(f"[FlexivTeleop] Failed to load config {config_path}: {exc}")
            return

        examples_path = os.environ.get("SIMATE_FLEXIV_EXAMPLES_PATH", "").strip()
        try:
            self._bridge = FlexivBridgeRuntime(config, log=self._log, examples_path=examples_path)
            self._bridge.setup()
        except Exception as exc:
            carb.log_error(f"[FlexivTeleop] Failed to set up Flexiv bridge: {exc}")
            carb.log_error(traceback.format_exc())
            return

        bindings = [SideBinding.from_robot_config(robot) for robot in config.teleop_robots]
        if not bindings:
            carb.log_error("[FlexivTeleop] No enabled teleop robot bindings in config.")
            return

        self._sink = FlexivRdkCartesianSink(
            RdkSinkSettings(
                stream_hz=_env_float("SIMATE_FLEXIV_RDK_STREAM_HZ", 250.0),
                switch_mode=_env_bool("SIMATE_FLEXIV_RDK_SWITCH_MODE", True),
                clear_fault=_env_bool("SIMATE_FLEXIV_RDK_CLEAR_FAULT", False),
                servo_on=_env_bool("SIMATE_FLEXIV_RDK_SERVO_ON", False),
                verbose=_env_bool("SIMATE_FLEXIV_RDK_VERBOSE", False),
            ),
            log=self._log,
        )
        adapter = FlexivStudioIKController(bindings, target_sink=self._sink, log=self._log)

        teleop_window = await self._open_teleop_window(adapter, bindings)
        recorder_window = None
        if _env_bool("SIMATE_FLEXIV_TELEOP_OPEN_RECORDER", False):
            recorder_window = await self._open_recorder_window()

        self._windows = (teleop_window, recorder_window)
        builtins._flexiv_studio_teleop_runtime = {
            "bridge": self._bridge,
            "sink": self._sink,
            "adapter": adapter,
            "windows": self._windows,
        }

        if _env_bool("SIMATE_FLEXIV_TELEOP_AUTO_PLAY", True):
            omni.timeline.get_timeline_interface().play()

        carb.log_warn(
            "[FlexivTeleop] Ready. In Teleop, Connect the headset, keep the IK side enabled, "
            "and use Play/Stop for simulation."
        )

    async def _open_teleop_window(self, adapter: FlexivStudioIKController, bindings: list[SideBinding]):
        from isaacsim.replicator.teleop import (
            BimanualControllerProfile,
            ControllerSideProfile,
            GraspControllerProfile,
            LocomotionProfile,
            TeleopProfile,
            TeleopSettingsProfile,
        )
        from isaacsim.replicator.teleop.ui.teleop_window import TeleopWindow

        app = omni.kit.app.get_app()
        teleop_window = TeleopWindow(title="Flexiv Studio Teleop")
        teleop_window.visible = True

        old_ik = getattr(teleop_window, "_ik_controller", None)
        if old_ik is not None:
            try:
                old_ik.set_on_status_changed(None)
                for side in ("left", "right"):
                    old_ik.disable(side)
                    old_ik.destroy(side)
            except Exception:
                pass

        teleop_window._ik_controller = adapter
        teleop_window._teleop_manager.set_ik_controller(adapter)
        if getattr(teleop_window, "_ik_panel", None) is not None:
            teleop_window._ik_panel._ik = adapter
            adapter.set_on_status_changed(teleop_window._ik_panel.on_reachability_changed)

        profile = self._build_profile(bindings, TeleopProfile, TeleopSettingsProfile, BimanualControllerProfile, ControllerSideProfile, GraspControllerProfile, LocomotionProfile)
        ok, message = teleop_window.apply_teleop_profile(profile)
        carb.log_warn(f"[FlexivTeleop] Applied Flexiv profile: ok={ok}, message={message}")

        for _ in range(5):
            await app.next_update_async()
        return teleop_window

    @staticmethod
    def _build_profile(
        bindings: list[SideBinding],
        TeleopProfile,
        TeleopSettingsProfile,
        BimanualControllerProfile,
        ControllerSideProfile,
        GraspControllerProfile,
        LocomotionProfile,
    ):
        side_profiles = {
            "left": ControllerSideProfile(enabled=False, settings={}),
            "right": ControllerSideProfile(enabled=False, settings={}),
        }
        for binding in bindings:
            side_profiles[binding.side] = ControllerSideProfile(
                enabled=True,
                settings={
                    "robot_path": binding.prim_path,
                    "ee_link": binding.ee_link,
                    "solver": "position-based",
                    "method": "singular-value-decomposition",
                    "gain": 5.0,
                    "vr_target_filter": 0.0,
                    "max_joint_step": 0.0,
                    "ee_rot_x_deg": binding.ee_rot_x_deg,
                    "ee_rot_y_deg": binding.ee_rot_y_deg,
                    "ee_rot_z_deg": binding.ee_rot_z_deg,
                },
            )
        return TeleopProfile(
            session=TeleopSettingsProfile(
                coordinate_system="isaac_sim",
                tracking_space_enabled=False,
                tracking_space_path="",
                marker_scale=0.05,
                anchor_rotation_mode="fixed",
                anchor_fixed_height=True,
            ),
            ik=BimanualControllerProfile(left=side_profiles["left"], right=side_profiles["right"]),
            grasp=GraspControllerProfile(),
            locomotion=LocomotionProfile(enabled=False, settings={}),
        )

    async def _open_recorder_window(self):
        from isaacsim.replicator.episode_recorder.ui.episode_recorder_window import EpisodeRecorderWindow

        app = omni.kit.app.get_app()
        recorder_window = EpisodeRecorderWindow(title="Episode Recorder")
        recorder_window.visible = True
        for _ in range(5):
            await app.next_update_async()
        return recorder_window
