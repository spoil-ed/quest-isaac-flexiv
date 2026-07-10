import asyncio
import builtins
import os
import time
from pathlib import Path

import carb
import omni.ext
import omni.kit.app
import omni.usd
from isaacsim.storage.native import get_assets_root_path


DEFAULT_PROFILE = "floating_xarm_dex3.yaml"
DEFAULT_STAGE_RELATIVE = "/Isaac/Samples/Replicator/Teleop/teleop_scenario_floating_xarm_dex3.usd"
FLEXIV_WORKFLOWS = {"flexiv-studio", "flexiv-studio-teleop", "flexiv_studio", "flexiv_studio_teleop"}


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_float(name: str, default: float = 0.0) -> float:
    try:
        return float(os.environ.get(name, str(default)))
    except ValueError:
        return default


class TeleopDemoLoaderExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        self._task = asyncio.ensure_future(self._run_workflow())

    def on_shutdown(self) -> None:
        self._task = None

    async def _run_workflow(self) -> None:
        app = omni.kit.app.get_app()
        for _ in range(30):
            await app.next_update_async()

        workflow = os.environ.get("SIMATE_TELEOP_WORKFLOW", "teleop").strip().lower()
        if workflow in FLEXIV_WORKFLOWS:
            carb.log_warn("[TeleopSDG] Flexiv Studio workflow is handled by simate.flexiv_studio_teleop.")
            return
        if workflow not in {"teleop", "record", "replay"}:
            carb.log_warn(f"[TeleopSDG] Unknown workflow {workflow!r}; falling back to teleop.")
            workflow = "teleop"

        stage_url = self._resolve_stage_url(workflow)
        if not stage_url:
            return
        if not await self._open_stage(stage_url):
            return

        teleop_window = None
        if workflow in {"teleop", "record"}:
            teleop_window = await self._open_teleop_window()

        recorder_window = await self._open_recorder_window()
        builtins._teleop_sdg_windows = (teleop_window, recorder_window)

        if workflow == "record":
            await self._setup_recording(recorder_window)
        elif workflow == "replay":
            await self._setup_replay(recorder_window)
        else:
            carb.log_warn("[TeleopSDG] Teleop demo, Teleop window, and Episode Recorder are ready.")

    def _resolve_stage_url(self, workflow: str) -> str | None:
        explicit_stage = os.environ.get("SIMATE_TELEOP_STAGE", "").strip()
        if explicit_stage:
            return explicit_stage

        if workflow == "replay":
            hdf5 = os.environ.get("SIMATE_TELEOP_HDF5", "").strip()
            if hdf5:
                snapshot = Path(hdf5).expanduser().resolve().parent / "stage_snapshot.usd"
                if snapshot.is_file():
                    return str(snapshot)

        assets_root = get_assets_root_path()
        if not assets_root:
            carb.log_error("[TeleopSDG] Could not resolve Isaac Sim assets root.")
            return None
        return assets_root + DEFAULT_STAGE_RELATIVE

    async def _open_stage(self, stage_url: str) -> bool:
        app = omni.kit.app.get_app()
        ctx = omni.usd.get_context()
        carb.log_warn(f"[TeleopSDG] Opening stage: {stage_url}")
        if not ctx.open_stage(stage_url):
            carb.log_error(f"[TeleopSDG] Failed to open stage: {stage_url}")
            return False

        for _ in range(900):
            await app.next_update_async()
            stage = ctx.get_stage()
            try:
                remaining = ctx.get_stage_loading_status()[2]
            except Exception:
                remaining = 0
            if stage is not None and remaining == 0:
                return True

        carb.log_error("[TeleopSDG] Timed out waiting for stage to load.")
        return False

    async def _open_teleop_window(self):
        app = omni.kit.app.get_app()
        from isaacsim.replicator.teleop import get_builtin_teleop_profiles_dir, load_teleop_profile
        from isaacsim.replicator.teleop.ui.teleop_window import TeleopWindow

        teleop_window = TeleopWindow(title="Teleop")
        teleop_window.visible = True

        profile_path = os.environ.get("SIMATE_TELEOP_PROFILE", "").strip()
        if not profile_path:
            profile_path = os.path.join(get_builtin_teleop_profiles_dir(), DEFAULT_PROFILE)

        profile, errors = load_teleop_profile(profile_path)
        if profile is None:
            carb.log_error(f"[TeleopSDG] Failed to load profile {profile_path}: {errors}")
            return teleop_window

        ok, message = teleop_window.apply_teleop_profile(profile)
        carb.log_warn(f"[TeleopSDG] Applied profile {os.path.basename(profile_path)}: ok={ok}, message={message}")
        for _ in range(5):
            await app.next_update_async()
        return teleop_window

    async def _open_recorder_window(self):
        app = omni.kit.app.get_app()
        from isaacsim.replicator.episode_recorder.ui.episode_recorder_window import EpisodeRecorderWindow

        recorder_window = EpisodeRecorderWindow(title="Episode Recorder")
        recorder_window.visible = True
        for _ in range(5):
            await app.next_update_async()
        return recorder_window

    async def _setup_recording(self, recorder_window) -> None:
        app = omni.kit.app.get_app()
        panel = getattr(recorder_window, "_panel", None)
        if panel is None:
            carb.log_error("[TeleopSDG] Episode Recorder panel is unavailable.")
            return

        output_dir = os.environ.get("SIMATE_TELEOP_RECORD_OUTPUT_DIR", "").strip()
        if not output_dir:
            output_dir = os.path.abspath("recordings/teleop_hdf5")
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        root_path = os.environ.get("SIMATE_TELEOP_RECORD_ROOT_PATH", "/World").strip() or "/World"
        file_prefix = os.environ.get("SIMATE_TELEOP_RECORD_FILE_PREFIX", "episode").strip() or "episode"
        auto_start_on_play = _env_bool("SIMATE_TELEOP_RECORD_AUTO_START_ON_PLAY", True)

        panel._root_path_field.model.set_value(root_path)
        panel._output_dir_field.model.set_value(output_dir)
        panel._file_prefix_field.model.set_value(file_prefix)
        if panel._auto_start_model is not None:
            panel._auto_start_model.set_value(auto_start_on_play)

        panel._on_discover_clicked()
        for _ in range(5):
            await app.next_update_async()

        panel._on_export_snapshot_clicked()
        for _ in range(5):
            await app.next_update_async()

        if _env_bool("SIMATE_TELEOP_RECORD_OPEN_SESSION", True):
            panel._open_session()
            for _ in range(5):
                await app.next_update_async()

        carb.log_warn(
            "[TeleopSDG] Recording session is ready. Press Play to start an episode; "
            "press Stop to end it. Close Session when all trajectories are collected."
        )

        if _env_bool("SIMATE_TELEOP_RECORD_AUTO_PLAY", False):
            await self._auto_play_recording()

    async def _auto_play_recording(self) -> None:
        import omni.timeline

        app = omni.kit.app.get_app()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        duration = _env_float("SIMATE_TELEOP_RECORD_DURATION", 0.0)
        if duration <= 0:
            carb.log_warn("[TeleopSDG] Auto-play started without duration; stop the timeline manually.")
            return

        start = time.perf_counter()
        while time.perf_counter() - start < duration:
            await app.next_update_async()
        timeline.stop()
        carb.log_warn(f"[TeleopSDG] Auto-play stopped after {duration:g}s.")

    async def _setup_replay(self, recorder_window) -> None:
        app = omni.kit.app.get_app()
        panel = getattr(recorder_window, "_panel", None)
        if panel is None:
            carb.log_error("[TeleopSDG] Episode Recorder panel is unavailable.")
            return

        hdf5 = os.environ.get("SIMATE_TELEOP_HDF5", "").strip()
        if not hdf5:
            carb.log_error("[TeleopSDG] SIMATE_TELEOP_HDF5 is required for replay workflow.")
            return
        hdf5_path = str(Path(hdf5).expanduser().resolve())
        if not os.path.isfile(hdf5_path):
            carb.log_error(f"[TeleopSDG] HDF5 file not found: {hdf5_path}")
            return

        if panel._replay_file_field is not None:
            panel._replay_file_field.model.set_value(hdf5_path)
        panel._on_replay_load()
        for _ in range(10):
            await app.next_update_async()

        episode = int(os.environ.get("SIMATE_TELEOP_REPLAY_EPISODE", "0"))
        if panel._replay_episode_combo is not None:
            panel._replay_episode_combo.model.get_item_value_model().set_value(episode)

        if _env_bool("SIMATE_TELEOP_REPLAY_AUTOSTART", True):
            panel._start_replay()
            carb.log_warn(f"[TeleopSDG] Replay loaded and started: {hdf5_path} episode={episode}")
        else:
            carb.log_warn(f"[TeleopSDG] Replay loaded: {hdf5_path} episode={episode}")
