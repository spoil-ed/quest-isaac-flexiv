#!/usr/bin/env python3
"""Run strict Stage1 single-Rizon4 real-runtime validation."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FLEXIV_PYTHON = Path(sys.executable)
DEFAULT_CONFIG = REPO_ROOT / "configs/pipelines/stage1_single_rizon4_data_collection.yaml"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "datasets/stage1_records"
DEFAULT_LEROBOT_OUTPUT_ROOT = REPO_ROOT / "datasets/lerobot"
DEFAULT_SAMPLE_ENDPOINT = os.environ.get("FLEXIV_STAGE1_SAMPLE_ENDPOINT", "tcp://127.0.0.1:5690")
DEFAULT_BRIDGE_ENDPOINT = os.environ.get("FLEXIV_STAGE1_BRIDGE_ENDPOINT", "tcp://127.0.0.1:5691")
DEFAULT_TARGET_POSE_UDP_HOST = os.environ.get("FLEXIV_TARGET_POSE_UDP_HOST", "127.0.0.1")
DEFAULT_TARGET_POSE_UDP_PORT = int(os.environ.get("FLEXIV_TARGET_POSE_UDP_PORT", "55678"))
DEFAULT_QUEST_TARGET_UDP_HOST = os.environ.get("FLEXIV_QUEST_TARGET_UDP_HOST", "127.0.0.1")
DEFAULT_QUEST_TARGET_UDP_PORT = int(os.environ.get("FLEXIV_QUEST_TARGET_UDP_PORT", "55679"))


def _import_stage1_helpers() -> None:
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))


_import_stage1_helpers()

from flexiv_data_collection.protocol import JsonLineReqClient, parse_tcp_endpoint  # noqa: E402
from flexiv_data_collection.real_validation import (  # noqa: E402
    EXPECTED_STAGE1_BACKEND,
    STAGE1_CAMERA_KEYS,
    Stage1SampleMonitor,
    summarize_stage1_single_arm_frames,
)


def json_print(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, ensure_ascii=False, default=str), flush=True)


def load_config(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    config_path = path.expanduser()
    if not config_path.exists():
        return {}
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Config must be a YAML mapping: {config_path}")
    return data


def resolve_config_path(value: Any, *, base: Path) -> Path | None:
    if value in (None, ""):
        return None
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def cfg_get(config: dict[str, Any], *keys: str) -> Any:
    current: Any = config
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def first_defined(*values: Any) -> Any:
    for value in values:
        if value is not None:
            return value
    return None


def endpoint_connect_host(host: str) -> str:
    return "127.0.0.1" if host in {"", "0.0.0.0", "::"} else host


def path_value(value: Any) -> Path | None:
    if value is None:
        return None
    return Path(str(value)).expanduser()


def env_path(name: str) -> Path | None:
    value = os.environ.get(name)
    return path_value(value) if value else None


def tail_log(path: Path, lines: int = 120) -> str:
    if not path.exists() or not path.is_file():
        return ""
    text = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(text[-lines:])


def assert_tcp_port_free(host: str, port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind((host, int(port)))
        except OSError as exc:
            raise RuntimeError(f"TCP port {host}:{port} is already in use; refusing to reuse an external gateway") from exc


def assert_udp_port_free(host: str, port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((host, int(port)))
        except OSError as exc:
            raise RuntimeError(f"UDP port {host}:{port} is already in use; refusing to share the real validation loop") from exc


def assert_required_path(path: Path | None, label: str, *, executable: bool = False, directory: bool = False) -> None:
    if path is None:
        raise RuntimeError(f"{label} is not configured; set it in the environment config or pass the matching CLI flag")
    if directory and not path.is_dir():
        raise RuntimeError(f"{label} does not exist or is not a directory: {path}")
    if (not directory) and not path.exists():
        raise RuntimeError(f"{label} does not exist: {path}")
    if executable and not os.access(path, os.X_OK):
        raise RuntimeError(f"{label} is not executable: {path}")


def wait_tcp(host: str, port: int, *, timeout: float, check_processes) -> None:
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        check_processes()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            if sock.connect_ex((host, int(port))) == 0:
                return
        time.sleep(0.25)
    raise TimeoutError(f"Timed out waiting for TCP {host}:{port}")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--flexiv-python", type=Path, default=DEFAULT_FLEXIV_PYTHON)
    parser.add_argument("--rdk-python", type=Path)
    parser.add_argument("--isaac-python", type=Path)
    parser.add_argument("--isaacsim-root", type=Path)
    parser.add_argument("--serial-number")
    parser.add_argument("--joint-group")
    parser.add_argument("--usd", type=Path)
    parser.add_argument("--examples-ext", type=Path)
    parser.add_argument("--camera-config", type=Path)
    parser.add_argument("--output-root", type=Path)
    parser.add_argument("--lerobot-output-root", type=Path)
    parser.add_argument("--repo-id-prefix")
    parser.add_argument("--action-mode")
    parser.add_argument("--record-frames", type=int)
    parser.add_argument("--record-fps", type=float)
    parser.add_argument("--image-size")
    parser.add_argument("--sample-endpoint")
    parser.add_argument("--bridge-endpoint")
    parser.add_argument("--target-pose-udp-host")
    parser.add_argument("--target-pose-udp-port", type=int)
    parser.add_argument("--quest-target-udp-host")
    parser.add_argument("--quest-target-udp-port", type=int)
    parser.add_argument("--fake-host")
    parser.add_argument("--fake-side")
    parser.add_argument("--fake-axis")
    parser.add_argument("--fake-quat-wxyz")
    parser.add_argument("--fake-amplitude-m", type=float)
    parser.add_argument("--fake-frames", type=int)
    parser.add_argument("--fake-rate-hz", type=float)
    parser.add_argument("--gateway-fps", type=float)
    parser.add_argument("--gateway-jpeg-quality", type=int)
    parser.add_argument("--probe-timeout-sec", type=float, default=150.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=240.0)
    parser.add_argument("--min-left-q-delta", type=float)
    parser.add_argument("--min-probe-left-q-delta", type=float, default=1e-4)
    parser.add_argument("--min-left-torque-norm", type=float)
    parser.add_argument("--keep-running-on-failure", action="store_true")
    return finalize_args(parser.parse_args(argv))


def finalize_args(args: argparse.Namespace) -> argparse.Namespace:
    args.config = Path(args.config).expanduser().resolve()
    pipeline_config = load_config(args.config)
    config_base = args.config.parent

    environment_config_path = resolve_config_path(
        first_defined(cfg_get(pipeline_config, "environment_config"), cfg_get(pipeline_config, "environment")),
        base=config_base,
    )
    scene_config_path = resolve_config_path(
        first_defined(cfg_get(pipeline_config, "scene_config"), cfg_get(pipeline_config, "scene")),
        base=config_base,
    )
    environment_config = load_config(environment_config_path) if environment_config_path is not None else {}
    scene_config = load_config(scene_config_path) if scene_config_path is not None else {}

    # Backward compatibility for the previous single-file Stage1 config shape.
    legacy_runtime_config = cfg_get(pipeline_config, "runtime") or {}
    environment = {**legacy_runtime_config, **environment_config}
    scene_robot = cfg_get(scene_config, "robot") or {}
    runtime_ws = path_value(first_defined(cfg_get(environment, "isaac_sim_ws"), env_path("ISAAC_SIM_WS")))
    configured_usd = (
        runtime_ws / "exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4_with_Grav.usd"
        if runtime_ws is not None
        else None
    )
    configured_examples_ext = (
        runtime_ws / "exts/isaacsim.robot.manipulators.examples"
        if runtime_ws is not None
        else None
    )

    record_task_dir = path_value(cfg_get(pipeline_config, "record", "task_dir"))
    configured_output_root = record_task_dir.parent if record_task_dir is not None else None

    args.environment_config = environment_config_path
    args.scene_config = scene_config_path
    args.pipeline_config_data = pipeline_config
    args.environment_config_data = environment_config
    args.scene_config_data = scene_config

    args.rdk_python = path_value(first_defined(args.rdk_python, cfg_get(environment, "rdk_python"), env_path("FLEXIV_RDK_PYTHON")))
    args.isaac_python = path_value(first_defined(args.isaac_python, cfg_get(environment, "isaac_python"), env_path("ISAAC_PYTHON")))
    args.isaacsim_root = path_value(first_defined(args.isaacsim_root, cfg_get(environment, "isaacsim_root"), env_path("ISAACSIM_ROOT")))
    args.usd = path_value(first_defined(args.usd, cfg_get(scene_robot, "usd"), cfg_get(environment, "usd"), env_path("FLEXIV_RIZON4_USD"), configured_usd))
    args.examples_ext = path_value(
        first_defined(
            args.examples_ext,
            cfg_get(scene_robot, "examples_ext"),
            cfg_get(environment, "examples_ext"),
            env_path("FLEXIV_EXAMPLES_EXT"),
            configured_examples_ext,
        )
    )
    args.camera_config = path_value(first_defined(args.camera_config, scene_config_path, args.config))
    args.output_root = path_value(
        first_defined(
            args.output_root,
            cfg_get(pipeline_config, "validation", "output_root"),
            cfg_get(environment, "record_output_root"),
            configured_output_root,
            env_path("FLEXIV_STAGE1_OUTPUT_ROOT"),
            DEFAULT_OUTPUT_ROOT,
        )
    )
    args.lerobot_output_root = path_value(
        first_defined(
            args.lerobot_output_root,
            cfg_get(pipeline_config, "convert", "output_root"),
            cfg_get(environment, "lerobot_output_root"),
            env_path("LEROBOT_OUTPUT_ROOT"),
            DEFAULT_LEROBOT_OUTPUT_ROOT,
        )
    )

    serial_value = first_defined(args.serial_number, cfg_get(scene_robot, "serial_number"), cfg_get(pipeline_config, "serial_number"))
    args.serial_number = "" if serial_value is None else str(serial_value)
    args.joint_group = str(first_defined(args.joint_group, cfg_get(scene_robot, "joint_group"), cfg_get(pipeline_config, "joint_group"), "ARM_1"))
    args.robot_prim_path = str(first_defined(cfg_get(scene_robot, "prim_path"), "/World/Flexiv/Rizon4"))
    args.scene_camera_names = [str(camera.get("name", f"cam_{idx}")) for idx, camera in enumerate(cfg_get(scene_config, "cameras") or cfg_get(pipeline_config, "cameras") or [])]
    args.repo_id_prefix = str(
        first_defined(args.repo_id_prefix, cfg_get(pipeline_config, "convert", "repo_id"), "qiming/quest_isaac_flexiv_stage1_single_rizon4_real")
    )
    args.action_mode = str(first_defined(args.action_mode, cfg_get(pipeline_config, "convert", "action_mode"), "qpos"))

    args.run_name_prefix = str(
        first_defined(
            cfg_get(pipeline_config, "validation", "run_name_prefix"),
            cfg_get(pipeline_config, "record", "task_name"),
            "quest_isaac_flexiv_stage1_single_rizon4_real",
        )
    )
    args.record_frames = int(first_defined(args.record_frames, cfg_get(pipeline_config, "record", "max_frames"), 120))
    args.record_fps = float(first_defined(args.record_fps, cfg_get(pipeline_config, "record", "fps"), 10.0))
    args.image_size = str(first_defined(args.image_size, cfg_get(pipeline_config, "record", "image_size"), "640x480"))

    args.sample_endpoint = str(first_defined(args.sample_endpoint, cfg_get(pipeline_config, "gateway", "sample_endpoint"), DEFAULT_SAMPLE_ENDPOINT))
    args.bridge_endpoint = str(first_defined(args.bridge_endpoint, cfg_get(pipeline_config, "gateway", "bridge_endpoint"), DEFAULT_BRIDGE_ENDPOINT))
    args.gateway_fps = float(first_defined(args.gateway_fps, cfg_get(pipeline_config, "gateway", "fps"), 15.0))
    args.gateway_jpeg_quality = int(first_defined(args.gateway_jpeg_quality, cfg_get(pipeline_config, "gateway", "jpeg_quality"), 90))

    args.target_pose_udp_host = str(
        first_defined(args.target_pose_udp_host, cfg_get(pipeline_config, "target_pose", "host"), DEFAULT_TARGET_POSE_UDP_HOST)
    )
    args.target_pose_udp_port = int(
        first_defined(args.target_pose_udp_port, cfg_get(pipeline_config, "target_pose", "port"), DEFAULT_TARGET_POSE_UDP_PORT)
    )
    args.quest_target_udp_host = str(
        first_defined(
            args.quest_target_udp_host,
            cfg_get(pipeline_config, "quest_target", "host"),
            cfg_get(pipeline_config, "fake_sender", "host"),
            DEFAULT_QUEST_TARGET_UDP_HOST,
        )
    )
    args.quest_target_udp_port = int(
        first_defined(
            args.quest_target_udp_port,
            cfg_get(pipeline_config, "quest_target", "port"),
            cfg_get(pipeline_config, "fake_sender", "port"),
            DEFAULT_QUEST_TARGET_UDP_PORT,
        )
    )

    args.fake_host = str(first_defined(args.fake_host, cfg_get(pipeline_config, "fake_sender", "host"), args.quest_target_udp_host))
    args.fake_side = str(first_defined(args.fake_side, cfg_get(pipeline_config, "fake_sender", "side"), "right"))
    args.fake_axis = str(first_defined(args.fake_axis, cfg_get(pipeline_config, "fake_sender", "axis"), "x"))
    args.fake_quat_wxyz = str(first_defined(args.fake_quat_wxyz, cfg_get(pipeline_config, "fake_sender", "quat_wxyz"), "0.0,0.70710678,0.0,0.70710678"))
    args.fake_amplitude_m = float(first_defined(args.fake_amplitude_m, cfg_get(pipeline_config, "fake_sender", "amplitude_m"), 0.02))
    args.fake_frames = int(first_defined(args.fake_frames, cfg_get(pipeline_config, "fake_sender", "frames"), 900))
    args.fake_rate_hz = float(first_defined(args.fake_rate_hz, cfg_get(pipeline_config, "fake_sender", "rate_hz"), 30.0))
    args.min_left_q_delta = float(first_defined(args.min_left_q_delta, cfg_get(pipeline_config, "validation", "min_left_q_delta"), 0.005))
    args.min_left_torque_norm = float(first_defined(args.min_left_torque_norm, cfg_get(pipeline_config, "validation", "min_left_torque_norm"), 1e-8))
    return args


class RealValidationRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.stamp = time.strftime("%Y%m%d_%H%M%S")
        self.run_root = args.output_root.expanduser().resolve() / f"{args.run_name_prefix}_{self.stamp}"
        self.raw_dir = self.run_root / "raw"
        self.log_dir = self.run_root / "logs"
        self.repo_id = f"{args.repo_id_prefix}_{self.stamp}"
        self.dataset_root = args.lerobot_output_root.expanduser().resolve() / self.repo_id
        self.report_path = self.run_root / "stage1_single_rizon4_real_validation.json"
        self.summary_path = self.run_root / "stage1_single_rizon4_real_summary.json"
        self.sample_endpoint = args.sample_endpoint
        self.bridge_endpoint = args.bridge_endpoint
        self.sample_host, self.sample_port = parse_tcp_endpoint(self.sample_endpoint)
        self.bridge_host, self.bridge_port = parse_tcp_endpoint(self.bridge_endpoint)
        self.sample_connect_host = endpoint_connect_host(self.sample_host)
        self.bridge_connect_host = endpoint_connect_host(self.bridge_host)
        self.processes: dict[str, subprocess.Popen] = {}
        self.logs: dict[str, Path] = {}

    def prepare(self) -> None:
        if not self.args.serial_number:
            raise RuntimeError("serial_number is not configured; set scene.robot.serial_number or pass --serial-number")
        assert_required_path(self.args.rdk_python, "rdk_python", executable=True)
        assert_required_path(self.args.isaac_python, "isaac_python", executable=True)
        assert_required_path(self.args.isaacsim_root, "isaacsim_root", directory=True)
        assert_required_path(self.args.usd, "scene robot USD")
        assert_required_path(self.args.examples_ext, "scene examples_ext", directory=True)
        assert_required_path(self.args.camera_config, "scene_config/camera_config")
        assert_tcp_port_free(self.sample_host, self.sample_port)
        assert_tcp_port_free(self.bridge_host, self.bridge_port)
        assert_udp_port_free(self.args.target_pose_udp_host, self.args.target_pose_udp_port)
        assert_udp_port_free(self.args.quest_target_udp_host, self.args.quest_target_udp_port)
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        json_print(
            {
                "event": "run_root",
                "run_root": self.run_root,
                "raw_dir": self.raw_dir,
                "dataset_root": self.dataset_root,
                "serial_number": self.args.serial_number,
                "config": self.args.config,
                "environment_config": self.args.environment_config,
                "scene_config": self.args.scene_config,
                "scene_robot_prim_path": self.args.robot_prim_path,
                "scene_cameras": self.args.scene_camera_names,
                "sample_endpoint": self.sample_endpoint,
                "bridge_endpoint": self.bridge_endpoint,
                "target_pose_udp": f"{self.args.target_pose_udp_host}:{self.args.target_pose_udp_port}",
                "quest_target_udp": f"{self.args.quest_target_udp_host}:{self.args.quest_target_udp_port}",
            }
        )

    def start(self, name: str, command: list[str | Path], *, env: dict[str, str] | None = None) -> subprocess.Popen:
        log_path = self.log_dir / f"{name}.log"
        self.logs[name] = log_path
        full_env = os.environ.copy()
        full_env["PYTHONUNBUFFERED"] = "1"
        if env:
            full_env.update(env)
        json_print({"event": "start", "name": name, "log": log_path, "cmd": [str(item) for item in command]})
        log_file = log_path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            [str(item) for item in command],
            cwd=REPO_ROOT,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            text=True,
            env=full_env,
            start_new_session=True,
        )
        log_file.close()
        self.processes[name] = process
        json_print({"event": "started", "name": name, "pid": process.pid})
        return process

    def stop(self, name: str, *, timeout: float = 8.0) -> None:
        process = self.processes.get(name)
        if process is None or process.poll() is not None:
            return
        json_print({"event": "stop", "name": name, "pid": process.pid})
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return
            time.sleep(0.2)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def cleanup(self) -> None:
        for name in ("fake_sender", "recorder", "isaac_follow", "rdk_target_streamer", "stage1_gateway"):
            self.stop(name)

    def check_processes(self) -> None:
        required = {"stage1_gateway", "rdk_target_streamer", "isaac_follow"}
        for name, process in self.processes.items():
            returncode = process.poll()
            if returncode is not None and name in required:
                raise RuntimeError(f"{name} exited early rc={returncode}\n--- tail {self.logs[name]} ---\n{tail_log(self.logs[name])}")

    def wait_log(self, name: str, needles: list[str], *, timeout: float, any_one: bool = True) -> None:
        path = self.logs[name]
        deadline = time.monotonic() + float(timeout)
        seen: set[str] = set()
        last_report = 0.0
        while time.monotonic() < deadline:
            self.check_processes()
            text = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
            for needle in needles:
                if needle in text:
                    seen.add(needle)
            if (any_one and seen) or ((not any_one) and all(needle in seen for needle in needles)):
                json_print({"event": "wait_log_ok", "name": name, "seen": sorted(seen)})
                return
            now = time.monotonic()
            if now - last_report > 20.0:
                json_print({"event": "wait_log", "name": name, "needles": needles, "seen": sorted(seen), "tail": tail_log(path, 10)})
                last_report = now
            time.sleep(1.0)
        raise TimeoutError(f"Timed out waiting for {needles} in {path}\n--- tail ---\n{tail_log(path, 160)}")

    def run_checked(self, name: str, command: list[str | Path], *, timeout: float = 300.0) -> Path:
        log_path = self.log_dir / f"{name}.log"
        json_print({"event": "run", "name": name, "log": log_path, "cmd": [str(item) for item in command]})
        with log_path.open("w", encoding="utf-8") as log_file:
            result = subprocess.run(
                [str(item) for item in command],
                cwd=REPO_ROOT,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout,
            )
        if result.returncode != 0:
            raise RuntimeError(f"{name} failed rc={result.returncode}\n--- tail {log_path} ---\n{tail_log(log_path, 180)}")
        json_print({"event": "run_ok", "name": name, "tail": tail_log(log_path, 20)})
        return log_path

    def request_gateway_sample(self, client: JsonLineReqClient) -> dict[str, Any]:
        client.send_json({"type": "sample_request", "stamp_ns": time.time_ns()})
        reply = client.recv_json(timeout=5.0)
        if reply.get("type") == "error":
            raise RuntimeError(str(reply.get("error")))
        return reply

    def wait_valid_gateway_sample(self) -> dict[str, Any]:
        monitor = Stage1SampleMonitor(
            min_servo_cycle_delta=5,
            min_left_q_delta=float(self.args.min_probe_left_q_delta),
            min_left_torque_norm=float(self.args.min_left_torque_norm),
            expected_serial=self.args.serial_number,
        )
        client = JsonLineReqClient(self.sample_endpoint, timeout=5.0)
        deadline = time.monotonic() + float(self.args.probe_timeout_sec)
        last_report = 0.0
        try:
            while time.monotonic() < deadline:
                self.check_processes()
                try:
                    status = monitor.observe(self.request_gateway_sample(client))
                except Exception as exc:
                    status = {"ready": False, "error": str(exc)}
                if status.get("ready"):
                    json_print({"event": "valid_gateway_sample", "status": status})
                    return status
                now = time.monotonic()
                if now - last_report > 5.0:
                    json_print(
                        {
                            "event": "probe_gateway_sample",
                            "status": status,
                            "monitor": monitor.last_status,
                            "isaac_tail": tail_log(self.logs.get("isaac_follow", Path()), 8),
                        }
                    )
                    last_report = now
                time.sleep(0.2)
        finally:
            client.close()
        raise TimeoutError(
            "Gateway did not produce a valid fresh Stage1 single-arm sample\n"
            f"last_status={monitor.last_status}\n--- Isaac tail ---\n{tail_log(self.logs.get('isaac_follow', Path()), 120)}"
        )

    def run(self) -> dict[str, Any]:
        self.prepare()
        self.start(
            "stage1_gateway",
            [
                self.args.flexiv_python,
                REPO_ROOT / "scripts/start_data_gateway.py",
                "--backend",
                "bridge",
                "--sample-endpoint",
                self.sample_endpoint,
                "--bridge-endpoint",
                self.bridge_endpoint,
                "--fps",
                str(float(self.args.gateway_fps)),
                "--image-size",
                self.args.image_size,
                "--camera-keys",
                ",".join(STAGE1_CAMERA_KEYS),
            ],
        )
        wait_tcp(self.sample_connect_host, self.sample_port, timeout=20.0, check_processes=self.check_processes)
        wait_tcp(self.bridge_connect_host, self.bridge_port, timeout=20.0, check_processes=self.check_processes)

        self.start(
            "rdk_target_streamer",
            [
                self.args.rdk_python,
                REPO_ROOT / "scripts/rdk_target_streamer.py",
                "--host",
                self.args.target_pose_udp_host,
                "--port",
                str(int(self.args.target_pose_udp_port)),
                "--serial-number",
                self.args.serial_number,
                "--joint-group",
                self.args.joint_group,
                "--max-age-sec",
                "1.0",
                "--log-hz",
                "5",
            ],
        )
        self.start(
            "isaac_follow",
            [
                self.args.isaac_python,
                REPO_ROOT / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/follow_ball_with_studio.py",
                "--serial-number",
                self.args.serial_number,
                "--rdk-serial-number",
                self.args.serial_number,
                "--joint-group",
                self.args.joint_group,
                "--robot-prim-path",
                self.args.robot_prim_path,
                "--usd",
                self.args.usd,
                "--examples-ext",
                self.args.examples_ext,
                "--headless",
                "--smoke-test",
                "--max-frames",
                "3600",
                "--physics-hz",
                "30",
                "--render-hz",
                "10",
                "--control-source",
                "studio-bridge",
                "--enable-quest-target-udp",
                "--quest-target-udp-host",
                self.args.quest_target_udp_host,
                "--quest-target-udp-port",
                str(int(self.args.quest_target_udp_port)),
                "--quest-target-mode",
                "relative",
                "--quest-position-scale",
                "0.5",
                "--target-pose-udp-host",
                self.args.target_pose_udp_host,
                "--target-pose-udp-port",
                str(int(self.args.target_pose_udp_port)),
                "--target-pose-publish-hz",
                "30",
                "--rdk-target-hz",
                "30",
                "--command-timeout-ms",
                "1",
                "--target-drive-warmup-cycles",
                "2",
                "--target-drive-required-valid-cycles",
                "1",
                "--state-torque-log-hz",
                "2",
                "--gateway-endpoint",
                self.bridge_endpoint,
                "--gateway-fps",
                str(float(self.args.gateway_fps)),
                "--gateway-jpeg-quality",
                str(int(self.args.gateway_jpeg_quality)),
                "--scene-config",
                self.args.camera_config,
            ],
            env={"ISAACSIM_ROOT": str(self.args.isaacsim_root)},
        )
        self.wait_log(
            "isaac_follow",
            ["Quest target UDP listening", "Ready. control_source=studio-bridge"],
            timeout=float(self.args.startup_timeout_sec),
            any_one=False,
        )
        self.wait_log("isaac_follow", ["Stage1 gateway connected"], timeout=120.0)
        self.start(
            "fake_sender",
            [
                self.args.flexiv_python,
                REPO_ROOT / "scripts/fake_rizon4_quest_sender.py",
                "--host",
                self.args.fake_host,
                "--port",
                str(int(self.args.quest_target_udp_port)),
                "--serial-number",
                self.args.serial_number,
                "--joint-group",
                self.args.joint_group,
                "--side",
                self.args.fake_side,
                "--axis",
                self.args.fake_axis,
                "--amplitude-m",
                str(float(self.args.fake_amplitude_m)),
                "--frames",
                str(int(self.args.fake_frames)),
                "--rate-hz",
                str(float(self.args.fake_rate_hz)),
                "--quat-wxyz",
                self.args.fake_quat_wxyz,
            ],
        )
        self.wait_valid_gateway_sample()
        self.start(
            "recorder",
            [
                self.args.flexiv_python,
                REPO_ROOT / "scripts/record_unitree_json.py",
                "--gateway-endpoint",
                self.sample_endpoint,
                "--task-dir",
                self.raw_dir,
                "--fps",
                str(float(self.args.record_fps)),
                "--episodes",
                "1",
                "--image-size",
                self.args.image_size,
                "--max-frames",
                str(int(self.args.record_frames)),
                "--auto-start",
                "--task-goal",
                "quest-isaac-flexiv Stage1 strict single-Rizon4 real validation",
                "--task-desc",
                "Old Isaac TargetFrame/RDK/Studio single-arm loop with Stage1 Unitree JSON recording",
                "--task-steps",
                "fake quest target; single Isaac Rizon4; Stage1 gateway; Unitree JSON; LeRobot conversion; H264 validation",
            ],
        )
        recorder_rc = self.processes["recorder"].wait(timeout=max(120.0, self.args.record_frames / self.args.record_fps + 90.0))
        if recorder_rc != 0:
            raise RuntimeError(f"recorder failed rc={recorder_rc}\n--- tail ---\n{tail_log(self.logs['recorder'], 180)}")

        self.stop("fake_sender")
        self.stop("isaac_follow")
        self.stop("rdk_target_streamer")
        self.stop("stage1_gateway")

        data_jsons = sorted(self.raw_dir.glob("episode_*/data.json"))
        if not data_jsons:
            raise RuntimeError(f"No Unitree data.json produced under {self.raw_dir}")
        data_json = data_jsons[-1]
        self.run_checked(
            "convert_lerobot",
            [
                self.args.flexiv_python,
                REPO_ROOT / "scripts/convert_unitree_json_to_lerobot.py",
                "--raw-dir",
                self.raw_dir,
                "--repo-id",
                self.repo_id,
                "--output-root",
                self.args.lerobot_output_root,
                "--action-mode",
                self.args.action_mode,
                "--fps",
                str(float(self.args.record_fps)),
            ],
        )
        self.run_checked(
            "validate_artifacts",
            [
                self.args.flexiv_python,
                REPO_ROOT / "scripts/validate_data_artifacts.py",
                "--raw-dir",
                self.raw_dir,
                "--dataset-root",
                self.dataset_root,
                "--out",
                self.report_path,
                "--strict-single-arm",
                "--expected-serial",
                self.args.serial_number,
                "--min-left-q-delta",
                str(float(self.args.min_left_q_delta)),
                "--min-left-torque-norm",
                str(float(self.args.min_left_torque_norm)),
                "--min-servo-cycle-delta",
                "5",
            ],
        )
        config_report = {
            "pipeline": str(self.args.config),
            "environment": str(self.args.environment_config) if self.args.environment_config else None,
            "scene": str(self.args.scene_config) if self.args.scene_config else None,
            "serial_number": self.args.serial_number,
            "usd": str(self.args.usd),
            "examples_ext": str(self.args.examples_ext),
            "robot_prim_path": self.args.robot_prim_path,
            "cameras": self.args.scene_camera_names,
        }
        report_payload = json.loads(self.report_path.read_text(encoding="utf-8"))
        report_payload["config"] = config_report
        self.report_path.write_text(json.dumps(report_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        frames = json.loads(data_json.read_text(encoding="utf-8")).get("data") or []
        strict_summary = summarize_stage1_single_arm_frames(
            frames,
            expected_serial=self.args.serial_number,
            min_left_q_delta=float(self.args.min_left_q_delta),
            min_left_torque_norm=float(self.args.min_left_torque_norm),
            min_servo_cycle_delta=5,
        )
        mp4s = sorted(self.dataset_root.glob("videos/**/*.mp4"))
        acceptance = {
            "strict_stage1_single_arm_real_validation": True,
            "single_robot": True,
            "single_camera": True,
            "external_gateway_reused": False,
            "backend_required": EXPECTED_STAGE1_BACKEND,
            "unitree_json": str(data_json),
            "dataset_root": str(self.dataset_root),
            "mp4": str(mp4s[0]) if mp4s else None,
            "validation_report": str(self.report_path),
            "config": config_report,
            **strict_summary,
            "logs": {name: str(path) for name, path in self.logs.items()},
        }
        self.summary_path.write_text(json.dumps(acceptance, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        json_print({"event": "success", "summary": self.summary_path, "acceptance": acceptance})
        return acceptance


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    runner = RealValidationRunner(args)
    try:
        runner.run()
    except Exception as exc:
        json_print({"event": "error", "error": str(exc)})
        for name, path in runner.logs.items():
            json_print({"event": "log_tail", "name": name, "path": path, "tail": tail_log(path)})
        if not args.keep_running_on_failure:
            runner.cleanup()
        raise
    finally:
        if not args.keep_running_on_failure:
            runner.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
