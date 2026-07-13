#!/usr/bin/env python3
"""Shared helpers for the local Flexiv/Isaac runtime scripts."""

from __future__ import annotations

import os
import signal
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = REPO_ROOT / "logs"
DEFAULT_ENVIRONMENT_CONFIG = Path(
    os.environ.get("FLEXIV_ENVIRONMENT_CONFIG", REPO_ROOT / "configs/environments/local_flexiv_runtime.yaml")
)


def load_environment_config(path: Path = DEFAULT_ENVIRONMENT_CONFIG) -> dict[str, str]:
    config_path = Path(path).expanduser()
    if not config_path.exists():
        return {}
    try:
        import yaml
    except ImportError:
        return {}
    data = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    return data if isinstance(data, dict) else {}


_ENVIRONMENT_CONFIG = load_environment_config()
ISAAC_PYTHON = Path(os.environ.get("ISAAC_PYTHON") or _ENVIRONMENT_CONFIG.get("isaac_python", "python"))
STUDIO_ROOT = Path(os.environ.get("STUDIO_ROOT") or _ENVIRONMENT_CONFIG.get("studio_root", "FlexivElementsStudio"))
FLEXIV_QUEST_FOLLOW = (
    REPO_ROOT
    / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/follow_ball_with_studio.py"
)
FLEXIV_QUEST_BRIDGE = (
    REPO_ROOT
    / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/flexiv_isaac_bridge_app.py"
)
FLEXIV_QUEST_BRIDGE_CONFIG = (
    REPO_ROOT
    / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/app_config.yaml"
)
DEFAULT_INITIAL_Q = ["0", "-0.698132", "0", "1.5708", "0", "0.698132", "0"]


def python_executable_or_current(value: str | Path | None) -> Path:
    """Use the active interpreter when an empty CLI value became Path('.')."""
    if value is None or str(value).strip() in ("", "."):
        return Path(sys.executable)
    return Path(value).expanduser()


def studio_env(studio_root: Path = STUDIO_ROOT) -> dict[str, str]:
    root = Path(studio_root).expanduser()
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = str(root / "lib") + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    env["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(root / "plugins")
    env["QTWEBENGINE_DISABLE_SANDBOX"] = "1"
    env["PATH"] = str(root / "bin") + (":" + env["PATH"] if env.get("PATH") else "")
    return env


def timestamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def start_background(
    command: list[str],
    *,
    cwd: Path,
    log_prefix: str,
    env: dict[str, str] | None = None,
) -> tuple[int, Path, Path]:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = timestamp()
    stdout_path = LOG_DIR / f"{log_prefix}_{stamp}.stdout.log"
    stderr_path = LOG_DIR / f"{log_prefix}_{stamp}.stderr.log"
    stdout_file = stdout_path.open("w", encoding="utf-8")
    stderr_file = stderr_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [str(item) for item in command],
        cwd=Path(cwd),
        env=env,
        stdout=stdout_file,
        stderr=stderr_file,
        start_new_session=True,
    )
    stdout_file.close()
    stderr_file.close()
    return process.pid, stdout_path, stderr_path


def pgrep_commands() -> list[tuple[int, str]]:
    result = subprocess.run(["pgrep", "-af", "."], text=True, capture_output=True, check=False)
    rows: list[tuple[int, str]] = []
    for line in result.stdout.splitlines():
        parts = line.split(maxsplit=1)
        if len(parts) == 2 and parts[0].isdigit():
            rows.append((int(parts[0]), parts[1]))
    return rows


def find_process_by_executable(executable_name: str) -> int | None:
    """Return an existing process whose argv[0] has the requested basename."""

    expected = str(executable_name)
    for pid, command in pgrep_commands():
        try:
            argv = shlex.split(command)
        except ValueError:
            continue
        if argv and Path(argv[0]).name == expected:
            return pid
    return None


def print_already_running(label: str, pid: int) -> None:
    print(f"{label}_PID={int(pid)}", flush=True)
    print(f"{label}_ALREADY_RUNNING=1", flush=True)


def process_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def stop_matching(needles: list[str], *, timeout: float = 8.0) -> list[int]:
    own_pid = os.getpid()
    targets = [
        (pid, command)
        for pid, command in pgrep_commands()
        if pid != own_pid
        and "pgrep -af" not in command
        and not command.startswith("/bin/bash -lc")
        and any(needle in command for needle in needles)
    ]
    for pid, _command in targets:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        if not any(process_exists(pid) for pid, _command in targets):
            return [pid for pid, _command in targets]
        time.sleep(0.2)
    for pid, _command in targets:
        if process_exists(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return [pid for pid, _command in targets]


def wait_port(host: str, port: int, *, timeout: float) -> bool:
    deadline = time.monotonic() + float(timeout)
    while time.monotonic() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            if sock.connect_ex((host, int(port))) == 0:
                return True
        time.sleep(0.5)
    return False


def print_started(label: str, pid: int, stdout_path: Path, stderr_path: Path) -> None:
    print(f"{label}_PID={pid}", flush=True)
    print(f"{label}_STDOUT={stdout_path}", flush=True)
    print(f"{label}_STDERR={stderr_path}", flush=True)
