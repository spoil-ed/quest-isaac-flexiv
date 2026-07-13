#!/usr/bin/env python3
"""Start the external RDK target-pose streamer."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


RDK_COMPAT_PATH = flexiv_runtime.REPO_ROOT / ".deps" / "flexivrdk_1_9_1"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=flexiv_runtime.ISAAC_PYTHON)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=45678)
    parser.add_argument("--serial-number", default="Rizon4-I0LIRN")
    parser.add_argument("--joint-group", default="ARM_1")
    parser.add_argument("--network-interface-whitelist", default="")
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--log-hz", type=float, default=2.0)
    args = parser.parse_args(argv)
    args.python = flexiv_runtime.python_executable_or_current(args.python)
    return args


def build_env(base_env: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(os.environ if base_env is None else base_env)
    if RDK_COMPAT_PATH.exists():
        current = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = str(RDK_COMPAT_PATH) + (":" + current if current else "")
    return env


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.python),
        str(flexiv_runtime.REPO_ROOT / "scripts" / "rdk_target_streamer.py"),
        "--host",
        str(args.host),
        "--port",
        str(args.port),
        "--serial-number",
        str(args.serial_number),
        "--joint-group",
        str(args.joint_group),
        "--max-age-sec",
        str(float(args.max_age_sec)),
        "--log-hz",
        str(float(args.log_hz)),
    ]
    if args.network_interface_whitelist:
        command.extend(["--network-interface-whitelist", str(args.network_interface_whitelist)])
    return command


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="rdk_target_streamer",
        env=build_env(),
    )
    flexiv_runtime.print_started("RDK_TARGET_STREAMER", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
