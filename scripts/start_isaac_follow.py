#!/usr/bin/env python3
"""Start Isaac's Flexiv Quest ball-following scene."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
RDK_COMPAT_PATH = flexiv_runtime.REPO_ROOT / ".deps" / "flexivrdk_1_9_1"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-python", type=Path, default=flexiv_runtime.ISAAC_PYTHON)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--rdk-target-hz", type=float, default=30.0)
    parser.add_argument("--rdk-network-interface-whitelist", default="")
    parser.add_argument("--manual-play", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    return parser.parse_args(argv)


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.isaac_python),
        str(flexiv_runtime.FLEXIV_QUEST_FOLLOW),
        "--control-source",
        "rdk-cartesian",
        "--serial-number",
        str(args.serial_number),
        "--rdk-target-hz",
        f"{float(args.rdk_target_hz):g}",
        "--initial-q",
        *flexiv_runtime.DEFAULT_INITIAL_Q,
    ]
    if args.rdk_network_interface_whitelist:
        command.extend(["--rdk-network-interface-whitelist", str(args.rdk_network_interface_whitelist)])
    if args.manual_play:
        command.append("--manual-play")
    if args.headless:
        command.append("--headless")
    if args.enable_quest_target_udp:
        command.append("--enable-quest-target-udp")
    return command


def build_env(base_env: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(os.environ if base_env is None else base_env)
    if RDK_COMPAT_PATH.exists():
        current = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = str(RDK_COMPAT_PATH) + (":" + current if current else "")
    return env


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="isaac_follow_ball",
        env=build_env(),
    )
    flexiv_runtime.print_started("ISAAC_FOLLOW", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
