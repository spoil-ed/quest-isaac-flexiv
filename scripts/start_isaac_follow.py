#!/usr/bin/env python3
"""Start Isaac's Flexiv Quest target-frame following scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-python", type=Path, default=flexiv_runtime.ISAAC_PYTHON)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--manual-play", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    parser.add_argument("--rdk-target-hz", type=float, default=None)
    return parser.parse_args(argv)


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.isaac_python),
        str(flexiv_runtime.FLEXIV_QUEST_FOLLOW),
        "--control-source",
        "studio-bridge",
        "--serial-number",
        str(args.serial_number),
        "--quest-target-mode",
        "relative",
        "--quest-position-scale",
        "1.0",
        "--initial-q",
        *flexiv_runtime.DEFAULT_INITIAL_Q,
    ]
    if args.manual_play:
        command.append("--manual-play")
    if args.headless:
        command.append("--headless")
    if args.enable_quest_target_udp:
        command.append("--enable-quest-target-udp")
    if args.rdk_target_hz is not None:
        command.extend(["--rdk-target-hz", str(float(args.rdk_target_hz))])
    return command


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="isaac_follow_target",
    )
    flexiv_runtime.print_started("ISAAC_FOLLOW", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
