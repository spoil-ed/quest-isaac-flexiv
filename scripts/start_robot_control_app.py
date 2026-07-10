#!/usr/bin/env python3
"""Start RobotControlApp only."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio-root", type=Path, default=flexiv_runtime.STUDIO_ROOT)
    parser.add_argument("--serial", default="A02L-00-M6-I0LIRN")
    parser.add_argument("--control-box", default="CX01-02-P1-00034")
    parser.add_argument("--user-data", default="./user_data_ui//./simDir/simulator0/user_data/")
    parser.add_argument("--config", default="./specs//robots/FlexivA02L/flexivCfg.xml")
    return parser.parse_args(argv)


def build_command(args: argparse.Namespace) -> list[str]:
    return [
        "./RobotControlApp",
        "-u",
        str(args.user_data),
        "-c",
        str(args.config),
        "-m",
        "MotionBarSimulation",
        "-s",
        str(args.serial),
        "-x",
        str(args.control_box),
        "-n",
        "-g",
    ]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=args.studio_root,
        log_prefix="robot_control_app",
        env=flexiv_runtime.studio_env(args.studio_root),
    )
    flexiv_runtime.print_started("ROBOT_CONTROL_APP", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
