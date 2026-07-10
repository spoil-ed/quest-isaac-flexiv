#!/usr/bin/env python3
"""Start FlexivSimulation only."""

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
    parser.add_argument(
        "--robot-urdf",
        default="./user_data_ui//./simDir/simulator0/user_data/settings/generated_robot_A02L-M6_abs_path.urdf",
    )
    parser.add_argument(
        "--robot-srdf",
        default="./user_data_ui//./simDir/simulator0/user_data/settings/generated_robot_A02L-M6_abs_path.srdf",
    )
    parser.add_argument(
        "--scene-urdf",
        default="./user_data_ui//./simDir/simulator0/user_data/settings//user_scene_abs_path.urdf",
    )
    parser.add_argument(
        "--param",
        default="./user_data_ui//./simDir/simulator0/A02L-00-M6-I0LIRN/arm_driver_param.xml",
    )
    parser.add_argument("--group-state", default="home")
    return parser.parse_args(argv)


def build_command(args: argparse.Namespace) -> list[str]:
    return [
        "./FlexivSimulation",
        "--robot_urdf",
        str(args.robot_urdf),
        "--robot_srdf",
        str(args.robot_srdf),
        "--scene_urdf",
        str(args.scene_urdf),
        "--param",
        str(args.param),
        "--group_state",
        str(args.group_state),
    ]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=args.studio_root,
        log_prefix="flexiv_simulation",
        env=flexiv_runtime.studio_env(args.studio_root),
    )
    flexiv_runtime.print_started("FLEXIV_SIMULATION", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
