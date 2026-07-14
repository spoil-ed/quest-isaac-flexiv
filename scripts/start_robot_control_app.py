#!/usr/bin/env python3
"""Start RobotControlApp only."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


def _relative_to_studio_root(path: Path, studio_root: Path) -> str:
    try:
        return str(path.relative_to(studio_root))
    except ValueError:
        return str(path)


def _find_one(paths: list[Path], *, description: str) -> Path:
    if not paths:
        raise FileNotFoundError(f"Could not find {description} under Elements Studio")
    return sorted(paths, key=lambda path: path.stat().st_mtime, reverse=True)[0]


def discover_robot_control_args(studio_root: Path) -> dict[str, str]:
    root = Path(studio_root).expanduser()
    simulator_root = root / "user_data_ui" / "simDir" / "simulator0"
    param_path = _find_one(
        list(simulator_root.glob("*/arm_driver_param.xml")),
        description="simulator arm driver parameter file",
    )
    config_paths = list((root / "specs" / "robots").glob("*/flexivCfg.xml"))
    if config_paths:
        config_path = _find_one(config_paths, description="robot control config")
    else:
        # RobotControlApp unlocks the encrypted specs directory during startup.
        # On a true cold start the config therefore does not exist yet; infer the
        # official relative path from serials such as A02L-00-M6-I0LIRN.
        model = param_path.parent.name.split("-", 1)[0]
        if not model:
            raise FileNotFoundError("Could not infer robot model from simulator parameters")
        config_path = root / "specs" / "robots" / f"Flexiv{model}" / "flexivCfg.xml"
    return {
        "serial": param_path.parent.name,
        "config": _relative_to_studio_root(config_path, root),
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio-root", type=Path, default=flexiv_runtime.STUDIO_ROOT)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--control-box", default="CX01-02-P1-00034")
    parser.add_argument("--user-data", default="./user_data_ui//./simDir/simulator0/user_data/")
    parser.add_argument("--config", default=None)
    args = parser.parse_args(argv)
    if args.serial is None or args.config is None:
        discovered = discover_robot_control_args(args.studio_root)
        args.serial = args.serial or discovered["serial"]
        args.config = args.config or discovered["config"]
    return args


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
    existing_pid = flexiv_runtime.find_process_by_executable("RobotControlApp")
    if existing_pid is not None:
        flexiv_runtime.print_already_running("ROBOT_CONTROL_APP", existing_pid)
        return 0
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
