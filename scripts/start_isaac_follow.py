#!/usr/bin/env python3
"""Start Isaac's Flexiv Quest ball-following scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isaac-python", type=Path, default=flexiv_runtime.ISAAC_PYTHON)
    parser.add_argument("--studio-grpc-address", default="127.0.0.1:18001")
    parser.add_argument("--physics-hz", type=float, default=None, help="Default omitted; script uses 2000 Hz.")
    parser.add_argument("--render-hz", type=float, default=60.0)
    parser.add_argument("--studio-jog-hz", type=float, default=30.0)
    parser.add_argument("--state-torque-log-hz", type=float, default=5.0)
    parser.add_argument("--command-timeout-ms", type=int, default=5)
    parser.add_argument("--target-drive-warmup-cycles", type=int, default=2000)
    parser.add_argument("--target-drive-required-valid-cycles", type=int, default=50)
    parser.add_argument("--max-target-drive-norm", type=float, default=200.0)
    parser.add_argument("--manual-play", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    return parser.parse_args(argv)


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.isaac_python),
        str(flexiv_runtime.FLEXIV_QUEST_FOLLOW),
        "--control-source",
        "studio-jog",
        "--render-hz",
        f"{float(args.render_hz):g}",
        "--studio-jog-hz",
        f"{float(args.studio_jog_hz):g}",
        "--studio-jog-position-deadband",
        "0.005",
        "--studio-jog-max-step-size",
        "0.01",
        "--studio-jog-vel-scale",
        "0.4",
        "--studio-grpc-address",
        str(args.studio_grpc_address),
        "--state-torque-log-hz",
        f"{float(args.state_torque_log_hz):g}",
        "--command-timeout-ms",
        str(int(args.command_timeout_ms)),
        "--target-drive-warmup-cycles",
        str(int(args.target_drive_warmup_cycles)),
        "--target-drive-required-valid-cycles",
        str(int(args.target_drive_required_valid_cycles)),
        "--max-target-drive-norm",
        f"{float(args.max_target_drive_norm):g}",
        "--initial-q",
        *flexiv_runtime.DEFAULT_INITIAL_Q,
    ]
    if args.physics_hz is not None:
        command.extend(["--physics-hz", f"{float(args.physics_hz):g}"])
    if args.manual_play:
        command.append("--manual-play")
    if args.headless:
        command.append("--headless")
    if args.enable_quest_target_udp:
        command.append("--enable-quest-target-udp")
    return command


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="isaac_follow_ball",
    )
    flexiv_runtime.print_started("ISAAC_FOLLOW", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
