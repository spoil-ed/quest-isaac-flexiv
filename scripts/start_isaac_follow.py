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
    parser.add_argument("--rdk-serial-number", default=None)
    parser.add_argument("--joint-group", default=None)
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--robot-prim-path", default=None)
    parser.add_argument("--robot-name", default=None)
    parser.add_argument("--end-effector-prim-name", default=None)
    parser.add_argument("--usd", type=Path, default=None)
    parser.add_argument("--examples-ext", type=Path, default=None)
    parser.add_argument("--manual-play", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    parser.add_argument("--quest-target-udp-host", default=None)
    parser.add_argument("--quest-target-udp-port", type=int, default=None)
    parser.add_argument("--target-pose-udp-host", default=None)
    parser.add_argument("--target-pose-udp-port", type=int, default=None)
    parser.add_argument("--target-pose-publish-hz", type=float, default=None)
    parser.add_argument("--rdk-target-hz", type=float, default=None)
    parser.add_argument("--command-timeout-ms", type=int, default=None)
    parser.add_argument("--gateway-endpoint", default="")
    parser.add_argument("--gateway-fps", type=float, default=None)
    parser.add_argument("--gateway-jpeg-quality", type=int, default=None)
    parser.add_argument("--camera-config", type=Path, default=None)
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
    if args.rdk_serial_number is not None:
        command.extend(["--rdk-serial-number", str(args.rdk_serial_number)])
    if args.joint_group is not None:
        command.extend(["--joint-group", str(args.joint_group)])
    if args.scene_config is not None:
        command.extend(["--scene-config", str(args.scene_config)])
    if args.robot_prim_path is not None:
        command.extend(["--robot-prim-path", str(args.robot_prim_path)])
    if args.robot_name is not None:
        command.extend(["--robot-name", str(args.robot_name)])
    if args.end_effector_prim_name is not None:
        command.extend(["--end-effector-prim-name", str(args.end_effector_prim_name)])
    if args.usd is not None:
        command.extend(["--usd", str(args.usd)])
    if args.examples_ext is not None:
        command.extend(["--examples-ext", str(args.examples_ext)])
    if args.manual_play:
        command.append("--manual-play")
    if args.headless:
        command.append("--headless")
    if args.enable_quest_target_udp:
        command.append("--enable-quest-target-udp")
    if args.quest_target_udp_host is not None:
        command.extend(["--quest-target-udp-host", str(args.quest_target_udp_host)])
    if args.quest_target_udp_port is not None:
        command.extend(["--quest-target-udp-port", str(int(args.quest_target_udp_port))])
    if args.target_pose_udp_host is not None:
        command.extend(["--target-pose-udp-host", str(args.target_pose_udp_host)])
    if args.target_pose_udp_port is not None:
        command.extend(["--target-pose-udp-port", str(int(args.target_pose_udp_port))])
    if args.target_pose_publish_hz is not None:
        command.extend(["--target-pose-publish-hz", str(float(args.target_pose_publish_hz))])
    if args.rdk_target_hz is not None:
        command.extend(["--rdk-target-hz", str(float(args.rdk_target_hz))])
    if args.command_timeout_ms is not None:
        command.extend(["--command-timeout-ms", str(int(args.command_timeout_ms))])
    if args.gateway_endpoint:
        command.extend(["--gateway-endpoint", str(args.gateway_endpoint)])
    if args.gateway_fps is not None:
        command.extend(["--gateway-fps", str(float(args.gateway_fps))])
    if args.gateway_jpeg_quality is not None:
        command.extend(["--gateway-jpeg-quality", str(int(args.gateway_jpeg_quality))])
    if args.camera_config is not None:
        command.extend(["--camera-config", str(args.camera_config)])
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
