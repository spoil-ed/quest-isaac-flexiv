#!/usr/bin/env python3
"""Start Isaac's dual-Flexiv Quest target-frame following scene."""

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
    parser.add_argument("--scene-config", type=Path, default=None)
    parser.add_argument("--left-serial-number", default=None)
    parser.add_argument("--right-serial-number", default=None)
    parser.add_argument("--joint-group", default=None)
    parser.add_argument("--usd", type=Path, default=None)
    parser.add_argument("--examples-ext", type=Path, default=None)
    parser.add_argument("--manual-play", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--physics-hz", type=float, default=None)
    parser.add_argument("--render-hz", type=float, default=None)
    parser.add_argument("--enable-quest-target-udp", action="store_true")
    parser.add_argument("--quest-target-udp-host", default=None)
    parser.add_argument("--quest-target-udp-port", type=int, default=None)
    parser.add_argument("--quest-target-max-age-sec", type=float, default=None)
    parser.add_argument("--quest-target-mode", choices=("absolute", "relative"), default="relative")
    parser.add_argument("--quest-relative-orientation-mode", choices=("packet", "reference", "current"), default="packet")
    parser.add_argument("--quest-axis-map", default=None)
    parser.add_argument("--quest-position-scale", type=float, default=1.0)
    parser.add_argument("--quest-position-deadband-m", type=float, default=None)
    parser.add_argument("--quest-workspace-min", default=None)
    parser.add_argument("--quest-workspace-max", default=None)
    parser.add_argument("--left-target-pose-udp-host", default=None)
    parser.add_argument("--left-target-pose-udp-port", type=int, default=None)
    parser.add_argument("--right-target-pose-udp-host", default=None)
    parser.add_argument("--right-target-pose-udp-port", type=int, default=None)
    parser.add_argument("--target-pose-publish-hz", type=float, default=None)
    parser.add_argument("--command-timeout-ms", type=int, default=None)
    parser.add_argument("--max-linear-speed-m-s", type=float, default=None)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=None)
    parser.add_argument("--max-joint-speed-rad-s", type=float, default=None)
    parser.add_argument("--max-target-drive-abs", type=float, default=None)
    parser.add_argument("--max-target-drive-norm", type=float, default=None)
    parser.add_argument("--target-drive-scale", type=float, default=None)
    parser.add_argument("--gateway-endpoint", default="")
    parser.add_argument("--gateway-fps", type=float, default=None)
    parser.add_argument("--gateway-jpeg-quality", type=int, default=None)
    parser.add_argument("--coordinated-reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    args = parser.parse_args(argv)
    args.isaac_python = flexiv_runtime.python_executable_or_current(args.isaac_python)
    return args


def _maybe_extend(command: list[str], option: str, value) -> None:
    if value is not None:
        command.extend([option, str(value)])


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.isaac_python),
        str(flexiv_runtime.FLEXIV_DUAL_QUEST_FOLLOW),
        "--quest-target-mode",
        str(args.quest_target_mode),
        "--quest-relative-orientation-mode",
        str(args.quest_relative_orientation_mode),
        "--quest-position-scale",
        str(float(args.quest_position_scale)),
    ]
    for option, value in (
        ("--scene-config", args.scene_config),
        ("--left-serial-number", args.left_serial_number),
        ("--right-serial-number", args.right_serial_number),
        ("--joint-group", args.joint_group),
        ("--usd", args.usd),
        ("--examples-ext", args.examples_ext),
    ):
        _maybe_extend(command, option, value)
    if args.manual_play:
        command.append("--manual-play")
    if args.headless:
        command.append("--headless")
    for option, value in (
        ("--physics-hz", args.physics_hz),
        ("--render-hz", args.render_hz),
        ("--quest-target-udp-host", args.quest_target_udp_host),
        ("--quest-target-udp-port", args.quest_target_udp_port),
        ("--quest-target-max-age-sec", args.quest_target_max_age_sec),
        ("--quest-axis-map", args.quest_axis_map),
        ("--quest-position-deadband-m", args.quest_position_deadband_m),
        ("--quest-workspace-min", args.quest_workspace_min),
        ("--quest-workspace-max", args.quest_workspace_max),
        ("--left-target-pose-udp-host", args.left_target_pose_udp_host),
        ("--left-target-pose-udp-port", args.left_target_pose_udp_port),
        ("--right-target-pose-udp-host", args.right_target_pose_udp_host),
        ("--right-target-pose-udp-port", args.right_target_pose_udp_port),
        ("--target-pose-publish-hz", args.target_pose_publish_hz),
        ("--command-timeout-ms", args.command_timeout_ms),
        ("--max-linear-speed-m-s", args.max_linear_speed_m_s),
        ("--max-angular-speed-rad-s", args.max_angular_speed_rad_s),
        ("--max-joint-speed-rad-s", args.max_joint_speed_rad_s),
        ("--max-target-drive-abs", args.max_target_drive_abs),
        ("--max-target-drive-norm", args.max_target_drive_norm),
        ("--target-drive-scale", args.target_drive_scale),
        ("--gateway-fps", args.gateway_fps),
        ("--gateway-jpeg-quality", args.gateway_jpeg_quality),
    ):
        _maybe_extend(command, option, value)
    if args.enable_quest_target_udp:
        command.append("--enable-quest-target-udp")
    if args.gateway_endpoint:
        command.extend(["--gateway-endpoint", str(args.gateway_endpoint)])
    command.append("--coordinated-reset" if args.coordinated_reset else "--no-coordinated-reset")
    command.extend(["--reset-settle-sec", str(float(args.reset_settle_sec))])
    return command


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="isaac_dual_follow_target",
    )
    flexiv_runtime.print_started("ISAAC_DUAL_FOLLOW", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
