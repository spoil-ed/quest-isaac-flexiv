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
    parser.add_argument("--target-pose-udp-host", default=None)
    parser.add_argument("--target-pose-udp-port", type=int, default=None)
    parser.add_argument("--target-pose-publish-hz", type=float, default=None)
    parser.add_argument("--rdk-target-hz", type=float, default=None)
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
    parser.add_argument("--camera-config", type=Path, default=None)
    parser.add_argument("--coordinated-reset", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--reset-settle-sec", type=float, default=2.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=20.0)
    parser.add_argument("--reset-position-tolerance-m", type=float, default=0.01)
    parser.add_argument("--reset-angular-tolerance-rad", type=float, default=0.10)
    parser.add_argument("--reset-joint-speed-tolerance-rad-s", type=float, default=0.05)
    args = parser.parse_args(argv)
    args.isaac_python = flexiv_runtime.python_executable_or_current(args.isaac_python)
    return args


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.isaac_python),
        str(flexiv_runtime.FLEXIV_QUEST_FOLLOW),
        "--control-source",
        "studio-bridge",
        "--serial-number",
        str(args.serial_number),
        "--quest-target-mode",
        str(args.quest_target_mode),
        "--quest-relative-orientation-mode",
        str(args.quest_relative_orientation_mode),
        "--quest-position-scale",
        str(float(args.quest_position_scale)),
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
    if args.physics_hz is not None:
        command.extend(["--physics-hz", str(float(args.physics_hz))])
    if args.render_hz is not None:
        command.extend(["--render-hz", str(float(args.render_hz))])
    if args.enable_quest_target_udp:
        command.append("--enable-quest-target-udp")
    if args.quest_target_udp_host is not None:
        command.extend(["--quest-target-udp-host", str(args.quest_target_udp_host)])
    if args.quest_target_udp_port is not None:
        command.extend(["--quest-target-udp-port", str(int(args.quest_target_udp_port))])
    if args.quest_target_max_age_sec is not None:
        command.extend(["--quest-target-max-age-sec", str(float(args.quest_target_max_age_sec))])
    if args.quest_axis_map is not None:
        command.extend(["--quest-axis-map", str(args.quest_axis_map)])
    if args.quest_position_deadband_m is not None:
        command.extend(["--quest-position-deadband-m", str(float(args.quest_position_deadband_m))])
    if args.quest_workspace_min is not None:
        command.extend(["--quest-workspace-min", str(args.quest_workspace_min)])
    if args.quest_workspace_max is not None:
        command.extend(["--quest-workspace-max", str(args.quest_workspace_max)])
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
    for option, value in (
        ("--max-linear-speed-m-s", args.max_linear_speed_m_s),
        ("--max-angular-speed-rad-s", args.max_angular_speed_rad_s),
        ("--max-joint-speed-rad-s", args.max_joint_speed_rad_s),
        ("--max-target-drive-abs", args.max_target_drive_abs),
        ("--max-target-drive-norm", args.max_target_drive_norm),
        ("--target-drive-scale", args.target_drive_scale),
    ):
        if value is not None:
            command.extend([option, str(float(value))])
    if args.gateway_endpoint:
        command.extend(["--gateway-endpoint", str(args.gateway_endpoint)])
    if args.gateway_fps is not None:
        command.extend(["--gateway-fps", str(float(args.gateway_fps))])
    if args.gateway_jpeg_quality is not None:
        command.extend(["--gateway-jpeg-quality", str(int(args.gateway_jpeg_quality))])
    if args.camera_config is not None:
        command.extend(["--camera-config", str(args.camera_config)])
    command.append("--coordinated-reset" if args.coordinated_reset else "--no-coordinated-reset")
    command.extend(["--reset-settle-sec", str(float(args.reset_settle_sec))])
    command.extend(["--reset-timeout-sec", str(float(args.reset_timeout_sec))])
    command.extend(["--reset-position-tolerance-m", str(float(args.reset_position_tolerance_m))])
    command.extend(["--reset-angular-tolerance-rad", str(float(args.reset_angular_tolerance_rad))])
    command.extend(
        ["--reset-joint-speed-tolerance-rad-s", str(float(args.reset_joint_speed_tolerance_rad_s))]
    )
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
