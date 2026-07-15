#!/usr/bin/env python3
"""Start the host-side Flexiv DRDK dual-arm target streamer."""

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
    parser.add_argument("--left-port", type=int, default=57680)
    parser.add_argument("--right-port", type=int, default=57681)
    parser.add_argument("--left-serial-number", default="Rizon4-qSaFLh")
    parser.add_argument("--right-serial-number", default="Rizon4-I0LIRN")
    parser.add_argument("--joint-group", default="ARM_1")
    parser.add_argument("--left-status-host", default="127.0.0.1")
    parser.add_argument("--left-status-port", type=int, default=57682)
    parser.add_argument("--right-status-host", default="127.0.0.1")
    parser.add_argument("--right-status-port", type=int, default=57683)
    parser.add_argument("--left-translation-in-world", default="0,0,0")
    parser.add_argument("--right-translation-in-world", default="0,0,0")
    parser.add_argument("--left-nullspace-posture", default="")
    parser.add_argument("--right-nullspace-posture", default="")
    parser.add_argument("--nullspace-tracking-weight", type=float, default=0.5)
    parser.add_argument("--network-interface-whitelist", default="")
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--enable-timeout-sec", type=float, default=15.0)
    parser.add_argument("--max-linear-speed-m-s", type=float, default=0.5)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=0.75)
    parser.add_argument("--max-linear-acc-m-s2", type=float, default=2.0)
    parser.add_argument("--max-angular-acc-rad-s2", type=float, default=5.0)
    parser.add_argument("--clear-fault", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--strict-clear-fault", action=argparse.BooleanOptionalAction, default=True)
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
        str(flexiv_runtime.REPO_ROOT / "scripts" / "drdk_target_streamer.py"),
        "--host",
        str(args.host),
        "--left-port",
        str(args.left_port),
        "--right-port",
        str(args.right_port),
        "--left-serial-number",
        str(args.left_serial_number),
        "--right-serial-number",
        str(args.right_serial_number),
        "--joint-group",
        str(args.joint_group),
        "--left-status-host",
        str(args.left_status_host),
        "--left-status-port",
        str(args.left_status_port),
        "--right-status-host",
        str(args.right_status_host),
        "--right-status-port",
        str(args.right_status_port),
        "--left-translation-in-world",
        str(args.left_translation_in_world),
        "--right-translation-in-world",
        str(args.right_translation_in_world),
        "--nullspace-tracking-weight",
        str(args.nullspace_tracking_weight),
        "--max-age-sec",
        str(args.max_age_sec),
        "--enable-timeout-sec",
        str(args.enable_timeout_sec),
        "--max-linear-speed-m-s",
        str(args.max_linear_speed_m_s),
        "--max-angular-speed-rad-s",
        str(args.max_angular_speed_rad_s),
        "--max-linear-acc-m-s2",
        str(args.max_linear_acc_m_s2),
        "--max-angular-acc-rad-s2",
        str(args.max_angular_acc_rad_s2),
        "--log-hz",
        str(args.log_hz),
        "--clear-fault" if args.clear_fault else "--no-clear-fault",
        "--strict-clear-fault" if args.strict_clear_fault else "--no-strict-clear-fault",
    ]
    for option, value in (
        ("--left-nullspace-posture", args.left_nullspace_posture),
        ("--right-nullspace-posture", args.right_nullspace_posture),
        ("--network-interface-whitelist", args.network_interface_whitelist),
    ):
        if value:
            command.extend([option, str(value)])
    return command


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="drdk_target_streamer_dual",
        env=build_env(),
    )
    flexiv_runtime.print_started("DRDK_TARGET_STREAMER", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
