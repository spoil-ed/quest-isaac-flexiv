#!/usr/bin/env python3
"""Stop local Flexiv/Isaac runtime processes."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


DEFAULT_NEEDLES = [
    "rdk_target_streamer.py",
    "follow_ball_with_studio.py",
    "dual_follow_with_studio.py",
    "rizon4_quest_target_publisher.py",
    "FlexivSimulation",
    "RobotControlApp",
    "FlexivElementsStudio",
]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=8.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    stopped = flexiv_runtime.stop_matching(DEFAULT_NEEDLES, timeout=float(args.timeout))
    for pid in stopped:
        print(f"STOPPED_PID={pid}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
