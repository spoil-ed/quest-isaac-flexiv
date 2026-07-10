#!/usr/bin/env python3
"""Print current Flexiv/Isaac process and port status."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


PROCESS_LABELS = {
    "FlexivElementsStudio": "STUDIO_UI",
    "RobotControlApp": "ROBOT_CONTROL_APP",
    "FlexivSimulation": "FLEXIV_SIMULATION",
    "follow_ball_with_studio.py": "ISAAC_FOLLOW",
    "rdk_target_streamer.py": "RDK_TARGET_STREAMER",
}


def matching_processes() -> list[tuple[str, int, str]]:
    rows = []
    for pid, command in flexiv_runtime.pgrep_commands():
        if command.startswith("/bin/bash -lc") or "pgrep -af" in command:
            continue
        for needle, label in PROCESS_LABELS.items():
            if needle in command:
                rows.append((label, pid, command))
                break
    return rows


def listening_ports() -> str:
    result = subprocess.run(["ss", "-ltnp"], text=True, capture_output=True, check=False)
    lines = [
        line
        for line in result.stdout.splitlines()
        if ":18001" in line or ":15001" in line or ":17006" in line
    ]
    return "\n".join(lines)


def main() -> int:
    for label, pid, command in matching_processes():
        print(f"{label}_PID={pid} {command}", flush=True)
    ports = listening_ports()
    if ports:
        print(ports, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
