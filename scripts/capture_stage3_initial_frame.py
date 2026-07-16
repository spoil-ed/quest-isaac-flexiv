#!/usr/bin/env python3
"""Capture one rendered initial frame for a Stage3 dual-arm scene."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DUAL_APP = (
    REPO_ROOT
    / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/dual_follow_with_studio.py"
)
DEFAULT_SCENE = REPO_ROOT / "configs/scenes/pick_place_redblock_flexiv_dual.yaml"
DEFAULT_OUTPUT = REPO_ROOT / "datasets/stage1_records/init_scenes/pick_place_redblock_initial_frame.jpg"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-config", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--isaac-python", type=Path, default=None)
    parser.add_argument("--isaacsim-root", type=Path, default=None)
    parser.add_argument("--isaac-sim-ws", type=Path, default=None)
    parser.add_argument("--usd", type=Path, default=None)
    parser.add_argument("--examples-ext", type=Path, default=None)
    parser.add_argument("--frames", type=int, default=45)
    parser.add_argument("--camera-name", default="cam_front")
    parser.add_argument("--physics-hz", type=float, default=2000.0)
    parser.add_argument("--render-hz", type=float, default=30.0)
    parser.add_argument("--base-port", type=int, default=59680)
    parser.add_argument("--target-axis-length", type=float, default=0.30)
    parser.add_argument("--target-axis-radius", type=float, default=0.010)
    return parser.parse_args(argv)


def _path_from_cli_or_env(path: Path | None, env_name: str) -> Path | None:
    if path is not None:
        return path
    value = os.environ.get(env_name)
    return Path(value) if value else None


def _required_path(path: Path | None, label: str, *, executable: bool = False) -> Path:
    if path is None:
        raise RuntimeError(f"{label} is not configured; pass it explicitly or set the matching environment variable")
    resolved = path.expanduser().resolve()
    if not resolved.exists():
        raise RuntimeError(f"{label} does not exist: {resolved}")
    if executable and not os.access(resolved, os.X_OK):
        raise RuntimeError(f"{label} is not executable: {resolved}")
    return resolved


def build_command(args: argparse.Namespace) -> list[str]:
    isaac_python = _required_path(_path_from_cli_or_env(args.isaac_python, "ISAAC_PYTHON"), "Isaac Python", executable=True)
    scene_config = _required_path(args.scene_config, "scene config")
    isaac_sim_ws = _path_from_cli_or_env(args.isaac_sim_ws, "ISAAC_SIM_WS")
    if isaac_sim_ws is None:
        repository_workspace = REPO_ROOT / "isaac_sim_ws"
        if repository_workspace.exists():
            isaac_sim_ws = repository_workspace
    usd = args.usd
    examples_ext = args.examples_ext
    if usd is None and isaac_sim_ws is not None:
        usd = isaac_sim_ws / "exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4_with_Grav.usd"
    if examples_ext is None and isaac_sim_ws is not None:
        examples_ext = isaac_sim_ws / "exts/isaacsim.robot.manipulators.examples"
    usd = _required_path(usd, "Rizon4 USD")
    examples_ext = _required_path(examples_ext, "Flexiv examples extension")
    output = args.output.expanduser().resolve()
    max_frames = max(int(args.frames) + 120, 180)
    base_port = int(args.base_port)
    return [
        str(isaac_python),
        str(DUAL_APP),
        "--scene-config",
        str(scene_config),
        "--usd",
        str(usd),
        "--examples-ext",
        str(examples_ext),
        "--headless",
        "--smoke-test",
        "--max-frames",
        str(max_frames),
        "--physics-hz",
        str(float(args.physics_hz)),
        "--render-hz",
        str(float(args.render_hz)),
        "--left-target-pose-udp-port",
        str(base_port),
        "--right-target-pose-udp-port",
        str(base_port + 1),
        "--left-rdk-status-udp-port",
        str(base_port + 2),
        "--right-rdk-status-udp-port",
        str(base_port + 3),
        "--quest-target-udp-port",
        str(base_port + 4),
        "--target-axis-length",
        str(float(args.target_axis_length)),
        "--target-axis-radius",
        str(float(args.target_axis_radius)),
        "--capture-initial-frame",
        str(output),
        "--capture-camera-name",
        str(args.camera_name),
        "--capture-after-frames",
        str(max(1, int(args.frames))),
    ]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    isaacsim_root = _path_from_cli_or_env(args.isaacsim_root, "ISAACSIM_ROOT")
    if isaacsim_root is not None:
        os.environ["ISAACSIM_ROOT"] = str(isaacsim_root.expanduser().resolve())
    command = build_command(args)
    print(" ".join(command), flush=True)
    return subprocess.run(command, cwd=str(REPO_ROOT), check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
