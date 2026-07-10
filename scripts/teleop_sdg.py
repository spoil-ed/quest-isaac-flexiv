#!/usr/bin/env python3
"""Launch Isaac Sim teleop SDG workflows from outside Kit."""

from __future__ import annotations

import argparse
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path


PYTHON_BIN = Path("/home/simate/miniconda3/envs/isaacsim/bin/python")
ISAACSIM_BIN = Path("/home/simate/miniconda3/envs/isaacsim/bin/isaacsim")
CLOUDXR_ENV = Path("/home/simate/.cloudxr/run/cloudxr.env")
CLOUDXR_RUN_DIR = Path("/home/simate/.cloudxr/run")
CLOUDXR_RUNTIME_JSON = Path("/home/simate/.cloudxr/openxr_cloudxr.json")
FLEXIV_EXAMPLES_PATH = Path(
    "/home/simate/workspace/isaacsim-flexiv/isaac_sim_ws/exts/isaacsim.robot.manipulators.examples"
)
DEFAULT_FLEXIV_TELEOP_CONFIG = Path("configs/flexiv_studio_teleop.yaml")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Launch Isaac Sim teleop, recording, and HDF5 replay workflows."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    def add_launch_options(subparser: argparse.ArgumentParser, *, cloudxr: bool) -> None:
        subparser.add_argument("--foreground", action="store_true", help="Run Isaac Sim in the foreground.")
        subparser.add_argument("--dry-run", action="store_true", help="Print launch command and exit.")
        subparser.add_argument("--stage", help="USD stage to open instead of the built-in teleop demo.")
        subparser.add_argument("--profile", help="Teleop profile YAML to load instead of floating_xarm_dex3.yaml.")
        if cloudxr:
            subparser.add_argument("--no-cloudxr", action="store_true", help="Do not start or source CloudXR.")

    teleop = subparsers.add_parser("teleop", help="Start the normal teleop demo.")
    add_launch_options(teleop, cloudxr=True)

    flexiv = subparsers.add_parser(
        "flexiv-studio",
        help="Start Flexiv Studio/RDK teleop; Isaac Teleop IK is replaced by the Flexiv stack.",
    )
    add_launch_options(flexiv, cloudxr=True)
    flexiv.add_argument(
        "--config",
        default=str(DEFAULT_FLEXIV_TELEOP_CONFIG),
        help="Flexiv Studio teleop YAML config.",
    )
    flexiv.add_argument("--stream-hz", type=float, default=250.0, help="RDK Cartesian streaming frequency.")
    flexiv.add_argument(
        "--no-switch-mode",
        action="store_true",
        help="Do not switch RDK to RT_CARTESIAN_MOTION_FORCE automatically.",
    )
    flexiv.add_argument("--clear-fault", action="store_true", help="Ask RDK to clear an existing robot fault.")
    flexiv.add_argument("--servo-on", action="store_true", help="Call ServoOn() before streaming if supported.")
    flexiv.add_argument("--rdk-verbose", action="store_true", help="Enable verbose RDK connection logs.")
    flexiv.add_argument("--no-auto-play", action="store_true", help="Do not auto-start the Isaac timeline.")
    flexiv.add_argument("--open-recorder", action="store_true", help="Open Episode Recorder beside Teleop.")

    record = subparsers.add_parser("record", help="Open a recording session; use Play/Stop manually.")
    add_launch_options(record, cloudxr=True)
    record.add_argument(
        "--output-dir",
        default="recordings/teleop_hdf5",
        help="Directory where HDF5 sessions and stage snapshots are written.",
    )
    record.add_argument("--file-prefix", default="episode", help="HDF5 filename prefix.")
    record.add_argument("--root-path", default="/World", help="USD root path to discover recordable prims under.")
    record.add_argument(
        "--auto-play",
        action="store_true",
        help="Start the timeline automatically after opening the recording session.",
    )
    record.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="With --auto-play, stop the timeline after this many seconds. 0 leaves it running.",
    )

    replay = subparsers.add_parser("replay", help="Load a recorded HDF5 session and start replay.")
    replay.add_argument("--hdf5", required=True, help="Recorded HDF5 session to load.")
    replay.add_argument("--stage", help="USD stage to open before replay; defaults to sibling stage_snapshot.usd.")
    replay.add_argument("--episode", type=int, default=0, help="Episode index to replay.")
    replay.add_argument("--load-only", action="store_true", help="Load the replay file without starting playback.")
    replay.add_argument("--foreground", action="store_true", help="Run Isaac Sim in the foreground.")
    replay.add_argument("--dry-run", action="store_true", help="Print launch command and exit.")

    return parser.parse_args(argv)


def workspace_root() -> Path:
    return Path(__file__).resolve().parents[1]


def resolve_path(value: str | os.PathLike[str], root: Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def load_export_file(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}

    env: dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line.startswith("export "):
            continue
        try:
            tokens = shlex.split(line[len("export ") :])
        except ValueError:
            continue
        for token in tokens:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            if key:
                env[key] = value
    return env


def workflow_env(args: argparse.Namespace, root: Path) -> dict[str, str]:
    env = {"SIMATE_TELEOP_WORKFLOW": args.command}

    stage = getattr(args, "stage", None)
    if stage:
        env["SIMATE_TELEOP_STAGE"] = str(resolve_path(stage, root))

    profile = getattr(args, "profile", None)
    if profile:
        env["SIMATE_TELEOP_PROFILE"] = str(resolve_path(profile, root))

    if args.command == "record":
        output_dir = resolve_path(args.output_dir, root)
        env.update(
            {
                "SIMATE_TELEOP_RECORD_OUTPUT_DIR": str(output_dir),
                "SIMATE_TELEOP_RECORD_FILE_PREFIX": args.file_prefix,
                "SIMATE_TELEOP_RECORD_ROOT_PATH": args.root_path,
                "SIMATE_TELEOP_RECORD_OPEN_SESSION": "1",
                "SIMATE_TELEOP_RECORD_AUTO_START_ON_PLAY": "1",
                "SIMATE_TELEOP_RECORD_AUTO_PLAY": "1" if args.auto_play else "0",
                "SIMATE_TELEOP_RECORD_DURATION": str(max(0.0, args.duration)),
            }
        )
    elif args.command == "flexiv-studio":
        env.update(
            {
                "SIMATE_FLEXIV_TELEOP_CONFIG": str(resolve_path(args.config, root)),
                "SIMATE_FLEXIV_EXAMPLES_PATH": str(FLEXIV_EXAMPLES_PATH),
                "SIMATE_FLEXIV_RDK_STREAM_HZ": str(max(1.0, args.stream_hz)),
                "SIMATE_FLEXIV_RDK_SWITCH_MODE": "0" if args.no_switch_mode else "1",
                "SIMATE_FLEXIV_RDK_CLEAR_FAULT": "1" if args.clear_fault else "0",
                "SIMATE_FLEXIV_RDK_SERVO_ON": "1" if args.servo_on else "0",
                "SIMATE_FLEXIV_RDK_VERBOSE": "1" if args.rdk_verbose else "0",
                "SIMATE_FLEXIV_TELEOP_AUTO_PLAY": "0" if args.no_auto_play else "1",
                "SIMATE_FLEXIV_TELEOP_OPEN_RECORDER": "1" if args.open_recorder else "0",
            }
        )
    elif args.command == "replay":
        env.update(
            {
                "SIMATE_TELEOP_HDF5": str(resolve_path(args.hdf5, root)),
                "SIMATE_TELEOP_REPLAY_EPISODE": str(args.episode),
                "SIMATE_TELEOP_REPLAY_AUTOSTART": "0" if args.load_only else "1",
            }
        )

    return env


def build_isaac_command(root: Path) -> list[str]:
    return [
        str(ISAACSIM_BIN),
        "--ext-folder",
        str(root / "local_exts"),
        "--enable",
        "isaacsim.replicator.teleop",
        "--enable",
        "isaacsim.replicator.teleop.ui",
        "--enable",
        "isaacsim.replicator.episode_recorder.ui",
        "--enable",
        "simate.teleop_demo_loader",
        "--enable",
        "simate.flexiv_studio_teleop",
    ]


def cloudxr_is_running() -> bool:
    result = subprocess.run(
        ["pgrep", "-f", "python .* -m isaacteleop.cloudxr|python -m isaacteleop.cloudxr"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def ensure_cloudxr(root: Path, env: dict[str, str]) -> None:
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)

    if cloudxr_is_running():
        print("CloudXR is already running.")
    else:
        log_path = log_dir / f"cloudxr_{time.strftime('%Y%m%d_%H%M%S')}.stdout.log"
        log_file = log_path.open("w")
        process = subprocess.Popen(
            [str(PYTHON_BIN), "-m", "isaacteleop.cloudxr", "--accept-eula"],
            stdin=subprocess.DEVNULL,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        print(f"Started CloudXR PID: {process.pid}")
        print(f"CloudXR log: {log_path}")

    for _ in range(30):
        if CLOUDXR_ENV.is_file():
            break
        time.sleep(1)

    env.update(load_export_file(CLOUDXR_ENV))
    env.setdefault("NV_CXR_RUNTIME_DIR", str(CLOUDXR_RUN_DIR))
    env.setdefault("XR_RUNTIME_JSON", str(CLOUDXR_RUNTIME_JSON))


def add_graphics_env(env: dict[str, str]) -> None:
    env["VK_ICD_FILENAMES"] = "/usr/share/vulkan/icd.d/nvidia_icd.json"
    env["VK_DRIVER_FILES"] = "/usr/share/vulkan/icd.d/nvidia_icd.json"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"


def launch(args: argparse.Namespace, root: Path) -> int:
    env = os.environ.copy()
    env.update(workflow_env(args, root))

    if args.command in {"teleop", "record", "flexiv-studio"} and not getattr(args, "no_cloudxr", False):
        ensure_cloudxr(root, env)
    add_graphics_env(env)

    command = build_isaac_command(root)
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"isaacsim_teleop_sdg_{args.command}_{time.strftime('%Y%m%d_%H%M%S')}.stdout.log"

    if args.dry_run:
        print("Command:")
        print(" ".join(shlex.quote(part) for part in command))
        print("Workflow environment:")
        for key in sorted(k for k in env if k.startswith(("SIMATE_TELEOP_", "SIMATE_FLEXIV_"))):
            print(f"{key}={env[key]}")
        print(f"Log: {log_path}")
        return 0

    if args.command == "replay" and not Path(env["SIMATE_TELEOP_HDF5"]).is_file():
        print(f"HDF5 file not found: {env['SIMATE_TELEOP_HDF5']}", file=sys.stderr)
        return 2

    if args.foreground:
        print(f"Isaac Sim foreground log mirror: {log_path}")
        with log_path.open("w") as log_file:
            return subprocess.run(command, env=env, stdout=log_file, stderr=subprocess.STDOUT, check=False).returncode

    log_file = log_path.open("w")
    process = subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
    )
    print(f"Started Isaac Sim PID: {process.pid}")
    print(f"Isaac Sim log: {log_path}")
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    return launch(args, workspace_root())


if __name__ == "__main__":
    raise SystemExit(main())
