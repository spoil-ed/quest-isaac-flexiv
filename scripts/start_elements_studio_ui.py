#!/usr/bin/env python3
"""Start Flexiv Elements Studio UI only."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio-root", type=Path, default=flexiv_runtime.STUDIO_ROOT)
    return parser.parse_args(argv)


def build_command(_args: argparse.Namespace) -> list[str]:
    return ["./FlexivElementsStudio", "-p", "ubuntu_pc"]


def main(argv: list[str] | None = None) -> int:
    existing_pid = flexiv_runtime.find_process_by_executable("FlexivElementsStudio")
    if existing_pid is not None:
        flexiv_runtime.print_already_running("STUDIO_UI", existing_pid)
        return 0
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=args.studio_root,
        log_prefix="flexiv_elements_studio_ui",
        env=flexiv_runtime.studio_env(args.studio_root),
    )
    flexiv_runtime.print_started("STUDIO_UI", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
