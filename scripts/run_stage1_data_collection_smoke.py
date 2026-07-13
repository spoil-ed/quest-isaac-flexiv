#!/usr/bin/env python3
"""Run a no-Isaac Stage1 data collection smoke test.

This verifies the self-contained gateway -> recorder -> Unitree JSON -> H264
LeRobot-style conversion path. It is a data-toolchain smoke test, not a real
Isaac/Studio/RDK closed-loop validation.
"""

from __future__ import annotations

import argparse
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path


def wait_for_gateway(python: str, repo_root: Path, endpoint: str, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    command = [
        python,
        "-c",
        (
            "import sys;"
            f"sys.path.insert(0,{str(repo_root)!r});"
            "from flexiv_data_collection.protocol import JsonLineReqClient;"
            f"c=JsonLineReqClient({endpoint!r}, timeout=1.0);"
            "c.send_json({'type':'sample_request'});"
            "print(c.recv_json(timeout=5).get('type'));"
            "c.close()"
        ),
    ]
    while time.monotonic() < deadline:
        result = subprocess.run(command, cwd=repo_root, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if result.returncode == 0:
            return
        time.sleep(0.2)
    raise TimeoutError(f"gateway did not become ready at {endpoint}")


def run_checked(command: list[str], *, cwd: Path) -> None:
    print("[stage1-smoke] " + " ".join(command), flush=True)
    subprocess.run(command, cwd=cwd, check=True)


def free_tcp_endpoint() -> str:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind(("127.0.0.1", 0))
        host, port = sock.getsockname()
        return f"tcp://{host}:{port}"
    finally:
        sock.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=Path("/tmp/quest_isaac_flexiv_stage1_smoke"))
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--repo-id", default="local/flexiv_stage1_smoke")
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--sample-endpoint", default="")
    parser.add_argument("--bridge-endpoint", default="")
    parser.add_argument("--camera-keys", default="color_0")
    parser.add_argument("--keep-output", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    repo_root = Path(__file__).resolve().parents[1]
    output_root = args.output_root.expanduser().resolve()
    raw_dir = output_root / "raw"
    lerobot_root = output_root / "lerobot"
    report_path = output_root / "stage1_smoke_validation.json"
    sample_endpoint = args.sample_endpoint or free_tcp_endpoint()
    bridge_endpoint = args.bridge_endpoint or free_tcp_endpoint()
    if output_root.exists() and not args.keep_output:
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    gateway_cmd = [
        args.python,
        str(repo_root / "scripts" / "start_data_gateway.py"),
        "--backend",
        "fake",
        "--sample-endpoint",
        sample_endpoint,
        "--bridge-endpoint",
        bridge_endpoint,
        "--fps",
        str(args.fps),
        "--camera-keys",
        str(args.camera_keys),
    ]
    gateway = subprocess.Popen(gateway_cmd, cwd=repo_root)
    try:
        wait_for_gateway(args.python, repo_root, sample_endpoint)
        if gateway.poll() is not None:
            raise RuntimeError(f"gateway exited early with code {gateway.returncode}")
        run_checked(
            [
                args.python,
                str(repo_root / "scripts" / "record_unitree_json.py"),
                "--gateway-endpoint",
                sample_endpoint,
                "--fps",
                str(args.fps),
                "--episodes",
                "1",
                "--task-dir",
                str(raw_dir),
                "--image-size",
                "640x480",
                "--max-frames",
                str(args.frames),
                "--auto-start",
            ],
            cwd=repo_root,
        )
        run_checked(
            [
                args.python,
                str(repo_root / "scripts" / "convert_unitree_json_to_lerobot.py"),
                "--raw-dir",
                str(raw_dir),
                "--repo-id",
                args.repo_id,
                "--output-root",
                str(lerobot_root),
            ],
            cwd=repo_root,
        )
        dataset_root = lerobot_root / args.repo_id
        run_checked(
            [
                args.python,
                str(repo_root / "scripts" / "validate_data_artifacts.py"),
                "--raw-dir",
                str(raw_dir),
                "--dataset-root",
                str(dataset_root),
                "--out",
                str(report_path),
            ],
            cwd=repo_root,
        )
        print(f"[stage1-smoke] raw Unitree JSON: {raw_dir}", flush=True)
        print(f"[stage1-smoke] LeRobot-style dataset: {dataset_root}", flush=True)
        print(f"[stage1-smoke] validation report: {report_path}", flush=True)
    finally:
        gateway.terminate()
        try:
            gateway.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            gateway.kill()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
