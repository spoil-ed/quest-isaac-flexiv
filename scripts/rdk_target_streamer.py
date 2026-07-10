#!/usr/bin/env python3
"""Stream Isaac target-pose UDP packets to the Flexiv runtime through RDK."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
UTILS_DIR = REPO_ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators"
RDK_COMPAT_PATH = REPO_ROOT / ".deps" / "flexivrdk_1_9_1"

for path in (str(RDK_COMPAT_PATH), str(UTILS_DIR)):
    if Path(path).exists() and path not in sys.path:
        sys.path.insert(0, path)

from elements_studio_utils import RdkRuntimeController, RdkRuntimeSettings


DEFAULT_SERIAL_NUMBER = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 45678


def parse_target_pose_packet(
    packet: dict,
    *,
    serial_number: str,
    joint_group: str,
    max_age_sec: float,
    now: float | None = None,
) -> list[float] | None:
    if not isinstance(packet, dict):
        return None
    if packet.get("schema") != "flexiv_target_pose.v1":
        return None
    if str(packet.get("serial", "")) != str(serial_number):
        return None
    if str(packet.get("joint_group", "")) != str(joint_group):
        return None
    try:
        pose = [float(value) for value in packet["pose_base_tcp_des"]]
    except (KeyError, TypeError, ValueError):
        return None
    if len(pose) != 7 or not all(math.isfinite(value) for value in pose):
        return None
    if max_age_sec > 0.0 and packet.get("monotonic_time") is not None:
        try:
            current_time = time.monotonic() if now is None else float(now)
            if current_time - float(packet["monotonic_time"]) > float(max_age_sec):
                return None
        except (TypeError, ValueError):
            return None
    return pose


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--joint-group", default=DEFAULT_JOINT_GROUP)
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--network-interface-whitelist", default="")
    parser.add_argument("--log-hz", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((str(args.host), int(args.port)))
    sock.settimeout(0.5)
    controller = RdkRuntimeController(
        RdkRuntimeSettings(
            serial_number=args.serial_number,
            joint_group=args.joint_group,
            network_interface_whitelist=args.network_interface_whitelist,
            switch_mode=True,
            clear_fault=True,
            servo_on=False,
            verbose=True,
        ),
        log=lambda message: print(f"[RdkTargetStreamer] {message}", flush=True),
    )
    print(
        f"[RdkTargetStreamer] listening {args.host}:{args.port} serial={args.serial_number} joint_group={args.joint_group}",
        flush=True,
    )
    last_log_time = 0.0
    while True:
        try:
            data, _addr = sock.recvfrom(65536)
        except socket.timeout:
            continue
        try:
            packet = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        pose = parse_target_pose_packet(
            packet,
            serial_number=args.serial_number,
            joint_group=args.joint_group,
            max_age_sec=float(args.max_age_sec),
        )
        if pose is None:
            continue
        try:
            controller.send_pose(pose)
        except Exception as exc:
            print(f"[RdkTargetStreamer] send failed: {exc}", flush=True)
            controller = RdkRuntimeController(
                controller.settings,
                log=lambda message: print(f"[RdkTargetStreamer] {message}", flush=True),
            )
            continue
        now = time.monotonic()
        if float(args.log_hz) > 0.0 and now - last_log_time >= 1.0 / float(args.log_hz):
            print(f"[RdkTargetStreamer] sent pose_xyz={[round(value, 4) for value in pose[:3]]}", flush=True)
            last_log_time = now


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
