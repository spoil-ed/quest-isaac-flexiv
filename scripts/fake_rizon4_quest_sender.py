#!/usr/bin/env python3
"""Send fake Quest target packets to the old Isaac UDP target port."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path
from typing import Iterable


DEFAULT_SERIAL_NUMBER = "Rizon4-VIHhZM"
DEFAULT_RIGHT_SERIAL_NUMBER = "Rizon4-WE7ssd"
DEFAULT_JOINT_GROUP = "ARM_1"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 45679
DEFAULT_QUAT_WXYZ = (0.0, 0.70710678, 0.0, 0.70710678)


def _float_list(values: Iterable[float], expected_len: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values")
    return result


def parse_csv_floats(value: str, expected_len: int, name: str) -> list[float]:
    return _float_list((item.strip() for item in value.split(",") if item.strip()), expected_len, name)


def normalize_quat_wxyz(values: Iterable[float]) -> list[float]:
    quat = _float_list(values, 4, "quaternion")
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [value / norm for value in quat]


def delta_for_frame(frame: int, frames: int, axis: str, amplitude_m: float, *, cycles: float = 0.5) -> list[float]:
    if frames <= 1:
        phase = 1.0
    else:
        phase = frame / float(frames - 1)
    value = float(amplitude_m) * math.sin(2.0 * math.pi * float(cycles) * phase)
    delta = [0.0, 0.0, 0.0]
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    delta[axis_index] = value
    return delta


def build_fake_quest_packet(
    *,
    seq: int,
    side: str,
    serial_number: str,
    joint_group: str,
    controller_delta_base: list[float],
    quat_wxyz: list[float],
    now: float,
    reason: str = "fake_stage1",
) -> dict:
    return {
        "schema": "rizon4_quest_target.v1",
        "serial": str(serial_number),
        "joint_group": str(joint_group),
        "seq": int(seq),
        "side": str(side),
        "pose_base_tcp_des": [*controller_delta_base, *normalize_quat_wxyz(quat_wxyz)],
        "controller_position_openxr": [0.0, 0.0, 0.0],
        "controller_delta_base": [float(value) for value in controller_delta_base],
        "monotonic_time": float(now),
        "reason": str(reason),
    }


def build_fake_dual_quest_packets(
    *,
    seq: int,
    left_serial_number: str,
    right_serial_number: str,
    joint_group: str,
    left_delta_base: list[float],
    right_delta_base: list[float],
    quat_wxyz: list[float],
    now: float,
) -> list[dict]:
    return [
        build_fake_quest_packet(
            seq=seq,
            side="left",
            serial_number=left_serial_number,
            joint_group=joint_group,
            controller_delta_base=left_delta_base,
            quat_wxyz=quat_wxyz,
            now=now,
            reason="fake_stage2_dual",
        ),
        build_fake_quest_packet(
            seq=seq,
            side="right",
            serial_number=right_serial_number,
            joint_group=joint_group,
            controller_delta_base=right_delta_base,
            quat_wxyz=quat_wxyz,
            now=now,
            reason="fake_stage2_dual",
        ),
    ]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--serial-number", default=DEFAULT_SERIAL_NUMBER)
    parser.add_argument("--left-serial-number", default=None)
    parser.add_argument("--right-serial-number", default=DEFAULT_RIGHT_SERIAL_NUMBER)
    parser.add_argument("--joint-group", default=DEFAULT_JOINT_GROUP)
    parser.add_argument("--side", choices=["left", "right"], default="right")
    parser.add_argument("--dual", action="store_true", help="Send left and right controller packets to one endpoint.")
    parser.add_argument("--axis", choices=["x", "y", "z"], default="x")
    parser.add_argument("--right-axis", choices=["x", "y", "z"], default=None)
    parser.add_argument("--amplitude-m", type=float, default=0.005)
    parser.add_argument(
        "--cycles",
        type=float,
        default=0.5,
        help="Number of sine cycles across --frames. Default 0.5 preserves the legacy single half-sine motion.",
    )
    parser.add_argument(
        "--same-direction",
        action="store_true",
        help="In --dual mode, move both arms in the same signed direction instead of opposed motion.",
    )
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--quat-wxyz", default=",".join(str(value) for value in DEFAULT_QUAT_WXYZ))
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    quat = parse_csv_floats(args.quat_wxyz, 4, "quat_wxyz")
    address = (args.host, int(args.port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / max(float(args.rate_hz), 1e-6)
    try:
        for frame in range(max(1, int(args.frames))):
            now = time.monotonic()
            if args.dual:
                left_delta = delta_for_frame(
                    frame,
                    max(1, int(args.frames)),
                    args.axis,
                    args.amplitude_m,
                    cycles=float(args.cycles),
                )
                right_amplitude = float(args.amplitude_m) if args.same_direction else -float(args.amplitude_m)
                right_delta = delta_for_frame(
                    frame,
                    max(1, int(args.frames)),
                    args.right_axis or args.axis,
                    right_amplitude,
                    cycles=float(args.cycles),
                )
                packets = build_fake_dual_quest_packets(
                    seq=frame,
                    left_serial_number=args.left_serial_number or args.serial_number,
                    right_serial_number=args.right_serial_number,
                    joint_group=args.joint_group,
                    left_delta_base=left_delta,
                    right_delta_base=right_delta,
                    quat_wxyz=quat,
                    now=now,
                )
            else:
                packets = [
                    build_fake_quest_packet(
                        seq=frame,
                        side=args.side,
                        serial_number=args.serial_number,
                        joint_group=args.joint_group,
                        controller_delta_base=delta_for_frame(
                            frame,
                            max(1, int(args.frames)),
                            args.axis,
                            args.amplitude_m,
                            cycles=float(args.cycles),
                        ),
                        quat_wxyz=quat,
                        now=now,
                    )
                ]
            if args.dry_run:
                payload = packets if args.dual else packets[0]
                print(json.dumps(payload, indent=2, sort_keys=True))
                break
            for packet in packets:
                sock.sendto(json.dumps(packet, separators=(",", ":"), sort_keys=True).encode("utf-8"), address)
            if frame % max(1, int(args.rate_hz)) == 0:
                print(
                    f"[fake_rizon4_quest_sender] sent seq={frame} "
                    f"packets={len(packets)} "
                    f"delta={[round(v, 5) for v in packets[0]['controller_delta_base']]} "
                    f"to {args.host}:{args.port}",
                    flush=True,
                )
            time.sleep(period)
    except KeyboardInterrupt:
        return 130
    finally:
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
