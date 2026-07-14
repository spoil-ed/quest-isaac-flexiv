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
from typing import Any, Iterable


DEFAULT_SERIAL_NUMBER = "Rizon4-VIHhZM"
DEFAULT_RIGHT_SERIAL_NUMBER = "Rizon4-WE7ssd"
DEFAULT_JOINT_GROUP = "ARM_1"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 45679
DEFAULT_QUAT_WXYZ = (0.0, 0.70710678, 0.0, 0.70710678)


BUILTIN_TRAJECTORY_PROFILES: dict[str, dict[str, list[tuple[float, tuple[float, float, float]]]]] = {
    "pick_place_redblock_dual": {
        "left": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.12, (0.35, -0.20, 0.10)),
            (0.28, (0.80, -0.15, -0.15)),
            (0.48, (0.55, 0.20, 0.18)),
            (0.68, (0.85, 0.45, -0.08)),
            (0.86, (0.45, 0.18, 0.10)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
        "right": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.18, (0.25, 0.18, 0.08)),
            (0.36, (0.65, 0.14, -0.12)),
            (0.56, (0.50, -0.22, 0.16)),
            (0.76, (0.78, -0.45, -0.05)),
            (0.90, (0.35, -0.16, 0.08)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
    },
    "pick_redblock_into_drawer_dual": {
        "left": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.18, (0.35, -0.25, 0.05)),
            (0.38, (0.78, -0.20, -0.12)),
            (0.62, (0.70, 0.15, -0.08)),
            (0.82, (0.42, 0.18, 0.12)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
        "right": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.16, (0.28, 0.22, 0.08)),
            (0.34, (0.62, 0.30, -0.10)),
            (0.54, (0.78, 0.36, -0.12)),
            (0.74, (0.55, 0.18, 0.08)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
    },
    "stack_rgyblock_dual": {
        "left": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.15, (0.45, -0.28, -0.12)),
            (0.35, (0.68, -0.08, 0.14)),
            (0.55, (0.80, 0.12, -0.08)),
            (0.78, (0.45, 0.26, 0.12)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
        "right": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.12, (0.40, 0.30, -0.10)),
            (0.32, (0.70, 0.08, 0.16)),
            (0.55, (0.84, -0.12, -0.06)),
            (0.80, (0.42, -0.28, 0.10)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
    },
    "move_cylinder_dual": {
        "left": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.18, (0.42, -0.35, -0.10)),
            (0.42, (0.72, -0.28, -0.04)),
            (0.62, (0.76, 0.12, 0.06)),
            (0.84, (0.36, 0.28, 0.10)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
        "right": [
            (0.00, (0.0, 0.0, 0.0)),
            (0.18, (0.42, 0.35, -0.10)),
            (0.42, (0.72, 0.28, -0.04)),
            (0.62, (0.76, -0.12, 0.06)),
            (0.84, (0.36, -0.28, 0.10)),
            (1.00, (0.0, 0.0, 0.0)),
        ],
    },
}


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


def _phase_for_frame(frame: int, frames: int, *, cycles: float) -> float:
    if frames <= 1:
        return 1.0
    progress = max(0.0, min(1.0, frame / float(frames - 1)))
    if cycles <= 1.0:
        return progress
    return (progress * float(cycles)) % 1.0


def _interpolate_waypoints(
    phase: float,
    waypoints: list[tuple[float, tuple[float, float, float]]],
    *,
    amplitude_m: float,
) -> list[float]:
    if not waypoints:
        return [0.0, 0.0, 0.0]
    points = sorted((float(t), tuple(float(v) for v in delta)) for t, delta in waypoints)
    if phase <= points[0][0]:
        return [float(amplitude_m) * value for value in points[0][1]]
    for (t0, d0), (t1, d1) in zip(points, points[1:]):
        if phase <= t1:
            span = max(t1 - t0, 1e-9)
            alpha = (phase - t0) / span
            return [float(amplitude_m) * (d0[idx] + (d1[idx] - d0[idx]) * alpha) for idx in range(3)]
    return [float(amplitude_m) * value for value in points[-1][1]]


def _waypoints_from_config(raw: Any) -> list[tuple[float, tuple[float, float, float]]]:
    if not isinstance(raw, list):
        raise ValueError("trajectory waypoints must be a list")
    result = []
    for item in raw:
        if isinstance(item, dict):
            phase = float(item.get("phase", item.get("t", 0.0)))
            delta = item.get("delta") or item.get("controller_delta_base")
        else:
            phase = float(item[0])
            delta = item[1]
        values = _float_list(delta, 3, "trajectory delta")
        result.append((phase, (values[0], values[1], values[2])))
    return result


def load_trajectory_config(path: Path) -> dict[str, list[tuple[float, tuple[float, float, float]]]]:
    config_path = Path(path).expanduser().resolve()
    raw = config_path.read_text(encoding="utf-8")
    if config_path.suffix.lower() == ".json":
        data = json.loads(raw) or {}
    else:
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required for trajectory YAML files") from exc
        data = yaml.safe_load(raw) or {}
    if not isinstance(data, dict):
        raise ValueError("trajectory config must contain a mapping")
    waypoints = data.get("waypoints") if isinstance(data.get("waypoints"), dict) else data
    return {
        "left": _waypoints_from_config(waypoints.get("left") or []),
        "right": _waypoints_from_config(waypoints.get("right") or []),
    }


def trajectory_deltas_for_frame(
    *,
    frame: int,
    frames: int,
    profile: str,
    amplitude_m: float,
    cycles: float,
    trajectory_config: dict[str, list[tuple[float, tuple[float, float, float]]]] | None = None,
) -> tuple[list[float], list[float]]:
    if trajectory_config:
        waypoints = trajectory_config
    else:
        if profile not in BUILTIN_TRAJECTORY_PROFILES:
            raise ValueError(f"Unknown trajectory profile {profile!r}; known={sorted(BUILTIN_TRAJECTORY_PROFILES)}")
        waypoints = BUILTIN_TRAJECTORY_PROFILES[profile]
    phase = _phase_for_frame(frame, max(1, int(frames)), cycles=float(cycles))
    return (
        _interpolate_waypoints(phase, waypoints.get("left", []), amplitude_m=float(amplitude_m)),
        _interpolate_waypoints(phase, waypoints.get("right", []), amplitude_m=float(amplitude_m)),
    )


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
    parser.add_argument(
        "--trajectory-profile",
        default="sine",
        help="Built-in dual-arm task trajectory profile. 'sine' preserves the legacy axis motion.",
    )
    parser.add_argument("--trajectory-config", type=Path, default=None, help="Optional JSON/YAML left/right waypoint config.")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    quat = parse_csv_floats(args.quat_wxyz, 4, "quat_wxyz")
    trajectory_config = load_trajectory_config(args.trajectory_config) if args.trajectory_config is not None else None
    address = (args.host, int(args.port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / max(float(args.rate_hz), 1e-6)
    try:
        for frame in range(max(1, int(args.frames))):
            now = time.monotonic()
            if args.dual:
                if args.trajectory_profile != "sine" or trajectory_config is not None:
                    left_delta, right_delta = trajectory_deltas_for_frame(
                        frame=frame,
                        frames=max(1, int(args.frames)),
                        profile=args.trajectory_profile,
                        amplitude_m=float(args.amplitude_m),
                        cycles=float(args.cycles),
                        trajectory_config=trajectory_config,
                    )
                else:
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
