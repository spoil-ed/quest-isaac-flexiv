#!/usr/bin/env python3
"""Plot live dual-arm q, dq, torque, and protection ratios from Isaac state UDP."""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time
from collections import deque


SCHEMA = "flexiv_dual_arm_state.v1"
JOINTS = tuple(f"J{index}" for index in range(1, 8))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=57684)
    parser.add_argument("--window-sec", type=float, default=30.0)
    parser.add_argument("--refresh-hz", type=float, default=10.0)
    parser.add_argument("--trigger-ratio", type=float, default=0.85)
    args = parser.parse_args(argv)
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    if args.window_sec <= 0.0 or args.refresh_hz <= 0.0:
        parser.error("--window-sec and --refresh-hz must be positive")
    if not 0.0 < args.trigger_ratio <= 1.0:
        parser.error("--trigger-ratio must be in (0, 1]")
    return args


def _vector(value, *, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise ValueError(f"{name} must contain seven values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} contains non-finite values")
    return result


def parse_state_packet(data: bytes) -> dict:
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict) or packet.get("schema") != SCHEMA:
        raise ValueError("unexpected state schema")
    parsed = {"monotonic_time": float(packet["monotonic_time"]), "arms": {}}
    for side in ("left", "right"):
        arm = packet["arms"][side]
        torque = arm["torque"]
        parsed["arms"][side] = {
            "q": _vector(arm.get("q"), name=f"{side}.q"),
            "dq": _vector(arm.get("dq"), name=f"{side}.dq"),
            "tau": _vector(torque.get("tau"), name=f"{side}.tau"),
            "tau_ext": _vector(torque.get("tau_ext"), name=f"{side}.tau_ext"),
            "tau_max": _vector(torque.get("tau_max"), name=f"{side}.tau_max"),
            "ratio": _vector(torque.get("ratio"), name=f"{side}.ratio"),
            "frozen": bool(torque.get("frozen", False)),
        }
    return parsed


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.host, args.port))
    listener.setblocking(False)

    history = {
        side: {
            "time": deque(),
            "q": [deque() for _ in JOINTS],
            "dq": [deque() for _ in JOINTS],
            "tau": [deque() for _ in JOINTS],
            "ratio": [deque() for _ in JOINTS],
            "frozen": False,
        }
        for side in ("left", "right")
    }
    start_time = time.monotonic()
    figure, axes = plt.subplots(4, 2, sharex="col", figsize=(15, 12))
    figure.canvas.manager.set_window_title("Flexiv dual-arm q/dq/torque monitor")
    colors = plt.get_cmap("tab10").colors
    lines = {side: {key: [] for key in ("q", "dq", "tau", "ratio")} for side in history}
    for column, side in enumerate(("left", "right")):
        q_axis = axes[0][column]
        dq_axis = axes[1][column]
        tau_axis = axes[2][column]
        ratio_axis = axes[3][column]
        for index, joint in enumerate(JOINTS):
            (q_line,) = q_axis.plot([], [], label=joint, color=colors[index])
            (dq_line,) = dq_axis.plot([], [], label=joint, color=colors[index])
            (tau_line,) = tau_axis.plot([], [], label=joint, color=colors[index])
            (ratio_line,) = ratio_axis.plot([], [], label=joint, color=colors[index])
            lines[side]["q"].append(q_line)
            lines[side]["dq"].append(dq_line)
            lines[side]["tau"].append(tau_line)
            lines[side]["ratio"].append(ratio_line)
        for axis, label in (
            (q_axis, "q [rad]"),
            (dq_axis, "dq [rad/s]"),
            (tau_axis, "measured tau [Nm]"),
        ):
            axis.axhline(0.0, color="black", linewidth=0.7)
            axis.set_ylabel(label)
            axis.grid(True, alpha=0.25)
        q_axis.legend(ncol=4, fontsize=8, loc="upper left")
        ratio_axis.axhline(args.trigger_ratio, color="red", linestyle="--", label="trigger")
        ratio_axis.set_ylim(0.0, max(1.05, args.trigger_ratio + 0.1))
        ratio_axis.set_ylabel("max torque ratio")
        ratio_axis.set_xlabel("time [s]")
        ratio_axis.grid(True, alpha=0.25)

    def update(_frame):
        latest = None
        while True:
            try:
                data, _address = listener.recvfrom(65535)
            except BlockingIOError:
                break
            try:
                latest = parse_state_packet(data)
            except (KeyError, TypeError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                continue
        if latest is not None:
            sample_time = time.monotonic() - start_time
            cutoff = sample_time - args.window_sec
            for side in history:
                side_history = history[side]
                arm = latest["arms"][side]
                side_history["time"].append(sample_time)
                for index in range(7):
                    for key in ("q", "dq", "tau", "ratio"):
                        side_history[key][index].append(arm[key][index])
                side_history["frozen"] = arm["frozen"]
                while side_history["time"] and side_history["time"][0] < cutoff:
                    side_history["time"].popleft()
                    for key in ("q", "dq", "tau", "ratio"):
                        for values in side_history[key]:
                            values.popleft()

        for column, side in enumerate(("left", "right")):
            side_history = history[side]
            times = list(side_history["time"])
            for index in range(7):
                for key in ("q", "dq", "tau", "ratio"):
                    lines[side][key][index].set_data(times, side_history[key][index])
            if times:
                axes[0][column].set_xlim(max(0.0, times[-1] - args.window_sec), max(args.window_sec, times[-1]))
                for row in range(3):
                    axes[row][column].relim()
                    axes[row][column].autoscale_view(scalex=False, scaley=True)
            peak = max((max(values, default=0.0) for values in side_history["ratio"]), default=0.0)
            state = "FROZEN" if side_history["frozen"] else "ACTIVE"
            axes[0][column].set_title(f"{side.upper()}  {state}  peak={peak:.3f}")
        return [line for side_lines in lines.values() for group in side_lines.values() for line in group]

    animation = FuncAnimation(
        figure,
        update,
        interval=max(20, int(1000.0 / args.refresh_hz)),
        cache_frame_data=False,
    )
    figure._torque_animation = animation
    figure.tight_layout()
    print(f"[state-plot] listening on udp://{args.host}:{args.port}", flush=True)
    try:
        plt.show()
    finally:
        listener.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
