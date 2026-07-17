#!/usr/bin/env python3
"""Send synchronized dual-arm Cartesian targets through Flexiv DRDK RobotPair."""

from __future__ import annotations

import argparse
import json
import math
import select
import socket
import sys
import time
from collections import deque
from pathlib import Path
from typing import NamedTuple


REPO_ROOT = Path(__file__).resolve().parents[1]
UTILS_DIR = REPO_ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators"
RDK_COMPAT_PATH = REPO_ROOT / ".deps" / "flexivrdk_1_9_1"
for path in (RDK_COMPAT_PATH, UTILS_DIR):
    if path.exists() and str(path) not in sys.path:
        sys.path.insert(0, str(path))

from control_helpers import format_pose_xyz_quat  # noqa: E402
from rdk_target_streamer import parse_target_pose_packet  # noqa: E402


DEFAULT_LEFT_SERIAL = "Rizon4-qSaFLh"
DEFAULT_RIGHT_SERIAL = "Rizon4-I0LIRN"
DEFAULT_JOINT_GROUP = "ARM_1"


class TargetCommand(NamedTuple):
    servo_cycle: int
    pose: list[float]
    control_active: bool


def _normalize_quat_wxyz(quaternion) -> list[float]:
    values = [float(value) for value in quaternion]
    norm = math.sqrt(sum(value * value for value in values))
    if len(values) != 4 or not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("quaternion must contain four finite values with non-zero norm")
    return [value / norm for value in values]


def _quat_multiply_wxyz(left, right) -> list[float]:
    lw, lx, ly, lz = _normalize_quat_wxyz(left)
    rw, rx, ry, rz = _normalize_quat_wxyz(right)
    return _normalize_quat_wxyz(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def rebase_relative_pose(incoming_pose, incoming_anchor, output_anchor) -> list[float]:
    """Apply motion relative to an input anchor on top of a held output pose."""

    incoming = [float(value) for value in incoming_pose]
    input_zero = [float(value) for value in incoming_anchor]
    output_zero = [float(value) for value in output_anchor]
    if any(len(pose) != 7 for pose in (incoming, input_zero, output_zero)):
        raise ValueError("Cartesian poses must contain seven values")
    xyz = [output_zero[index] + incoming[index] - input_zero[index] for index in range(3)]
    input_zero_quat = _normalize_quat_wxyz(input_zero[3:])
    input_delta = _quat_multiply_wxyz(
        incoming[3:],
        [input_zero_quat[0], -input_zero_quat[1], -input_zero_quat[2], -input_zero_quat[3]],
    )
    return xyz + _quat_multiply_wxyz(input_delta, output_zero[3:])


class ContactWrenchGuard:
    """Freeze each TCP independently while its measured contact wrench is excessive."""

    SIDES = ("left", "right")

    def __init__(
        self,
        limits,
        *,
        enabled: bool = True,
        trigger_ratio: float,
        release_ratio: float,
        trigger_samples: int,
        release_dwell_sec: float,
    ) -> None:
        self.enabled = bool(enabled)
        self.limits = {
            side: [float(value) for value in side_limits]
            for side, side_limits in zip(self.SIDES, limits, strict=True)
        }
        self.trigger_ratio = float(trigger_ratio)
        self.release_ratio = float(release_ratio)
        self.trigger_samples = int(trigger_samples)
        self.release_dwell_sec = float(release_dwell_sec)
        self.reset()

    def reset(self) -> None:
        self.frozen = {side: False for side in self.SIDES}
        self.held_poses: dict[str, list[float] | None] = {side: None for side in self.SIDES}
        self.input_anchors: dict[str, list[float] | None] = {side: None for side in self.SIDES}
        self.output_anchors: dict[str, list[float] | None] = {side: None for side in self.SIDES}
        self.over_limit_samples = {side: 0 for side in self.SIDES}
        self.clear_since: dict[str, float | None] = {side: None for side in self.SIDES}
        self.latest_wrenches = {side: [0.0] * 6 for side in self.SIDES}

    def update(self, states, latest_input_poses: dict[str, list[float]], *, now: float) -> list[tuple[str, str]]:
        if not self.enabled:
            return []
        events: list[tuple[str, str]] = []
        for index, side in enumerate(self.SIDES):
            try:
                wrench = [float(value) for value in states[index].tcp_wrench]
                tcp_pose = [float(value) for value in states[index].tcp_pose]
            except (AttributeError, TypeError, ValueError):
                continue
            if len(wrench) != 6 or len(tcp_pose) != 7 or not all(
                math.isfinite(value) for value in wrench + tcp_pose
            ):
                continue
            self.latest_wrenches[side] = wrench
            limits = self.limits[side]
            over_trigger = any(
                abs(value) >= limit * self.trigger_ratio
                for value, limit in zip(wrench, limits, strict=True)
            )
            below_release = all(
                abs(value) <= limit * self.release_ratio
                for value, limit in zip(wrench, limits, strict=True)
            )
            if not self.frozen[side]:
                self.over_limit_samples[side] = (
                    self.over_limit_samples[side] + 1 if over_trigger else 0
                )
                if self.over_limit_samples[side] >= self.trigger_samples:
                    self.frozen[side] = True
                    self.held_poses[side] = tcp_pose
                    self.input_anchors[side] = None
                    self.output_anchors[side] = None
                    self.clear_since[side] = None
                    events.append((side, "frozen"))
                continue
            if below_release:
                self.clear_since[side] = (
                    float(now) if self.clear_since[side] is None else self.clear_since[side]
                )
            else:
                self.clear_since[side] = None
            if (
                self.clear_since[side] is not None
                and float(now) - self.clear_since[side] >= self.release_dwell_sec
            ):
                held_pose = self.held_poses[side]
                incoming_pose = latest_input_poses.get(side)
                if held_pose is None or incoming_pose is None:
                    continue
                self.frozen[side] = False
                self.held_poses[side] = None
                self.input_anchors[side] = list(incoming_pose)
                self.output_anchors[side] = list(held_pose)
                self.over_limit_samples[side] = 0
                self.clear_since[side] = None
                events.append((side, "released"))
        return events

    def command_pose(self, side: str, incoming_pose) -> list[float]:
        incoming = [float(value) for value in incoming_pose]
        if self.frozen[side] and self.held_poses[side] is not None:
            return list(self.held_poses[side])
        if self.input_anchors[side] is not None and self.output_anchors[side] is not None:
            return rebase_relative_pose(
                incoming,
                self.input_anchors[side],
                self.output_anchors[side],
            )
        return incoming


class JointTorqueGuard:
    """Roll each arm back to a recent safe target before a joint torque fault."""

    SIDES = ("left", "right")

    def __init__(
        self,
        limits,
        *,
        enabled: bool = True,
        trigger_ratio: float,
        release_ratio: float,
        trigger_samples: int,
        release_dwell_sec: float,
        prediction_horizon_sec: float,
        rollback_sec: float,
    ) -> None:
        self.enabled = bool(enabled)
        self.limits = {
            side: [float(value) for value in side_limits]
            for side, side_limits in zip(self.SIDES, limits, strict=True)
        }
        self.trigger_ratio = float(trigger_ratio)
        self.release_ratio = float(release_ratio)
        self.trigger_samples = int(trigger_samples)
        self.release_dwell_sec = float(release_dwell_sec)
        self.prediction_horizon_sec = float(prediction_horizon_sec)
        self.rollback_sec = float(rollback_sec)
        self.reset()

    def reset(
        self,
        initial_safe_poses: dict[str, list[float]] | None = None,
        *,
        now: float | None = None,
    ) -> None:
        self.frozen = {side: False for side in self.SIDES}
        self.held_poses: dict[str, list[float] | None] = {side: None for side in self.SIDES}
        self.input_anchors: dict[str, list[float] | None] = {side: None for side in self.SIDES}
        self.output_anchors: dict[str, list[float] | None] = {side: None for side in self.SIDES}
        self.over_limit_samples = {side: 0 for side in self.SIDES}
        self.clear_since: dict[str, float | None] = {side: None for side in self.SIDES}
        self.latest_tau = {side: [0.0] * len(self.limits[side]) for side in self.SIDES}
        self.latest_tau_dot = {side: [0.0] * len(self.limits[side]) for side in self.SIDES}
        self.latest_tau_ext = {side: [0.0] * len(self.limits[side]) for side in self.SIDES}
        self.latest_ratios = {side: [0.0] * len(self.limits[side]) for side in self.SIDES}
        self.trigger_joints: dict[str, int | None] = {side: None for side in self.SIDES}
        self.command_history: dict[str, deque[tuple[float, list[float]]]] = {
            side: deque() for side in self.SIDES
        }
        if initial_safe_poses:
            stamp = time.monotonic() if now is None else float(now)
            for side in self.SIDES:
                pose = initial_safe_poses.get(side)
                if pose is not None:
                    self.command_history[side].append((stamp, [float(value) for value in pose]))

    def record_command(self, side: str, pose, *, now: float) -> None:
        """Remember commands while healthy so a later trigger can roll back in time."""

        if not self.enabled or self.frozen[side]:
            return
        values = [float(value) for value in pose]
        if len(values) != 7 or not all(math.isfinite(value) for value in values):
            return
        history = self.command_history[side]
        history.append((float(now), values))
        keep_after = float(now) - max(2.0, 4.0 * self.rollback_sec)
        while len(history) > 1 and history[1][0] < keep_after:
            history.popleft()

    def _rollback_pose(self, side: str, tcp_pose: list[float], *, now: float) -> list[float]:
        history = self.command_history[side]
        cutoff = float(now) - self.rollback_sec
        eligible = [pose for stamp, pose in history if stamp <= cutoff]
        if eligible:
            return list(eligible[-1])
        if history:
            return list(history[0][1])
        return list(tcp_pose)

    def update(self, states, latest_input_poses: dict[str, list[float]], *, now: float) -> list[tuple[str, str]]:
        if not self.enabled:
            return []
        events: list[tuple[str, str]] = []
        for index, side in enumerate(self.SIDES):
            try:
                tau = [float(value) for value in states[index].tau]
                tau_dot = [float(value) for value in states[index].tau_dot]
                tau_ext = [float(value) for value in states[index].tau_ext]
                tcp_pose = [float(value) for value in states[index].tcp_pose]
            except (AttributeError, TypeError, ValueError):
                continue
            limits = self.limits[side]
            if any(len(values) != len(limits) for values in (tau, tau_dot, tau_ext)):
                continue
            if len(tcp_pose) != 7 or not all(
                math.isfinite(value) for values in (tau, tau_dot, tau_ext, tcp_pose) for value in values
            ):
                continue
            predicted_tau = [
                current + rate * self.prediction_horizon_sec
                for current, rate in zip(tau, tau_dot, strict=True)
            ]
            ratios = [
                max(abs(current), abs(predicted), abs(external)) / limit
                for current, predicted, external, limit in zip(
                    tau, predicted_tau, tau_ext, limits, strict=True
                )
            ]
            self.latest_tau[side] = tau
            self.latest_tau_dot[side] = tau_dot
            self.latest_tau_ext[side] = tau_ext
            self.latest_ratios[side] = ratios
            peak_joint = max(range(len(ratios)), key=ratios.__getitem__)
            over_trigger = ratios[peak_joint] >= self.trigger_ratio
            below_release = all(ratio <= self.release_ratio for ratio in ratios)
            if not self.frozen[side]:
                self.over_limit_samples[side] = (
                    self.over_limit_samples[side] + 1 if over_trigger else 0
                )
                if self.over_limit_samples[side] >= self.trigger_samples:
                    self.frozen[side] = True
                    self.held_poses[side] = self._rollback_pose(side, tcp_pose, now=now)
                    self.input_anchors[side] = None
                    self.output_anchors[side] = None
                    self.clear_since[side] = None
                    self.trigger_joints[side] = peak_joint
                    events.append((side, "frozen"))
                continue
            if below_release:
                self.clear_since[side] = (
                    float(now) if self.clear_since[side] is None else self.clear_since[side]
                )
            else:
                self.clear_since[side] = None
            if (
                self.clear_since[side] is not None
                and float(now) - self.clear_since[side] >= self.release_dwell_sec
            ):
                held_pose = self.held_poses[side]
                incoming_pose = latest_input_poses.get(side)
                if held_pose is None or incoming_pose is None:
                    continue
                self.frozen[side] = False
                self.held_poses[side] = None
                self.input_anchors[side] = list(incoming_pose)
                self.output_anchors[side] = list(held_pose)
                self.over_limit_samples[side] = 0
                self.clear_since[side] = None
                self.trigger_joints[side] = None
                self.command_history[side].clear()
                self.command_history[side].append((float(now), list(held_pose)))
                events.append((side, "released"))
        return events

    def command_pose(self, side: str, incoming_pose) -> list[float]:
        incoming = [float(value) for value in incoming_pose]
        if self.frozen[side] and self.held_poses[side] is not None:
            return list(self.held_poses[side])
        if self.input_anchors[side] is not None and self.output_anchors[side] is not None:
            return rebase_relative_pose(
                incoming,
                self.input_anchors[side],
                self.output_anchors[side],
            )
        return incoming


def joint_torque_limits(robot_pair, flexivrdk, joint_group: str) -> tuple[list[float], list[float]]:
    """Read both robots' software joint-torque limits from RobotPair.info()."""

    # DRDK 1.2.1 is ABI-compatible with the repository's RDK 1.9.1 wheel.
    # That RDK exposes the single-arm RobotInfo.tau_max directly as a list and
    # does not define JointGroup. Newer RDK wheels expose tau_max by group, so
    # support both layouts without forcing an incompatible RDK upgrade.
    joint_group_type = getattr(flexivrdk, "JointGroup", None)
    group = None
    if joint_group_type is not None:
        try:
            group = getattr(joint_group_type, str(joint_group))
        except (AttributeError, TypeError) as exc:
            raise ValueError(f"unknown Flexiv joint group: {joint_group}") from exc
    limits = []
    for side, info in zip(JointTorqueGuard.SIDES, robot_pair.info(), strict=True):
        try:
            raw_limits = info.tau_max if group is None else info.tau_max[group]
            values = [float(value) for value in raw_limits]
        except (AttributeError, KeyError, TypeError) as exc:
            raise RuntimeError(f"DRDK RobotInfo does not expose tau_max for {side}/{joint_group}") from exc
        if not values or not all(math.isfinite(value) and value > 0.0 for value in values):
            raise RuntimeError(f"invalid DRDK tau_max for {side}/{joint_group}: {values}")
        limits.append(values)
    return limits[0], limits[1]


def parse_csv_floats(value: str, *, expected: int, name: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in str(value).split(",") if item.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must contain numeric values") from exc
    if len(values) != expected or not all(math.isfinite(item) for item in values):
        raise argparse.ArgumentTypeError(f"{name} must contain {expected} finite comma-separated values")
    return values


def parse_target_command_packet(
    packet: dict,
    *,
    serial_number: str,
    joint_group: str,
    max_age_sec: float,
    now: float | None = None,
) -> TargetCommand | None:
    pose = parse_target_pose_packet(
        packet,
        serial_number=serial_number,
        joint_group=joint_group,
        max_age_sec=max_age_sec,
        now=now,
    )
    if pose is None:
        return None
    try:
        servo_cycle = int(packet["servo_cycle"])
    except (KeyError, TypeError, ValueError):
        return None
    if servo_cycle < 0:
        return None
    return TargetCommand(
        servo_cycle=servo_cycle,
        pose=list(pose),
        control_active=bool(packet.get("control_active", True)),
    )


def parse_reset_request_seq(packet: dict, *, serial_number: str, joint_group: str) -> int | None:
    """Return a coordinated-reset sequence carried by a normal target packet."""

    if str(packet.get("schema", "")) != "flexiv_target_pose.v1":
        return None
    if str(packet.get("serial", "")) != str(serial_number):
        return None
    if str(packet.get("joint_group", "")) != str(joint_group):
        return None
    try:
        reset_seq = int(packet.get("reset_seq", 0))
    except (TypeError, ValueError):
        return None
    return reset_seq if reset_seq > 0 else None


def pop_synchronized_target_pair(
    command_buffers: dict[str, dict[int, TargetCommand]], *, after_cycle: int
) -> tuple[TargetCommand, TargetCommand] | None:
    common_cycles = set(command_buffers["left"]).intersection(command_buffers["right"])
    eligible_cycles = [cycle for cycle in common_cycles if cycle > int(after_cycle)]
    if not eligible_cycles:
        return None
    cycle = max(eligible_cycles)
    pair = (command_buffers["left"][cycle], command_buffers["right"][cycle])
    for buffer in command_buffers.values():
        for buffered_cycle in tuple(buffer):
            if buffered_cycle <= cycle:
                del buffer[buffered_cycle]
    return pair


def buffer_target_command(buffer: dict[int, TargetCommand], command: TargetCommand, *, limit: int = 64) -> None:
    buffer[command.servo_cycle] = command
    overflow = len(buffer) - max(1, int(limit))
    if overflow > 0:
        for cycle in sorted(buffer)[:overflow]:
            del buffer[cycle]


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--left-port", type=int, default=57680)
    parser.add_argument("--right-port", type=int, default=57681)
    parser.add_argument("--left-serial-number", default=DEFAULT_LEFT_SERIAL)
    parser.add_argument("--right-serial-number", default=DEFAULT_RIGHT_SERIAL)
    parser.add_argument("--joint-group", default=DEFAULT_JOINT_GROUP)
    parser.add_argument("--left-status-host", default="127.0.0.1")
    parser.add_argument("--left-status-port", type=int, default=57682)
    parser.add_argument("--right-status-host", default="127.0.0.1")
    parser.add_argument("--right-status-port", type=int, default=57683)
    parser.add_argument("--left-translation-in-world", default="0,0,0")
    parser.add_argument("--right-translation-in-world", default="0,0,0")
    parser.add_argument(
        "--cartesian-impedance-control",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Explicitly configure DRDK Cartesian stiffness and damping after each mode switch.",
    )
    parser.add_argument("--left-cartesian-stiffness", default="10000,10000,10000,1500,1500,1500")
    parser.add_argument("--right-cartesian-stiffness", default="10000,10000,10000,1500,1500,1500")
    parser.add_argument("--left-cartesian-damping-ratio", default="0.7,0.7,0.7,0.7,0.7,0.7")
    parser.add_argument("--right-cartesian-damping-ratio", default="0.7,0.7,0.7,0.7,0.7,0.7")
    parser.add_argument(
        "--self-collision-monitor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable the official DRDK SelfCollisionMonitor for inter-arm collision stopping.",
    )
    parser.add_argument(
        "--self-collision-min-distance-m",
        type=float,
        default=0.05,
        help="Minimum permitted distance between the two robots.",
    )
    parser.add_argument(
        "--self-collision-loop-interval-ms",
        type=int,
        default=10,
        help="Background SelfCollisionMonitor interval; 10 ms is 100 Hz.",
    )
    parser.add_argument(
        "--self-collision-skip-link",
        action="append",
        default=[],
        help="Robot link name excluded from inter-arm collision checks; repeat as needed.",
    )
    parser.add_argument("--left-max-contact-wrench", default="20,20,20,3,3,3")
    parser.add_argument("--right-max-contact-wrench", default="20,20,20,3,3,3")
    parser.add_argument(
        "--contact-wrench-control",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--contact-wrench-freeze-trigger-ratio", type=float, default=0.95)
    parser.add_argument("--contact-wrench-release-ratio", type=float, default=0.70)
    parser.add_argument("--contact-wrench-trigger-samples", type=int, default=3)
    parser.add_argument("--contact-wrench-release-dwell-sec", type=float, default=0.30)
    parser.add_argument(
        "--joint-torque-control",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Roll back a target before measured or predicted joint torque reaches its limit.",
    )
    parser.add_argument("--joint-torque-trigger-ratio", type=float, default=0.85)
    parser.add_argument("--joint-torque-release-ratio", type=float, default=0.70)
    parser.add_argument("--joint-torque-trigger-samples", type=int, default=1)
    parser.add_argument("--joint-torque-release-dwell-sec", type=float, default=0.30)
    parser.add_argument("--joint-torque-prediction-horizon-sec", type=float, default=0.02)
    parser.add_argument("--joint-torque-rollback-sec", type=float, default=0.10)
    parser.add_argument("--left-nullspace-posture", required=True)
    parser.add_argument("--right-nullspace-posture", required=True)
    parser.add_argument(
        "--left-startup-waypoint",
        action="append",
        default=[],
        help="Optional 7-DoF NRT joint waypoint used only during initial startup; repeat with the right option.",
    )
    parser.add_argument(
        "--right-startup-waypoint",
        action="append",
        default=[],
        help="Optional 7-DoF NRT joint waypoint used only during initial startup; repeat with the left option.",
    )
    parser.add_argument("--nullspace-tracking-weight", type=float, default=0.5)
    parser.add_argument("--network-interface-whitelist", default="")
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--connect-timeout-sec", type=float, default=30.0)
    parser.add_argument("--enable-timeout-sec", type=float, default=15.0)
    parser.add_argument("--initial-joint-timeout-sec", type=float, default=45.0)
    parser.add_argument("--initial-joint-handoff-sec", type=float, default=0.5)
    parser.add_argument("--initial-joint-settle-sec", type=float, default=0.5)
    parser.add_argument("--initial-joint-tolerance-rad", type=float, default=0.02)
    parser.add_argument("--initial-joint-speed-tolerance-rad-s", type=float, default=0.03)
    parser.add_argument("--initial-joint-max-vel-rad-s", type=float, default=0.5)
    parser.add_argument("--initial-joint-max-acc-rad-s2", type=float, default=1.0)
    parser.add_argument(
        "--reset-joint-max-vel-rad-s",
        type=float,
        default=0.2,
        help="Joint velocity limit used only while recovering from a coordinated reset.",
    )
    parser.add_argument(
        "--reset-joint-max-acc-rad-s2",
        type=float,
        default=0.4,
        help="Joint acceleration limit used only while recovering from a coordinated reset.",
    )
    parser.add_argument(
        "--reset-max-attempts",
        type=int,
        default=3,
        help="Maximum bounded Stop/ClearFault/Enable/NRT recovery attempts per reset sequence.",
    )
    parser.add_argument(
        "--reset-retry-delay-sec",
        type=float,
        default=0.5,
        help="Delay after a recoverable mid-motion failure before retrying the same reset sequence.",
    )
    parser.add_argument("--max-linear-speed-m-s", type=float, default=0.5)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=0.75)
    parser.add_argument("--max-linear-acc-m-s2", type=float, default=2.0)
    parser.add_argument("--max-angular-acc-rad-s2", type=float, default=5.0)
    parser.add_argument("--clear-fault", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--strict-clear-fault", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--log-hz", type=float, default=2.0)
    args = parser.parse_args(argv)
    if args.left_serial_number == args.right_serial_number:
        parser.error("left and right serial numbers must be different")
    if not 0.1 <= float(args.nullspace_tracking_weight) <= 1.0:
        parser.error("--nullspace-tracking-weight must be within [0.1, 1.0]")
    for option in (
        "initial_joint_timeout_sec",
        "initial_joint_settle_sec",
        "initial_joint_tolerance_rad",
        "initial_joint_speed_tolerance_rad_s",
        "initial_joint_max_vel_rad_s",
        "initial_joint_max_acc_rad_s2",
        "reset_joint_max_vel_rad_s",
        "reset_joint_max_acc_rad_s2",
    ):
        if float(getattr(args, option)) <= 0.0:
            parser.error(f"--{option.replace('_', '-')} must be positive")
    if float(args.initial_joint_handoff_sec) < 0.0:
        parser.error("--initial-joint-handoff-sec must be non-negative")
    if int(args.reset_max_attempts) < 1:
        parser.error("--reset-max-attempts must be at least 1")
    if float(args.reset_retry_delay_sec) < 0.0:
        parser.error("--reset-retry-delay-sec must be non-negative")
    if float(args.self_collision_min_distance_m) <= 0.0:
        parser.error("--self-collision-min-distance-m must be positive")
    if int(args.self_collision_loop_interval_ms) < 1:
        parser.error("--self-collision-loop-interval-ms must be at least 1")
    if not 0.0 < float(args.contact_wrench_freeze_trigger_ratio) <= 1.0:
        parser.error("--contact-wrench-freeze-trigger-ratio must be within (0, 1]")
    if not 0.0 <= float(args.contact_wrench_release_ratio) < float(
        args.contact_wrench_freeze_trigger_ratio
    ):
        parser.error("--contact-wrench-release-ratio must be below the freeze trigger ratio")
    if int(args.contact_wrench_trigger_samples) < 1:
        parser.error("--contact-wrench-trigger-samples must be at least 1")
    if float(args.contact_wrench_release_dwell_sec) < 0.0:
        parser.error("--contact-wrench-release-dwell-sec must be non-negative")
    if not 0.0 < float(args.joint_torque_trigger_ratio) < 1.0:
        parser.error("--joint-torque-trigger-ratio must be within (0, 1)")
    if not 0.0 <= float(args.joint_torque_release_ratio) < float(
        args.joint_torque_trigger_ratio
    ):
        parser.error("--joint-torque-release-ratio must be below the trigger ratio")
    if int(args.joint_torque_trigger_samples) < 1:
        parser.error("--joint-torque-trigger-samples must be at least 1")
    if float(args.joint_torque_release_dwell_sec) < 0.0:
        parser.error("--joint-torque-release-dwell-sec must be non-negative")
    if float(args.joint_torque_prediction_horizon_sec) < 0.0:
        parser.error("--joint-torque-prediction-horizon-sec must be non-negative")
    if float(args.joint_torque_rollback_sec) < 0.0:
        parser.error("--joint-torque-rollback-sec must be non-negative")
    args.left_translation_in_world = parse_csv_floats(
        args.left_translation_in_world, expected=3, name="left translation"
    )
    args.right_translation_in_world = parse_csv_floats(
        args.right_translation_in_world, expected=3, name="right translation"
    )
    args.left_max_contact_wrench = parse_csv_floats(
        args.left_max_contact_wrench, expected=6, name="left maximum contact wrench"
    )
    args.right_max_contact_wrench = parse_csv_floats(
        args.right_max_contact_wrench, expected=6, name="right maximum contact wrench"
    )
    args.left_cartesian_stiffness = parse_csv_floats(
        args.left_cartesian_stiffness, expected=6, name="left Cartesian stiffness"
    )
    args.right_cartesian_stiffness = parse_csv_floats(
        args.right_cartesian_stiffness, expected=6, name="right Cartesian stiffness"
    )
    args.left_cartesian_damping_ratio = parse_csv_floats(
        args.left_cartesian_damping_ratio, expected=6, name="left Cartesian damping ratio"
    )
    args.right_cartesian_damping_ratio = parse_csv_floats(
        args.right_cartesian_damping_ratio, expected=6, name="right Cartesian damping ratio"
    )
    if any(
        value < 0.0
        for stiffness in (args.left_cartesian_stiffness, args.right_cartesian_stiffness)
        for value in stiffness
    ):
        parser.error("Cartesian stiffness values must be non-negative")
    if any(
        not 0.3 <= value <= 0.8
        for damping in (args.left_cartesian_damping_ratio, args.right_cartesian_damping_ratio)
        for value in damping
    ):
        parser.error("Cartesian damping ratios must be within [0.3, 0.8]")
    if any(
        value <= 0.0
        for limits in (args.left_max_contact_wrench, args.right_max_contact_wrench)
        for value in limits
    ):
        parser.error("maximum contact wrench values must be positive")
    args.left_nullspace_posture = parse_csv_floats(
        args.left_nullspace_posture, expected=7, name="left null-space posture"
    )
    args.right_nullspace_posture = parse_csv_floats(
        args.right_nullspace_posture, expected=7, name="right null-space posture"
    )
    if len(args.left_startup_waypoint) != len(args.right_startup_waypoint):
        parser.error("left and right startup waypoints must be supplied in pairs")
    args.left_startup_waypoint = [
        parse_csv_floats(value, expected=7, name="left startup waypoint")
        for value in args.left_startup_waypoint
    ]
    args.right_startup_waypoint = [
        parse_csv_floats(value, expected=7, name="right startup waypoint")
        for value in args.right_startup_waypoint
    ]
    return args


def _wait_until_operational(robot_pair, timeout_sec: float) -> None:
    if robot_pair.operational():
        return
    robot_pair.Enable()
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    while not robot_pair.operational() and time.monotonic() < deadline:
        time.sleep(0.1)
    if not robot_pair.operational():
        raise TimeoutError(f"DRDK robot pair did not become operational within {float(timeout_sec):.3f}s")


def _switch_mode_and_wait(robot_pair, target_mode, *, timeout_sec: float) -> None:
    """Switch both robots and wait until the pair reports the requested mode."""

    if any(current_mode != target_mode for current_mode in robot_pair.mode()):
        robot_pair.SwitchMode(target_mode)
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    while any(current_mode != target_mode for current_mode in robot_pair.mode()):
        if not robot_pair.connected() or robot_pair.fault() or not robot_pair.operational():
            raise RuntimeError(f"DRDK robot pair failed while switching to mode {target_mode}")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"DRDK robot pair did not enter mode {target_mode} within {timeout_sec:.3f}s")
        time.sleep(0.01)
    # RobotPair.mode() can reflect the requested state before the runtime has
    # completed its controller handoff.  One short stabilization interval
    # prevents the first command from racing that internal transition.
    time.sleep(0.05)
    if any(current_mode != target_mode for current_mode in robot_pair.mode()):
        raise RuntimeError(f"DRDK robot pair left requested mode {target_mode} during handoff")


def _connect_robot_pair(args: argparse.Namespace, *, flexivdrdk):
    whitelist = [item.strip() for item in str(args.network_interface_whitelist).split(",") if item.strip()]
    deadline = time.monotonic() + max(0.0, float(args.connect_timeout_sec))
    last_error: Exception | None = None
    while True:
        try:
            return flexivdrdk.RobotPair(
                (str(args.left_serial_number), str(args.right_serial_number)),
                (list(args.left_translation_in_world), list(args.right_translation_in_world)),
                whitelist,
            )
        except Exception as exc:
            last_error = exc
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    "DRDK robot pair was not discoverable during initial startup "
                    f"within {float(args.connect_timeout_sec):.3f}s"
                ) from last_error
            time.sleep(0.1)


def start_self_collision_monitor(args: argparse.Namespace, *, robot_pair, flexivdrdk):
    """Create and start the official DRDK inter-arm collision monitor."""

    if not bool(args.self_collision_monitor):
        return None
    monitor_type = getattr(flexivdrdk, "SelfCollisionMonitor", None)
    if monitor_type is None:
        raise RuntimeError("installed flexivdrdk does not expose SelfCollisionMonitor")
    monitor = monitor_type(robot_pair, list(args.self_collision_skip_link))
    try:
        monitor.SetMinDistance(float(args.self_collision_min_distance_m))
        monitor.Start(int(args.self_collision_loop_interval_ms))
    except Exception:
        monitor.Stop()
        raise
    print(
        "[DrdkTargetStreamer] SelfCollisionMonitor started "
        f"min_distance={float(args.self_collision_min_distance_m):.3f}m "
        f"interval={int(args.self_collision_loop_interval_ms)}ms "
        f"skipped_links={list(args.self_collision_skip_link)}",
        flush=True,
    )
    return monitor


def stop_self_collision_monitor(monitor) -> None:
    """Stop a running monitor without masking streamer shutdown errors."""

    if monitor is None:
        return
    try:
        monitor.Stop()
    except Exception as exc:
        print(f"[DrdkTargetStreamer] SelfCollisionMonitor stop warning: {exc}", flush=True)


def _joint_errors(current_q, target_q) -> list[float]:
    return [
        math.atan2(math.sin(float(actual) - float(target)), math.cos(float(actual) - float(target)))
        for actual, target in zip(current_q, target_q)
    ]


def move_robot_pair_to_initial_q(
    args: argparse.Namespace,
    *,
    robot_pair,
    flexivrdk,
    progress_callback=None,
    use_startup_waypoints: bool = False,
):
    """Move an already connected pair to init_q and enter Cartesian mode."""

    _wait_until_operational(robot_pair, float(args.enable_timeout_sec))

    states = robot_pair.states()
    current_q = ([float(value) for value in states[0].q], [float(value) for value in states[1].q])
    nullspace_postures = (
        list(args.left_nullspace_posture),
        list(args.right_nullspace_posture),
    )
    if len(nullspace_postures[0]) != len(current_q[0]) or len(nullspace_postures[1]) != len(current_q[1]):
        raise ValueError("configured null-space posture length must match each robot DoF")

    joint_mode = flexivrdk.Mode.NRT_JOINT_POSITION
    _switch_mode_and_wait(robot_pair, joint_mode, timeout_sec=float(args.enable_timeout_sec))
    # Follow the official NRT joint-position example: establish the first
    # command from state sampled *after* the mode switch.  Sending a distant
    # task posture as the first command leaves the controller handoff
    # discontinuous and can trip the simulated torque protection.
    states = robot_pair.states()
    current_q = ([float(value) for value in states[0].q], [float(value) for value in states[1].q])
    dofs = (len(nullspace_postures[0]), len(nullspace_postures[1]))
    zero_velocities = ([0.0] * dofs[0], [0.0] * dofs[1])
    max_velocities = (
        [float(args.initial_joint_max_vel_rad_s)] * dofs[0],
        [float(args.initial_joint_max_vel_rad_s)] * dofs[1],
    )
    max_accelerations = (
        [float(args.initial_joint_max_acc_rad_s2)] * dofs[0],
        [float(args.initial_joint_max_acc_rad_s2)] * dofs[1],
    )
    robot_pair.SendJointPosition(
        current_q,
        zero_velocities,
        max_velocities,
        max_accelerations,
    )
    print(
        "[DrdkTargetStreamer] NRT joint mode handoff seeded from current q "
        f"left={[round(value, 6) for value in current_q[0]]} "
        f"right={[round(value, 6) for value in current_q[1]]}",
        flush=True,
    )
    handoff_deadline = time.monotonic() + float(args.initial_joint_handoff_sec)
    while time.monotonic() < handoff_deadline:
        if not robot_pair.connected() or robot_pair.fault() or not robot_pair.operational():
            raise RuntimeError("DRDK robot pair failed during NRT joint-mode handoff")
        states = robot_pair.states()
        if progress_callback is not None:
            progress_callback(states)
        time.sleep(0.02)

    trajectory_targets = []
    if use_startup_waypoints:
        trajectory_targets.extend(
            zip(args.left_startup_waypoint, args.right_startup_waypoint, strict=True)
        )
    trajectory_targets.append(nullspace_postures)
    for target_index, target_postures in enumerate(trajectory_targets, start=1):
        phase = "startup waypoint" if target_index < len(trajectory_targets) else "initial_q"
        robot_pair.SendJointPosition(
            target_postures,
            zero_velocities,
            max_velocities,
            max_accelerations,
        )
        print(
            f"[DrdkTargetStreamer] {phase} NRT joint trajectory started "
            f"left={[round(value, 6) for value in target_postures[0]]} "
            f"right={[round(value, 6) for value in target_postures[1]]} "
            f"max_vel={float(args.initial_joint_max_vel_rad_s):.3f}rad/s "
            f"max_acc={float(args.initial_joint_max_acc_rad_s2):.3f}rad/s^2",
            flush=True,
        )
        deadline = time.monotonic() + float(args.initial_joint_timeout_sec)
        settled_since = None
        last_progress_log = 0.0
        while True:
            if not robot_pair.connected() or robot_pair.fault() or not robot_pair.operational():
                raise RuntimeError(f"DRDK robot pair failed during {phase} joint-position motion")
            states = robot_pair.states()
            if progress_callback is not None:
                progress_callback(states)
            position_errors = (
                _joint_errors(states[0].q, target_postures[0]),
                _joint_errors(states[1].q, target_postures[1]),
            )
            max_position_error = max(abs(value) for errors in position_errors for value in errors)
            max_joint_speed = max(abs(float(value)) for state in states for value in state.dq)
            within_tolerance = bool(
                max_position_error <= float(args.initial_joint_tolerance_rad)
                and max_joint_speed <= float(args.initial_joint_speed_tolerance_rad_s)
            )
            now = time.monotonic()
            if now - last_progress_log >= 1.0:
                print(
                    f"[DrdkTargetStreamer] {phase} joint trajectory progress "
                    f"max_position_error={max_position_error:.6f}rad "
                    f"max_joint_speed={max_joint_speed:.6f}rad/s",
                    flush=True,
                )
                last_progress_log = now
            if within_tolerance:
                settled_since = now if settled_since is None else settled_since
                if now - settled_since >= float(args.initial_joint_settle_sec):
                    break
            else:
                settled_since = None
            if now >= deadline:
                raise TimeoutError(
                    f"DRDK {phase} joint-position motion timed out: "
                    f"max_position_error={max_position_error:.6f}rad "
                    f"max_joint_speed={max_joint_speed:.6f}rad/s"
                )
            time.sleep(0.02)

    print(
        "[DrdkTargetStreamer] initial_q reached and settled; switching to Cartesian mode",
        flush=True,
    )

    cartesian_mode = flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE
    _switch_mode_and_wait(robot_pair, cartesian_mode, timeout_sec=float(args.enable_timeout_sec))
    if args.cartesian_impedance_control:
        stiffness = (
            list(args.left_cartesian_stiffness),
            list(args.right_cartesian_stiffness),
        )
        damping_ratio = (
            list(args.left_cartesian_damping_ratio),
            list(args.right_cartesian_damping_ratio),
        )
        robot_pair.SetCartesianImpedance(stiffness, damping_ratio)
        print(
            "[DrdkTargetStreamer] Cartesian impedance configured "
            f"left_K={stiffness[0]} right_K={stiffness[1]} "
            f"left_Z={damping_ratio[0]} right_Z={damping_ratio[1]}",
            flush=True,
        )
    # The runtime resets the null-space reference whenever this mode is
    # re-entered, so install the task initq only after the Cartesian switch.
    robot_pair.SetNullSpacePosture(nullspace_postures)
    weight = float(args.nullspace_tracking_weight)
    robot_pair.SetNullSpaceObjectives(
        linear_manipulability=(0.0, 0.0),
        angular_manipulability=(0.0, 0.0),
        ref_positions_tracking=(weight, weight),
    )
    max_contact_wrenches = (
        list(args.left_max_contact_wrench),
        list(args.right_max_contact_wrench),
    )
    if args.contact_wrench_control:
        robot_pair.SetMaxContactWrench(max_contact_wrenches)
        print(
            "[DrdkTargetStreamer] maximum contact wrench configured "
            f"left={max_contact_wrenches[0]} right={max_contact_wrenches[1]}",
            flush=True,
        )
    return nullspace_postures


def initialize_connected_robot_pair(args: argparse.Namespace, *, robot_pair, flexivrdk, progress_callback=None):
    """Perform the startup init flow on an already connected RobotPair."""

    if robot_pair.fault():
        if not args.clear_fault:
            raise RuntimeError("DRDK robot pair fault is active")
        cleared = robot_pair.ClearFault()
        if args.strict_clear_fault and not all(bool(value) for value in cleared):
            raise RuntimeError(f"DRDK failed to clear both robot faults: {cleared}")
    nullspace_postures = move_robot_pair_to_initial_q(
        args,
        robot_pair=robot_pair,
        flexivrdk=flexivrdk,
        progress_callback=progress_callback,
        use_startup_waypoints=True,
    )
    return nullspace_postures


def initialize_robot_pair(args: argparse.Namespace, *, flexivdrdk, flexivrdk, progress_callback=None):
    robot_pair = _connect_robot_pair(args, flexivdrdk=flexivdrdk)
    nullspace_postures = initialize_connected_robot_pair(
        args,
        robot_pair=robot_pair,
        flexivrdk=flexivrdk,
        progress_callback=progress_callback,
    )
    return robot_pair, nullspace_postures


def recover_connected_robot_pair_to_initial_q(
    args: argparse.Namespace,
    *,
    robot_pair,
    flexivrdk,
    progress_callback=None,
    phase_callback=None,
):
    """Run a bounded, resumable coordinated reset for an already connected pair."""

    recovery_args = argparse.Namespace(**vars(args))
    recovery_args.initial_joint_max_vel_rad_s = min(
        float(args.initial_joint_max_vel_rad_s),
        float(args.reset_joint_max_vel_rad_s),
    )
    recovery_args.initial_joint_max_acc_rad_s2 = min(
        float(args.initial_joint_max_acc_rad_s2),
        float(args.reset_joint_max_acc_rad_s2),
    )
    max_attempts = int(args.reset_max_attempts)
    last_error: Exception | None = None

    for attempt in range(1, max_attempts + 1):
        if attempt > 1:
            if phase_callback is not None:
                phase_callback("reset_retrying")
            print(
                f"[DrdkTargetStreamer] reset recovery retry {attempt}/{max_attempts} "
                f"after: {last_error}",
                flush=True,
            )
            time.sleep(float(args.reset_retry_delay_sec))

        if not robot_pair.connected():
            raise RuntimeError("DRDK robot pair disconnected during coordinated reset") from last_error
        if phase_callback is not None:
            phase_callback("reset_stopping")
        try:
            robot_pair.Stop()
        except Exception as exc:
            print(
                f"[DrdkTargetStreamer] Stop() reported {exc}; "
                "continuing because the runtime may already be stopped",
                flush=True,
            )
        if phase_callback is not None:
            phase_callback("reset_clearing_fault")
        if robot_pair.fault():
            cleared = robot_pair.ClearFault()
            if not all(bool(value) for value in cleared):
                last_error = RuntimeError(f"DRDK failed to clear both robot faults: {cleared}")
                continue
        # Stop() leaves a healthy simulated robot in a stopped/idle controller
        # state while operational() may still report true. Enable explicitly so
        # the following NRT mode switch is applicable even without an active fault.
        robot_pair.Enable()
        try:
            _wait_until_operational(robot_pair, float(args.enable_timeout_sec))
            return move_robot_pair_to_initial_q(
                recovery_args,
                robot_pair=robot_pair,
                flexivrdk=flexivrdk,
                progress_callback=progress_callback,
            )
        except Exception as exc:
            last_error = exc
            if not robot_pair.connected():
                break

    raise RuntimeError(
        f"DRDK coordinated reset exhausted {max_attempts} recovery attempts: {last_error}"
    ) from last_error


def _status_packet(
    *,
    serial: str,
    ready: bool,
    phase: str,
    reference_pose: list[float] | None,
    current_pose: list[float] | None,
    current_q: list[float] | None = None,
    tcp_wrench: list[float] | None = None,
    contact_frozen: bool = False,
    joint_tau: list[float] | None = None,
    joint_tau_dot: list[float] | None = None,
    joint_tau_ext: list[float] | None = None,
    joint_tau_max: list[float] | None = None,
    joint_torque_ratio: list[float] | None = None,
    joint_torque_frozen: bool = False,
    reset_seq: int = 0,
    error: str | None = None,
) -> dict:
    packet = {
        "schema": "flexiv_rdk_streamer_status.v1",
        "backend": "drdk",
        "serial": str(serial),
        "ready": bool(ready),
        "phase": str(phase),
        "monotonic_time": time.monotonic(),
        "reset_seq": int(reset_seq),
        "contact_frozen": bool(contact_frozen),
        "joint_torque_frozen": bool(joint_torque_frozen),
    }
    if reference_pose is not None:
        packet["reference_pose_base_tcp"] = list(reference_pose)
    if current_pose is not None:
        packet["current_pose_base_tcp"] = list(current_pose)
    if current_q is not None:
        packet["current_q"] = list(current_q)
    if tcp_wrench is not None:
        packet["tcp_wrench"] = list(tcp_wrench)
    for key, values in (
        ("joint_tau", joint_tau),
        ("joint_tau_dot", joint_tau_dot),
        ("joint_tau_ext", joint_tau_ext),
        ("joint_tau_max", joint_tau_max),
        ("joint_torque_ratio", joint_torque_ratio),
    ):
        if values is not None:
            packet[key] = list(values)
    if error:
        packet["error"] = str(error)
    return packet


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    import flexivdrdk
    import flexivrdk

    serials = {"left": args.left_serial_number, "right": args.right_serial_number}
    ports = {"left": int(args.left_port), "right": int(args.right_port)}
    target_sockets: dict[socket.socket, str] = {}
    for side in ("left", "right"):
        target_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target_socket.bind((str(args.host), ports[side]))
        target_socket.setblocking(False)
        target_sockets[target_socket] = side
    status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status_addresses = {
        "left": (str(args.left_status_host), int(args.left_status_port)),
        "right": (str(args.right_status_host), int(args.right_status_port)),
    }

    command_buffers: dict[str, dict[int, TargetCommand]] = {"left": {}, "right": {}}
    robot_pair = None
    collision_monitor = None
    reference_poses: dict[str, list[float]] = {}
    latest_input_poses: dict[str, list[float]] = {}
    contact_guard = ContactWrenchGuard(
        (args.left_max_contact_wrench, args.right_max_contact_wrench),
        enabled=bool(args.contact_wrench_control),
        trigger_ratio=float(args.contact_wrench_freeze_trigger_ratio),
        release_ratio=float(args.contact_wrench_release_ratio),
        trigger_samples=int(args.contact_wrench_trigger_samples),
        release_dwell_sec=float(args.contact_wrench_release_dwell_sec),
    )
    joint_guard: JointTorqueGuard | None = None
    last_sent_cycle = -1
    last_status_time = 0.0
    last_log_time = 0.0
    pending_reset_seq = 0
    handled_reset_seq = 0
    recovery_required = False
    recovery_phase = "fault"
    status_error: str | None = None

    def publish_status(
        ready: bool,
        *,
        phase: str,
        current_poses: dict[str, list[float]] | None = None,
        current_qs: dict[str, list[float]] | None = None,
        tcp_wrenches: dict[str, list[float]] | None = None,
        contact_frozen: dict[str, bool] | None = None,
        joint_taus: dict[str, list[float]] | None = None,
        joint_tau_dots: dict[str, list[float]] | None = None,
        joint_tau_exts: dict[str, list[float]] | None = None,
        joint_torque_ratios: dict[str, list[float]] | None = None,
        joint_torque_frozen: dict[str, bool] | None = None,
        reset_seq: int | None = None,
        error: str | None = None,
    ) -> None:
        poses = current_poses or {}
        joint_positions = current_qs or {}
        wrenches = tcp_wrenches or {}
        frozen = contact_frozen or {}
        taus = joint_taus or {}
        tau_dots = joint_tau_dots or {}
        tau_exts = joint_tau_exts or {}
        torque_ratios = joint_torque_ratios or {}
        torque_frozen = joint_torque_frozen or {}
        for side in ("left", "right"):
            packet = _status_packet(
                serial=serials[side],
                ready=ready,
                phase=phase,
                reference_pose=reference_poses.get(side),
                current_pose=poses.get(side),
                current_q=joint_positions.get(side),
                tcp_wrench=wrenches.get(side),
                contact_frozen=bool(frozen.get(side, False)),
                joint_tau=taus.get(side),
                joint_tau_dot=tau_dots.get(side),
                joint_tau_ext=tau_exts.get(side),
                joint_tau_max=(joint_guard.limits[side] if joint_guard is not None else None),
                joint_torque_ratio=torque_ratios.get(side),
                joint_torque_frozen=bool(torque_frozen.get(side, False)),
                reset_seq=handled_reset_seq if reset_seq is None else int(reset_seq),
                error=error,
            )
            status_socket.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                status_addresses[side],
            )

    def publish_joint_initialization(states, *, reset_seq: int = 0) -> None:
        publish_status(
            False,
            phase="joint_initializing",
            current_poses={
                "left": [float(value) for value in states[0].tcp_pose],
                "right": [float(value) for value in states[1].tcp_pose],
            },
            current_qs={
                "left": [float(value) for value in states[0].q],
                "right": [float(value) for value in states[1].q],
            },
            reset_seq=reset_seq,
        )

    def latch_cartesian_references() -> None:
        nonlocal reference_poses, latest_input_poses
        states = robot_pair.states()
        reference_poses = {
            "left": [float(value) for value in states[0].tcp_pose],
            "right": [float(value) for value in states[1].tcp_pose],
        }
        latest_input_poses = {
            side: list(pose) for side, pose in reference_poses.items()
        }
        contact_guard.reset()
        if joint_guard is not None:
            joint_guard.reset(reference_poses, now=time.monotonic())
        print(
            "[DrdkTargetStreamer] RobotPair operational; latched TCP references "
            f"left={format_pose_xyz_quat(reference_poses['left'])} "
            f"right={format_pose_xyz_quat(reference_poses['right'])}",
            flush=True,
        )

    def recover_to_initial_q(reset_seq: int) -> None:
        nonlocal robot_pair, collision_monitor, last_sent_cycle
        nonlocal joint_guard, recovery_required, recovery_phase, status_error
        print(f"[DrdkTargetStreamer] reset seq={reset_seq}: stopping both robots", flush=True)
        publish_status(False, phase="reset_stopping", reset_seq=reset_seq)
        if robot_pair is None or not robot_pair.connected():
            stop_self_collision_monitor(collision_monitor)
            robot_pair = _connect_robot_pair(args, flexivdrdk=flexivdrdk)
            torque_limits = joint_torque_limits(robot_pair, flexivrdk, args.joint_group)
            joint_guard = JointTorqueGuard(
                torque_limits,
                enabled=bool(args.joint_torque_control),
                trigger_ratio=float(args.joint_torque_trigger_ratio),
                release_ratio=float(args.joint_torque_release_ratio),
                trigger_samples=int(args.joint_torque_trigger_samples),
                release_dwell_sec=float(args.joint_torque_release_dwell_sec),
                prediction_horizon_sec=float(args.joint_torque_prediction_horizon_sec),
                rollback_sec=float(args.joint_torque_rollback_sec),
            )
            collision_monitor = start_self_collision_monitor(
                args,
                robot_pair=robot_pair,
                flexivdrdk=flexivdrdk,
            )
        recover_connected_robot_pair_to_initial_q(
            args,
            robot_pair=robot_pair,
            flexivrdk=flexivrdk,
            progress_callback=lambda states: publish_joint_initialization(states, reset_seq=reset_seq),
            phase_callback=lambda phase: publish_status(False, phase=phase, reset_seq=reset_seq),
        )
        latch_cartesian_references()
        states = robot_pair.states()
        publish_status(
            True,
            phase="ready",
            current_poses=reference_poses,
            current_qs={
                "left": [float(value) for value in states[0].q],
                "right": [float(value) for value in states[1].q],
            },
            reset_seq=reset_seq,
        )
        for buffer in command_buffers.values():
            buffer.clear()
        last_sent_cycle = -1
        recovery_required = False
        recovery_phase = "fault"
        status_error = None
        print(f"[DrdkTargetStreamer] reset seq={reset_seq}: READY at initial_q", flush=True)

    print(
        "[DrdkTargetStreamer] listening "
        f"left={args.host}:{args.left_port}/{args.left_serial_number} "
        f"right={args.host}:{args.right_port}/{args.right_serial_number}",
        flush=True,
    )
    try:
        robot_pair = _connect_robot_pair(args, flexivdrdk=flexivdrdk)
        torque_limits = joint_torque_limits(robot_pair, flexivrdk, args.joint_group)
        joint_guard = JointTorqueGuard(
            torque_limits,
            enabled=bool(args.joint_torque_control),
            trigger_ratio=float(args.joint_torque_trigger_ratio),
            release_ratio=float(args.joint_torque_release_ratio),
            trigger_samples=int(args.joint_torque_trigger_samples),
            release_dwell_sec=float(args.joint_torque_release_dwell_sec),
            prediction_horizon_sec=float(args.joint_torque_prediction_horizon_sec),
            rollback_sec=float(args.joint_torque_rollback_sec),
        )
        print(
            "[DrdkTargetStreamer] joint torque guard configured "
            f"left_tau_max={[round(value, 3) for value in torque_limits[0]]} "
            f"right_tau_max={[round(value, 3) for value in torque_limits[1]]} "
            f"trigger={float(args.joint_torque_trigger_ratio):.2f} "
            f"release={float(args.joint_torque_release_ratio):.2f}",
            flush=True,
        )
        collision_monitor = start_self_collision_monitor(
            args,
            robot_pair=robot_pair,
            flexivdrdk=flexivdrdk,
        )
        try:
            nullspace_postures = initialize_connected_robot_pair(
                args,
                robot_pair=robot_pair,
                flexivrdk=flexivrdk,
                progress_callback=publish_joint_initialization,
            )
            latch_cartesian_references()
            states = robot_pair.states()
            print(
                "[DrdkTargetStreamer] null-space posture initialized "
                f"left={[round(value, 6) for value in nullspace_postures[0]]} "
                f"right={[round(value, 6) for value in nullspace_postures[1]]}",
                flush=True,
            )
            publish_status(
                True,
                phase="ready",
                current_poses=reference_poses,
                current_qs={
                    "left": [float(value) for value in states[0].q],
                    "right": [float(value) for value in states[1].q],
                },
            )
        except Exception as exc:
            recovery_required = True
            recovery_phase = "fault"
            status_error = str(exc)
            print(
                f"[DrdkTargetStreamer] startup initialization not ready: {status_error}; "
                "waiting for coordinated reset",
                flush=True,
            )
            publish_status(False, phase=recovery_phase, error=status_error)
        last_status_time = time.monotonic()

        while True:
            readable, _, _ = select.select(list(target_sockets), [], [], 0.05)
            for target_socket in readable:
                side = target_sockets[target_socket]
                while True:
                    try:
                        data, _address = target_socket.recvfrom(65536)
                    except BlockingIOError:
                        break
                    try:
                        packet = json.loads(data.decode("utf-8"))
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    reset_seq = parse_reset_request_seq(
                        packet,
                        serial_number=serials[side],
                        joint_group=args.joint_group,
                    )
                    if reset_seq is not None:
                        pending_reset_seq = max(pending_reset_seq, reset_seq)
                    command = parse_target_command_packet(
                        packet,
                        serial_number=serials[side],
                        joint_group=args.joint_group,
                        max_age_sec=float(args.max_age_sec),
                    )
                    if command is not None:
                        buffer_target_command(command_buffers[side], command)

            if pending_reset_seq > handled_reset_seq:
                handled_reset_seq = pending_reset_seq
                try:
                    recover_to_initial_q(handled_reset_seq)
                except Exception as exc:
                    recovery_required = True
                    recovery_phase = "reset_failed"
                    status_error = str(exc)
                    print(
                        f"[DrdkTargetStreamer] reset seq={handled_reset_seq} failed: {status_error}",
                        flush=True,
                    )
                    publish_status(
                        False,
                        phase=recovery_phase,
                        reset_seq=handled_reset_seq,
                        error=status_error,
                    )
                last_status_time = time.monotonic()
                continue

            collision_stopped = bool(
                collision_monitor is not None
                and robot_pair is not None
                and robot_pair.stopped()
            )
            healthy = bool(
                robot_pair is not None
                and robot_pair.connected()
                and not robot_pair.fault()
                and robot_pair.operational()
                and not collision_stopped
            )
            if recovery_required or not healthy:
                recovery_required = True
                if collision_stopped:
                    recovery_phase = "self_collision_stopped"
                    status_error = (
                        "DRDK SelfCollisionMonitor stopped both robots; "
                        "remove the inter-arm proximity condition, then request a coordinated reset"
                    )
                now = time.monotonic()
                if now - last_status_time >= 0.1:
                    publish_status(
                        False,
                        phase=recovery_phase,
                        error=status_error or "robot pair requires coordinated reset",
                    )
                    last_status_time = now
                continue

            synchronized = pop_synchronized_target_pair(command_buffers, after_cycle=last_sent_cycle)
            raw_command_poses = None
            synchronized_cycle = None
            if synchronized is not None and synchronized[0].servo_cycle > last_sent_cycle:
                left, right = synchronized
                raw_command_poses = (
                    list(left.pose if left.control_active else reference_poses["left"]),
                    list(right.pose if right.control_active else reference_poses["right"]),
                )
                latest_input_poses = {
                    "left": list(raw_command_poses[0]),
                    "right": list(raw_command_poses[1]),
                }
                synchronized_cycle = left.servo_cycle
            states = robot_pair.states()
            now = time.monotonic()
            guard_events = contact_guard.update(states, latest_input_poses, now=now)
            for side, event in guard_events:
                print(
                    f"[DrdkTargetStreamer] contact guard {event} {side} "
                    f"wrench={[round(value, 3) for value in contact_guard.latest_wrenches[side]]}",
                    flush=True,
                )
            post_contact_inputs = {
                side: contact_guard.command_pose(side, latest_input_poses[side])
                for side in ("left", "right")
            }
            torque_guard_events = joint_guard.update(states, post_contact_inputs, now=now)
            for side, event in torque_guard_events:
                ratios = joint_guard.latest_ratios[side]
                peak_joint = max(range(len(ratios)), key=ratios.__getitem__)
                print(
                    f"[DrdkTargetStreamer] joint torque guard {event} {side} "
                    f"joint=A{peak_joint + 1} ratio={ratios[peak_joint]:.3f} "
                    f"tau={joint_guard.latest_tau[side][peak_joint]:.3f}Nm "
                    f"tau_ext={joint_guard.latest_tau_ext[side][peak_joint]:.3f}Nm "
                    f"tau_dot={joint_guard.latest_tau_dot[side][peak_joint]:.3f}Nm/s",
                    flush=True,
                )
            command_poses = None
            if raw_command_poses is not None:
                contact_command_poses = (
                    contact_guard.command_pose("left", raw_command_poses[0]),
                    contact_guard.command_pose("right", raw_command_poses[1]),
                )
                command_poses = (
                    joint_guard.command_pose("left", contact_command_poses[0]),
                    joint_guard.command_pose("right", contact_command_poses[1]),
                )
                last_sent_cycle = int(synchronized_cycle)
            elif guard_events or torque_guard_events:
                contact_command_poses = (
                    contact_guard.command_pose("left", latest_input_poses["left"]),
                    contact_guard.command_pose("right", latest_input_poses["right"]),
                )
                command_poses = (
                    joint_guard.command_pose("left", contact_command_poses[0]),
                    joint_guard.command_pose("right", contact_command_poses[1]),
                )
            if command_poses is not None:
                robot_pair.SendCartesianMotionForce(
                    command_poses,
                    max_linear_vel=(float(args.max_linear_speed_m_s),) * 2,
                    max_angular_vel=(float(args.max_angular_speed_rad_s),) * 2,
                    max_linear_acc=(float(args.max_linear_acc_m_s2),) * 2,
                    max_angular_acc=(float(args.max_angular_acc_rad_s2),) * 2,
                )
                joint_guard.record_command("left", command_poses[0], now=now)
                joint_guard.record_command("right", command_poses[1], now=now)

            if command_poses is not None or now - last_status_time >= 0.1:
                current_poses = {
                    "left": [float(value) for value in states[0].tcp_pose],
                    "right": [float(value) for value in states[1].tcp_pose],
                }
                publish_status(
                    True,
                    phase="ready",
                    current_poses=current_poses,
                    current_qs={
                        "left": [float(value) for value in states[0].q],
                        "right": [float(value) for value in states[1].q],
                    },
                    tcp_wrenches={
                        "left": list(contact_guard.latest_wrenches["left"]),
                        "right": list(contact_guard.latest_wrenches["right"]),
                    },
                    contact_frozen=dict(contact_guard.frozen),
                    joint_taus=dict(joint_guard.latest_tau),
                    joint_tau_dots=dict(joint_guard.latest_tau_dot),
                    joint_tau_exts=dict(joint_guard.latest_tau_ext),
                    joint_torque_ratios=dict(joint_guard.latest_ratios),
                    joint_torque_frozen=dict(joint_guard.frozen),
                )
                last_status_time = now
                if command_poses is not None and args.log_hz > 0.0 and now - last_log_time >= 1.0 / args.log_hz:
                    print(
                        f"[DrdkTargetStreamer] sent synchronized cycle={last_sent_cycle} "
                        f"left={format_pose_xyz_quat(command_poses[0])} "
                        f"right={format_pose_xyz_quat(command_poses[1])}",
                        flush=True,
                    )
                    last_log_time = now
    except KeyboardInterrupt:
        publish_status(False, phase="stopped")
        return 130
    except Exception as exc:
        print(f"[DrdkTargetStreamer] startup failed: {exc}", flush=True)
        publish_status(False, phase="fault", error=str(exc))
        return 1
    finally:
        stop_self_collision_monitor(collision_monitor)
        for target_socket in target_sockets:
            target_socket.close()
        status_socket.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
