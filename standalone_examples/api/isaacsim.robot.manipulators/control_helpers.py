"""Control-loop helpers for the Flexiv Quest follow scene."""

from __future__ import annotations

import math
import time
from typing import NamedTuple


class TargetPosePublishGate(NamedTuple):
    period_cycles: int

    @classmethod
    def from_hz(cls, publish_hz: float, *, physics_freq: float) -> "TargetPosePublishGate":
        if publish_hz <= 0.0:
            return cls(period_cycles=1)
        return cls(period_cycles=max(1, int(round(float(physics_freq) / float(publish_hz)))))

    def should_publish(self, servo_cycle: int) -> bool:
        return int(servo_cycle) % self.period_cycles == 0


class StepRateLimiter:
    def __init__(self, step_hz: float, *, time_fn=time.perf_counter, sleep_fn=time.sleep) -> None:
        self._period = 0.0 if step_hz <= 0.0 else 1.0 / float(step_hz)
        self._time_fn = time_fn
        self._sleep_fn = sleep_fn
        self._next_time = self._time_fn()

    def sleep(self) -> None:
        if self._period <= 0.0:
            return
        self._next_time += self._period
        now = self._time_fn()
        delay = self._next_time - now
        if delay > 0.0:
            self._sleep_fn(delay)
        elif delay < -self._period:
            self._next_time = now


def format_float_list(values, *, precision: int = 4) -> str:
    return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"


def format_pose_xyz_quat(pose_base_tcp, *, precision: int = 4) -> str:
    pose = [float(value) for value in pose_base_tcp]
    if len(pose) != 7:
        raise ValueError("pose_base_tcp must contain 7 values")
    return f"pose_xyz={format_float_list(pose[:3], precision=precision)} pose_quat={format_float_list(pose[3:], precision=precision)}"


def format_state_torque_telemetry(
    *,
    servo_cycle: int,
    q,
    dq,
    target_drives,
    torque_norm: float,
    current_pose_base_tcp,
    control_pose_base_tcp,
) -> str:
    current_pose = [float(value) for value in current_pose_base_tcp]
    target_pose = [float(value) for value in control_pose_base_tcp]
    pos_error = [target_pose[index] - current_pose[index] for index in range(3)]
    return (
        "[FlexivStudioBall] state_torque "
        f"cycle={int(servo_cycle)} "
        f"q={format_float_list(q)} "
        f"dq={format_float_list(dq)} "
        f"tau={format_float_list(target_drives)} "
        f"tau_norm={float(torque_norm):.4f} "
        f"tcp_xyz={format_float_list(current_pose[:3])} "
        f"target_xyz={format_float_list(target_pose[:3])} "
        f"pos_err={format_float_list(pos_error)}"
    )


def should_poll_simplugin_target_drives(
    *,
    connected: bool,
    runtime_target_active: bool,
) -> bool:
    return bool(connected) and bool(runtime_target_active)


def target_pose_control_is_active(
    *,
    quest_target_receiver_enabled: bool,
    latest_quest_target,
) -> bool:
    if quest_target_receiver_enabled:
        return latest_quest_target is not None
    return True


def cartesian_pose_changed(
    reference_pose,
    candidate_pose,
    *,
    position_tolerance_m: float = 1e-4,
    orientation_tolerance_rad: float = 1e-3,
) -> bool:
    """Return whether two RDK poses differ beyond translation/rotation tolerances.

    Poses use Flexiv order ``[x, y, z, qw, qx, qy, qz]``. Quaternion sign is
    ignored, so ``q`` and ``-q`` represent the same orientation.
    """

    reference = [float(value) for value in reference_pose]
    candidate = [float(value) for value in candidate_pose]
    if len(reference) != 7 or len(candidate) != 7:
        raise ValueError("Cartesian poses must contain 7 values")
    linear_error = math.sqrt(sum((candidate[index] - reference[index]) ** 2 for index in range(3)))
    if linear_error > max(0.0, float(position_tolerance_m)):
        return True

    def normalized_quaternion(pose):
        quaternion = pose[3:]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm <= 0.0:
            raise ValueError("Cartesian pose quaternion must be non-zero")
        return [value / norm for value in quaternion]

    reference_quaternion = normalized_quaternion(reference)
    candidate_quaternion = normalized_quaternion(candidate)
    dot = abs(sum(a * b for a, b in zip(reference_quaternion, candidate_quaternion)))
    angular_error = 2.0 * math.acos(min(1.0, max(-1.0, dot)))
    return angular_error > max(0.0, float(orientation_tolerance_rad))


def rdk_streamer_status_is_ready(
    packet,
    *,
    serial_number: str,
    max_age_sec: float,
    now: float | None = None,
) -> bool:
    """Validate a fresh positive readiness packet from one external streamer."""

    if not isinstance(packet, dict):
        return False
    if packet.get("schema") != "flexiv_rdk_streamer_status.v1":
        return False
    if str(packet.get("serial", "")) != str(serial_number):
        return False
    if packet.get("ready") is not True:
        return False
    try:
        packet_time = float(packet["monotonic_time"])
    except (KeyError, TypeError, ValueError):
        return False
    current_time = time.monotonic() if now is None else float(now)
    return float(max_age_sec) <= 0.0 or current_time - packet_time <= float(max_age_sec)
