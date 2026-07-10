"""Control-loop helpers for the Flexiv Quest follow scene."""

from __future__ import annotations

import time
from typing import NamedTuple

from elements_studio_utils import CartJogCommand


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


def pose_error_to_cart_jog_cmd(
    pose_error_base_tcp: list[float],
    *,
    position_deadband: float,
    max_step_size: float,
    vel_scale: float,
) -> CartJogCommand:
    if len(pose_error_base_tcp) != 7:
        raise ValueError("pose_error_base_tcp must contain 7 values")
    position_error = [float(value) for value in pose_error_base_tcp[:3]]
    axis = max(range(3), key=lambda index: abs(position_error[index]))
    error = position_error[axis]
    if abs(error) <= float(position_deadband):
        return CartJogCommand(jog_index=0, jog_dir=0, step_size=0.0, jog_axis_type=0, vel_scale=0.0)
    return CartJogCommand(
        jog_index=axis,
        jog_dir=1 if error > 0.0 else 2,
        step_size=min(abs(error), float(max_step_size)),
        jog_axis_type=0,
        vel_scale=float(vel_scale),
    )


def select_cart_jog_command(
    *,
    control_pose_base_tcp: list[float],
    current_pose_base_tcp: list[float],
    can_jog: bool,
    position_deadband: float,
    max_step_size: float,
    vel_scale: float,
) -> tuple[CartJogCommand, list[float]]:
    pose_error_base_tcp = [
        float(control_pose_base_tcp[0]) - float(current_pose_base_tcp[0]),
        float(control_pose_base_tcp[1]) - float(current_pose_base_tcp[1]),
        float(control_pose_base_tcp[2]) - float(current_pose_base_tcp[2]),
        1.0,
        0.0,
        0.0,
        0.0,
    ]
    if not can_jog:
        return (
            CartJogCommand(jog_index=0, jog_dir=0, step_size=0.0, jog_axis_type=0, vel_scale=0.0),
            pose_error_base_tcp,
        )
    return (
        pose_error_to_cart_jog_cmd(
            pose_error_base_tcp,
            position_deadband=position_deadband,
            max_step_size=max_step_size,
            vel_scale=vel_scale,
        ),
        pose_error_base_tcp,
    )


class VirtualJogFeedback:
    def __init__(self) -> None:
        self._pose_base_tcp: list[float] | None = None

    def reset(self) -> None:
        self._pose_base_tcp = None

    def current_pose(self, measured_pose_base_tcp: list[float]) -> list[float]:
        measured_pose = [float(value) for value in measured_pose_base_tcp]
        if len(measured_pose) != 7:
            raise ValueError("measured_pose_base_tcp must contain 7 values")
        if self._pose_base_tcp is None:
            self._pose_base_tcp = list(measured_pose)
        return list(self._pose_base_tcp)

    def advance(self, command: CartJogCommand) -> None:
        if self._pose_base_tcp is None:
            return
        if command.jog_dir not in {1, 2} or command.step_size <= 0.0:
            return
        if command.jog_index not in {0, 1, 2}:
            return
        sign = 1.0 if command.jog_dir == 1 else -1.0
        self._pose_base_tcp[command.jog_index] += sign * float(command.step_size)


def format_float_list(values, *, precision: int = 4) -> str:
    return "[" + ", ".join(f"{float(value):.{precision}f}" for value in values) + "]"


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


def format_jog_command_telemetry(
    *,
    servo_cycle: int,
    command: CartJogCommand,
    pose_error_base_tcp,
    can_jog: bool,
) -> str:
    return (
        "[FlexivStudioBall] jog_cmd "
        f"cycle={int(servo_cycle)} "
        f"can_jog={bool(can_jog)} "
        f"jog_index={int(command.jog_index)} "
        f"jog_dir={int(command.jog_dir)} "
        f"step_size={float(command.step_size):.4f} "
        f"axis_type={int(command.jog_axis_type)} "
        f"vel_scale={float(command.vel_scale):.4f} "
        f"pos_err={format_float_list([float(value) for value in pose_error_base_tcp[:3]])}"
    )


def should_poll_simplugin_target_drives(
    *,
    connected: bool,
    disable_simplugin_target_drives: bool,
    runtime_target_active: bool,
) -> bool:
    return bool(connected) and bool(runtime_target_active) and not bool(disable_simplugin_target_drives)


def target_pose_control_is_active(
    *,
    quest_target_receiver_enabled: bool,
    latest_quest_target,
) -> bool:
    _ = quest_target_receiver_enabled
    _ = latest_quest_target
    return True
