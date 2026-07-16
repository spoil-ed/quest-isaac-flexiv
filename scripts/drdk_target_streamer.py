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
    parser.add_argument("--left-nullspace-posture", required=True)
    parser.add_argument("--right-nullspace-posture", required=True)
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
    ):
        if float(getattr(args, option)) <= 0.0:
            parser.error(f"--{option.replace('_', '-')} must be positive")
    if float(args.initial_joint_handoff_sec) < 0.0:
        parser.error("--initial-joint-handoff-sec must be non-negative")
    args.left_translation_in_world = parse_csv_floats(
        args.left_translation_in_world, expected=3, name="left translation"
    )
    args.right_translation_in_world = parse_csv_floats(
        args.right_translation_in_world, expected=3, name="right translation"
    )
    args.left_nullspace_posture = parse_csv_floats(
        args.left_nullspace_posture, expected=7, name="left null-space posture"
    )
    args.right_nullspace_posture = parse_csv_floats(
        args.right_nullspace_posture, expected=7, name="right null-space posture"
    )
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


def _joint_errors(current_q, target_q) -> list[float]:
    return [
        math.atan2(math.sin(float(actual) - float(target)), math.cos(float(actual) - float(target)))
        for actual, target in zip(current_q, target_q)
    ]


def initialize_robot_pair(args: argparse.Namespace, *, flexivdrdk, flexivrdk, progress_callback=None):
    robot_pair = _connect_robot_pair(args, flexivdrdk=flexivdrdk)
    if robot_pair.fault():
        if not args.clear_fault:
            raise RuntimeError("DRDK robot pair fault is active")
        cleared = robot_pair.ClearFault()
        if args.strict_clear_fault and not all(bool(value) for value in cleared):
            raise RuntimeError(f"DRDK failed to clear both robot faults: {cleared}")
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
    if any(current_mode != joint_mode for current_mode in robot_pair.mode()):
        robot_pair.SwitchMode(joint_mode)
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

    robot_pair.SendJointPosition(
        nullspace_postures,
        zero_velocities,
        max_velocities,
        max_accelerations,
    )
    print(
        "[DrdkTargetStreamer] initial NRT joint trajectory started "
        f"left={[round(value, 6) for value in nullspace_postures[0]]} "
        f"right={[round(value, 6) for value in nullspace_postures[1]]} "
        f"max_vel={float(args.initial_joint_max_vel_rad_s):.3f}rad/s "
        f"max_acc={float(args.initial_joint_max_acc_rad_s2):.3f}rad/s^2",
        flush=True,
    )
    deadline = time.monotonic() + float(args.initial_joint_timeout_sec)
    settled_since = None
    last_progress_log = 0.0
    while True:
        if not robot_pair.connected() or robot_pair.fault() or not robot_pair.operational():
            raise RuntimeError("DRDK robot pair failed during initial joint-position motion")
        states = robot_pair.states()
        if progress_callback is not None:
            progress_callback(states)
        position_errors = (
            _joint_errors(states[0].q, nullspace_postures[0]),
            _joint_errors(states[1].q, nullspace_postures[1]),
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
                "[DrdkTargetStreamer] initial joint trajectory progress "
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
                "DRDK initial joint-position motion timed out: "
                f"max_position_error={max_position_error:.6f}rad "
                f"max_joint_speed={max_joint_speed:.6f}rad/s"
            )
        time.sleep(0.02)

    print(
        "[DrdkTargetStreamer] initial_q reached and settled; switching to Cartesian mode",
        flush=True,
    )

    cartesian_mode = flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE
    if any(current_mode != cartesian_mode for current_mode in robot_pair.mode()):
        robot_pair.SwitchMode(cartesian_mode)
    # The runtime resets the null-space reference whenever this mode is
    # re-entered, so install the task initq only after the Cartesian switch.
    robot_pair.SetNullSpacePosture(nullspace_postures)
    weight = float(args.nullspace_tracking_weight)
    robot_pair.SetNullSpaceObjectives(
        linear_manipulability=(0.0, 0.0),
        angular_manipulability=(0.0, 0.0),
        ref_positions_tracking=(weight, weight),
    )
    return robot_pair, nullspace_postures


def _status_packet(
    *,
    serial: str,
    ready: bool,
    phase: str,
    reference_pose: list[float] | None,
    current_pose: list[float] | None,
    current_q: list[float] | None = None,
) -> dict:
    packet = {
        "schema": "flexiv_rdk_streamer_status.v1",
        "backend": "drdk",
        "serial": str(serial),
        "ready": bool(ready),
        "phase": str(phase),
        "monotonic_time": time.monotonic(),
    }
    if reference_pose is not None:
        packet["reference_pose_base_tcp"] = list(reference_pose)
    if current_pose is not None:
        packet["current_pose_base_tcp"] = list(current_pose)
    if current_q is not None:
        packet["current_q"] = list(current_q)
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
    reference_poses: dict[str, list[float]] = {}
    last_sent_cycle = -1
    last_status_time = 0.0
    last_log_time = 0.0

    def publish_status(
        ready: bool,
        *,
        phase: str,
        current_poses: dict[str, list[float]] | None = None,
        current_qs: dict[str, list[float]] | None = None,
    ) -> None:
        poses = current_poses or {}
        joint_positions = current_qs or {}
        for side in ("left", "right"):
            packet = _status_packet(
                serial=serials[side],
                ready=ready,
                phase=phase,
                reference_pose=reference_poses.get(side),
                current_pose=poses.get(side),
                current_q=joint_positions.get(side),
            )
            status_socket.sendto(
                json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                status_addresses[side],
            )

    print(
        "[DrdkTargetStreamer] listening "
        f"left={args.host}:{args.left_port}/{args.left_serial_number} "
        f"right={args.host}:{args.right_port}/{args.right_serial_number}",
        flush=True,
    )
    try:
        def publish_joint_initialization(states) -> None:
            publish_status(
                True,
                phase="joint_initializing",
                current_poses={
                    "left": [float(value) for value in states[0].tcp_pose],
                    "right": [float(value) for value in states[1].tcp_pose],
                },
                current_qs={
                    "left": [float(value) for value in states[0].q],
                    "right": [float(value) for value in states[1].q],
                },
            )

        # Use the runtime's official NRT joint trajectory generator to reach
        # task initq first. Only then enter Cartesian mode, reinstall initq as
        # its null-space reference, latch TCP, and announce task readiness.
        robot_pair, nullspace_postures = initialize_robot_pair(
            args,
            flexivdrdk=flexivdrdk,
            flexivrdk=flexivrdk,
            progress_callback=publish_joint_initialization,
        )
        states = robot_pair.states()
        reference_poses = {
            "left": [float(value) for value in states[0].tcp_pose],
            "right": [float(value) for value in states[1].tcp_pose],
        }
        print(
            "[DrdkTargetStreamer] RobotPair operational; latched TCP references "
            f"left={format_pose_xyz_quat(reference_poses['left'])} "
            f"right={format_pose_xyz_quat(reference_poses['right'])}",
            flush=True,
        )
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
                    command = parse_target_command_packet(
                        packet,
                        serial_number=serials[side],
                        joint_group=args.joint_group,
                        max_age_sec=float(args.max_age_sec),
                    )
                    if command is not None:
                        buffer_target_command(command_buffers[side], command)

            if not robot_pair.connected() or robot_pair.fault() or not robot_pair.operational():
                raise RuntimeError("DRDK robot pair is disconnected, faulted, or not operational")

            synchronized = pop_synchronized_target_pair(command_buffers, after_cycle=last_sent_cycle)
            command_poses = None
            if synchronized is not None and synchronized[0].servo_cycle > last_sent_cycle:
                left, right = synchronized
                command_poses = (
                    list(left.pose if left.control_active else reference_poses["left"]),
                    list(right.pose if right.control_active else reference_poses["right"]),
                )
                robot_pair.SendCartesianMotionForce(
                    command_poses,
                    max_linear_vel=(float(args.max_linear_speed_m_s),) * 2,
                    max_angular_vel=(float(args.max_angular_speed_rad_s),) * 2,
                    max_linear_acc=(float(args.max_linear_acc_m_s2),) * 2,
                    max_angular_acc=(float(args.max_angular_acc_rad_s2),) * 2,
                )
                last_sent_cycle = left.servo_cycle

            now = time.monotonic()
            if command_poses is not None or now - last_status_time >= 0.1:
                states = robot_pair.states()
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
        print(f"[DrdkTargetStreamer] fault latched: {exc}", flush=True)
        publish_status(False, phase="fault")
        return 1
    finally:
        for target_socket in target_sockets:
            target_socket.close()
        status_socket.close()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
