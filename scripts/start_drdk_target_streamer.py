#!/usr/bin/env python3
"""Start the host-side Flexiv DRDK dual-arm target streamer."""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import yaml

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import flexiv_runtime


RDK_COMPAT_PATH = flexiv_runtime.REPO_ROOT / ".deps" / "flexivrdk_1_9_1"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=flexiv_runtime.ISAAC_PYTHON)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--left-port", type=int, default=57680)
    parser.add_argument("--right-port", type=int, default=57681)
    parser.add_argument("--left-serial-number", default="Rizon4-qSaFLh")
    parser.add_argument("--right-serial-number", default="Rizon4-I0LIRN")
    parser.add_argument("--joint-group", default="ARM_1")
    parser.add_argument("--left-status-host", default="127.0.0.1")
    parser.add_argument("--left-status-port", type=int, default=57682)
    parser.add_argument("--right-status-host", default="127.0.0.1")
    parser.add_argument("--right-status-port", type=int, default=57683)
    parser.add_argument(
        "--pipeline-config",
        type=Path,
        default=None,
        help="Pipeline YAML whose control.drdk section supplies DRDK safety parameters.",
    )
    parser.add_argument("--left-translation-in-world", default=None)
    parser.add_argument("--right-translation-in-world", default=None)
    parser.add_argument("--cartesian-impedance-control", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--left-cartesian-stiffness", default=None)
    parser.add_argument("--right-cartesian-stiffness", default=None)
    parser.add_argument("--left-cartesian-damping-ratio", default=None)
    parser.add_argument("--right-cartesian-damping-ratio", default=None)
    parser.add_argument(
        "--output-torque-regulator",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    parser.add_argument("--output-torque-limiting-factor", type=float, default=None)
    parser.add_argument("--output-torque-error-threshold", type=int, default=None)
    parser.add_argument("--safety-password-env", default=None)
    parser.add_argument(
        "--self-collision-monitor",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    parser.add_argument("--self-collision-min-distance-m", type=float, default=None)
    parser.add_argument("--self-collision-loop-interval-ms", type=int, default=None)
    parser.add_argument("--self-collision-skip-link", action="append", default=None)
    parser.add_argument("--contact-wrench-control", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--left-max-contact-wrench", default=None)
    parser.add_argument("--right-max-contact-wrench", default=None)
    parser.add_argument("--contact-wrench-freeze-trigger-ratio", type=float, default=None)
    parser.add_argument("--contact-wrench-release-ratio", type=float, default=None)
    parser.add_argument("--contact-wrench-trigger-samples", type=int, default=None)
    parser.add_argument("--contact-wrench-release-dwell-sec", type=float, default=None)
    parser.add_argument("--joint-torque-control", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--joint-torque-trigger-ratio", type=float, default=None)
    parser.add_argument("--joint-torque-release-ratio", type=float, default=None)
    parser.add_argument("--joint-torque-trigger-samples", type=int, default=None)
    parser.add_argument("--joint-torque-release-dwell-sec", type=float, default=None)
    parser.add_argument("--joint-torque-prediction-horizon-sec", type=float, default=None)
    parser.add_argument("--joint-torque-rollback-sec", type=float, default=None)
    parser.add_argument("--target-resampling-control", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument("--target-resample-rate-hz", type=float, default=None)
    parser.add_argument("--target-prediction-horizon-sec", type=float, default=None)
    parser.add_argument("--target-velocity-filter-alpha", type=float, default=None)
    parser.add_argument("--target-feedforward-scale", type=float, default=None)
    parser.add_argument("--target-max-linear-feedforward-m-s", type=float, default=None)
    parser.add_argument("--target-max-angular-feedforward-rad-s", type=float, default=None)
    parser.add_argument("--target-torque-soft-ratio", type=float, default=None)
    parser.add_argument("--target-min-motion-scale", type=float, default=None)
    parser.add_argument("--target-linear-velocity-deadband-m-s", type=float, default=None)
    parser.add_argument("--target-angular-velocity-deadband-rad-s", type=float, default=None)
    parser.add_argument(
        "--scene-config",
        type=Path,
        required=True,
        help="Scene YAML whose left/right initial_q values define the DRDK null-space posture.",
    )
    parser.add_argument("--nullspace-tracking-weight", type=float, default=1.0)
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
    parser.add_argument("--reset-joint-max-vel-rad-s", type=float, default=0.2)
    parser.add_argument("--reset-joint-max-acc-rad-s2", type=float, default=0.4)
    parser.add_argument("--reset-max-attempts", type=int, default=3)
    parser.add_argument("--reset-retry-delay-sec", type=float, default=0.5)
    parser.add_argument(
        "--reset-motion-method",
        choices=("send_joint_position", "movej"),
        default=None,
    )
    parser.add_argument("--max-linear-speed-m-s", type=float, default=0.5)
    parser.add_argument("--max-angular-speed-rad-s", type=float, default=0.75)
    parser.add_argument("--max-linear-acc-m-s2", type=float, default=2.0)
    parser.add_argument("--max-angular-acc-rad-s2", type=float, default=5.0)
    parser.add_argument("--clear-fault", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--strict-clear-fault", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--log-hz", type=float, default=2.0)
    args = parser.parse_args(argv)
    args.python = flexiv_runtime.python_executable_or_current(args.python)
    drdk_control = {}
    if args.pipeline_config is not None:
        args.pipeline_config = args.pipeline_config.expanduser().resolve()
        drdk_control = load_drdk_control(args.pipeline_config)
    output_torque_regulator = drdk_control.get("output_torque_regulator") or {}
    self_collision = drdk_control.get("self_collision_monitor") or {}
    cartesian_impedance = drdk_control.get("cartesian_impedance") or {}
    contact_wrench = drdk_control.get("contact_wrench") or {}
    joint_torque = drdk_control.get("joint_torque") or {}
    target_resampling = drdk_control.get("target_resampling") or {}
    reset_motion = drdk_control.get("reset_motion") or {}
    args.reset_motion_method = str(
        _configured(
            args.reset_motion_method,
            reset_motion.get("method"),
            "send_joint_position",
        )
    )
    if args.reset_motion_method not in {"send_joint_position", "movej"}:
        raise ValueError(
            "reset motion method must be 'send_joint_position' or 'movej'"
        )
    args.output_torque_regulator = _configured(
        args.output_torque_regulator,
        output_torque_regulator.get("enabled"),
        False,
    )
    args.output_torque_limiting_factor = float(
        _configured(
            args.output_torque_limiting_factor,
            output_torque_regulator.get("limiting_factor"),
            0.85,
        )
    )
    args.output_torque_error_threshold = int(
        _configured(
            args.output_torque_error_threshold,
            output_torque_regulator.get("error_threshold"),
            50,
        )
    )
    args.safety_password_env = str(
        _configured(
            args.safety_password_env,
            output_torque_regulator.get("password_env"),
            "FLEXIV_SAFETY_PASSWORD",
        )
    ).strip()
    if not 0.0 < args.output_torque_limiting_factor <= 1.0:
        raise ValueError("output torque limiting_factor must be within (0, 1]")
    if args.output_torque_error_threshold < 1:
        raise ValueError("output torque error_threshold must be at least 1")
    if args.output_torque_regulator and not args.safety_password_env:
        raise ValueError("output torque regulator requires a non-empty password_env name")
    args.cartesian_impedance_control = _configured(
        args.cartesian_impedance_control, cartesian_impedance.get("enabled"), False
    )
    args.left_cartesian_stiffness = _vector6_csv(
        _configured(
            args.left_cartesian_stiffness,
            cartesian_impedance.get("left_stiffness"),
            [10000, 10000, 10000, 1500, 1500, 1500],
        ),
        name="left Cartesian stiffness",
        minimum=0.0,
    )
    args.right_cartesian_stiffness = _vector6_csv(
        _configured(
            args.right_cartesian_stiffness,
            cartesian_impedance.get("right_stiffness"),
            [10000, 10000, 10000, 1500, 1500, 1500],
        ),
        name="right Cartesian stiffness",
        minimum=0.0,
    )
    args.left_cartesian_damping_ratio = _vector6_csv(
        _configured(
            args.left_cartesian_damping_ratio,
            cartesian_impedance.get("left_damping_ratio"),
            [0.7] * 6,
        ),
        name="left Cartesian damping ratio",
        minimum=0.3,
        maximum=0.8,
    )
    args.right_cartesian_damping_ratio = _vector6_csv(
        _configured(
            args.right_cartesian_damping_ratio,
            cartesian_impedance.get("right_damping_ratio"),
            [0.7] * 6,
        ),
        name="right Cartesian damping ratio",
        minimum=0.3,
        maximum=0.8,
    )
    args.self_collision_monitor = _configured(
        args.self_collision_monitor, self_collision.get("enabled"), False
    )
    args.self_collision_min_distance_m = _configured(
        args.self_collision_min_distance_m, self_collision.get("min_distance_m"), 0.05
    )
    args.self_collision_loop_interval_ms = _configured(
        args.self_collision_loop_interval_ms, self_collision.get("loop_interval_ms"), 10
    )
    args.self_collision_skip_link = _configured(
        args.self_collision_skip_link, self_collision.get("skipped_links"), []
    )
    args.contact_wrench_control = _configured(
        args.contact_wrench_control, contact_wrench.get("enabled"), True
    )
    args.left_max_contact_wrench = _wrench_csv(
        _configured(args.left_max_contact_wrench, contact_wrench.get("left_limit"), [30, 30, 30, 5, 5, 5]),
        name="left contact wrench limit",
    )
    args.right_max_contact_wrench = _wrench_csv(
        _configured(args.right_max_contact_wrench, contact_wrench.get("right_limit"), [30, 30, 30, 5, 5, 5]),
        name="right contact wrench limit",
    )
    args.contact_wrench_freeze_trigger_ratio = _configured(
        args.contact_wrench_freeze_trigger_ratio, contact_wrench.get("freeze_trigger_ratio"), 0.90
    )
    args.contact_wrench_release_ratio = _configured(
        args.contact_wrench_release_ratio, contact_wrench.get("release_ratio"), 0.55
    )
    args.contact_wrench_trigger_samples = _configured(
        args.contact_wrench_trigger_samples, contact_wrench.get("trigger_samples"), 1
    )
    args.contact_wrench_release_dwell_sec = _configured(
        args.contact_wrench_release_dwell_sec, contact_wrench.get("release_dwell_sec"), 0.12
    )
    args.joint_torque_control = _configured(
        args.joint_torque_control, joint_torque.get("enabled"), True
    )
    args.joint_torque_trigger_ratio = _configured(
        args.joint_torque_trigger_ratio, joint_torque.get("trigger_ratio"), 0.72
    )
    args.joint_torque_release_ratio = _configured(
        args.joint_torque_release_ratio, joint_torque.get("release_ratio"), 0.55
    )
    args.joint_torque_trigger_samples = _configured(
        args.joint_torque_trigger_samples, joint_torque.get("trigger_samples"), 1
    )
    args.joint_torque_release_dwell_sec = _configured(
        args.joint_torque_release_dwell_sec, joint_torque.get("release_dwell_sec"), 0.15
    )
    args.joint_torque_prediction_horizon_sec = _configured(
        args.joint_torque_prediction_horizon_sec,
        joint_torque.get("prediction_horizon_sec"),
        0.025,
    )
    args.joint_torque_rollback_sec = _configured(
        args.joint_torque_rollback_sec, joint_torque.get("rollback_sec"), 0.05
    )
    args.target_resampling_control = _configured(
        args.target_resampling_control, target_resampling.get("enabled"), False
    )
    args.target_resample_rate_hz = float(
        _configured(args.target_resample_rate_hz, target_resampling.get("rate_hz"), 250.0)
    )
    args.target_prediction_horizon_sec = float(
        _configured(
            args.target_prediction_horizon_sec,
            target_resampling.get("prediction_horizon_sec"),
            0.01,
        )
    )
    args.target_velocity_filter_alpha = float(
        _configured(
            args.target_velocity_filter_alpha,
            target_resampling.get("velocity_filter_alpha"),
            0.35,
        )
    )
    args.target_feedforward_scale = float(
        _configured(
            args.target_feedforward_scale,
            target_resampling.get("feedforward_scale"),
            0.5,
        )
    )
    args.target_max_linear_feedforward_m_s = float(
        _configured(
            args.target_max_linear_feedforward_m_s,
            target_resampling.get("max_linear_feedforward_m_s"),
            0.25,
        )
    )
    args.target_max_angular_feedforward_rad_s = float(
        _configured(
            args.target_max_angular_feedforward_rad_s,
            target_resampling.get("max_angular_feedforward_rad_s"),
            1.0,
        )
    )
    args.target_torque_soft_ratio = float(
        _configured(
            args.target_torque_soft_ratio,
            target_resampling.get("torque_soft_ratio"),
            0.65,
        )
    )
    args.target_min_motion_scale = float(
        _configured(
            args.target_min_motion_scale,
            target_resampling.get("min_motion_scale"),
            0.25,
        )
    )
    args.target_linear_velocity_deadband_m_s = float(
        _configured(
            args.target_linear_velocity_deadband_m_s,
            target_resampling.get("linear_velocity_deadband_m_s"),
            0.005,
        )
    )
    args.target_angular_velocity_deadband_rad_s = float(
        _configured(
            args.target_angular_velocity_deadband_rad_s,
            target_resampling.get("angular_velocity_deadband_rad_s"),
            0.02,
        )
    )
    if args.target_resample_rate_hz <= 0.0:
        raise ValueError("target resampling rate_hz must be positive")
    if args.target_prediction_horizon_sec < 0.0:
        raise ValueError("target resampling prediction_horizon_sec must be non-negative")
    if not 0.0 <= args.target_velocity_filter_alpha <= 1.0:
        raise ValueError("target resampling velocity_filter_alpha must be within [0, 1]")
    if not 0.0 <= args.target_feedforward_scale <= 1.0:
        raise ValueError("target resampling feedforward_scale must be within [0, 1]")
    if min(
        args.target_max_linear_feedforward_m_s,
        args.target_max_angular_feedforward_rad_s,
    ) < 0.0:
        raise ValueError("target resampling feed-forward limits must be non-negative")
    if not 0.0 <= args.target_torque_soft_ratio < args.joint_torque_trigger_ratio:
        raise ValueError("target resampling torque_soft_ratio must be below joint torque trigger_ratio")
    if not 0.0 <= args.target_min_motion_scale <= 1.0:
        raise ValueError("target resampling min_motion_scale must be within [0, 1]")
    if min(
        args.target_linear_velocity_deadband_m_s,
        args.target_angular_velocity_deadband_rad_s,
    ) < 0.0:
        raise ValueError("target resampling velocity deadbands must be non-negative")
    args.scene_config = args.scene_config.expanduser().resolve()
    (
        args.left_nullspace_posture,
        args.right_nullspace_posture,
        args.left_startup_waypoint,
        args.right_startup_waypoint,
    ) = load_initial_q(args.scene_config)
    scene_translations = load_robot_translations(args.scene_config)
    args.left_translation_in_world = (
        args.left_translation_in_world or scene_translations["left"]
    )
    args.right_translation_in_world = (
        args.right_translation_in_world or scene_translations["right"]
    )
    return args


def _configured(cli_value, config_value, fallback):
    if cli_value is not None:
        return cli_value
    return fallback if config_value is None else config_value


def _wrench_csv(value, *, name: str) -> str:
    if isinstance(value, str):
        items = [item.strip() for item in value.split(",") if item.strip()]
    elif isinstance(value, (list, tuple)):
        items = list(value)
    else:
        raise ValueError(f"{name} must contain six numeric values")
    try:
        numbers = [float(item) for item in items]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain six numeric values") from exc
    if len(numbers) != 6 or not all(math.isfinite(item) and item > 0.0 for item in numbers):
        raise ValueError(f"{name} must contain six positive finite values")
    return ",".join(str(item) for item in numbers)


def _vector6_csv(
    value,
    *,
    name: str,
    minimum: float,
    maximum: float | None = None,
) -> str:
    if isinstance(value, str):
        items = [item.strip() for item in value.split(",") if item.strip()]
    elif isinstance(value, (list, tuple)):
        items = list(value)
    else:
        raise ValueError(f"{name} must contain six numeric values")
    try:
        numbers = [float(item) for item in items]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain six numeric values") from exc
    if len(numbers) != 6 or not all(math.isfinite(item) for item in numbers):
        raise ValueError(f"{name} must contain six finite values")
    if any(item < minimum or (maximum is not None and item > maximum) for item in numbers):
        range_text = f"[{minimum}, {maximum}]" if maximum is not None else f"[{minimum}, inf)"
        raise ValueError(f"{name} values must be within {range_text}")
    return ",".join(str(item) for item in numbers)


def load_drdk_control(pipeline_config: Path) -> dict:
    data = yaml.safe_load(pipeline_config.read_text(encoding="utf-8")) or {}
    control = data.get("control") or {}
    drdk = control.get("drdk") or {}
    if not isinstance(drdk, dict):
        raise ValueError(f"{pipeline_config}: control.drdk must be a mapping")
    for key in (
        "cartesian_impedance",
        "output_torque_regulator",
        "self_collision_monitor",
        "contact_wrench",
        "joint_torque",
        "target_resampling",
        "reset_motion",
    ):
        value = drdk.get(key)
        if value is not None and not isinstance(value, dict):
            raise ValueError(f"{pipeline_config}: control.drdk.{key} must be a mapping")
    return drdk


def load_initial_q(scene_config: Path) -> tuple[str, str, list[str], list[str]]:
    data = yaml.safe_load(scene_config.read_text(encoding="utf-8")) or {}
    robots = data.get("robots") or []
    postures: dict[str, str] = {}
    waypoints: dict[str, list[str]] = {}
    for robot in robots:
        side = str(robot.get("side", "")).strip().lower()
        if side not in {"left", "right"}:
            continue
        initial_q = robot.get("initial_q")
        if not isinstance(initial_q, list) or len(initial_q) != 7:
            raise ValueError(f"{scene_config}: robots[{side}].initial_q must contain 7 joint values")
        try:
            postures[side] = ",".join(str(float(value)) for value in initial_q)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{scene_config}: robots[{side}].initial_q must contain only numeric values"
            ) from exc
        raw_waypoints = robot.get("initial_q_waypoints") or []
        if not isinstance(raw_waypoints, list):
            raise ValueError(f"{scene_config}: robots[{side}].initial_q_waypoints must be a list")
        waypoints[side] = []
        for index, waypoint in enumerate(raw_waypoints):
            if not isinstance(waypoint, list) or len(waypoint) != 7:
                raise ValueError(
                    f"{scene_config}: robots[{side}].initial_q_waypoints[{index}] must contain 7 values"
                )
            try:
                waypoints[side].append(",".join(str(float(value)) for value in waypoint))
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"{scene_config}: robots[{side}].initial_q_waypoints[{index}] must contain only numeric values"
                ) from exc
    missing = [side for side in ("left", "right") if side not in postures]
    if missing:
        raise ValueError(f"{scene_config}: missing robot initial_q for {', '.join(missing)}")
    if len(waypoints["left"]) != len(waypoints["right"]):
        raise ValueError(f"{scene_config}: left and right initial_q_waypoints must have equal length")
    return postures["left"], postures["right"], waypoints["left"], waypoints["right"]


def load_robot_translations(scene_config: Path) -> dict[str, str]:
    """Load the two robot base translations in their shared scene world."""

    data = yaml.safe_load(scene_config.read_text(encoding="utf-8")) or {}
    translations: dict[str, str] = {}
    for robot in data.get("robots") or []:
        side = str(robot.get("side", "")).strip().lower()
        if side not in {"left", "right"}:
            continue
        position = robot.get("position")
        if isinstance(position, dict):
            raw_values = [position.get(axis) for axis in ("x", "y", "z")]
        elif isinstance(position, (list, tuple)) and len(position) == 3:
            raw_values = list(position)
        else:
            raise ValueError(
                f"{scene_config}: robots[{side}].position must define finite x, y and z"
            )
        try:
            values = [float(value) for value in raw_values]
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"{scene_config}: robots[{side}].position must define finite x, y and z"
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"{scene_config}: robots[{side}].position must define finite x, y and z"
            )
        translations[side] = ",".join(str(value) for value in values)
    missing = [side for side in ("left", "right") if side not in translations]
    if missing:
        raise ValueError(f"{scene_config}: missing robot position for {', '.join(missing)}")
    return translations


def build_env(base_env: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(os.environ if base_env is None else base_env)
    if RDK_COMPAT_PATH.exists():
        current = env.get("PYTHONPATH", "")
        env["PYTHONPATH"] = str(RDK_COMPAT_PATH) + (":" + current if current else "")
    return env


def build_command(args: argparse.Namespace) -> list[str]:
    command = [
        str(args.python),
        str(flexiv_runtime.REPO_ROOT / "scripts" / "drdk_target_streamer.py"),
        "--host",
        str(args.host),
        "--left-port",
        str(args.left_port),
        "--right-port",
        str(args.right_port),
        "--left-serial-number",
        str(args.left_serial_number),
        "--right-serial-number",
        str(args.right_serial_number),
        "--joint-group",
        str(args.joint_group),
        "--left-status-host",
        str(args.left_status_host),
        "--left-status-port",
        str(args.left_status_port),
        "--right-status-host",
        str(args.right_status_host),
        "--right-status-port",
        str(args.right_status_port),
        f"--left-translation-in-world={args.left_translation_in_world}",
        f"--right-translation-in-world={args.right_translation_in_world}",
        "--cartesian-impedance-control"
        if args.cartesian_impedance_control
        else "--no-cartesian-impedance-control",
        f"--left-cartesian-stiffness={args.left_cartesian_stiffness}",
        f"--right-cartesian-stiffness={args.right_cartesian_stiffness}",
        f"--left-cartesian-damping-ratio={args.left_cartesian_damping_ratio}",
        f"--right-cartesian-damping-ratio={args.right_cartesian_damping_ratio}",
        "--output-torque-regulator"
        if args.output_torque_regulator
        else "--no-output-torque-regulator",
        "--output-torque-limiting-factor",
        str(args.output_torque_limiting_factor),
        "--output-torque-error-threshold",
        str(args.output_torque_error_threshold),
        "--safety-password-env",
        str(args.safety_password_env),
        "--self-collision-monitor" if args.self_collision_monitor else "--no-self-collision-monitor",
        "--self-collision-min-distance-m",
        str(args.self_collision_min_distance_m),
        "--self-collision-loop-interval-ms",
        str(args.self_collision_loop_interval_ms),
        "--contact-wrench-control" if args.contact_wrench_control else "--no-contact-wrench-control",
        f"--left-max-contact-wrench={args.left_max_contact_wrench}",
        f"--right-max-contact-wrench={args.right_max_contact_wrench}",
        "--contact-wrench-freeze-trigger-ratio",
        str(args.contact_wrench_freeze_trigger_ratio),
        "--contact-wrench-release-ratio",
        str(args.contact_wrench_release_ratio),
        "--contact-wrench-trigger-samples",
        str(args.contact_wrench_trigger_samples),
        "--contact-wrench-release-dwell-sec",
        str(args.contact_wrench_release_dwell_sec),
        "--joint-torque-control" if args.joint_torque_control else "--no-joint-torque-control",
        "--joint-torque-trigger-ratio",
        str(args.joint_torque_trigger_ratio),
        "--joint-torque-release-ratio",
        str(args.joint_torque_release_ratio),
        "--joint-torque-trigger-samples",
        str(args.joint_torque_trigger_samples),
        "--joint-torque-release-dwell-sec",
        str(args.joint_torque_release_dwell_sec),
        "--joint-torque-prediction-horizon-sec",
        str(args.joint_torque_prediction_horizon_sec),
        "--joint-torque-rollback-sec",
        str(args.joint_torque_rollback_sec),
        "--target-resampling-control"
        if args.target_resampling_control
        else "--no-target-resampling-control",
        "--target-resample-rate-hz",
        str(args.target_resample_rate_hz),
        "--target-prediction-horizon-sec",
        str(args.target_prediction_horizon_sec),
        "--target-velocity-filter-alpha",
        str(args.target_velocity_filter_alpha),
        "--target-feedforward-scale",
        str(args.target_feedforward_scale),
        "--target-max-linear-feedforward-m-s",
        str(args.target_max_linear_feedforward_m_s),
        "--target-max-angular-feedforward-rad-s",
        str(args.target_max_angular_feedforward_rad_s),
        "--target-torque-soft-ratio",
        str(args.target_torque_soft_ratio),
        "--target-min-motion-scale",
        str(args.target_min_motion_scale),
        "--target-linear-velocity-deadband-m-s",
        str(args.target_linear_velocity_deadband_m_s),
        "--target-angular-velocity-deadband-rad-s",
        str(args.target_angular_velocity_deadband_rad_s),
        f"--left-nullspace-posture={args.left_nullspace_posture}",
        f"--right-nullspace-posture={args.right_nullspace_posture}",
        "--nullspace-tracking-weight",
        str(args.nullspace_tracking_weight),
        "--max-age-sec",
        str(args.max_age_sec),
        "--connect-timeout-sec",
        str(args.connect_timeout_sec),
        "--enable-timeout-sec",
        str(args.enable_timeout_sec),
        "--initial-joint-timeout-sec",
        str(args.initial_joint_timeout_sec),
        "--initial-joint-handoff-sec",
        str(args.initial_joint_handoff_sec),
        "--initial-joint-settle-sec",
        str(args.initial_joint_settle_sec),
        "--initial-joint-tolerance-rad",
        str(args.initial_joint_tolerance_rad),
        "--initial-joint-speed-tolerance-rad-s",
        str(args.initial_joint_speed_tolerance_rad_s),
        "--initial-joint-max-vel-rad-s",
        str(args.initial_joint_max_vel_rad_s),
        "--initial-joint-max-acc-rad-s2",
        str(args.initial_joint_max_acc_rad_s2),
        "--reset-joint-max-vel-rad-s",
        str(args.reset_joint_max_vel_rad_s),
        "--reset-joint-max-acc-rad-s2",
        str(args.reset_joint_max_acc_rad_s2),
        "--reset-max-attempts",
        str(args.reset_max_attempts),
        "--reset-retry-delay-sec",
        str(args.reset_retry_delay_sec),
        "--reset-motion-method",
        str(args.reset_motion_method),
        "--max-linear-speed-m-s",
        str(args.max_linear_speed_m_s),
        "--max-angular-speed-rad-s",
        str(args.max_angular_speed_rad_s),
        "--max-linear-acc-m-s2",
        str(args.max_linear_acc_m_s2),
        "--max-angular-acc-rad-s2",
        str(args.max_angular_acc_rad_s2),
        "--log-hz",
        str(args.log_hz),
        "--clear-fault" if args.clear_fault else "--no-clear-fault",
        "--strict-clear-fault" if args.strict_clear_fault else "--no-strict-clear-fault",
    ]
    for left_waypoint, right_waypoint in zip(
        args.left_startup_waypoint, args.right_startup_waypoint, strict=True
    ):
        command.extend(
            [
                f"--left-startup-waypoint={left_waypoint}",
                f"--right-startup-waypoint={right_waypoint}",
            ]
        )
    for option, value in (("--network-interface-whitelist", args.network_interface_whitelist),):
        if value:
            command.extend([option, str(value)])
    for link_name in args.self_collision_skip_link:
        command.extend(["--self-collision-skip-link", str(link_name)])
    return command


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    pid, stdout_path, stderr_path = flexiv_runtime.start_background(
        build_command(args),
        cwd=flexiv_runtime.REPO_ROOT,
        log_prefix="drdk_target_streamer_dual",
        env=build_env(),
    )
    flexiv_runtime.print_started("DRDK_TARGET_STREAMER", pid, stdout_path, stderr_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
