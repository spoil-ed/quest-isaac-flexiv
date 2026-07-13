"""Strict Stage1 single-Rizon4 validation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable

from flexiv_data_collection.schema import (
    BODY,
    LEFT_ARM,
    RIGHT_ARM,
    RIGHT_EE,
    validate_unitree_sample,
)


EXPECTED_STAGE1_BACKEND = "quest_isaac_flexiv_stage1"
EXPECTED_STAGE1_SERIAL = "Rizon4-VIHhZM"
STAGE1_CAMERA_KEYS = ("color_0",)
STAGE1_CAMERA_NAMES = ("cam_front",)


def vector_norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _field_values(container: dict[str, Any], part: str, field: str) -> list[float]:
    payload = container.get(part) or {}
    return [float(value) for value in payload.get(field) or []]


def _assert_zero(values: Iterable[float], *, name: str, tolerance: float) -> None:
    norm = vector_norm(values)
    if norm > float(tolerance):
        raise ValueError(f"{name} must be zero for Stage1 single-arm data, got norm={norm:.6g}")


def extract_stage1_bridge_state(sample: dict[str, Any]) -> dict[str, Any]:
    sim_state = sample.get("sim_state") or {}
    if sim_state.get("backend") == EXPECTED_STAGE1_BACKEND:
        return sim_state
    bridge = sim_state.get("bridge") or {}
    if bridge.get("backend") == EXPECTED_STAGE1_BACKEND:
        return bridge
    return {}


def validate_stage1_single_arm_sample(
    sample: dict[str, Any],
    *,
    expected_serial: str = EXPECTED_STAGE1_SERIAL,
    required_camera_keys: tuple[str, ...] = STAGE1_CAMERA_KEYS,
    exact_camera_keys: bool = True,
    require_stage1_backend: bool = True,
    zero_tolerance: float = 1e-10,
) -> None:
    validate_unitree_sample(sample)
    colors = sample.get("colors") or {}
    missing_cameras = [key for key in required_camera_keys if key not in colors]
    if missing_cameras:
        raise ValueError(f"Missing Stage1 camera frames: {missing_cameras}")
    if exact_camera_keys and set(colors) != set(required_camera_keys):
        raise ValueError(f"Stage1 single-arm validation only allows cameras {list(required_camera_keys)}, got {sorted(colors)}")

    if require_stage1_backend:
        bridge = extract_stage1_bridge_state(sample)
        if not bridge:
            raise ValueError(f"Stage1 sample backend must be {EXPECTED_STAGE1_BACKEND}")
        serial = str(bridge.get("serial", ""))
        if serial != str(expected_serial):
            raise ValueError(f"Stage1 sample serial must be {expected_serial}, got {serial!r}")

    for key in ("states", "actions"):
        container = sample[key]
        for field in ("qpos", "qvel", "torque"):
            _assert_zero(
                _field_values(container, RIGHT_ARM, field),
                name=f"{key}.{RIGHT_ARM}.{field}",
                tolerance=zero_tolerance,
            )
            _assert_zero(
                _field_values(container, RIGHT_EE, field),
                name=f"{key}.{RIGHT_EE}.{field}",
                tolerance=zero_tolerance,
            )
        body = container.get(BODY) or {}
        for field in ("qpos", "qvel", "torque"):
            _assert_zero(body.get(field) or [], name=f"{key}.{BODY}.{field}", tolerance=zero_tolerance)


def summarize_stage1_single_arm_frames(
    frames: list[dict[str, Any]],
    *,
    expected_serial: str = EXPECTED_STAGE1_SERIAL,
    required_camera_keys: tuple[str, ...] = STAGE1_CAMERA_KEYS,
    exact_camera_keys: bool = True,
    min_left_q_delta: float = 0.0,
    min_left_torque_norm: float = 0.0,
    min_servo_cycle_delta: int = 0,
    zero_tolerance: float = 1e-10,
) -> dict[str, Any]:
    if not frames:
        raise ValueError("Stage1 validation requires at least one frame")

    color_counts = {key: 0 for key in required_camera_keys}
    servo_cycles: list[int] = []
    left_q0 = list(frames[0]["states"][LEFT_ARM]["qpos"])
    left_q_delta_norm = 0.0
    max_left_torque_norm = 0.0

    for idx, frame in enumerate(frames):
        try:
            validate_stage1_single_arm_sample(
                frame,
                expected_serial=expected_serial,
                required_camera_keys=required_camera_keys,
                exact_camera_keys=exact_camera_keys,
                require_stage1_backend=True,
                zero_tolerance=zero_tolerance,
            )
        except Exception as exc:
            raise ValueError(f"Stage1 single-arm frame {idx}: {exc}") from exc

        colors = frame.get("colors") or {}
        for key in required_camera_keys:
            if colors.get(key):
                color_counts[key] += 1

        bridge = extract_stage1_bridge_state(frame)
        if bridge.get("servo_cycle") is not None:
            servo_cycles.append(int(bridge["servo_cycle"]))
        q = frame["states"][LEFT_ARM]["qpos"]
        left_q_delta_norm = max(left_q_delta_norm, vector_norm(float(b) - float(a) for a, b in zip(left_q0, q)))
        max_left_torque_norm = max(max_left_torque_norm, vector_norm(frame["actions"][LEFT_ARM].get("torque") or []))

    missing_frames = {key: len(frames) - count for key, count in color_counts.items()}
    if any(value for value in missing_frames.values()):
        raise ValueError(f"Stage1 camera frame counts must match frame count {len(frames)}, missing={missing_frames}")
    if left_q_delta_norm < float(min_left_q_delta):
        raise ValueError(f"left_q_delta_norm {left_q_delta_norm:.6g} < required {float(min_left_q_delta):.6g}")
    if max_left_torque_norm <= float(min_left_torque_norm):
        raise ValueError(f"max_left_torque_norm {max_left_torque_norm:.6g} <= required {float(min_left_torque_norm):.6g}")
    servo_cycle_delta = (max(servo_cycles) - min(servo_cycles)) if servo_cycles else 0
    if servo_cycle_delta < int(min_servo_cycle_delta):
        raise ValueError(f"servo_cycle_delta {servo_cycle_delta} < required {int(min_servo_cycle_delta)}")

    return {
        "strict_stage1_single_arm": True,
        "expected_serial": expected_serial,
        "required_camera_keys": list(required_camera_keys),
        "frames": len(frames),
        "color_counts": color_counts,
        "left_q_delta_norm": left_q_delta_norm,
        "max_left_torque_norm": max_left_torque_norm,
        "servo_cycle_delta": servo_cycle_delta,
        "right_arm_zero": True,
    }


@dataclass
class Stage1SampleMonitor:
    """Track live gateway samples until they prove the stream is fresh."""

    min_servo_cycle_delta: int = 5
    min_left_q_delta: float = 0.0
    min_left_torque_norm: float = 1e-8
    expected_serial: str = EXPECTED_STAGE1_SERIAL
    required_camera_keys: tuple[str, ...] = STAGE1_CAMERA_KEYS
    zero_tolerance: float = 1e-10
    first_servo_cycle: int | None = None
    first_left_q: list[float] | None = None
    max_servo_cycle_delta: int = 0
    max_left_q_delta_norm: float = 0.0
    max_left_torque_norm: float = 0.0
    color_ready: bool = False
    last_status: dict[str, Any] = field(default_factory=dict)

    def observe(self, sample: dict[str, Any]) -> dict[str, Any]:
        validate_stage1_single_arm_sample(
            sample,
            expected_serial=self.expected_serial,
            required_camera_keys=self.required_camera_keys,
            exact_camera_keys=True,
            require_stage1_backend=True,
            zero_tolerance=self.zero_tolerance,
        )
        bridge = extract_stage1_bridge_state(sample)
        servo_cycle = int(bridge.get("servo_cycle") or 0)
        left_q = [float(value) for value in sample["states"][LEFT_ARM]["qpos"]]
        if self.first_servo_cycle is None:
            self.first_servo_cycle = servo_cycle
        if self.first_left_q is None:
            self.first_left_q = list(left_q)
        self.max_servo_cycle_delta = max(self.max_servo_cycle_delta, servo_cycle - self.first_servo_cycle)
        self.max_left_q_delta_norm = max(
            self.max_left_q_delta_norm,
            vector_norm(float(b) - float(a) for a, b in zip(self.first_left_q, left_q)),
        )
        self.max_left_torque_norm = max(
            self.max_left_torque_norm,
            vector_norm(sample["actions"][LEFT_ARM].get("torque") or []),
        )
        colors = sample.get("colors") or {}
        self.color_ready = self.color_ready or all(colors.get(key) for key in self.required_camera_keys)
        ready = (
            self.max_servo_cycle_delta >= int(self.min_servo_cycle_delta)
            and self.max_left_q_delta_norm >= float(self.min_left_q_delta)
            and self.max_left_torque_norm > float(self.min_left_torque_norm)
            and self.color_ready
        )
        self.last_status = {
            "ready": ready,
            "servo_cycle": servo_cycle,
            "servo_cycle_delta": self.max_servo_cycle_delta,
            "left_q_delta_norm": self.max_left_q_delta_norm,
            "max_left_torque_norm": self.max_left_torque_norm,
            "color_ready": self.color_ready,
        }
        return dict(self.last_status)
