"""Strict Stage2 dual-Rizon4 validation helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable

from flexiv_data_collection.schema import (
    BODY,
    LEFT_ARM,
    RIGHT_ARM,
    validate_unitree_sample,
)


EXPECTED_STAGE2_BACKEND = "quest_isaac_flexiv_stage2_dual"
DEFAULT_STAGE2_LEFT_SERIAL = "Rizon4-VIHhZM"
DEFAULT_STAGE2_RIGHT_SERIAL = "Rizon4-WE7ssd"
STAGE2_DEFAULT_CAMERA_KEYS = ("color_0",)
STAGE2_DEFAULT_CAMERA_NAMES = ("cam_front",)


def vector_norm(values: Iterable[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def _field_values(container: dict[str, Any], part: str, field: str) -> list[float]:
    payload = container.get(part) or {}
    return [float(value) for value in payload.get(field) or []]


def extract_stage2_bridge_state(sample: dict[str, Any]) -> dict[str, Any]:
    sim_state = sample.get("sim_state") or {}
    if sim_state.get("backend") == EXPECTED_STAGE2_BACKEND:
        return sim_state
    bridge = sim_state.get("bridge") or {}
    if bridge.get("backend") == EXPECTED_STAGE2_BACKEND:
        return bridge
    return {}


def _serial_from_bridge(bridge: dict[str, Any], side: str) -> str:
    serials = bridge.get("serials") or {}
    if isinstance(serials, dict) and serials.get(side) is not None:
        return str(serials[side])
    return str(bridge.get(f"{side}_serial", ""))


def _servo_cycle_from_bridge(bridge: dict[str, Any], side: str | None = None) -> int | None:
    if side is not None:
        cycles = bridge.get("servo_cycles") or {}
        if isinstance(cycles, dict) and cycles.get(side) is not None:
            return int(cycles[side])
    if bridge.get("servo_cycle") is not None:
        return int(bridge["servo_cycle"])
    return None


def _target_frame_xyz(bridge: dict[str, Any], side: str) -> list[float] | None:
    target_frames = bridge.get("target_frames") or {}
    target_frame = target_frames.get(side) if isinstance(target_frames, dict) else None
    if not isinstance(target_frame, dict):
        target_frame = bridge.get(f"{side}_target_frame") or {}
    pose = target_frame.get("base_tcp_pose")
    if isinstance(pose, list) and len(pose) >= 3:
        return [float(value) for value in pose[:3]]
    world_position = target_frame.get("world_position")
    if isinstance(world_position, list) and len(world_position) >= 3:
        return [float(value) for value in world_position[:3]]
    return None


def validate_stage2_dual_arm_sample(
    sample: dict[str, Any],
    *,
    expected_left_serial: str = DEFAULT_STAGE2_LEFT_SERIAL,
    expected_right_serial: str = DEFAULT_STAGE2_RIGHT_SERIAL,
    required_camera_keys: tuple[str, ...] = STAGE2_DEFAULT_CAMERA_KEYS,
    exact_camera_keys: bool = True,
    require_stage2_backend: bool = True,
) -> None:
    validate_unitree_sample(sample)
    colors = sample.get("colors") or {}
    missing_cameras = [key for key in required_camera_keys if key not in colors]
    if missing_cameras:
        raise ValueError(f"Missing Stage2 camera frames: {missing_cameras}")
    if exact_camera_keys and set(colors) != set(required_camera_keys):
        raise ValueError(f"Stage2 dual-arm validation only allows cameras {list(required_camera_keys)}, got {sorted(colors)}")

    body = (sample.get("states") or {}).get(BODY) or {}
    for field in ("qpos", "qvel", "torque"):
        if body.get(field):
            raise ValueError(f"states.{BODY}.{field} must be empty for Stage2 dual-arm data")

    if require_stage2_backend:
        bridge = extract_stage2_bridge_state(sample)
        if not bridge:
            raise ValueError(f"Stage2 sample backend must be {EXPECTED_STAGE2_BACKEND}")
        left_serial = _serial_from_bridge(bridge, "left")
        right_serial = _serial_from_bridge(bridge, "right")
        if left_serial != str(expected_left_serial):
            raise ValueError(f"Stage2 left serial must be {expected_left_serial}, got {left_serial!r}")
        if right_serial != str(expected_right_serial):
            raise ValueError(f"Stage2 right serial must be {expected_right_serial}, got {right_serial!r}")


def summarize_stage2_dual_arm_frames(
    frames: list[dict[str, Any]],
    *,
    expected_left_serial: str = DEFAULT_STAGE2_LEFT_SERIAL,
    expected_right_serial: str = DEFAULT_STAGE2_RIGHT_SERIAL,
    required_camera_keys: tuple[str, ...] = STAGE2_DEFAULT_CAMERA_KEYS,
    exact_camera_keys: bool = True,
    min_left_q_delta: float = 0.0,
    min_right_q_delta: float = 0.0,
    min_left_torque_norm: float = 0.0,
    min_right_torque_norm: float = 0.0,
    min_left_target_frame_delta: float = 0.0,
    min_right_target_frame_delta: float = 0.0,
    min_servo_cycle_delta: int = 0,
) -> dict[str, Any]:
    if not frames:
        raise ValueError("Stage2 validation requires at least one frame")

    color_counts = {key: 0 for key in required_camera_keys}
    servo_cycles: list[int] = []
    left_servo_cycles: list[int] = []
    right_servo_cycles: list[int] = []
    left_q0 = list(frames[0]["states"][LEFT_ARM]["qpos"])
    right_q0 = list(frames[0]["states"][RIGHT_ARM]["qpos"])
    left_q_delta_norm = 0.0
    right_q_delta_norm = 0.0
    max_left_torque_norm = 0.0
    max_right_torque_norm = 0.0
    left_target_frame_xyz0: list[float] | None = None
    right_target_frame_xyz0: list[float] | None = None
    left_target_frame_delta_norm = 0.0
    right_target_frame_delta_norm = 0.0

    for idx, frame in enumerate(frames):
        try:
            validate_stage2_dual_arm_sample(
                frame,
                expected_left_serial=expected_left_serial,
                expected_right_serial=expected_right_serial,
                required_camera_keys=required_camera_keys,
                exact_camera_keys=exact_camera_keys,
                require_stage2_backend=True,
            )
        except Exception as exc:
            raise ValueError(f"Stage2 dual-arm frame {idx}: {exc}") from exc

        colors = frame.get("colors") or {}
        for key in required_camera_keys:
            if colors.get(key):
                color_counts[key] += 1

        bridge = extract_stage2_bridge_state(frame)
        cycle = _servo_cycle_from_bridge(bridge)
        left_cycle = _servo_cycle_from_bridge(bridge, "left")
        right_cycle = _servo_cycle_from_bridge(bridge, "right")
        if cycle is not None:
            servo_cycles.append(cycle)
        if left_cycle is not None:
            left_servo_cycles.append(left_cycle)
        if right_cycle is not None:
            right_servo_cycles.append(right_cycle)
        left_target_frame_xyz = _target_frame_xyz(bridge, "left")
        if left_target_frame_xyz is not None:
            if left_target_frame_xyz0 is None:
                left_target_frame_xyz0 = list(left_target_frame_xyz)
            left_target_frame_delta_norm = max(
                left_target_frame_delta_norm,
                vector_norm(float(b) - float(a) for a, b in zip(left_target_frame_xyz0, left_target_frame_xyz)),
            )
        right_target_frame_xyz = _target_frame_xyz(bridge, "right")
        if right_target_frame_xyz is not None:
            if right_target_frame_xyz0 is None:
                right_target_frame_xyz0 = list(right_target_frame_xyz)
            right_target_frame_delta_norm = max(
                right_target_frame_delta_norm,
                vector_norm(float(b) - float(a) for a, b in zip(right_target_frame_xyz0, right_target_frame_xyz)),
            )

        left_q = frame["states"][LEFT_ARM]["qpos"]
        right_q = frame["states"][RIGHT_ARM]["qpos"]
        left_q_delta_norm = max(left_q_delta_norm, vector_norm(float(b) - float(a) for a, b in zip(left_q0, left_q)))
        right_q_delta_norm = max(right_q_delta_norm, vector_norm(float(b) - float(a) for a, b in zip(right_q0, right_q)))
        max_left_torque_norm = max(max_left_torque_norm, vector_norm(_field_values(frame["actions"], LEFT_ARM, "torque")))
        max_right_torque_norm = max(max_right_torque_norm, vector_norm(_field_values(frame["actions"], RIGHT_ARM, "torque")))

    missing_frames = {key: len(frames) - count for key, count in color_counts.items()}
    if any(value for value in missing_frames.values()):
        raise ValueError(f"Stage2 camera frame counts must match frame count {len(frames)}, missing={missing_frames}")
    if left_q_delta_norm < float(min_left_q_delta):
        raise ValueError(f"left_q_delta_norm {left_q_delta_norm:.6g} < required {float(min_left_q_delta):.6g}")
    if right_q_delta_norm < float(min_right_q_delta):
        raise ValueError(f"right_q_delta_norm {right_q_delta_norm:.6g} < required {float(min_right_q_delta):.6g}")
    if max_left_torque_norm <= float(min_left_torque_norm):
        raise ValueError(f"max_left_torque_norm {max_left_torque_norm:.6g} <= required {float(min_left_torque_norm):.6g}")
    if max_right_torque_norm <= float(min_right_torque_norm):
        raise ValueError(f"max_right_torque_norm {max_right_torque_norm:.6g} <= required {float(min_right_torque_norm):.6g}")
    if left_target_frame_delta_norm < float(min_left_target_frame_delta):
        raise ValueError(
            "left_target_frame_delta_norm "
            f"{left_target_frame_delta_norm:.6g} < required {float(min_left_target_frame_delta):.6g}"
        )
    if right_target_frame_delta_norm < float(min_right_target_frame_delta):
        raise ValueError(
            "right_target_frame_delta_norm "
            f"{right_target_frame_delta_norm:.6g} < required {float(min_right_target_frame_delta):.6g}"
        )

    servo_cycle_delta = (max(servo_cycles) - min(servo_cycles)) if servo_cycles else 0
    left_servo_cycle_delta = (max(left_servo_cycles) - min(left_servo_cycles)) if left_servo_cycles else servo_cycle_delta
    right_servo_cycle_delta = (max(right_servo_cycles) - min(right_servo_cycles)) if right_servo_cycles else servo_cycle_delta
    if max(servo_cycle_delta, left_servo_cycle_delta, right_servo_cycle_delta) < int(min_servo_cycle_delta):
        raise ValueError(
            "servo_cycle_delta "
            f"{max(servo_cycle_delta, left_servo_cycle_delta, right_servo_cycle_delta)} < required {int(min_servo_cycle_delta)}"
        )

    return {
        "strict_stage2_dual_arm": True,
        "expected_left_serial": expected_left_serial,
        "expected_right_serial": expected_right_serial,
        "required_camera_keys": list(required_camera_keys),
        "frames": len(frames),
        "color_counts": color_counts,
        "left_q_delta_norm": left_q_delta_norm,
        "right_q_delta_norm": right_q_delta_norm,
        "max_left_torque_norm": max_left_torque_norm,
        "max_right_torque_norm": max_right_torque_norm,
        "left_target_frame_delta_norm": left_target_frame_delta_norm,
        "right_target_frame_delta_norm": right_target_frame_delta_norm,
        "servo_cycle_delta": servo_cycle_delta,
        "left_servo_cycle_delta": left_servo_cycle_delta,
        "right_servo_cycle_delta": right_servo_cycle_delta,
    }


@dataclass
class Stage2SampleMonitor:
    """Track live gateway samples until they prove the dual-arm stream is fresh."""

    min_servo_cycle_delta: int = 5
    min_left_q_delta: float = 0.0
    min_right_q_delta: float = 0.0
    min_left_torque_norm: float = 1e-8
    min_right_torque_norm: float = 1e-8
    expected_left_serial: str = DEFAULT_STAGE2_LEFT_SERIAL
    expected_right_serial: str = DEFAULT_STAGE2_RIGHT_SERIAL
    required_camera_keys: tuple[str, ...] = STAGE2_DEFAULT_CAMERA_KEYS
    first_servo_cycle: int | None = None
    first_left_q: list[float] | None = None
    first_right_q: list[float] | None = None
    max_servo_cycle_delta: int = 0
    max_left_q_delta_norm: float = 0.0
    max_right_q_delta_norm: float = 0.0
    max_left_torque_norm: float = 0.0
    max_right_torque_norm: float = 0.0
    color_ready: bool = False
    last_status: dict[str, Any] = field(default_factory=dict)

    def observe(self, sample: dict[str, Any]) -> dict[str, Any]:
        validate_stage2_dual_arm_sample(
            sample,
            expected_left_serial=self.expected_left_serial,
            expected_right_serial=self.expected_right_serial,
            required_camera_keys=self.required_camera_keys,
            exact_camera_keys=True,
            require_stage2_backend=True,
        )
        bridge = extract_stage2_bridge_state(sample)
        servo_cycle = _servo_cycle_from_bridge(bridge) or 0
        left_q = [float(value) for value in sample["states"][LEFT_ARM]["qpos"]]
        right_q = [float(value) for value in sample["states"][RIGHT_ARM]["qpos"]]
        if self.first_servo_cycle is None:
            self.first_servo_cycle = servo_cycle
        if self.first_left_q is None:
            self.first_left_q = list(left_q)
        if self.first_right_q is None:
            self.first_right_q = list(right_q)
        self.max_servo_cycle_delta = max(self.max_servo_cycle_delta, servo_cycle - self.first_servo_cycle)
        self.max_left_q_delta_norm = max(
            self.max_left_q_delta_norm,
            vector_norm(float(b) - float(a) for a, b in zip(self.first_left_q, left_q)),
        )
        self.max_right_q_delta_norm = max(
            self.max_right_q_delta_norm,
            vector_norm(float(b) - float(a) for a, b in zip(self.first_right_q, right_q)),
        )
        self.max_left_torque_norm = max(
            self.max_left_torque_norm,
            vector_norm(sample["actions"][LEFT_ARM].get("torque") or []),
        )
        self.max_right_torque_norm = max(
            self.max_right_torque_norm,
            vector_norm(sample["actions"][RIGHT_ARM].get("torque") or []),
        )
        colors = sample.get("colors") or {}
        self.color_ready = self.color_ready or all(colors.get(key) for key in self.required_camera_keys)
        ready = (
            self.max_servo_cycle_delta >= int(self.min_servo_cycle_delta)
            and self.max_left_q_delta_norm >= float(self.min_left_q_delta)
            and self.max_right_q_delta_norm >= float(self.min_right_q_delta)
            and self.max_left_torque_norm > float(self.min_left_torque_norm)
            and self.max_right_torque_norm > float(self.min_right_torque_norm)
            and self.color_ready
        )
        self.last_status = {
            "ready": ready,
            "servo_cycle": servo_cycle,
            "servo_cycle_delta": self.max_servo_cycle_delta,
            "left_q_delta_norm": self.max_left_q_delta_norm,
            "right_q_delta_norm": self.max_right_q_delta_norm,
            "max_left_torque_norm": self.max_left_torque_norm,
            "max_right_torque_norm": self.max_right_torque_norm,
            "color_ready": self.color_ready,
        }
        return dict(self.last_status)
