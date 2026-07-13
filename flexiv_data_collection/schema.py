"""Flexiv Unitree-JSON schema helpers.

The vector order is intentionally compatible with the Flexiv LeRobot replay
path used by the newer data pipeline:

    left_arm[7], left_gripper, right_arm[7], right_gripper
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


ARM_DOF = 7
GRIPPER_DOF = 1
FLEXIV_VECTOR_DIM = 16
FLEXIV_ROBOT_TYPE = "Flexiv_Dual_Rizon4_Grav"

LEFT_ARM = "left_arm"
LEFT_EE = "left_ee"
RIGHT_ARM = "right_arm"
RIGHT_EE = "right_ee"
BODY = "body"

FLEXIV_PART_ORDER = (LEFT_ARM, LEFT_EE, RIGHT_ARM, RIGHT_EE)
FLEXIV_PART_DIMS = {
    LEFT_ARM: ARM_DOF,
    LEFT_EE: GRIPPER_DOF,
    RIGHT_ARM: ARM_DOF,
    RIGHT_EE: GRIPPER_DOF,
}

FLEXIV_MOTOR_NAMES = (
    "left_joint_1",
    "left_joint_2",
    "left_joint_3",
    "left_joint_4",
    "left_joint_5",
    "left_joint_6",
    "left_joint_7",
    "left_gripper",
    "right_joint_1",
    "right_joint_2",
    "right_joint_3",
    "right_joint_4",
    "right_joint_5",
    "right_joint_6",
    "right_joint_7",
    "right_gripper",
)

FLEXIV_CAMERA_NAMES = (
    "cam_front",
    "cam_left_wrist",
    "cam_right_wrist",
)

FLEXIV_CAMERA_TO_IMAGE_KEY = {
    "color_0": "cam_front",
    "color_1": "cam_left_wrist",
    "color_2": "cam_right_wrist",
}

FLEXIV_JSON_DATA_NAMES = (
    "left_arm.qpos",
    "left_ee.qpos",
    "right_arm.qpos",
    "right_ee.qpos",
)


@dataclass(frozen=True)
class FlexivVectorParts:
    left_arm: list[float]
    left_gripper: float
    right_arm: list[float]
    right_gripper: float

    def as_vector(self) -> list[float]:
        return [
            *_float_list(self.left_arm, expected_len=ARM_DOF, name=LEFT_ARM),
            normalize_gripper(self.left_gripper),
            *_float_list(self.right_arm, expected_len=ARM_DOF, name=RIGHT_ARM),
            normalize_gripper(self.right_gripper),
        ]


def normalize_gripper(value: float) -> float:
    return min(1.0, max(0.0, float(value)))


def _float_list(values: Iterable[float], *, expected_len: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected_len:
        raise ValueError(f"{name} must contain {expected_len} values, got {len(result)}")
    return result


def _optional_float_list(values: Iterable[float] | None, *, expected_len: int, name: str) -> list[float] | None:
    if values is None:
        return None
    return _float_list(values, expected_len=expected_len, name=name)


def split_flexiv_vector(vector: Iterable[float]) -> FlexivVectorParts:
    values = _float_list(vector, expected_len=FLEXIV_VECTOR_DIM, name="Flexiv vector")
    return FlexivVectorParts(
        left_arm=values[0:7],
        left_gripper=normalize_gripper(values[7]),
        right_arm=values[8:15],
        right_gripper=normalize_gripper(values[15]),
    )


def part_payload(
    qpos: Iterable[float],
    *,
    qvel: Iterable[float] | None = None,
    torque: Iterable[float] | None = None,
) -> dict[str, list[float]]:
    qpos_list = [float(value) for value in qpos]
    return {
        "qpos": qpos_list,
        "qvel": [] if qvel is None else [float(value) for value in qvel],
        "torque": [] if torque is None else [float(value) for value in torque],
    }


def unitree_parts_from_vector(
    qpos: Iterable[float],
    *,
    qvel: Iterable[float] | None = None,
    torque: Iterable[float] | None = None,
) -> dict[str, dict[str, list[float]]]:
    parts = split_flexiv_vector(qpos)
    qvel_values = _optional_float_list(qvel, expected_len=FLEXIV_VECTOR_DIM, name="qvel")
    torque_values = _optional_float_list(torque, expected_len=FLEXIV_VECTOR_DIM, name="torque")
    return {
        LEFT_ARM: part_payload(
            parts.left_arm,
            qvel=None if qvel_values is None else qvel_values[0:ARM_DOF],
            torque=None if torque_values is None else torque_values[0:ARM_DOF],
        ),
        LEFT_EE: part_payload(
            [parts.left_gripper],
            qvel=None if qvel_values is None else qvel_values[ARM_DOF : ARM_DOF + 1],
            torque=None if torque_values is None else torque_values[ARM_DOF : ARM_DOF + 1],
        ),
        RIGHT_ARM: part_payload(
            parts.right_arm,
            qvel=None if qvel_values is None else qvel_values[ARM_DOF + 1 : 2 * ARM_DOF + 1],
            torque=None if torque_values is None else torque_values[ARM_DOF + 1 : 2 * ARM_DOF + 1],
        ),
        RIGHT_EE: part_payload(
            [parts.right_gripper],
            qvel=None if qvel_values is None else qvel_values[-1:],
            torque=None if torque_values is None else torque_values[-1:],
        ),
        BODY: part_payload([]),
    }


def unitree_parts_from_single_arm(
    qpos: Iterable[float],
    *,
    qvel: Iterable[float] | None = None,
    torque: Iterable[float] | None = None,
    gripper: float = 0.0,
    arm: str = "left",
) -> dict[str, dict[str, list[float]]]:
    """Build the canonical 16D Unitree part mapping from one Rizon arm.

    Stage1 records the old single-Rizon4 pipeline. The output remains compatible
    with the dual-Flexiv 16D schema by filling the inactive arm with zeros.
    """

    q = _float_list(qpos, expected_len=ARM_DOF, name="qpos")
    dq = _optional_float_list(qvel, expected_len=ARM_DOF, name="qvel")
    tau = _optional_float_list(torque, expected_len=ARM_DOF, name="torque")
    zero_arm = [0.0] * ARM_DOF
    zero_optional = [0.0] * ARM_DOF
    if arm == "left":
        vector = [*q, normalize_gripper(gripper), *zero_arm, 0.0]
        qvel_vector = None if dq is None else [*dq, 0.0, *zero_optional, 0.0]
        torque_vector = None if tau is None else [*tau, 0.0, *zero_optional, 0.0]
    elif arm == "right":
        vector = [*zero_arm, 0.0, *q, normalize_gripper(gripper)]
        qvel_vector = None if dq is None else [*zero_optional, 0.0, *dq, 0.0]
        torque_vector = None if tau is None else [*zero_optional, 0.0, *tau, 0.0]
    else:
        raise ValueError("arm must be 'left' or 'right'")
    return unitree_parts_from_vector(vector, qvel=qvel_vector, torque=torque_vector)


def _get_qpos(container: dict[str, Any], part: str) -> list[float]:
    if part not in container:
        raise KeyError(f"Missing Unitree JSON part: {part}")
    qpos = container[part].get("qpos")
    if qpos is None:
        raise KeyError(f"Missing Unitree JSON qpos for part: {part}")
    return _float_list(qpos, expected_len=FLEXIV_PART_DIMS[part], name=f"{part}.qpos")


def _get_field(container: dict[str, Any], part: str, field: str, expected: int) -> list[float]:
    payload = container.get(part) or {}
    values = payload.get(field) or []
    if not values:
        return [0.0] * expected
    return _float_list(values, expected_len=expected, name=f"{part}.{field}")


def unitree_parts_to_vector(container: dict[str, Any]) -> list[float]:
    return [
        *_get_qpos(container, LEFT_ARM),
        *_get_qpos(container, LEFT_EE),
        *_get_qpos(container, RIGHT_ARM),
        *_get_qpos(container, RIGHT_EE),
    ]


def unitree_parts_to_full_vector(container: dict[str, Any], field: str) -> list[float]:
    return [
        *_get_field(container, LEFT_ARM, field, ARM_DOF),
        *_get_field(container, LEFT_EE, field, GRIPPER_DOF),
        *_get_field(container, RIGHT_ARM, field, ARM_DOF),
        *_get_field(container, RIGHT_EE, field, GRIPPER_DOF),
    ]


def validate_unitree_sample(sample: dict[str, Any]) -> None:
    for key in ("states", "actions"):
        if key not in sample or sample[key] is None:
            raise KeyError(f"Missing Unitree JSON sample field: {key}")
        vector = unitree_parts_to_vector(sample[key])
        if len(vector) != FLEXIV_VECTOR_DIM:
            raise ValueError(f"{key} must map to {FLEXIV_VECTOR_DIM} values")


def fake_target_to_joint_vector(
    left_position: Iterable[float],
    right_position: Iterable[float],
    *,
    left_gripper: float,
    right_gripper: float,
) -> list[float]:
    """Deterministic fake mapping used by the no-Isaac smoke test."""

    left = _float_list(left_position, expected_len=3, name="left_position")
    right = _float_list(right_position, expected_len=3, name="right_position")
    left_q = [left[0], left[1], left[2], 0.35 + left[1], -0.25, 0.15, -0.1]
    right_q = [right[0], right[1], right[2], -0.35 + right[1], 0.25, -0.15, 0.1]
    return FlexivVectorParts(
        left_arm=left_q,
        left_gripper=left_gripper,
        right_arm=right_q,
        right_gripper=right_gripper,
    ).as_vector()
