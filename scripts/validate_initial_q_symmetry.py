#!/usr/bin/env python3
"""Validate dual-Rizon4 initial_q poses with the transforms stored in the USD."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = (
    REPO_ROOT
    / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/app_config.yaml"
)


def rotation(axis: str, angle: float) -> np.ndarray:
    unit = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[axis]
    x, y, z = unit
    c, s, d = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array(
        [
            [c + x * x * d, x * y * d - z * s, x * z * d + y * s],
            [y * x * d + z * s, c + y * y * d, y * z * d - x * s],
            [z * x * d - y * s, z * y * d + x * s, c + z * z * d],
        ]
    )


def quaternion_rotation(w: float, x: float, y: float, z: float) -> np.ndarray:
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = (value / norm for value in (w, x, y, z))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform(position: np.ndarray, orientation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = orientation
    result[:3, 3] = position
    return result


def config_vector(value: dict[str, float]) -> np.ndarray:
    return np.array([float(value[key]) for key in ("x", "y", "z")])


def config_quaternion(value: dict[str, float]) -> np.ndarray:
    return quaternion_rotation(*(float(value[key]) for key in ("w", "x", "y", "z")))


def euler_xyz(value: dict[str, float]) -> np.ndarray:
    angles = {axis: math.radians(float(value[axis.lower()])) for axis in "XYZ"}
    return rotation("Z", angles["Z"]) @ rotation("Y", angles["Y"]) @ rotation("X", angles["X"])


def orientation_error(actual: np.ndarray, expected: np.ndarray) -> float:
    cosine = max(-1.0, min(1.0, (np.trace(expected.T @ actual) - 1.0) / 2.0))
    return math.acos(cosine)


@dataclass(frozen=True)
class RizonModel:
    axes: tuple[str, ...]
    local_transforms: tuple[np.ndarray, ...]
    link7_to_flange: np.ndarray
    lower_limits: np.ndarray
    upper_limits: np.ndarray

    def forward(
        self, q: np.ndarray, base_position: np.ndarray, base_orientation: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        current = transform(base_position, base_orientation)
        pivots = []
        for axis, local, angle in zip(self.axes, self.local_transforms, q):
            current = current @ local
            pivots.append(current[:3, 3].copy())
            current = current @ transform(np.zeros(3), rotation(axis, float(angle)))
        return current @ self.link7_to_flange, np.asarray(pivots)


def _usd_quaternion(value) -> np.ndarray:
    imaginary = value.GetImaginary()
    return quaternion_rotation(value.GetReal(), imaginary[0], imaginary[1], imaginary[2])


def load_rizon_model(usd_path: Path) -> RizonModel:
    try:
        from pxr import Usd
    except ImportError as exc:
        raise RuntimeError("pxr is unavailable; run this script in the isaacsim conda environment") from exc

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise RuntimeError(f"failed to open USD: {usd_path}")

    axes = []
    locals_ = []
    lower = []
    upper = []
    zero_link7 = np.eye(4)
    for index in range(1, 8):
        joint = stage.GetPrimAtPath(f"/Rizon4/joints/joint{index}")
        if not joint.IsValid():
            raise RuntimeError(f"missing /Rizon4/joints/joint{index} in {usd_path}")
        position = np.asarray(joint.GetAttribute("physics:localPos0").Get(), dtype=float)
        orientation = _usd_quaternion(joint.GetAttribute("physics:localRot0").Get())
        local = transform(position, orientation)
        axes.append(str(joint.GetAttribute("physics:axis").Get()))
        locals_.append(local)
        zero_link7 = zero_link7 @ local
        lower.append(math.radians(float(joint.GetAttribute("physics:lowerLimit").Get())))
        upper.append(math.radians(float(joint.GetAttribute("physics:upperLimit").Get())))

    flange = stage.GetPrimAtPath("/Rizon4/flange")
    flange_position = np.asarray(flange.GetAttribute("xformOp:translate").Get(), dtype=float)
    flange_orientation = _usd_quaternion(flange.GetAttribute("xformOp:orient").Get())
    link7_to_flange = np.linalg.inv(zero_link7) @ transform(flange_position, flange_orientation)
    return RizonModel(
        tuple(axes), tuple(locals_), link7_to_flange, np.asarray(lower), np.asarray(upper)
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--max-elbow-error-mm", type=float, default=20.0)
    parser.add_argument("--min-limit-margin-deg", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scene_path = args.scene.expanduser().resolve()
    scene = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    robots = {robot["side"]: robot for robot in scene["robots"]}
    if set(robots) != {"left", "right"}:
        raise ValueError(f"{scene_path}: expected exactly one left and one right robot")

    results = {}
    for side in ("left", "right"):
        robot = robots[side]
        usd_path = (scene_path.parent / robot["usd"]).resolve()
        model = load_rizon_model(usd_path)
        q = np.asarray(robot["initial_q"], dtype=float)
        flange, pivots = model.forward(
            q, config_vector(robot["position"]), config_quaternion(robot["orientation"])
        )
        expected_position = config_vector(robot["target"]["position"])
        expected_orientation = euler_xyz(robot["target"]["euler_deg"])
        position_error = np.linalg.norm(flange[:3, 3] - expected_position)
        angle_error = orientation_error(flange[:3, :3], expected_orientation)
        margins = np.minimum(q - model.lower_limits, model.upper_limits - q)
        results[side] = (flange, pivots, position_error, angle_error, float(np.min(margins)))
        print(
            f"{side}: TCP error={position_error * 1000:.4f} mm / "
            f"{math.degrees(angle_error):.5f} deg, "
            f"minimum hard-limit margin={math.degrees(float(np.min(margins))):.2f} deg"
        )

    left_pivots = results["left"][1]
    right_pivots = results["right"][1]
    mirror = np.diag([1.0, -1.0, 1.0])
    pivot_errors = np.linalg.norm(left_pivots @ mirror - right_pivots, axis=1)
    elbow_error = float(pivot_errors[3])
    print(f"left J4 elbow:  {np.round(left_pivots[3], 5).tolist()}")
    print(f"right J4 elbow: {np.round(right_pivots[3], 5).tolist()}")
    print(f"J4 mirror error={elbow_error * 1000:.2f} mm")
    print(f"J1..J7 pivot mirror errors (mm)={np.round(pivot_errors * 1000, 2).tolist()}")

    passed = (
        max(results[side][2] for side in results) <= 1e-4
        and max(results[side][3] for side in results) <= math.radians(0.01)
        and min(results[side][4] for side in results) >= math.radians(args.min_limit_margin_deg)
        and elbow_error <= args.max_elbow_error_mm / 1000.0
    )
    print("SUCCESS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
