"""Pose helpers for Isaac world poses and Flexiv RDK pose vectors."""

from __future__ import annotations

import math
from collections.abc import Iterable


Vector3 = tuple[float, float, float]
QuatXYZW = tuple[float, float, float, float]


def _vec3(values: Iterable[float]) -> Vector3:
    x, y, z = values
    return float(x), float(y), float(z)


def _quat_xyzw(values: Iterable[float]) -> QuatXYZW:
    x, y, z, w = values
    return normalize_quat_xyzw((float(x), float(y), float(z), float(w)))


def wxyz_to_xyzw(values: Iterable[float]) -> QuatXYZW:
    """Convert a Flexiv/Isaac config quaternion from wxyz to xyzw."""
    w, x, y, z = values
    return _quat_xyzw((x, y, z, w))


def normalize_quat_xyzw(q: QuatXYZW) -> QuatXYZW:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quat_conjugate_xyzw(q: QuatXYZW) -> QuatXYZW:
    x, y, z, w = normalize_quat_xyzw(q)
    return -x, -y, -z, w


def quat_mul_xyzw(a: QuatXYZW, b: QuatXYZW) -> QuatXYZW:
    ax, ay, az, aw = normalize_quat_xyzw(a)
    bx, by, bz, bw = normalize_quat_xyzw(b)
    return normalize_quat_xyzw(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def rotate_vector(q: QuatXYZW, v: Vector3) -> Vector3:
    qx, qy, qz, qw = normalize_quat_xyzw(q)
    vx, vy, vz = v

    # q * [v, 0] * conj(q), expanded to avoid temporary tuple churn.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def axis_angle_quat(axis: Vector3, degrees: float) -> QuatXYZW:
    ax, ay, az = axis
    radians = math.radians(float(degrees))
    half = radians * 0.5
    s = math.sin(half)
    return normalize_quat_xyzw((ax * s, ay * s, az * s, math.cos(half)))


def rotation_offset_quat_xyzw(x_deg: float, y_deg: float, z_deg: float) -> QuatXYZW:
    """Compose local XYZ rotation offsets in the same order as Isaac Teleop."""
    qx = axis_angle_quat((1.0, 0.0, 0.0), x_deg)
    qy = axis_angle_quat((0.0, 1.0, 0.0), y_deg)
    qz = axis_angle_quat((0.0, 0.0, 1.0), z_deg)
    return quat_mul_xyzw(quat_mul_xyzw(qx, qy), qz)


def apply_local_rotation_offset(
    orientation_xyzw: QuatXYZW | None,
    x_deg: float,
    y_deg: float,
    z_deg: float,
) -> QuatXYZW:
    base = _quat_xyzw(orientation_xyzw or (0.0, 0.0, 0.0, 1.0))
    return quat_mul_xyzw(base, rotation_offset_quat_xyzw(x_deg, y_deg, z_deg))


def world_to_base_pose(
    world_position: Iterable[float],
    world_orientation_xyzw: Iterable[float] | None,
    base_position: Iterable[float],
    base_orientation_xyzw: Iterable[float],
) -> tuple[Vector3, QuatXYZW]:
    """Convert an Isaac world-space target pose into the robot base frame."""
    wp = _vec3(world_position)
    wo = _quat_xyzw(world_orientation_xyzw or (0.0, 0.0, 0.0, 1.0))
    bp = _vec3(base_position)
    bo = _quat_xyzw(base_orientation_xyzw)

    inv_base = quat_conjugate_xyzw(bo)
    rel_world = (wp[0] - bp[0], wp[1] - bp[1], wp[2] - bp[2])
    rel_pos = rotate_vector(inv_base, rel_world)
    rel_ori = quat_mul_xyzw(inv_base, wo)
    return rel_pos, rel_ori


def flexiv_pose_vector(position: Iterable[float], orientation_xyzw: Iterable[float]) -> list[float]:
    """Return Flexiv RDK pose vector [x, y, z, qw, qx, qy, qz]."""
    x, y, z = _vec3(position)
    qx, qy, qz, qw = _quat_xyzw(orientation_xyzw)
    return [x, y, z, qw, qx, qy, qz]
