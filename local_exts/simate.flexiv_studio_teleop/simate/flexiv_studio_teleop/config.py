"""Configuration loading for the Flexiv Studio teleop workflow."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .pose import QuatXYZW, Vector3, wxyz_to_xyzw


DEFAULT_INITIAL_Q = [0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0]
REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_USD = str(
    REPO_ROOT
    / "isaac_sim_ws/exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd"
)


@dataclass(frozen=True)
class TeleopSideConfig:
    side: str = "right"
    enabled: bool = True
    joint_group: str = "ARM_1"
    ee_link: str = "flange"
    ee_rot_x_deg: float = 0.0
    ee_rot_y_deg: float = 0.0
    ee_rot_z_deg: float = 0.0


@dataclass(frozen=True)
class RobotConfig:
    serial_number: str
    name: str
    usd: str
    position: Vector3
    orientation_xyzw: QuatXYZW
    initial_q: tuple[float, ...]
    teleop: TeleopSideConfig | None = None

    @property
    def prim_name(self) -> str:
        return self.name.replace(" ", "").replace("-", "_")

    @property
    def prim_path(self) -> str:
        return f"/World/Flexiv/{self.prim_name}"


@dataclass(frozen=True)
class FlexivTeleopConfig:
    robots: tuple[RobotConfig, ...]
    env_usd: str = ""
    gpu_dynamics: bool = False
    cameras: tuple[dict[str, Any], ...] = field(default_factory=tuple)
    motion: dict[str, Any] = field(default_factory=dict)

    @property
    def teleop_robots(self) -> tuple[RobotConfig, ...]:
        return tuple(robot for robot in self.robots if robot.teleop and robot.teleop.enabled)


def _position(data: dict[str, Any] | None) -> Vector3:
    data = data or {}
    return (
        float(data.get("x", 0.0)),
        float(data.get("y", 0.0)),
        float(data.get("z", 0.0)),
    )


def _orientation_xyzw(data: dict[str, Any] | None) -> QuatXYZW:
    data = data or {}
    return wxyz_to_xyzw(
        (
            float(data.get("w", 1.0)),
            float(data.get("x", 0.0)),
            float(data.get("y", 0.0)),
            float(data.get("z", 0.0)),
        )
    )


def _teleop_side(data: dict[str, Any] | None, *, default_side: str, default_enabled: bool) -> TeleopSideConfig:
    data = data or {}
    return TeleopSideConfig(
        side=str(data.get("side", default_side)).strip().lower() or default_side,
        enabled=bool(data.get("enabled", default_enabled)),
        joint_group=str(data.get("joint_group", "ARM_1")).strip() or "ARM_1",
        ee_link=str(data.get("ee_link", "flange")).strip() or "flange",
        ee_rot_x_deg=float(data.get("ee_rot_x_deg", 0.0)),
        ee_rot_y_deg=float(data.get("ee_rot_y_deg", 0.0)),
        ee_rot_z_deg=float(data.get("ee_rot_z_deg", 0.0)),
    )


def _robot(data: dict[str, Any], index: int) -> RobotConfig:
    serial = str(data.get("serial_number", data.get("name", f"Rizon4_{index:05d}"))).strip()
    if not serial:
        raise ValueError(f"robots[{index}] must define serial_number or name")
    name = str(data.get("name", serial)).strip() or serial
    initial_q = tuple(float(q) for q in data.get("initial_q", DEFAULT_INITIAL_Q))
    if len(initial_q) != 7:
        raise ValueError(f"robots[{index}].initial_q must contain 7 values")
    default_side = "right" if index == 0 else "left"
    return RobotConfig(
        serial_number=serial,
        name=name,
        usd=str(data.get("usd", DEFAULT_USD)).strip() or DEFAULT_USD,
        position=_position(data.get("position")),
        orientation_xyzw=_orientation_xyzw(data.get("orientation")),
        initial_q=initial_q,
        teleop=_teleop_side(data.get("teleop"), default_side=default_side, default_enabled=index == 0),
    )


def load_config(path: str | Path) -> FlexivTeleopConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as file_obj:
        raw = yaml.safe_load(file_obj) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Expected YAML mapping in {config_path}")

    robot_data = raw.get("robots")
    if robot_data is None and "robot" in raw:
        robot_data = [raw["robot"]]
    if not isinstance(robot_data, list) or not robot_data:
        raise ValueError(f"{config_path} must contain a non-empty robots list")

    robots = tuple(_robot(item, idx) for idx, item in enumerate(robot_data))
    return FlexivTeleopConfig(
        robots=robots,
        env_usd=str(raw.get("env_usd", "") or ""),
        gpu_dynamics=bool(raw.get("gpu_dynamics", False)),
        cameras=tuple(raw.get("cameras", []) or []),
        motion=dict(raw.get("motion", {}) or {}),
    )
