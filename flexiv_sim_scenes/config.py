"""YAML schema helpers for config-driven Flexiv simulation scenes."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]


def _default_unitree_asset_root() -> Path:
    configured = os.environ.get("UNITREE_SIM_ISAACLAB_ASSETS")
    if configured:
        return Path(configured).expanduser().resolve()
    workspace_assets = REPO_ROOT.parent / "unitree" / "unitree_sim_isaaclab" / "assets"
    return workspace_assets.resolve()


DEFAULT_UNITREE_ASSET_ROOT = _default_unitree_asset_root()
SUPPORTED_OBJECT_TYPES = {"usd", "articulation", "cuboid", "cylinder"}


@dataclass(frozen=True)
class SceneObjectSpec:
    name: str
    object_type: str
    prim_path: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    scale: tuple[float, float, float]
    usd_path: Path | None = None
    size: tuple[float, float, float] | None = None
    radius: float | None = None
    height: float | None = None
    color: tuple[float, float, float] | None = None
    mass: float | None = None
    collision: bool = True
    rigid_body: bool = False
    kinematic: bool = False
    disable_gravity: bool = False
    joint_positions: dict[str, float] | None = None
    metadata: dict[str, Any] | None = None

    def summary(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "name": self.name,
            "type": self.object_type,
            "prim_path": self.prim_path,
            "position": list(self.position),
            "orientation": list(self.orientation),
            "scale": list(self.scale),
        }
        if self.usd_path is not None:
            payload["usd_path"] = str(self.usd_path)
        if self.size is not None:
            payload["size"] = list(self.size)
        if self.radius is not None:
            payload["radius"] = self.radius
        if self.height is not None:
            payload["height"] = self.height
        if self.color is not None:
            payload["color"] = list(self.color)
        if self.mass is not None:
            payload["mass"] = self.mass
        if self.joint_positions:
            payload["joint_positions"] = dict(self.joint_positions)
        if self.metadata:
            payload["metadata"] = dict(self.metadata)
        payload["collision"] = self.collision
        payload["rigid_body"] = self.rigid_body
        payload["kinematic"] = self.kinematic
        payload["disable_gravity"] = self.disable_gravity
        return payload


def load_scene_config(path: Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    raw = config_path.read_text(encoding="utf-8")
    if config_path.suffix.lower() == ".json":
        data = json.loads(raw) or {}
    else:
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required for Stage3 scene YAML files") from exc
        data = yaml.safe_load(raw) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Scene config must contain a mapping: {config_path}")
    return data


def _as_tuple(values: Any, *, length: int, name: str) -> tuple[float, ...]:
    if isinstance(values, dict):
        keys = ("x", "y", "z") if length == 3 else ("w", "x", "y", "z")
        result = tuple(float(values[key]) for key in keys)
    else:
        result = tuple(float(item) for item in values)
    if len(result) != length:
        raise ValueError(f"{name} must contain {length} numeric values")
    return result


def _tuple3(value: Any, *, default: Iterable[float], name: str) -> tuple[float, float, float]:
    if value is None:
        return tuple(float(item) for item in default)  # type: ignore[return-value]
    result = _as_tuple(value, length=3, name=name)
    return (result[0], result[1], result[2])


def _quat(value: Any | None) -> tuple[float, float, float, float]:
    if value is None:
        return (1.0, 0.0, 0.0, 0.0)
    result = _as_tuple(value, length=4, name="orientation")
    return (result[0], result[1], result[2], result[3])


def _color(value: Any | None) -> tuple[float, float, float] | None:
    if value is None:
        return None
    result = _as_tuple(value, length=3, name="color")
    return (result[0], result[1], result[2])


def _expand_path_vars(value: str, *, asset_root: Path) -> str:
    expanded = os.path.expandvars(value)
    expanded = expanded.replace("${UNITREE_ASSET_ROOT}", str(asset_root))
    expanded = expanded.replace("$UNITREE_ASSET_ROOT", str(asset_root))
    expanded = expanded.replace("${REPO_ROOT}", str(REPO_ROOT))
    expanded = expanded.replace("$REPO_ROOT", str(REPO_ROOT))
    return expanded


def resolve_asset_path(value: Any, *, base: Path, asset_root: Path = DEFAULT_UNITREE_ASSET_ROOT) -> Path | None:
    if value in (None, ""):
        return None
    raw = _expand_path_vars(str(value), asset_root=asset_root)
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = base / path
    return path.resolve()


def _object_type(raw: dict[str, Any]) -> str:
    object_type = str(raw.get("type") or raw.get("asset_type") or "").strip().lower()
    if object_type not in SUPPORTED_OBJECT_TYPES:
        raise ValueError(
            f"scene object {raw.get('name') or raw.get('prim_path')!r} has unsupported type "
            f"{object_type!r}; expected one of {sorted(SUPPORTED_OBJECT_TYPES)}"
        )
    return object_type


def _joint_positions(raw: Any) -> dict[str, float] | None:
    if raw in (None, ""):
        return None
    if not isinstance(raw, dict):
        raise ValueError("joint_positions must be a mapping of joint name to position")
    return {str(key): float(value) for key, value in raw.items()}


def parse_scene_objects(
    scene_config: dict[str, Any],
    *,
    config_path: Path | None = None,
    validate_assets: bool = True,
    asset_root: Path = DEFAULT_UNITREE_ASSET_ROOT,
) -> list[SceneObjectSpec]:
    base = Path(config_path).expanduser().resolve().parent if config_path is not None else REPO_ROOT
    objects = scene_config.get("scene_objects") or []
    if not isinstance(objects, list):
        raise ValueError("scene_objects must be a list")

    specs: list[SceneObjectSpec] = []
    seen_prim_paths: set[str] = set()
    for idx, raw in enumerate(objects):
        if not isinstance(raw, dict):
            raise ValueError(f"scene_objects[{idx}] must be a mapping")
        object_type = _object_type(raw)
        name = str(raw.get("name") or f"{object_type}_{idx}")
        prim_path = str(raw.get("prim_path") or "").strip()
        if not prim_path.startswith("/"):
            raise ValueError(f"scene object {name!r} prim_path must be an absolute USD path")
        if prim_path in seen_prim_paths:
            raise ValueError(f"duplicate scene object prim_path: {prim_path}")
        seen_prim_paths.add(prim_path)

        usd_path = resolve_asset_path(raw.get("usd_path") or raw.get("usd"), base=base, asset_root=asset_root)
        if object_type in {"usd", "articulation"}:
            if usd_path is None:
                raise ValueError(f"scene object {name!r} requires usd_path/usd")
            if validate_assets and not usd_path.exists():
                raise FileNotFoundError(f"scene object {name!r} asset does not exist: {usd_path}")

        size = None
        radius = None
        height = None
        if object_type == "cuboid":
            size = _tuple3(raw.get("size"), default=(0.05, 0.05, 0.05), name=f"{name}.size")
        elif object_type == "cylinder":
            radius = float(raw.get("radius", 0.02))
            height = float(raw.get("height", 0.10))
            if radius <= 0.0 or height <= 0.0:
                raise ValueError(f"scene object {name!r} cylinder radius/height must be positive")

        specs.append(
            SceneObjectSpec(
                name=name,
                object_type=object_type,
                prim_path=prim_path,
                position=_tuple3(raw.get("position"), default=(0.0, 0.0, 0.0), name=f"{name}.position"),
                orientation=_quat(raw.get("orientation")),
                scale=_tuple3(raw.get("scale"), default=(1.0, 1.0, 1.0), name=f"{name}.scale"),
                usd_path=usd_path,
                size=size,
                radius=radius,
                height=height,
                color=_color(raw.get("color")),
                mass=float(raw["mass"]) if raw.get("mass") is not None else None,
                collision=bool(raw.get("collision", True)),
                rigid_body=bool(raw.get("rigid_body", object_type in {"cuboid", "cylinder"})),
                kinematic=bool(raw.get("kinematic", False)),
                disable_gravity=bool(raw.get("disable_gravity", False)),
                joint_positions=_joint_positions(raw.get("joint_positions")),
                metadata=raw.get("metadata") if isinstance(raw.get("metadata"), dict) else None,
            )
        )
    return specs


def summarize_scene_objects(
    scene_config: dict[str, Any],
    *,
    config_path: Path | None = None,
    validate_assets: bool = False,
) -> list[dict[str, Any]]:
    return [
        spec.summary()
        for spec in parse_scene_objects(scene_config, config_path=config_path, validate_assets=validate_assets)
    ]


def scene_task_metadata(scene_config: dict[str, Any]) -> dict[str, Any]:
    task = scene_config.get("task") or {}
    return dict(task) if isinstance(task, dict) else {}
