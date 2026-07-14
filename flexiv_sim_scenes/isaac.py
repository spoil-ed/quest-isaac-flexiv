"""Isaac runtime builders for Stage3 task scene objects."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .config import SceneObjectSpec, parse_scene_objects, scene_task_metadata


def _stage_from_world(world: Any):
    stage = getattr(world, "stage", None)
    if stage is not None:
        return stage
    try:
        import omni.usd

        return omni.usd.get_context().get_stage()
    except Exception as exc:
        raise RuntimeError("Could not resolve the active USD stage for scene object loading") from exc


def _set_xform(stage: Any, spec: SceneObjectSpec, *, scale: tuple[float, float, float] | None = None) -> None:
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(spec.prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Cannot set xform for missing prim {spec.prim_path}")
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*spec.position))
    w, x, y, z = spec.orientation
    xformable.AddOrientOp().Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
    xformable.AddScaleOp().Set(Gf.Vec3f(*(scale or spec.scale)))


def _apply_display_color(stage: Any, spec: SceneObjectSpec) -> None:
    if spec.color is None:
        return
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(spec.prim_path)
    if prim.IsValid():
        UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(*spec.color)])


def _apply_physics(stage: Any, spec: SceneObjectSpec) -> None:
    from pxr import UsdPhysics

    prim = stage.GetPrimAtPath(spec.prim_path)
    if not prim.IsValid():
        return
    if spec.collision:
        UsdPhysics.CollisionAPI.Apply(prim)
    if spec.rigid_body:
        rigid = UsdPhysics.RigidBodyAPI.Apply(prim)
        if spec.kinematic:
            rigid.CreateKinematicEnabledAttr(True)
        if spec.disable_gravity:
            rigid.CreateStartsAsleepAttr(False)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if spec.mass is not None:
            mass_api.CreateMassAttr(float(spec.mass))


def _define_cuboid(stage: Any, spec: SceneObjectSpec) -> None:
    from pxr import UsdGeom

    cube = UsdGeom.Cube.Define(stage, spec.prim_path)
    cube.CreateSizeAttr(1.0)
    sx, sy, sz = spec.size or (0.05, 0.05, 0.05)
    ox, oy, oz = spec.scale
    _set_xform(stage, spec, scale=(sx * ox, sy * oy, sz * oz))
    _apply_display_color(stage, spec)
    _apply_physics(stage, spec)


def _define_cylinder(stage: Any, spec: SceneObjectSpec) -> None:
    from pxr import UsdGeom

    cylinder = UsdGeom.Cylinder.Define(stage, spec.prim_path)
    cylinder.CreateRadiusAttr(float(spec.radius or 0.02))
    cylinder.CreateHeightAttr(float(spec.height or 0.10))
    cylinder.CreateAxisAttr("Z")
    _set_xform(stage, spec)
    _apply_display_color(stage, spec)
    _apply_physics(stage, spec)


def _set_existing_attr(prim: Any, names: tuple[str, ...], value: float) -> bool:
    for name in names:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            try:
                attr.Set(float(value))
                return True
            except Exception:
                continue
    return False


def _apply_initial_joint_positions(stage: Any, spec: SceneObjectSpec) -> dict[str, bool]:
    from pxr import Usd

    if not spec.joint_positions:
        return {}
    root = stage.GetPrimAtPath(spec.prim_path)
    if not root.IsValid():
        return {name: False for name in spec.joint_positions}
    status = {name: False for name in spec.joint_positions}
    for prim in Usd.PrimRange(root):
        name = prim.GetName()
        if name not in spec.joint_positions:
            continue
        status[name] = _set_existing_attr(
            prim,
            (
                "state:linear:physics:position",
                "state:angular:physics:position",
                "drive:linear:physics:targetPosition",
                "drive:angular:physics:targetPosition",
            ),
            spec.joint_positions[name],
        )
    return status


def _add_usd_reference(stage: Any, spec: SceneObjectSpec) -> dict[str, Any]:
    from isaacsim.core.utils.stage import add_reference_to_stage

    add_reference_to_stage(usd_path=str(spec.usd_path), prim_path=spec.prim_path)
    _set_xform(stage, spec)
    joint_status = _apply_initial_joint_positions(stage, spec)
    return {"joint_position_status": joint_status} if joint_status else {}


def build_scene_objects(
    world: Any,
    scene_config: dict[str, Any],
    *,
    config_path: Path | None = None,
) -> list[dict[str, Any]]:
    """Build configured Stage3 scene objects in the active Isaac stage."""

    specs = parse_scene_objects(scene_config, config_path=config_path, validate_assets=True)
    stage = _stage_from_world(world)
    summaries: list[dict[str, Any]] = []
    for spec in specs:
        extra: dict[str, Any] = {}
        if spec.object_type in {"usd", "articulation"}:
            extra = _add_usd_reference(stage, spec)
        elif spec.object_type == "cuboid":
            _define_cuboid(stage, spec)
        elif spec.object_type == "cylinder":
            _define_cylinder(stage, spec)
        else:
            raise ValueError(f"Unsupported scene object type: {spec.object_type}")
        summary = spec.summary()
        summary.update(extra)
        summaries.append(summary)
    return summaries


def stage3_sim_state(scene_config: dict[str, Any], scene_objects: list[dict[str, Any]], *, config_path: Path | None) -> dict[str, Any]:
    task = scene_task_metadata(scene_config)
    return {
        "task": task,
        "scene_config": str(config_path) if config_path is not None else None,
        "scene_objects": scene_objects,
    }
