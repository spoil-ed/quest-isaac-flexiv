"""Controller-compatible adapter that replaces Isaac Teleop IK with Flexiv RDK."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Literal

from .config import RobotConfig
from .pose import QuatXYZW, Vector3, apply_local_rotation_offset, flexiv_pose_vector, world_to_base_pose
from .rdk_sink import FlexivRdkCartesianSink


@dataclass(frozen=True)
class SideBinding:
    side: str
    serial_number: str
    joint_group: str
    prim_path: str
    ee_link: str
    base_position: Vector3
    base_orientation_xyzw: QuatXYZW
    ee_rot_x_deg: float = 0.0
    ee_rot_y_deg: float = 0.0
    ee_rot_z_deg: float = 0.0

    @classmethod
    def from_robot_config(cls, robot: RobotConfig) -> "SideBinding":
        if robot.teleop is None:
            raise ValueError(f"robot {robot.serial_number} has no teleop section")
        return cls(
            side=robot.teleop.side,
            serial_number=robot.serial_number,
            joint_group=robot.teleop.joint_group,
            prim_path=robot.prim_path,
            ee_link=robot.teleop.ee_link,
            base_position=robot.position,
            base_orientation_xyzw=robot.orientation_xyzw,
            ee_rot_x_deg=robot.teleop.ee_rot_x_deg,
            ee_rot_y_deg=robot.teleop.ee_rot_y_deg,
            ee_rot_z_deg=robot.teleop.ee_rot_z_deg,
        )


@dataclass
class IKValidationResultCompat:
    valid: bool
    message: str
    articulation_path: str = ""
    link_names: list[str] = field(default_factory=list)
    dof_names: list[str] = field(default_factory=list)
    num_dofs: int = 0
    arm_dofs: int | None = None


@dataclass
class _ArmState:
    binding: SideBinding | None = None
    path: str = ""
    ee_link_name: str = ""
    ee_rot_x_deg: float = 0.0
    ee_rot_y_deg: float = 0.0
    ee_rot_z_deg: float = 0.0
    configured: bool = False
    running: bool = False
    solver_type: Any = "position-based"
    ik_method: Any = "singular-value-decomposition"
    gain: float = 5.0
    vr_target_filter: float = 0.0
    max_joint_step: float = 0.0
    pink_qp_solver: str = "daqp"
    pink_task_gain: float = 0.5
    pink_posture_cost: float = 0.001
    pink_lm_damping: float = 1.0
    num_arm_dofs: int = 7
    latest_pose: list[float] | None = None


class FlexivStudioIKController:
    """Drop-in replacement for Isaac Teleop's RobotIKController.

    It accepts the same TeleopManager calls as the built-in IK controller, but
    converts each wrist target into a Flexiv base-frame TCP pose and streams it
    to the Studio/RDK stack. It never writes joint targets to the Isaac
    articulation; Isaac motion comes back through flexivsimplugin target_drives.
    """

    def __init__(
        self,
        bindings: list[SideBinding] | tuple[SideBinding, ...],
        target_sink: FlexivRdkCartesianSink | None = None,
        *,
        require_timeline_playing: bool = True,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self._arms = {"left": _ArmState(), "right": _ArmState()}
        self._target_sink = target_sink
        self._require_timeline_playing = require_timeline_playing
        self._log = log or (lambda _msg: None)
        self._on_status_changed: Callable[[str, bool], None] | None = None
        self._target_coordinate_system: Any = None
        self._default_solver_type = self._load_default_solver_type()
        self._default_ik_method = self._load_default_ik_method()
        for binding in bindings:
            side = binding.side.lower()
            if side not in self._arms:
                raise ValueError(f"Unsupported teleop side: {binding.side}")
            arm = self._arms[side]
            arm.binding = binding
            arm.path = binding.prim_path
            arm.ee_link_name = binding.ee_link
            arm.ee_rot_x_deg = binding.ee_rot_x_deg
            arm.ee_rot_y_deg = binding.ee_rot_y_deg
            arm.ee_rot_z_deg = binding.ee_rot_z_deg
            arm.solver_type = self._default_solver_type
            arm.ik_method = self._default_ik_method

    @staticmethod
    def _load_default_solver_type() -> Any:
        try:
            from isaacsim.replicator.teleop.controllers import IKSolverType

            return IKSolverType.POSITION_BASED
        except Exception:
            return "position-based"

    @staticmethod
    def _load_default_ik_method() -> Any:
        try:
            from isaacsim.replicator.teleop.controllers import IKMethod

            return IKMethod.SVD
        except Exception:
            return "singular-value-decomposition"

    def _arm(self, side: str) -> _ArmState:
        return self._arms[side.lower()]

    def set_coordinate_system(self, target_coordinate_system: Any) -> None:
        self._target_coordinate_system = target_coordinate_system

    def set_on_status_changed(self, callback: Callable[[str, bool], None] | None) -> None:
        self._on_status_changed = callback

    def set_articulation_path(self, side: Literal["left", "right"], prim_path: str | None) -> None:
        arm = self._arm(side)
        path = prim_path or ""
        if arm.path != path:
            self.destroy(side)
        arm.path = path

    def set_ee_link_name(self, side: Literal["left", "right"], name: str) -> None:
        self._arm(side).ee_link_name = name or ""

    def set_ee_rotation_offsets(
        self,
        side: Literal["left", "right"],
        x_deg: float = 0.0,
        y_deg: float = 0.0,
        z_deg: float = 0.0,
    ) -> None:
        arm = self._arm(side)
        arm.ee_rot_x_deg = float(x_deg)
        arm.ee_rot_y_deg = float(y_deg)
        arm.ee_rot_z_deg = float(z_deg)

    def compute_arm_dofs(self, side: Literal["left", "right"]) -> int | None:
        return self._arm(side).num_arm_dofs

    def set_num_arm_dofs(self, side: Literal["left", "right"], n: int) -> None:
        self._arm(side).num_arm_dofs = max(1, int(n))

    def set_solver_type(self, side: Literal["left", "right"], solver_type: Any) -> tuple[bool, str]:
        self._arm(side).solver_type = solver_type
        return True, "Flexiv Studio stack owns IK/control"

    def get_solver_type(self, side: Literal["left", "right"]) -> Any:
        return self._arm(side).solver_type

    @staticmethod
    def get_solver_availability(_solver_type: Any) -> tuple[bool, str]:
        return True, ""

    def set_ik_method(self, side: Literal["left", "right"], method: Any) -> None:
        self._arm(side).ik_method = method

    def get_ik_method(self, side: Literal["left", "right"]) -> Any:
        return self._arm(side).ik_method

    def set_gain(self, side: Literal["left", "right"], value: float) -> None:
        self._arm(side).gain = float(value)

    def get_gain(self, side: Literal["left", "right"]) -> float:
        return self._arm(side).gain

    def set_vr_target_filter(self, side: Literal["left", "right"], value: float) -> None:
        self._arm(side).vr_target_filter = float(value)

    def set_max_joint_step(self, side: Literal["left", "right"], value: float) -> None:
        self._arm(side).max_joint_step = float(value)

    def set_pink_task_gain(self, side: Literal["left", "right"], value: float) -> None:
        self._arm(side).pink_task_gain = float(value)

    def get_pink_task_gain(self, side: Literal["left", "right"]) -> float:
        return self._arm(side).pink_task_gain

    def set_pink_qp_solver(self, side: Literal["left", "right"], solver_name: str) -> tuple[bool, str]:
        self._arm(side).pink_qp_solver = solver_name or "daqp"
        return True, "ok"

    def get_pink_qp_solver(self, side: Literal["left", "right"]) -> str:
        return self._arm(side).pink_qp_solver

    @staticmethod
    def get_pink_qp_solver_names() -> tuple[str, ...]:
        return ("daqp",)

    @staticmethod
    def get_pink_qp_solver_availability(_solver_name: str) -> tuple[bool, str]:
        return True, ""

    def set_pink_posture_cost(self, side: Literal["left", "right"], value: float) -> None:
        self._arm(side).pink_posture_cost = float(value)

    def get_pink_posture_cost(self, side: Literal["left", "right"]) -> float:
        return self._arm(side).pink_posture_cost

    def set_pink_lm_damping(self, side: Literal["left", "right"], value: float) -> None:
        self._arm(side).pink_lm_damping = float(value)

    def get_pink_lm_damping(self, side: Literal["left", "right"]) -> float:
        return self._arm(side).pink_lm_damping

    def validate(self, side: Literal["left", "right"]) -> IKValidationResultCompat:
        arm = self._arm(side)
        if not arm.path:
            return IKValidationResultCompat(False, "Set prim path first.")

        stage_result = self._validate_against_stage(arm)
        if stage_result is not None:
            return stage_result

        binding = arm.binding
        if binding is not None and arm.path == binding.prim_path:
            link_names = [binding.ee_link]
            valid = bool(arm.ee_link_name and arm.ee_link_name in link_names)
            return IKValidationResultCompat(
                valid=valid,
                message=(
                    f"Flexiv Studio target sink: {binding.serial_number}/{binding.joint_group}"
                    if valid
                    else f"Select EE link for {binding.serial_number}"
                ),
                articulation_path=arm.path,
                link_names=link_names,
                dof_names=[f"joint_{idx}" for idx in range(7)],
                num_dofs=7,
                arm_dofs=7,
            )
        return IKValidationResultCompat(False, f"No Flexiv binding for '{arm.path}'.")

    def _validate_against_stage(self, arm: _ArmState) -> IKValidationResultCompat | None:
        try:
            from isaacsim.core.experimental.prims import Articulation

            art_paths = Articulation.fetch_articulation_root_api_prim_paths(arm.path)
            art_path = art_paths[0] if art_paths else None
            if art_path is None:
                return None
            robot = Articulation(art_path)
            link_names = list(robot.link_names)
            dof_names = list(robot.dof_names)
            valid = bool(arm.ee_link_name and arm.ee_link_name in link_names)
            return IKValidationResultCompat(
                valid=valid,
                message=(
                    f"Flexiv Studio target sink: {arm.ee_link_name}, DOFs: {min(7, len(dof_names))}/{len(dof_names)}"
                    if valid
                    else f"Articulation: {art_path} ({len(dof_names)} DOFs). Select EE link."
                ),
                articulation_path=art_path,
                link_names=link_names,
                dof_names=dof_names,
                num_dofs=len(dof_names),
                arm_dofs=min(7, len(dof_names)),
            )
        except Exception:
            return None

    def configure(self, side: Literal["left", "right"]) -> bool:
        result = self.validate(side)
        arm = self._arm(side)
        arm.configured = result.valid
        return result.valid

    def enable(self, side: Literal["left", "right"]) -> bool:
        arm = self._arm(side)
        if not arm.configured and not self.configure(side):
            return False
        arm.running = True
        if self._target_sink is not None and arm.binding is not None:
            self._target_sink.start_binding(arm.binding.serial_number, arm.binding.joint_group)
        if self._on_status_changed is not None:
            self._on_status_changed(side, True)
        return True

    def disable(self, side: Literal["left", "right"]) -> None:
        arm = self._arm(side)
        arm.running = False
        if self._target_sink is not None and arm.binding is not None:
            self._target_sink.clear_target(arm.binding.serial_number, arm.binding.joint_group)

    def destroy(self, side: Literal["left", "right"]) -> None:
        self.disable(side)
        arm = self._arm(side)
        arm.configured = False
        arm.latest_pose = None

    def is_configured(self, side: Literal["left", "right"]) -> bool:
        return self._arm(side).configured

    def is_running(self, side: Literal["left", "right"]) -> bool:
        return self._arm(side).running

    def is_reachable(self, _side: Literal["left", "right"]) -> bool:
        return True

    def update_targets(
        self,
        left_pos: tuple[float, float, float] | None,
        left_orient: tuple[float, float, float, float] | None,
        right_pos: tuple[float, float, float] | None,
        right_orient: tuple[float, float, float, float] | None,
    ) -> None:
        if self._require_timeline_playing and not self._timeline_is_playing():
            return

        poses = {"left": (left_pos, left_orient), "right": (right_pos, right_orient)}
        for side, (pos, orient) in poses.items():
            arm = self._arm(side)
            binding = arm.binding
            if not arm.running or binding is None or pos is None:
                continue

            target_orient = apply_local_rotation_offset(
                orient,
                arm.ee_rot_x_deg,
                arm.ee_rot_y_deg,
                arm.ee_rot_z_deg,
            )
            base_pos, base_orient = world_to_base_pose(
                pos,
                target_orient,
                binding.base_position,
                binding.base_orientation_xyzw,
            )
            pose = flexiv_pose_vector(base_pos, base_orient)
            arm.latest_pose = pose
            if self._target_sink is not None:
                self._target_sink.set_target(binding.serial_number, binding.joint_group, pose)

    @staticmethod
    def _timeline_is_playing() -> bool:
        try:
            import omni.timeline

            return bool(omni.timeline.get_timeline_interface().is_playing())
        except Exception:
            return True
