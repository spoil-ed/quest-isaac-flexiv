import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXT_ROOT = ROOT / "local_exts" / "simate.flexiv_studio_teleop"
SCRIPT = ROOT / "scripts" / "teleop_sdg.py"
CONFIG = ROOT / "configs/flexiv_studio_teleop.yaml"
FOLLOW_BALL = (
    ROOT
    / "standalone_examples"
    / "api"
    / "isaacsim.robot.manipulators"
    / "flexiv_quest"
    / "follow_ball_with_studio.py"
)


if str(EXT_ROOT) not in sys.path:
    sys.path.insert(0, str(EXT_ROOT))


def load_teleop_sdg():
    spec = importlib.util.spec_from_file_location("teleop_sdg", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_follow_ball():
    spec = importlib.util.spec_from_file_location("follow_ball_with_studio", FOLLOW_BALL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeSink:
    def __init__(self):
        self.started = []
        self.targets = []
        self.cleared = []

    def start_binding(self, serial_number, joint_group):
        self.started.append((serial_number, joint_group))

    def set_target(self, serial_number, joint_group, pose):
        self.targets.append((serial_number, joint_group, list(pose)))

    def clear_target(self, serial_number, joint_group):
        self.cleared.append((serial_number, joint_group))


class FlexivStudioTeleopTests(unittest.TestCase):
    def test_camera_look_at_points_usd_negative_z_at_target(self):
        mod = load_follow_ball()
        position = [2.2, 0.0, 1.45]
        target = [0.0, 0.0, 0.55]

        quat = mod.camera_look_at_quat_wxyz(position, target)
        w, x, y, z = quat
        camera_forward = [
            -2.0 * (x * z + w * y),
            -2.0 * (y * z - w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        ]
        delta = [target[i] - position[i] for i in range(3)]
        norm = sum(value * value for value in delta) ** 0.5
        expected = [value / norm for value in delta]

        for actual, wanted in zip(camera_forward, expected):
            self.assertAlmostEqual(actual, wanted)

    def test_camera_config_accepts_look_at_and_legacy_orientation(self):
        mod = load_follow_ball()
        position, orientation = mod.camera_pose_from_config(
            {
                "position": {"x": 2.2, "y": 0.0, "z": 1.45},
                "look_at": {"x": 0.0, "y": 0.0, "z": 0.55},
            }
        )
        self.assertEqual(position, [2.2, 0.0, 1.45])
        self.assertAlmostEqual(sum(value * value for value in orientation), 1.0)

        _position, legacy = mod.camera_pose_from_config(
            {
                "position": {"x": 1, "y": 2, "z": 3},
                "orientation": {"w": 1, "x": 0, "y": 0, "z": 0},
            }
        )
        self.assertEqual(legacy, [1.0, 0.0, 0.0, 0.0])

    def test_flexiv_pose_vector_uses_wxyz_quaternion_order(self):
        from simate.flexiv_studio_teleop.pose import flexiv_pose_vector

        self.assertEqual(
            flexiv_pose_vector((0.1, 0.2, 0.3), (0.5, 0.5, 0.5, 0.5)),
            [0.1, 0.2, 0.3, 0.5, 0.5, 0.5, 0.5],
        )

    def test_world_to_base_pose_applies_inverse_base_transform(self):
        from simate.flexiv_studio_teleop.pose import flexiv_pose_vector, world_to_base_pose

        pos, quat = world_to_base_pose(
            world_position=(1.25, 2.0, 3.5),
            world_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
            base_position=(1.0, 2.0, 3.0),
            base_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )

        self.assertEqual(flexiv_pose_vector(pos, quat), [0.25, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])

    def test_default_config_maps_right_controller_to_first_flexiv_robot(self):
        from simate.flexiv_studio_teleop.config import load_config

        config = load_config(CONFIG)

        self.assertEqual(len(config.robots), 1)
        robot = config.robots[0]
        self.assertEqual(robot.serial_number, "Rizon4-I0LIRN")
        self.assertEqual(robot.prim_path, "/World/Flexiv/Rizon4_I0LIRN")
        self.assertEqual(robot.teleop.side, "right")
        self.assertEqual(robot.teleop.joint_group, "ARM_1")
        self.assertFalse(config.motion.get("enabled"))

    def test_ik_adapter_streams_base_frame_flexiv_pose(self):
        from simate.flexiv_studio_teleop.ik_adapter import FlexivStudioIKController, SideBinding

        sink = FakeSink()
        binding = SideBinding(
            side="right",
            serial_number="Rizon4-ghsyIc",
            joint_group="ARM_1",
            prim_path="/World/Flexiv/Rizon4_ghsyIc",
            ee_link="flange",
            base_position=(1.0, 2.0, 3.0),
            base_orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        adapter = FlexivStudioIKController([binding], target_sink=sink, require_timeline_playing=False)

        self.assertTrue(adapter.enable("right"))
        adapter.update_targets(None, None, (1.25, 2.0, 3.5), (0.0, 0.0, 0.0, 1.0))

        self.assertEqual(sink.started, [("Rizon4-ghsyIc", "ARM_1")])
        self.assertEqual(sink.targets[-1], ("Rizon4-ghsyIc", "ARM_1", [0.25, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0]))

    def test_flexiv_studio_cli_sets_workflow_environment(self):
        teleop_sdg = load_teleop_sdg()

        args = teleop_sdg.parse_args(["flexiv-studio", "--no-cloudxr", "--stream-hz", "333"])
        env = teleop_sdg.workflow_env(args, ROOT)
        command = teleop_sdg.build_isaac_command(ROOT)

        self.assertEqual(env["SIMATE_TELEOP_WORKFLOW"], "flexiv-studio")
        self.assertEqual(env["SIMATE_FLEXIV_RDK_STREAM_HZ"], "333.0")
        self.assertEqual(env["SIMATE_FLEXIV_TELEOP_CONFIG"], str(CONFIG.resolve()))
        self.assertIn("simate.flexiv_studio_teleop", command)

    def test_follow_ball_base_to_world_inverts_world_to_base_pose(self):
        mod = load_follow_ball()
        base_position = (1.0, 2.0, 3.0)
        base_orientation = (1.0, 0.0, 0.0, 0.0)
        world_position = (1.25, 2.0, 3.5)
        world_orientation = (1.0, 0.0, 0.0, 0.0)

        pose = mod.world_target_to_flexiv_pose(
            world_position=world_position,
            world_orientation_wxyz=world_orientation,
            base_position=base_position,
            base_orientation_wxyz=base_orientation,
        )

        self.assertEqual(
            mod.flexiv_pose_to_world_target(
                pose_base_tcp_des=pose,
                base_position=base_position,
                base_orientation_wxyz=base_orientation,
            ),
            (world_position, world_orientation),
        )

    def test_follow_target_default_visual_is_xyz_frame(self):
        mod = load_follow_ball()

        self.assertEqual(mod.DEFAULT_TARGET_PRIM_PATH, "/World/TargetFrame")
        self.assertEqual(mod.DEFAULT_TARGET_NAME, "target_frame")
        self.assertIn("target_axis_length", mod.PARAM_OVERRIDE_KEYS)
        self.assertNotIn("ball_radius", mod.PARAM_OVERRIDE_KEYS)

    def test_follow_target_arrow_specs_describe_xyz_axes(self):
        mod = load_follow_ball()

        specs = mod.target_arrow_specs(axis_length=0.12, axis_radius=0.005)

        self.assertEqual([spec["axis"] for spec in specs], ["x", "x", "y", "y", "z", "z"])
        self.assertEqual([spec["kind"] for spec in specs], ["shaft", "head", "shaft", "head", "shaft", "head"])
        self.assertEqual(specs[0]["color"], (1.0, 0.05, 0.05))
        self.assertEqual(specs[2]["color"], (0.05, 0.75, 0.15))
        self.assertEqual(specs[4]["color"], (0.1, 0.35, 1.0))
        self.assertEqual(specs[0]["translation"], (0.043199999999999995, 0.0, 0.0))
        self.assertEqual(specs[4]["translation"], (0.0, 0.0, 0.043199999999999995))

    def test_follow_ball_rejects_wrong_quest_target_serial(self):
        mod = load_follow_ball()
        packet = {
            "schema": "rizon4_quest_target.v1",
            "serial": "Rizon4-WRONG",
            "joint_group": "ARM_1",
            "pose_base_tcp_des": [0, 0, 0, 1, 0, 0, 0],
            "monotonic_time": 10.0,
        }

        self.assertIsNone(
            mod.parse_quest_target_packet(
                packet,
                serial_number="Rizon4-I0LIRN",
                joint_group="ARM_1",
                now=10.1,
                max_age_sec=1.0,
            )
        )

    def test_relative_mapper_reference_orientation_ignores_packet_quat(self):
        mod = load_follow_ball()
        mapper = mod.QuestRelativeTargetMapper(
            axis_map=mod.parse_quest_axis_map("x,y,z"),
            scale=1.0,
            workspace_min=(-1.0, -1.0, -1.0),
            workspace_max=(1.0, 1.0, 1.0),
            position_deadband_m=0.0,
            orientation_mode="reference",
        )
        current = [0.30, 0.0, 0.40, 1.0, 0.0, 0.0, 0.0]
        packet = mod.QuestTargetPacket(
            seq=1,
            side="right",
            pose_base_tcp_des=[0.0, 0.0, 0.0, 0.0, 0.70710678, 0.0, 0.70710678],
            controller_position_openxr=[0.0, 0.0, 0.0],
            gripper_open_ratio=None,
            monotonic_time=1.0,
            controller_delta_base=[0.0, 0.0, 0.0],
        )
        self.assertEqual(mapper.update(packet, current), current)

        moved = mapper.update(packet._replace(seq=2, controller_delta_base=[0.02, 0.0, 0.0]), current)

        self.assertEqual(moved[:3], [0.32, 0.0, 0.4])
        self.assertEqual(moved[3:], [1.0, 0.0, 0.0, 0.0])

    def test_follow_ball_accepts_matching_quest_target_packet(self):
        mod = load_follow_ball()
        packet = {
            "schema": "rizon4_quest_target.v1",
            "serial": "Rizon4-I0LIRN",
            "joint_group": "ARM_1",
            "seq": 4,
            "side": "right",
            "pose_base_tcp_des": [0.4, -0.1, 0.6, 1, 0, 0, 0],
            "controller_position_openxr": [0.1, 0.2, -0.3],
            "monotonic_time": 10.0,
        }

        parsed = mod.parse_quest_target_packet(
            packet,
            serial_number="Rizon4-I0LIRN",
            joint_group="ARM_1",
            now=10.1,
            max_age_sec=1.0,
        )

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed.seq, 4)
        self.assertEqual(parsed.side, "right")
        self.assertEqual(parsed.pose_base_tcp_des, [0.4, -0.1, 0.6, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(parsed.controller_position_openxr, [0.1, 0.2, -0.3])
        self.assertEqual(parsed.monotonic_time, 10.0)

    def test_follow_ball_prefers_direct_quest_pose_over_ball_pose(self):
        mod = load_follow_ball()
        quest_target = mod.QuestTargetPacket(
            seq=5,
            side="right",
            pose_base_tcp_des=[0.6, -0.2, 0.7, 1.0, 0.0, 0.0, 0.0],
            controller_position_openxr=None,
            gripper_open_ratio=0.5,
            monotonic_time=10.0,
        )

        selected = mod.select_pose_base_tcp_des(
            quest_target=quest_target,
            world_position=(9.0, 9.0, 9.0),
            world_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            base_position=(1.0, 2.0, 3.0),
            base_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        )

        self.assertEqual(selected, quest_target.pose_base_tcp_des)

    def test_follow_ball_falls_back_to_ball_pose_without_quest_target(self):
        mod = load_follow_ball()

        selected = mod.select_pose_base_tcp_des(
            quest_target=None,
            world_position=(1.25, 2.0, 3.5),
            world_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
            base_position=(1.0, 2.0, 3.0),
            base_orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        )

        self.assertEqual(selected, [0.25, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0])

    def test_follow_ball_marks_quest_target_stale_after_timeout(self):
        mod = load_follow_ball()
        quest_target = mod.QuestTargetPacket(
            seq=5,
            side="right",
            pose_base_tcp_des=[0.6, -0.2, 0.7, 1.0, 0.0, 0.0, 0.0],
            controller_position_openxr=None,
            gripper_open_ratio=0.5,
            monotonic_time=10.0,
        )

        self.assertTrue(mod.quest_target_is_fresh(quest_target, now=10.2, max_age_sec=0.5))
        self.assertFalse(mod.quest_target_is_fresh(quest_target, now=10.7, max_age_sec=0.5))
        self.assertFalse(mod.quest_target_is_fresh(None, now=10.2, max_age_sec=0.5))

    def test_follow_ball_coordinate_observation_packet_contains_error(self):
        mod = load_follow_ball()
        quest_target = mod.QuestTargetPacket(
            seq=8,
            side="right",
            pose_base_tcp_des=[0.6, -0.2, 0.7, 1.0, 0.0, 0.0, 0.0],
            controller_position_openxr=None,
            gripper_open_ratio=0.5,
            monotonic_time=10.0,
        )

        packet = mod.build_coordinate_observation_packet(
            serial_number="Rizon4-I0LIRN",
            joint_group="ARM_1",
            servo_cycle=123,
            quest_target=quest_target,
            active=True,
            current_pose_base_tcp=[0.5, -0.1, 0.4, 1.0, 0.0, 0.0, 0.0],
            monotonic_time=10.25,
        )

        self.assertEqual(packet["schema"], "rizon4_quest_coordinate_observation.v1")
        self.assertTrue(packet["active"])
        self.assertEqual(packet["quest_seq"], 8)
        self.assertEqual(packet["target_pose_base_tcp"], quest_target.pose_base_tcp_des)
        self.assertEqual(packet["current_pose_base_tcp"], [0.5, -0.1, 0.4, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(packet["position_error"], [0.09999999999999998, -0.1, 0.29999999999999993])
        self.assertEqual(packet["age_sec"], 0.25)

    def test_follow_ball_maps_openxr_delta_to_rizon4_base_axes(self):
        mod = load_follow_ball()

        axis_map = mod.parse_quest_axis_map("-z,-x,y")
        mapped = mod.map_openxr_delta_to_base(
            [0.10, 0.20, -0.30],
            axis_map=axis_map,
            scale=0.5,
        )

        self.assertEqual(mapped, [0.15, -0.05, 0.1])

    def test_cartesian_pose_error_handles_quaternion_sign(self):
        mod = load_follow_ball()

        linear, angular = mod.cartesian_pose_error(
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.003, 0.004, 0.0, -1.0, 0.0, 0.0, 0.0],
        )

        self.assertAlmostEqual(linear, 0.005)
        self.assertAlmostEqual(angular, 0.0)

    def test_reset_timeout_allows_active_settling_grace(self):
        mod = load_follow_ball()

        self.assertFalse(
            mod.reset_timeout_expired(
                elapsed_sec=20.0,
                timeout_sec=20.0,
                settling_within_hysteresis=True,
            )
        )
        self.assertTrue(
            mod.reset_timeout_expired(
                elapsed_sec=20.0,
                timeout_sec=20.0,
                settling_within_hysteresis=False,
            )
        )

    def test_follow_ball_relative_mapper_anchors_position_and_uses_absolute_orientation(self):
        mod = load_follow_ball()
        mapper = mod.QuestRelativeTargetMapper(
            axis_map=mod.parse_quest_axis_map("-z,-x,y"),
            scale=0.5,
            workspace_min=(0.0, -1.0, 0.2),
            workspace_max=(1.0, 1.0, 1.4),
        )
        first = mod.QuestTargetPacket(
            seq=1,
            side="right",
            pose_base_tcp_des=[9.0, 9.0, 9.0, 0.0, 1.0, 0.0, 0.0],
            controller_position_openxr=[0.0, 1.0, -0.5],
            gripper_open_ratio=0.5,
            monotonic_time=10.0,
        )
        second = mod.QuestTargetPacket(
            seq=2,
            side="right",
            pose_base_tcp_des=[9.0, 9.0, 9.0, 0.0, 0.0, 1.0, 0.0],
            controller_position_openxr=[0.10, 1.20, -0.80],
            gripper_open_ratio=0.5,
            monotonic_time=10.1,
        )

        current_tcp = [0.40, -0.10, 0.70, 1.0, 0.0, 0.0, 0.0]

        self.assertEqual(mapper.update(first, current_tcp), current_tcp)
        mapped = mapper.update(second, current_tcp)
        for actual, expected in zip(mapped, [0.55, -0.15, 0.8, 0.0, 0.0, 1.0, 0.0]):
            self.assertAlmostEqual(actual, expected)

    def test_follow_ball_relative_mapper_prefers_transmitted_base_delta(self):
        mod = load_follow_ball()
        mapper = mod.QuestRelativeTargetMapper(
            axis_map=mod.parse_quest_axis_map("-z,-x,y"),
            scale=0.5,
            workspace_min=(0.0, -1.0, 0.2),
            workspace_max=(1.0, 1.0, 1.4),
        )
        first = mod.QuestTargetPacket(
            seq=1,
            side="right",
            pose_base_tcp_des=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            controller_position_openxr=[100.0, 100.0, 100.0],
            gripper_open_ratio=0.5,
            monotonic_time=10.0,
            controller_delta_base=[0.0, 0.0, 0.0],
        )
        second = mod.QuestTargetPacket(
            seq=2,
            side="right",
            pose_base_tcp_des=[0.3, -0.2, 0.1, 0.5, 0.5, 0.5, 0.5],
            controller_position_openxr=[200.0, 200.0, 200.0],
            gripper_open_ratio=0.5,
            monotonic_time=10.1,
            controller_delta_base=[0.3, -0.2, 0.1],
        )

        current_tcp = [0.40, -0.10, 0.70, 1.0, 0.0, 0.0, 0.0]

        self.assertEqual(mapper.update(first, current_tcp), current_tcp)
        self.assertEqual(mapper.update(second, current_tcp), [0.55, -0.2, 0.75, 0.5, 0.5, 0.5, 0.5])

    def test_follow_ball_relative_mapper_clips_position_to_workspace_bounds(self):
        mod = load_follow_ball()
        mapper = mod.QuestRelativeTargetMapper(
            axis_map=mod.parse_quest_axis_map("x,y,z"),
            scale=1.0,
            workspace_min=(0.15, -0.10, 0.50),
            workspace_max=(1.00, 0.10, 1.00),
        )
        first = mod.QuestTargetPacket(
            seq=1,
            side="right",
            pose_base_tcp_des=[9.0, 9.0, 9.0, 0.0, 1.0, 0.0, 0.0],
            controller_position_openxr=[0.0, 0.0, 0.0],
            gripper_open_ratio=0.5,
            monotonic_time=10.0,
        )
        second = mod.QuestTargetPacket(
            seq=2,
            side="right",
            pose_base_tcp_des=[9.0, 9.0, 9.0, 0.0, 1.0, 0.0, 0.0],
            controller_position_openxr=[-0.05, -0.20, 0.80],
            gripper_open_ratio=0.5,
            monotonic_time=10.1,
        )

        current_tcp = [0.092, -0.1195, 1.2635, 1.0, 0.0, 0.0, 0.0]

        self.assertEqual(mapper.update(first, current_tcp), current_tcp)
        mapped = mapper.update(second, current_tcp)
        for actual, expected in zip(mapped, [0.15, -0.10, 1.00, 0.0, 1.0, 0.0, 0.0]):
            self.assertAlmostEqual(actual, expected)

    def test_cartesian_target_limiter_enforces_translation_and_rotation_speed(self):
        mod = load_follow_ball()
        limiter = mod.CartesianTargetLimiter(
            workspace_min=(0.0, -1.0, 0.2),
            workspace_max=(1.0, 1.0, 1.4),
            max_linear_speed_m_s=0.10,
            max_angular_speed_rad_s=0.50,
        )
        start = [0.40, 0.0, 0.70, 1.0, 0.0, 0.0, 0.0]
        limiter.reset(start)

        limited = limiter.limit([0.80, 0.0, 0.70, 0.0, 1.0, 0.0, 0.0], dt=0.1)

        self.assertAlmostEqual(limited[0], 0.41)
        self.assertAlmostEqual(limited[1], 0.0)
        self.assertAlmostEqual(limited[2], 0.70)
        angular_step = 2.0 * __import__("math").acos(abs(limited[3]))
        self.assertAlmostEqual(angular_step, 0.05)

    def test_cartesian_target_limiter_does_not_jump_when_start_is_outside_workspace(self):
        mod = load_follow_ball()
        limiter = mod.CartesianTargetLimiter(
            workspace_min=(0.15, -1.0, 0.2),
            workspace_max=(1.0, 1.0, 1.4),
            max_linear_speed_m_s=0.10,
            max_angular_speed_rad_s=0.50,
        )
        limiter.reset([0.10, 0.0, 0.70, 1.0, 0.0, 0.0, 0.0])

        limited = limiter.limit([0.10, 0.0, 0.70, 1.0, 0.0, 0.0, 0.0], dt=0.1)

        self.assertAlmostEqual(limited[0], 0.11)


if __name__ == "__main__":
    unittest.main()
