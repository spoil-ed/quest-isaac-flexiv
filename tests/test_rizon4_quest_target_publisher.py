import importlib.util
import math
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "rizon4_quest_target_publisher.py"
TELEVUER_PATH = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "televuer"
    / "src"
    / "televuer"
    / "televuer.py"
)
spec = importlib.util.spec_from_file_location("rizon4_quest_target_publisher", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


def _pose(x, y, z):
    matrix = [
        [1.0, 0.0, 0.0, x],
        [0.0, 1.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return matrix


def _rot_z_90_pose(x=0.0, y=0.0, z=0.0):
    return [
        [0.0, -1.0, 0.0, x],
        [1.0, 0.0, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rotate_vector_wxyz(quat, vector):
    pure = [0.0, *vector]
    rotated = mod.quat_multiply_wxyz(
        mod.quat_multiply_wxyz(quat, pure),
        mod.quat_inverse_wxyz(quat),
    )
    return rotated[1:]


class Rizon4QuestTargetPublisherTests(unittest.TestCase):
    def test_cli_translation_deadband_defaults_to_zero(self):
        args = mod.parse_args([])

        self.assertEqual(args.position_deadband, 0.0)

    def test_cli_supports_one_televuer_session_for_both_controllers(self):
        args = mod.parse_args(["--side", "both"])

        self.assertEqual(args.side, "both")
        self.assertEqual(args.left_serial_number, "Rizon4-qSaFLh")
        self.assertEqual(args.right_serial_number, "Rizon4-I0LIRN")

    def test_televuer_streams_both_motion_controllers(self):
        source = TELEVUER_PATH.read_text(encoding="utf-8")
        motion_controllers = source.split("def _motion_controllers", maxsplit=1)[1].split(
            "def _copy_controller_pose", maxsplit=1
        )[0]

        self.assertIn("left=True", motion_controllers)
        self.assertIn("right=True", motion_controllers)

    def test_default_televuer_root_is_inside_repo(self):
        repo_root = Path(__file__).resolve().parents[1]

        self.assertTrue(mod.DEFAULT_TELEVUER_ROOT.is_relative_to(repo_root))
        self.assertTrue(mod.DEFAULT_CERT_FILE.is_relative_to(repo_root))
        self.assertTrue(mod.DEFAULT_KEY_FILE.is_relative_to(repo_root))

    def test_mapper_outputs_relative_origin_on_first_enable(self):
        mapper = mod.QuestRelativeMapper(position_delta_scale=3.0, engage_settle_sec=0.0)

        packet = mapper.update(_pose(0.2, -0.1, 0.4), enabled=True, seq=1, now=10.0)

        self.assertIsNotNone(packet)
        self.assertEqual(packet["pose_base_tcp_des"][:3], [0.0, 0.0, 0.0])
        self.assertEqual(packet["controller_delta_base"], [0.0, 0.0, 0.0])
        self.assertEqual(packet["reason"], "tracking")

    def test_mapper_scales_relative_delta_while_enabled(self):
        mapper = mod.QuestRelativeMapper(
            axis_map="x,y,z",
            position_delta_scale=3.0,
            engage_settle_sec=0.0,
            position_deadband=0.0,
        )
        mapper.update(_pose(0.2, -0.1, 0.4), enabled=True, seq=1, now=10.0)

        packet = mapper.update(_pose(0.21, -0.12, 0.45), enabled=True, seq=2, now=10.1)

        self.assertEqual(
            [round(value, 4) for value in packet["pose_base_tcp_des"][:3]],
            [0.03, -0.06, 0.15],
        )
        self.assertEqual(
            [round(value, 4) for value in packet["controller_delta_base"]],
            [0.03, -0.06, 0.15],
        )

    def test_default_axis_map_converts_local_controller_axes_to_base_axes(self):
        mapper = mod.QuestRelativeMapper(
            position_delta_scale=1.0,
            engage_settle_sec=0.0,
            position_deadband=0.0,
        )
        mapper.update(_pose(0.0, 0.0, 0.0), enabled=True, seq=1, now=10.0)

        forward = mapper.update(_pose(0.0, 0.0, -0.2), enabled=True, seq=2, now=10.1)
        left = mapper.update(_pose(-0.2, 0.0, 0.0), enabled=True, seq=3, now=10.2)
        up = mapper.update(_pose(0.0, 0.2, 0.0), enabled=True, seq=4, now=10.3)

        self.assertEqual(forward["controller_delta_base"], [0.2, 0.0, 0.0])
        self.assertEqual(left["controller_delta_base"], [0.0, 0.2, 0.0])
        self.assertEqual(up["controller_delta_base"], [0.0, 0.0, 0.2])

    def test_mapper_holds_zero_during_engage_settle_window(self):
        mapper = mod.QuestRelativeMapper(position_delta_scale=3.0, engage_settle_sec=0.15)
        mapper.update(_pose(0.2, -0.1, 0.4), enabled=True, seq=1, now=10.0)

        packet = mapper.update(_pose(0.25, -0.2, 0.5), enabled=True, seq=2, now=10.1)

        self.assertIsNone(packet)

    def test_mapper_first_packet_after_settle_is_exactly_zero(self):
        mapper = mod.QuestRelativeMapper(
            tcp_rot_offset_wxyz=[1.0, 0.0, 0.0, 0.0],
            engage_settle_sec=0.25,
        )

        self.assertIsNone(mapper.update(_pose(0.0, 0.0, 0.0), enabled=True, seq=1, now=10.0))
        self.assertIsNone(mapper.update(_pose(0.1, 0.0, 0.0), enabled=True, seq=2, now=10.1))
        packet = mapper.update(_rot_z_90_pose(0.2, 0.0, 0.0), enabled=True, seq=3, now=10.3)

        self.assertEqual(packet["controller_delta_base"], [0.0, 0.0, 0.0])
        self.assertEqual(packet["pose_base_tcp_des"][:3], [0.0, 0.0, 0.0])

    def test_orientation_uses_absolute_openxr_to_base_axis_mapping(self):
        mapper = mod.QuestRelativeMapper(
            tcp_rot_offset_wxyz=[1.0, 0.0, 0.0, 0.0],
            engage_settle_sec=0.0,
        )

        packet = mapper.update(_pose(0.0, 0.0, 0.0), enabled=True, seq=1, now=10.0)

        hand_forward_in_base = _rotate_vector_wxyz(packet["pose_base_tcp_des"][3:], [1.0, 0.0, 0.0])
        hand_left_in_base = _rotate_vector_wxyz(packet["pose_base_tcp_des"][3:], [0.0, 1.0, 0.0])
        hand_up_in_base = _rotate_vector_wxyz(packet["pose_base_tcp_des"][3:], [0.0, 0.0, 1.0])
        self.assertEqual([round(value, 4) for value in hand_forward_in_base], [1.0, 0.0, 0.0])
        self.assertEqual([round(value, 4) for value in hand_left_in_base], [0.0, 1.0, 0.0])
        self.assertEqual([round(value, 4) for value in hand_up_in_base], [0.0, 0.0, 1.0])

    def test_default_orientation_maps_hand_semantic_axes_to_tcp_zero(self):
        mapper = mod.QuestRelativeMapper(engage_settle_sec=0.0)

        packet = mapper.update(_pose(0.0, 0.0, 0.0), enabled=True, seq=1, now=10.0)

        tcp_forward_in_base = _rotate_vector_wxyz(packet["pose_base_tcp_des"][3:], [0.0, 0.0, -1.0])
        tcp_left_in_base = _rotate_vector_wxyz(packet["pose_base_tcp_des"][3:], [0.0, 1.0, 0.0])
        tcp_up_in_base = _rotate_vector_wxyz(packet["pose_base_tcp_des"][3:], [1.0, 0.0, 0.0])
        self.assertEqual([round(value, 4) for value in tcp_forward_in_base], [-1.0, 0.0, 0.0])
        self.assertEqual([round(value, 4) for value in tcp_left_in_base], [0.0, -1.0, 0.0])
        self.assertEqual([round(value, 4) for value in tcp_up_in_base], [0.0, 0.0, 1.0])

    def test_mapper_applies_position_deadband_after_settle(self):
        mapper = mod.QuestRelativeMapper(
            position_delta_scale=3.0,
            engage_settle_sec=0.0,
            position_deadband=0.02,
        )
        mapper.update(_pose(0.2, -0.1, 0.4), enabled=True, seq=1, now=10.0)

        packet = mapper.update(_pose(0.205, -0.102, 0.401), enabled=True, seq=2, now=10.1)

        self.assertEqual(packet["controller_delta_base"], [0.0, 0.0, 0.0])
        self.assertEqual(packet["pose_base_tcp_des"][:3], [0.0, 0.0, 0.0])

    def test_mapper_pauses_after_release(self):
        mapper = mod.QuestRelativeMapper(position_delta_scale=2.0)
        mapper.update(_pose(1.0, 1.0, 1.0), enabled=True, seq=1, now=10.0)
        mapper.update(_pose(1.1, 1.0, 0.9), enabled=True, seq=2, now=10.1)

        released = mapper.update(_pose(5.0, 5.0, 5.0), enabled=False, seq=3, now=10.2)

        self.assertIsNone(released)

    def test_build_packet_shape_and_schema(self):
        packet = mod.build_quest_packet(
            seq=7,
            side="right",
            pose_base_tcp_des=[0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0],
            controller_position_openxr=[1.0, 2.0, 3.0],
            controller_delta_base=[0.1, 0.2, 0.3],
            now=12.0,
            reason="tracking",
        )

        self.assertEqual(packet["schema"], "rizon4_quest_target.v1")
        self.assertEqual(packet["serial"], "Rizon4-I0LIRN")
        self.assertEqual(packet["joint_group"], "ARM_1")
        self.assertEqual(len(packet["pose_base_tcp_des"]), 7)
        self.assertEqual(packet["controller_delta_base"], [0.1, 0.2, 0.3])
        self.assertTrue(all(math.isfinite(value) for value in packet["pose_base_tcp_des"]))

    def test_build_gripper_packet_is_independent_of_arm_tracking(self):
        packet = mod.build_gripper_packet(
            seq=8,
            side="left",
            closed=True,
            now=12.5,
            serial_number="Rizon4-qSaFLh",
        )

        self.assertEqual(packet["schema"], "rizon4_quest_gripper.v1")
        self.assertEqual(packet["serial"], "Rizon4-qSaFLh")
        self.assertEqual(packet["side"], "left")
        self.assertTrue(packet["closed"])

    def test_build_input_packet_reports_pose_and_buttons_without_enabling_control(self):
        packet = mod.build_quest_input_packet(
            seq=9,
            side="left",
            motion_data_ready=True,
            controller_pose_openxr=[0.1, 1.2, -0.3, 1.0, 0.0, 0.0, 0.0],
            enable_button="squeeze",
            enable_value=0.2,
            enabled=False,
            gripper_button="trigger",
            gripper_value=0.7,
            gripper_closed=True,
            now=13.0,
            serial_number="Rizon4-qSaFLh",
        )

        self.assertEqual(packet["schema"], "rizon4_quest_input.v1")
        self.assertEqual(packet["serial"], "Rizon4-qSaFLh")
        self.assertEqual(len(packet["controller_pose_openxr"]), 7)
        self.assertFalse(packet["enabled"])
        self.assertTrue(packet["gripper_closed"])
        self.assertEqual(packet["axis_map"], mod.DEFAULT_AXIS_MAP)
        self.assertEqual(packet["position_delta_scale"], 1.0)
        self.assertEqual(len(packet["tcp_rot_offset_wxyz"]), 4)

    def test_select_enable_accepts_analog_squeeze_threshold(self):
        class FakeTeleVuer:
            right_ctrl_squeeze = False
            right_ctrl_squeezeValue = 0.7

        self.assertTrue(mod.select_enable(FakeTeleVuer(), "right", "squeeze", threshold=0.5))

    def test_select_enable_rejects_small_analog_squeeze_value(self):
        class FakeTeleVuer:
            right_ctrl_squeeze = False
            right_ctrl_squeezeValue = 0.2

        self.assertFalse(mod.select_enable(FakeTeleVuer(), "right", "squeeze", threshold=0.5))

    def test_gripper_defaults_to_trigger(self):
        args = mod.parse_args([])

        self.assertEqual(args.gripper_button, "trigger")
        self.assertEqual(args.gripper_threshold, 0.5)


if __name__ == "__main__":
    unittest.main()
