import importlib.util
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UTILS_DIR = ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators"
CONTROL_HELPERS = UTILS_DIR / "control_helpers.py"
ELEMENTS_UTILS = UTILS_DIR / "elements_studio_utils.py"


def load_control_helpers():
    elements_spec = importlib.util.spec_from_file_location("elements_studio_utils", ELEMENTS_UTILS)
    elements = importlib.util.module_from_spec(elements_spec)
    sys.modules[elements_spec.name] = elements
    elements_spec.loader.exec_module(elements)

    spec = importlib.util.spec_from_file_location("control_helpers", CONTROL_HELPERS)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ControlHelpersTests(unittest.TestCase):
    def test_format_pose_xyz_quat_includes_full_7d_pose(self):
        helpers = load_control_helpers()

        text = helpers.format_pose_xyz_quat([0.12345, -0.2, 0.35, 0.707106, 0.0, -0.707106, 0.0])

        self.assertEqual(
            text,
            "pose_xyz=[0.1235, -0.2000, 0.3500] pose_quat=[0.7071, 0.0000, -0.7071, 0.0000]",
        )

    def test_simplugin_target_drives_wait_for_runtime_target(self):
        helpers = load_control_helpers()

        self.assertFalse(
            helpers.should_poll_simplugin_target_drives(
                connected=True,
                runtime_target_active=False,
            )
        )
        self.assertTrue(
            helpers.should_poll_simplugin_target_drives(
                connected=True,
                runtime_target_active=True,
            )
        )

    def test_target_pose_control_pauses_without_fresh_quest_target(self):
        helpers = load_control_helpers()

        self.assertTrue(
            helpers.target_pose_control_is_active(
                quest_target_receiver_enabled=False,
                latest_quest_target=None,
            )
        )
        self.assertFalse(
            helpers.target_pose_control_is_active(
                quest_target_receiver_enabled=True,
                latest_quest_target=None,
            )
        )
        self.assertTrue(
            helpers.target_pose_control_is_active(
                quest_target_receiver_enabled=True,
                latest_quest_target=object(),
            )
        )

    def test_cartesian_pose_changed_uses_translation_and_quaternion_angle(self):
        helpers = load_control_helpers()
        reference = [0.4, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0]

        self.assertFalse(
            helpers.cartesian_pose_changed(
                reference,
                [0.40005, -0.1, 0.3, -1.0, 0.0, 0.0, 0.0],
                position_tolerance_m=1e-4,
            )
        )
        self.assertTrue(
            helpers.cartesian_pose_changed(
                reference,
                [0.4002, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0],
                position_tolerance_m=1e-4,
            )
        )
        half_angle = math.radians(0.2) / 2.0
        self.assertTrue(
            helpers.cartesian_pose_changed(
                reference,
                [0.4, -0.1, 0.3, math.cos(half_angle), math.sin(half_angle), 0.0, 0.0],
                orientation_tolerance_rad=math.radians(0.1),
            )
        )

    def test_rdk_streamer_status_requires_matching_fresh_positive_packet(self):
        helpers = load_control_helpers()
        packet = {
            "schema": "flexiv_rdk_streamer_status.v1",
            "serial": "Rizon4-test01",
            "ready": True,
            "monotonic_time": 10.0,
        }

        self.assertTrue(
            helpers.rdk_streamer_status_is_ready(
                packet,
                serial_number="Rizon4-test01",
                max_age_sec=0.5,
                now=10.4,
            )
        )
        self.assertFalse(
            helpers.rdk_streamer_status_is_ready(
                packet,
                serial_number="Rizon4-test01",
                max_age_sec=0.5,
                now=10.6,
            )
        )
        self.assertFalse(
            helpers.rdk_streamer_status_is_ready(
                {**packet, "ready": False},
                serial_number="Rizon4-test01",
                max_age_sec=0.5,
                now=10.1,
            )
        )

if __name__ == "__main__":
    unittest.main()
