import importlib.util
import json
import math
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/print_dual_arm_state.py"
SPEC = importlib.util.spec_from_file_location("print_dual_arm_state", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def state_packet() -> dict:
    arm = {
        "serial": "Rizon4-test",
        "q": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "dq": [0.0] * 7,
        "tcp_pose_base": [0.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
        "tcp_pose_world": [1.4, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0],
        "ready": True,
        "quest": {
            "available": True,
            "seq": 17,
            "age_sec": 0.01,
            "motion_data_ready": True,
            "enable_button": "squeeze",
            "enable_value": 0.8,
            "enabled": True,
            "gripper_button": "trigger",
            "gripper_value": 0.6,
            "gripper_closed": True,
            "controller_pose_openxr": [0.1, 1.2, -0.3, 1.0, 0.0, 0.0, 0.0],
            "controller_delta_base": [0.01, 0.02, 0.03],
            "target_packet_pose_base_tcp": [0.01, 0.02, 0.03, 1.0, 0.0, 0.0, 0.0],
            "mapped_goal_pose_base_tcp": [0.4, 0.1, 0.5, 1.0, 0.0, 0.0, 0.0],
        },
    }
    return {
        "schema": MODULE.SCHEMA,
        "servo_cycle": 42,
        "stamp_ns": 1_000_000_000,
        "arms": {"left": dict(arm), "right": dict(arm)},
    }


class DualArmStatePrinterTests(unittest.TestCase):
    def test_parse_and_format_state_packet(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))
        output = MODULE.format_state(packet, received_time=1.01)

        self.assertIn("cycle=42", output)
        self.assertIn("LEFT", output)
        self.assertIn("RIGHT", output)
        self.assertIn("q rad", output)
        self.assertIn("q deg", output)
        self.assertIn("TCP base", output)
        self.assertIn("TCP world", output)
        self.assertIn("Quest", output)
        self.assertIn("squeeze=0.800", output)
        self.assertIn("trigger=0.600", output)
        self.assertIn("OpenXR", output)
        self.assertIn("mapped dxyz", output)

    def test_rejects_wrong_joint_count(self):
        packet = state_packet()
        packet["arms"]["left"]["q"] = [0.0] * 6

        with self.assertRaisesRegex(ValueError, "left.q must contain 7 values"):
            MODULE.parse_state_packet(json.dumps(packet).encode("utf-8"))

    def test_quaternion_yaw_is_reported_in_degrees(self):
        half_angle = math.pi / 4.0
        rpy = MODULE.quaternion_wxyz_to_rpy_deg(
            [math.cos(half_angle), 0.0, 0.0, math.sin(half_angle)]
        )

        self.assertAlmostEqual(rpy[0], 0.0, places=6)
        self.assertAlmostEqual(rpy[1], 0.0, places=6)
        self.assertAlmostEqual(rpy[2], 90.0, places=6)


if __name__ == "__main__":
    unittest.main()
