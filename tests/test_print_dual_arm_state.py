import importlib.util
import copy
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
    # +90 degrees about base Y maps TCP local +Z to base +X.
    forward_x_quat = [math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0]
    arm = {
        "serial": "Rizon4-test",
        "q": [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
        "dq": [0.0] * 7,
        "tcp_pose_base": [0.4, 0.0, 0.5, *forward_x_quat],
        "tcp_pose_world": [1.4, 0.0, 0.5, *forward_x_quat],
        "ready": True,
        "quest": {
            "available": True,
            "seq": 17,
            "age_sec": 0.01,
            "motion_data_ready": True,
            "enable_button": "squeeze",
            "enable_value": 0.0,
            "enabled": False,
            "calibration_confirmed": False,
            "both_squeeze": False,
            "calibration_rotation_base_from_mapped": None,
            "gripper_button": "trigger",
            "gripper_value": 0.6,
            "gripper_closed": True,
            "control_active": False,
            "axis_map": "-z,-x,y",
            "publisher_position_scale": 1.0,
            "isaac_position_scale": 1.0,
            "publisher_position_deadband_m": 0.0,
            "isaac_position_deadband_m": 0.0,
            "engage_settle_sec": 0.25,
            "position_mode": "relative",
            "orientation_mode": "relative",
            "workspace_clipping": False,
            "tcp_rot_offset_wxyz": [0.0, 0.70710678, 0.0, 0.70710678],
            "controller_pose_openxr": [-0.2, 1.2, -0.3, 1.0, 0.0, 0.0, 0.0],
            "controller_delta_base": [0.01, 0.02, 0.03],
            "target_packet_pose_base_tcp": [0.01, 0.02, 0.03, *forward_x_quat],
            "mapped_goal_pose_base_tcp": [0.4, 0.0, 0.5, *forward_x_quat],
            "command_pose_base_tcp": [0.4, 0.0, 0.5, *forward_x_quat],
            "relative_reference": {
                "controller_orientation_base": forward_x_quat,
                "tcp_pose_base": [0.39, -0.02, 0.47, *forward_x_quat],
            },
        },
    }
    left = copy.deepcopy(arm)
    right = copy.deepcopy(arm)
    right["quest"]["controller_pose_openxr"][0] = 0.2
    return {
        "schema": MODULE.SCHEMA,
        "servo_cycle": 42,
        "stamp_ns": 1_000_000_000,
        "arms": {"left": left, "right": right},
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
        self.assertIn("squeeze=0.000", output)
        self.assertIn("trigger=0.600", output)
        self.assertIn("OpenXR", output)
        self.assertIn("mapped dxyz", output)
        self.assertIn("limited command", output)
        self.assertIn("SPACING PASS", output)
        self.assertIn("DIRECTION PASS", output)
        self.assertIn("Mapping:", output)
        self.assertIn("[-dOpenXR.z, -dOpenXR.x, +dOpenXR.y]", output)
        self.assertIn("q_goal=(q_packet * inverse(q_packet@engage))", output)
        self.assertIn("hold the last mapped goal", output)

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

    def test_axis_map_formula_explains_output_axes(self):
        self.assertEqual(
            MODULE.axis_map_formula("-z,-x,y"),
            "[-dOpenXR.z, -dOpenXR.x, +dOpenXR.y]",
        )

    def test_hand_gate_rejects_wrong_controller_forward_direction(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))
        packet["arms"]["left"]["quest"]["controller_pose_openxr"][3:7] = [0.0, 0.0, 1.0, 0.0]

        output = MODULE.format_concise_state(packet)

        self.assertIn("DIRECTION FAIL", output)
        self.assertIn("left_perp_error=0.0deg", output)
        self.assertIn("mutual_error=180.0deg", output)

    def test_tcp_forward_quaternion_maps_local_plus_z_to_base_plus_x(self):
        forward = MODULE.rotate_vector_wxyz(
            [math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0],
            MODULE.TCP_FORWARD_LOCAL,
        )

        self.assertEqual([round(value, 6) for value in forward], [1.0, 0.0, 0.0])

    def test_spacing_uses_euclidean_distance_instead_of_per_axis_error(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))
        packet["arms"]["right"]["quest"]["controller_pose_openxr"][2] = -0.25

        output = MODULE.format_concise_state(packet)

        self.assertIn("SPACING PASS", output)
        self.assertIn("distance=0.403m", output)

    def test_concise_gate_is_exactly_two_lines_and_does_not_require_squeeze(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))

        output = MODULE.format_concise_state(packet)

        self.assertEqual(len(output.splitlines()), 2)
        self.assertIn("SPACING PASS", output)
        self.assertIn("distance=0.400m", output)
        self.assertIn("frame=HOLD_BOTH_SQUEEZE", output)
        self.assertIn("DIRECTION PASS", output)
        self.assertIn("left_perp_error=0.0deg", output)
        self.assertIn("mutual_error=0.0deg", output)
        self.assertFalse(packet["arms"]["left"]["quest"]["enabled"])
        self.assertNotIn("q rad", output)

    def test_dual_squeeze_reports_confirming_then_locked_frame(self):
        raw = state_packet()
        for arm in raw["arms"].values():
            arm["quest"]["both_squeeze"] = True
        packet = MODULE.parse_state_packet(json.dumps(raw).encode("utf-8"))
        self.assertIn("frame=CONFIRMING", MODULE.format_concise_state(packet))

        for arm in packet["arms"].values():
            arm["quest"]["calibration_confirmed"] = True
            arm["quest"]["calibration_rotation_base_from_mapped"] = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        self.assertIn("frame=LOCKED", MODULE.format_concise_state(packet))

    def test_spacing_gate_rejects_wrong_euclidean_separation(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))
        packet["arms"]["right"]["quest"]["controller_pose_openxr"][0] = 0.15

        output = MODULE.format_concise_state(packet)

        self.assertIn("SPACING FAIL", output)
        self.assertIn("distance=0.350m", output)

    def test_direction_target_is_dynamically_perpendicular_to_diagonal_line(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))
        # Mapped delta is [+0.24, -0.32, 0], whose Euclidean length is 0.40 m.
        packet["arms"]["right"]["quest"]["controller_pose_openxr"][:3] = [0.12, 1.2, -0.54]
        # A +36.87 deg raw-Y rotation maps the controller forward to
        # [+0.8, +0.6, 0], perpendicular to that dynamic line in base XY.
        half_angle = math.radians(36.86989765) / 2.0
        quaternion = [math.cos(half_angle), 0.0, math.sin(half_angle), 0.0]
        packet["arms"]["left"]["quest"]["controller_pose_openxr"][3:7] = quaternion
        packet["arms"]["right"]["quest"]["controller_pose_openxr"][3:7] = quaternion

        output = MODULE.format_concise_state(packet)

        self.assertIn("SPACING PASS", output)
        self.assertIn("delta_base_xyz=[ 0.240, -0.320,  0.000]m", output)
        self.assertIn("DIRECTION PASS", output)
        self.assertIn("left_perp_error=0.0deg", output)

    def test_gate_waits_for_tracking_but_not_control_activation(self):
        packet = MODULE.parse_state_packet(json.dumps(state_packet()).encode("utf-8"))
        packet["arms"]["left"]["quest"]["motion_data_ready"] = False

        output = MODULE.format_concise_state(packet)

        self.assertEqual(
            output.splitlines(),
            [
                "SPACING WAIT | both tracked controller poses are required",
                "DIRECTION WAIT | both tracked controller poses are required",
            ],
        )


if __name__ == "__main__":
    unittest.main()
