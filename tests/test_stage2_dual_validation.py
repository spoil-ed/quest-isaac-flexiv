import unittest
from pathlib import Path

from flexiv_data_collection import schema
from flexiv_data_collection.dual_validation import (
    DEFAULT_STAGE2_LEFT_SERIAL,
    DEFAULT_STAGE2_RIGHT_SERIAL,
    EXPECTED_STAGE2_BACKEND,
    Stage2SampleMonitor,
    summarize_stage2_dual_arm_frames,
    validate_stage2_dual_arm_sample,
)
from flexiv_data_collection.validators import parse_args as parse_validator_args


def _sample(
    *,
    cycle=1,
    left_offset=0.0,
    right_offset=0.0,
    left_torque=0.1,
    right_torque=0.1,
    left_target_x=0.3,
    right_target_x=0.3,
):
    parts = schema.unitree_parts_from_dual_arms(
        [left_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [right_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        left_qvel=[0.0] * 7,
        right_qvel=[0.0] * 7,
        left_torque=[left_torque] * 7,
        right_torque=[right_torque] * 7,
    )
    return {
        "states": parts,
        "actions": parts,
        "colors": {"color_0": "colors/000000_color_0.jpg"},
        "sim_state": {
            "bridge": {
                "backend": EXPECTED_STAGE2_BACKEND,
                "serials": {
                    "left": DEFAULT_STAGE2_LEFT_SERIAL,
                    "right": DEFAULT_STAGE2_RIGHT_SERIAL,
                },
                "servo_cycle": cycle,
                "servo_cycles": {"left": cycle, "right": cycle},
                "target_frames": {
                    "left": {
                        "base_tcp_pose": [left_target_x, 0.35, 0.4, 1.0, 0.0, 0.0, 0.0],
                        "world_position": [left_target_x, 0.35, 0.4],
                    },
                    "right": {
                        "base_tcp_pose": [right_target_x, -0.35, 0.4, 1.0, 0.0, 0.0, 0.0],
                        "world_position": [right_target_x, -0.35, 0.4],
                    },
                },
            }
        },
    }


class Stage2DualValidationTests(unittest.TestCase):
    def test_dual_schema_roundtrip_populates_both_arms(self):
        parts = schema.unitree_parts_from_dual_arms([1, 2, 3, 4, 5, 6, 7], [8, 9, 10, 11, 12, 13, 14])

        self.assertEqual(parts["left_arm"]["qpos"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        self.assertEqual(parts["right_arm"]["qpos"], [8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0])
        self.assertEqual(len(schema.unitree_parts_to_vector(parts)), schema.FLEXIV_VECTOR_DIM)

    def test_strict_dual_requires_serials_and_backend(self):
        validate_stage2_dual_arm_sample(_sample())
        bad = _sample()
        bad["sim_state"]["bridge"]["serials"]["right"] = "Rizon4-WRONG"

        with self.assertRaisesRegex(ValueError, "right serial"):
            validate_stage2_dual_arm_sample(bad)

    def test_summary_requires_motion_and_torque_on_both_arms(self):
        frames = [_sample(cycle=1), _sample(cycle=7, left_offset=0.01, right_offset=0.002)]

        with self.assertRaisesRegex(ValueError, "right_q_delta_norm"):
            summarize_stage2_dual_arm_frames(
                frames,
                min_left_q_delta=0.005,
                min_right_q_delta=0.005,
                min_left_torque_norm=1e-8,
                min_right_torque_norm=1e-8,
                min_servo_cycle_delta=5,
            )

        result = summarize_stage2_dual_arm_frames(
            [_sample(cycle=1), _sample(cycle=7, left_offset=0.01, right_offset=0.01)],
            min_left_q_delta=0.005,
            min_right_q_delta=0.005,
            min_left_torque_norm=1e-8,
            min_right_torque_norm=1e-8,
            min_servo_cycle_delta=5,
        )
        self.assertTrue(result["strict_stage2_dual_arm"])

    def test_summary_requires_target_frame_motion_on_both_sides(self):
        frames = [
            _sample(cycle=1),
            _sample(cycle=7, left_offset=0.01, right_offset=0.01, left_target_x=0.315, right_target_x=0.304),
        ]

        with self.assertRaisesRegex(ValueError, "right_target_frame_delta_norm"):
            summarize_stage2_dual_arm_frames(
                frames,
                min_left_q_delta=0.005,
                min_right_q_delta=0.005,
                min_left_torque_norm=1e-8,
                min_right_torque_norm=1e-8,
                min_left_target_frame_delta=0.01,
                min_right_target_frame_delta=0.01,
                min_servo_cycle_delta=5,
            )

        result = summarize_stage2_dual_arm_frames(
            [
                _sample(cycle=1),
                _sample(cycle=7, left_offset=0.01, right_offset=0.01, left_target_x=0.315, right_target_x=0.315),
            ],
            min_left_q_delta=0.005,
            min_right_q_delta=0.005,
            min_left_torque_norm=1e-8,
            min_right_torque_norm=1e-8,
            min_left_target_frame_delta=0.01,
            min_right_target_frame_delta=0.01,
            min_servo_cycle_delta=5,
        )
        self.assertGreaterEqual(result["left_target_frame_delta_norm"], 0.01)
        self.assertGreaterEqual(result["right_target_frame_delta_norm"], 0.01)

    def test_monitor_requires_fresh_dual_motion(self):
        monitor = Stage2SampleMonitor(min_servo_cycle_delta=2, min_left_q_delta=0.005, min_right_q_delta=0.005)

        self.assertFalse(monitor.observe(_sample(cycle=1))["ready"])
        self.assertTrue(monitor.observe(_sample(cycle=4, left_offset=0.01, right_offset=0.01))["ready"])

    def test_validator_cli_requires_dual_serials(self):
        base = ["--raw-dir", str(Path("raw")), "--dataset-root", str(Path("dataset")), "--strict-dual-arm"]

        with self.assertRaises(SystemExit):
            parse_validator_args(base)
        args = parse_validator_args(
            [
                *base,
                "--expected-left-serial",
                "Rizon4-L",
                "--expected-right-serial",
                "Rizon4-R",
                "--required-camera-names",
                "cam_front",
            ]
        )
        self.assertTrue(args.strict_dual_arm)
        self.assertEqual(args.required_camera_keys_tuple, ("color_0",))


if __name__ == "__main__":
    unittest.main()
