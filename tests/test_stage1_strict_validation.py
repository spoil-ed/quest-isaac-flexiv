import unittest
from pathlib import Path

from flexiv_data_collection import schema
from flexiv_data_collection.real_validation import (
    EXPECTED_STAGE1_BACKEND,
    EXPECTED_STAGE1_SERIAL,
    Stage1SampleMonitor,
    summarize_stage1_single_arm_frames,
    validate_stage1_single_arm_sample,
)
from flexiv_data_collection.validators import parse_args as parse_validator_args
from flexiv_data_collection.validators import validate_unitree_json


def _sample(*, cycle=1, q_offset=0.0, torque=0.1, target_x=0.3, colors=None):
    parts = schema.unitree_parts_from_single_arm(
        [q_offset, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        qvel=[0.0] * 7,
        torque=[torque] * 7,
        arm="left",
    )
    return {
        "states": parts,
        "actions": parts,
        "colors": {"color_0": "colors/000000_color_0.jpg"} if colors is None else colors,
        "sim_state": {
            "bridge": {
                "backend": EXPECTED_STAGE1_BACKEND,
                "serial": EXPECTED_STAGE1_SERIAL,
                "servo_cycle": cycle,
                "target_frame": {
                    "base_tcp_pose": [target_x, 0.0, 0.4, 1.0, 0.0, 0.0, 0.0],
                    "world_position": [target_x, 0.0, 0.4],
                },
            }
        },
    }


class Stage1StrictValidationTests(unittest.TestCase):
    def test_validator_rejects_missing_or_empty_serial_in_strict_mode(self):
        base = ["--raw-dir", str(Path("raw")), "--dataset-root", str(Path("dataset")), "--strict-single-arm"]

        with self.assertRaises(SystemExit):
            parse_validator_args(base)
        with self.assertRaises(SystemExit):
            parse_validator_args([*base, "--expected-serial", ""])

    def test_strict_validation_error_includes_episode_path(self):
        import json
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            data_json = Path(tmp) / "episode_0000" / "data.json"
            data_json.parent.mkdir()
            data_json.write_text(json.dumps({"data": [_sample()]}), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, r"episode_0000/data\.json: left_q_delta_norm"):
                validate_unitree_json(
                    Path(tmp),
                    strict_single_arm=True,
                    expected_serial=EXPECTED_STAGE1_SERIAL,
                    min_left_q_delta=0.005,
                )

    def test_strict_single_arm_rejects_nonzero_right_arm(self):
        sample = _sample()
        sample["states"]["right_arm"]["qpos"][0] = 0.1

        with self.assertRaisesRegex(ValueError, "right_arm.qpos"):
            validate_stage1_single_arm_sample(sample)

    def test_strict_single_arm_rejects_extra_cameras(self):
        sample = _sample(colors={"color_0": "a.jpg", "color_1": "b.jpg"})

        with self.assertRaisesRegex(ValueError, "only allows cameras"):
            validate_stage1_single_arm_sample(sample)

    def test_monitor_requires_fresh_cycle_motion_and_torque(self):
        monitor = Stage1SampleMonitor(min_servo_cycle_delta=2, min_left_q_delta=0.01, min_left_torque_norm=1e-8)

        self.assertFalse(monitor.observe(_sample(cycle=1, q_offset=0.0, torque=0.1))["ready"])
        self.assertFalse(monitor.observe(_sample(cycle=1, q_offset=0.0, torque=0.1))["ready"])
        self.assertTrue(monitor.observe(_sample(cycle=4, q_offset=0.02, torque=0.1))["ready"])

    def test_episode_summary_enforces_motion_threshold(self):
        frames = [_sample(cycle=1, q_offset=0.0), _sample(cycle=5, q_offset=0.004)]

        with self.assertRaisesRegex(ValueError, "left_q_delta_norm"):
            summarize_stage1_single_arm_frames(frames, min_left_q_delta=0.005, min_left_torque_norm=1e-8)

        result = summarize_stage1_single_arm_frames(frames, min_left_q_delta=0.003, min_left_torque_norm=1e-8)
        self.assertTrue(result["right_arm_zero"])

    def test_episode_summary_enforces_target_frame_motion(self):
        frames = [_sample(cycle=1, q_offset=0.0, target_x=0.3), _sample(cycle=5, q_offset=0.01, target_x=0.304)]

        with self.assertRaisesRegex(ValueError, "target_frame_delta_norm"):
            summarize_stage1_single_arm_frames(
                frames,
                min_left_q_delta=0.005,
                min_left_torque_norm=1e-8,
                min_target_frame_delta=0.01,
            )

        result = summarize_stage1_single_arm_frames(
            [_sample(cycle=1, q_offset=0.0, target_x=0.3), _sample(cycle=5, q_offset=0.01, target_x=0.315)],
            min_left_q_delta=0.005,
            min_left_torque_norm=1e-8,
            min_target_frame_delta=0.01,
        )
        self.assertGreaterEqual(result["target_frame_delta_norm"], 0.01)


if __name__ == "__main__":
    unittest.main()
