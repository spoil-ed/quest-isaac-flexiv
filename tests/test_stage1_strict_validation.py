import unittest

from flexiv_data_collection import schema
from flexiv_data_collection.real_validation import (
    EXPECTED_STAGE1_BACKEND,
    EXPECTED_STAGE1_SERIAL,
    Stage1SampleMonitor,
    summarize_stage1_single_arm_frames,
    validate_stage1_single_arm_sample,
)


def _sample(*, cycle=1, q_offset=0.0, torque=0.1, colors=None):
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
            }
        },
    }


class Stage1StrictValidationTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
