import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "rdk_target_streamer.py"


def load_streamer():
    spec = importlib.util.spec_from_file_location("rdk_target_streamer", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RdkTargetStreamerTests(unittest.TestCase):
    def test_status_packets_include_live_rdk_tcp_pose(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn('packet["current_pose_base_tcp"]', source)
        self.assertIn("current_pose = controller.current_tcp_pose()", source)
        self.assertIn("publish_ready(True, current_pose)", source)

    def test_parse_target_pose_packet_accepts_matching_fresh_packet(self):
        streamer = load_streamer()

        pose = streamer.parse_target_pose_packet(
            {
                "schema": "flexiv_target_pose.v1",
                "serial": "Rizon4-I0LIRN",
                "joint_group": "ARM_1",
                "monotonic_time": 10.0,
                "pose_base_tcp_des": [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0],
            },
            serial_number="Rizon4-I0LIRN",
            joint_group="ARM_1",
            now=10.2,
            max_age_sec=0.5,
        )

        self.assertEqual(pose, [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])

    def test_parse_target_pose_packet_rejects_wrong_serial_and_stale_packets(self):
        streamer = load_streamer()
        packet = {
            "schema": "flexiv_target_pose.v1",
            "serial": "Rizon4-I0LIRN",
            "joint_group": "ARM_1",
            "monotonic_time": 10.0,
            "pose_base_tcp_des": [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0],
        }

        self.assertIsNone(
            streamer.parse_target_pose_packet(
                {**packet, "serial": "Rizon4-WRONG"},
                serial_number="Rizon4-I0LIRN",
                joint_group="ARM_1",
                now=10.2,
                max_age_sec=0.5,
            )
        )
        self.assertIsNone(
            streamer.parse_target_pose_packet(
                packet,
                serial_number="Rizon4-I0LIRN",
                joint_group="ARM_1",
                now=11.0,
                max_age_sec=0.5,
            )
        )


if __name__ == "__main__":
    unittest.main()
