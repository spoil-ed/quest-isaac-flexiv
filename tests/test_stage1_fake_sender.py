import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "fake_rizon4_quest_sender.py"


def load_module():
    spec = importlib.util.spec_from_file_location("fake_rizon4_quest_sender", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Stage1FakeSenderTests(unittest.TestCase):
    def test_builds_old_quest_target_schema(self):
        sender = load_module()
        packet = sender.build_fake_quest_packet(
            seq=7,
            side="right",
            serial_number="Rizon4-VIHhZM",
            joint_group="ARM_1",
            controller_delta_base=[0.001, 0.0, 0.0],
            quat_wxyz=[1.0, 0.0, 0.0, 0.0],
            now=123.0,
        )

        self.assertEqual(packet["schema"], "rizon4_quest_target.v1")
        self.assertEqual(packet["serial"], "Rizon4-VIHhZM")
        self.assertEqual(packet["joint_group"], "ARM_1")
        self.assertEqual(packet["pose_base_tcp_des"], [0.001, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(packet["controller_delta_base"], [0.001, 0.0, 0.0])

    def test_delta_sine_starts_and_ends_at_zero(self):
        sender = load_module()

        self.assertEqual(sender.delta_for_frame(0, 5, "x", 0.01), [0.0, 0.0, 0.0])
        end = sender.delta_for_frame(4, 5, "x", 0.01)
        self.assertAlmostEqual(end[0], 0.0, places=8)


if __name__ == "__main__":
    unittest.main()
