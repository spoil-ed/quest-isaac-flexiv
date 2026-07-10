import importlib.util
import math
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "rizon4_quest_target_publisher.py"
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


class Rizon4QuestTargetPublisherTests(unittest.TestCase):
    def test_default_televuer_root_is_inside_repo(self):
        repo_root = Path(__file__).resolve().parents[1]

        self.assertTrue(mod.DEFAULT_TELEVUER_ROOT.is_relative_to(repo_root))
        self.assertTrue(mod.DEFAULT_CERT_FILE.is_relative_to(repo_root))
        self.assertTrue(mod.DEFAULT_KEY_FILE.is_relative_to(repo_root))

    def test_mapper_outputs_relative_origin_on_first_enable(self):
        mapper = mod.QuestRelativeMapper(position_delta_scale=3.0)

        packet = mapper.update(_pose(0.2, -0.1, 0.4), enabled=True, seq=1, now=10.0)

        self.assertIsNotNone(packet)
        self.assertEqual(packet["pose_base_tcp_des"][:3], [0.0, 0.0, 0.0])
        self.assertEqual(packet["reason"], "tracking")

    def test_mapper_scales_relative_delta_while_enabled(self):
        mapper = mod.QuestRelativeMapper(position_delta_scale=3.0)
        mapper.update(_pose(0.2, -0.1, 0.4), enabled=True, seq=1, now=10.0)

        packet = mapper.update(_pose(0.21, -0.12, 0.45), enabled=True, seq=2, now=10.1)

        self.assertEqual(
            [round(value, 4) for value in packet["pose_base_tcp_des"][:3]],
            [0.03, -0.06, 0.15],
        )

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
            now=12.0,
            reason="tracking",
        )

        self.assertEqual(packet["schema"], "rizon4_quest_target.v1")
        self.assertEqual(packet["serial"], "Rizon4-I0LIRN")
        self.assertEqual(packet["joint_group"], "ARM_1")
        self.assertEqual(len(packet["pose_base_tcp_des"]), 7)
        self.assertTrue(all(math.isfinite(value) for value in packet["pose_base_tcp_des"]))


if __name__ == "__main__":
    unittest.main()
