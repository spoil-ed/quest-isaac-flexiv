import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "fake_rizon4_quest_sender.py"


def load_module():
    spec = importlib.util.spec_from_file_location("fake_rizon4_quest_sender", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Stage3FakeTrajectoryTests(unittest.TestCase):
    def test_builtin_stage3_profiles_move_both_arms(self):
        sender = load_module()

        for profile in (
            "pick_place_redblock_dual",
            "pick_redblock_into_drawer_dual",
            "stack_rgyblock_dual",
            "move_cylinder_dual",
        ):
            with self.subTest(profile=profile):
                left, right = sender.trajectory_deltas_for_frame(
                    frame=15,
                    frames=60,
                    profile=profile,
                    amplitude_m=0.075,
                    cycles=1.0,
                )
                self.assertEqual(len(left), 3)
                self.assertEqual(len(right), 3)
                self.assertGreater(sum(abs(value) for value in left), 0.01)
                self.assertGreater(sum(abs(value) for value in right), 0.01)

    def test_dual_dry_run_can_emit_task_profile_packets(self):
        sender = load_module()
        args = sender.parse_args(
            [
                "--dual",
                "--trajectory-profile",
                "pick_place_redblock_dual",
                "--amplitude-m",
                "0.075",
                "--cycles",
                "1",
                "--dry-run",
            ]
        )

        self.assertEqual(args.trajectory_profile, "pick_place_redblock_dual")
        left_delta, right_delta = sender.trajectory_deltas_for_frame(
            frame=20,
            frames=120,
            profile=args.trajectory_profile,
            amplitude_m=args.amplitude_m,
            cycles=args.cycles,
        )
        packets = sender.build_fake_dual_quest_packets(
            seq=20,
            left_serial_number="Rizon4-L",
            right_serial_number="Rizon4-R",
            joint_group="ARM_1",
            left_delta_base=left_delta,
            right_delta_base=right_delta,
            quat_wxyz=[1.0, 0.0, 0.0, 0.0],
            now=1.0,
        )

        self.assertEqual([packet["side"] for packet in packets], ["left", "right"])
        self.assertNotEqual(packets[0]["controller_delta_base"], [0.0, 0.0, 0.0])
        self.assertNotEqual(packets[1]["controller_delta_base"], [0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
