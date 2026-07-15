import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
SCRIPT = SCRIPTS / "drdk_target_streamer.py"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("drdk_target_streamer", SCRIPT)
mod = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(mod)


class DrdkTargetStreamerTests(unittest.TestCase):
    def test_parses_synchronized_pair(self):
        packet = {
            "schema": "flexiv_target_pose.v1",
            "serial": "Rizon4-L",
            "joint_group": "ARM_1",
            "servo_cycle": 100,
            "monotonic_time": 10.0,
            "pose_base_tcp_des": [0.3, 0.1, -0.6, 1.0, 0.0, 0.0, 0.0],
            "control_active": False,
        }
        left = mod.parse_target_command_packet(
            packet,
            serial_number="Rizon4-L",
            joint_group="ARM_1",
            max_age_sec=0.5,
            now=10.1,
        )
        right = left._replace(control_active=True)

        buffers = {"left": {100: left}, "right": {100: right}}
        pair = mod.pop_synchronized_target_pair(buffers, after_cycle=99)

        self.assertIsNotNone(pair)
        self.assertEqual(pair[0].servo_cycle, 100)
        self.assertFalse(pair[0].control_active)

    def test_rejects_mismatched_servo_cycles(self):
        left = mod.TargetCommand(100, [0.0] * 7, False)
        right = mod.TargetCommand(101, [0.0] * 7, False)

        buffers = {"left": {100: left}, "right": {101: right}}
        self.assertIsNone(mod.pop_synchronized_target_pair(buffers, after_cycle=99))

    def test_pairs_delayed_matching_cycle_instead_of_latest_arrival(self):
        left_100 = mod.TargetCommand(100, [0.0] * 7, False)
        left_101 = mod.TargetCommand(101, [0.1] * 7, True)
        right_100 = mod.TargetCommand(100, [0.2] * 7, False)
        buffers = {"left": {}, "right": {}}
        mod.buffer_target_command(buffers["left"], left_100)
        mod.buffer_target_command(buffers["left"], left_101)
        mod.buffer_target_command(buffers["right"], right_100)

        pair = mod.pop_synchronized_target_pair(buffers, after_cycle=99)

        self.assertEqual((pair[0].servo_cycle, pair[1].servo_cycle), (100, 100))
        self.assertIn(101, buffers["left"])

    def test_initializes_nullspace_from_measured_joint_positions(self):
        class State:
            def __init__(self, q, tcp_pose):
                self.q = q
                self.tcp_pose = tcp_pose

        class Pair:
            def __init__(self):
                self.switches = []
                self.postures = []
                self.objectives = []

            def fault(self):
                return False

            def operational(self):
                return True

            def mode(self):
                return ("IDLE", "IDLE")

            def SwitchMode(self, mode):
                self.switches.append(mode)

            def states(self):
                return (
                    State([0.1] * 7, [0.3, 0.1, -0.6, 1.0, 0.0, 0.0, 0.0]),
                    State([0.2] * 7, [0.3, -0.1, -0.6, 1.0, 0.0, 0.0, 0.0]),
                )

            def SetNullSpacePosture(self, postures):
                self.postures.append(postures)

            def SetNullSpaceObjectives(self, **objectives):
                self.objectives.append(objectives)

        pair = Pair()

        class Drdk:
            @staticmethod
            def RobotPair(*_args):
                return pair

        class Mode:
            NRT_CARTESIAN_MOTION_FORCE = "NRT_CARTESIAN_MOTION_FORCE"

        class Rdk:
            pass

        Rdk.Mode = Mode

        args = mod.parse_args(["--left-serial-number", "Rizon4-L", "--right-serial-number", "Rizon4-R"])

        initialized, postures = mod.initialize_robot_pair(args, flexivdrdk=Drdk, flexivrdk=Rdk)

        self.assertIs(initialized, pair)
        self.assertEqual(postures, ([0.1] * 7, [0.2] * 7))
        self.assertEqual(pair.postures, [([0.1] * 7, [0.2] * 7)])
        self.assertEqual(pair.objectives[0]["ref_positions_tracking"], (0.5, 0.5))

    def test_explicit_nullspace_posture_overrides_measured_q(self):
        args = mod.parse_args(
            [
                "--left-nullspace-posture",
                "0,1,2,3,4,5,6",
                "--right-nullspace-posture",
                "6,5,4,3,2,1,0",
            ]
        )

        self.assertEqual(args.left_nullspace_posture, [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(args.right_nullspace_posture, [6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0])


if __name__ == "__main__":
    unittest.main()
