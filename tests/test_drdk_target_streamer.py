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

    def test_initializes_nullspace_from_configured_initial_q(self):
        class State:
            def __init__(self, q, tcp_pose, dq=None):
                self.q = q
                self.dq = [0.0] * 7 if dq is None else dq
                self.tcp_pose = tcp_pose

        class Pair:
            def __init__(self):
                self.switches = []
                self.postures = []
                self.objectives = []
                self.joint_commands = []
                self.current_q = ([0.1] * 7, [0.2] * 7)
                self.current_mode = ("IDLE", "IDLE")

            def fault(self):
                return False

            def operational(self):
                return True

            def connected(self):
                return True

            def mode(self):
                return self.current_mode

            def SwitchMode(self, mode):
                self.switches.append(mode)
                self.current_mode = (mode, mode)

            def SendJointPosition(self, positions, velocities, max_vel, max_acc):
                self.joint_commands.append((positions, velocities, max_vel, max_acc))
                self.current_q = (list(positions[0]), list(positions[1]))

            def states(self):
                return (
                    State(self.current_q[0], [0.3, 0.1, -0.6, 1.0, 0.0, 0.0, 0.0]),
                    State(self.current_q[1], [0.3, -0.1, -0.6, 1.0, 0.0, 0.0, 0.0]),
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
            NRT_JOINT_POSITION = "NRT_JOINT_POSITION"
            NRT_CARTESIAN_MOTION_FORCE = "NRT_CARTESIAN_MOTION_FORCE"

        class Rdk:
            pass

        Rdk.Mode = Mode

        args = mod.parse_args(
            [
                "--left-serial-number",
                "Rizon4-L",
                "--right-serial-number",
                "Rizon4-R",
                "--left-nullspace-posture",
                "0,1,2,3,4,5,6",
                "--right-nullspace-posture",
                "6,5,4,3,2,1,0",
                "--initial-joint-settle-sec",
                "0.001",
                "--initial-joint-handoff-sec",
                "0",
            ]
        )

        initialized, postures = mod.initialize_robot_pair(args, flexivdrdk=Drdk, flexivrdk=Rdk)

        self.assertIs(initialized, pair)
        expected = ([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0])
        self.assertEqual(postures, expected)
        self.assertEqual(pair.switches, ["NRT_JOINT_POSITION", "NRT_CARTESIAN_MOTION_FORCE"])
        self.assertEqual(pair.joint_commands[0][0], ([0.1] * 7, [0.2] * 7))
        self.assertEqual(pair.joint_commands[1][0], expected)
        self.assertEqual(pair.postures, [expected])
        self.assertEqual(pair.objectives[0]["ref_positions_tracking"], (0.5, 0.5))

    def test_nullspace_postures_are_required(self):
        with self.assertRaises(SystemExit):
            mod.parse_args([])

    def test_parses_nullspace_postures(self):
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

    def test_status_packet_exposes_initialization_phase_and_joint_state(self):
        packet = mod._status_packet(
            serial="Rizon4-L",
            ready=True,
            phase="joint_initializing",
            reference_pose=None,
            current_pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            current_q=[0.1] * 7,
        )

        self.assertEqual(packet["phase"], "joint_initializing")
        self.assertEqual(packet["current_q"], [0.1] * 7)


if __name__ == "__main__":
    unittest.main()
