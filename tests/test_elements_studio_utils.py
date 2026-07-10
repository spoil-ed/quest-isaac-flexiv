import importlib.util
import math
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UTILS = ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators" / "elements_studio_utils.py"


def load_utils():
    spec = importlib.util.spec_from_file_location("elements_studio_utils", UTILS)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ElementsStudioUtilsTests(unittest.TestCase):
    def test_rdk_pose_from_xyzw_reorders_quaternion_to_wxyz(self):
        utils = load_utils()

        pose = utils.rdk_pose_from_position_quat_xyzw((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 2.0))

        self.assertEqual(pose, [0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0])

    def test_rdk_pose_to_position_quat_xyzw_reorders_quaternion_to_xyzw(self):
        utils = load_utils()

        position, quat_xyzw = utils.rdk_pose_to_position_quat_xyzw([0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0])

        self.assertEqual(position, (0.1, 0.2, 0.3))
        self.assertEqual(quat_xyzw, (0.0, 0.0, 0.0, 1.0))

    def test_studio_target_pose_from_rdk_pose_uses_euler_degrees(self):
        utils = load_utils()
        yaw_90 = math.sqrt(0.5)

        target = utils.studio_target_pose_from_rdk_pose([0.4, -0.1, 0.6, yaw_90, 0.0, 0.0, yaw_90])

        self.assertEqual(target[:3], ["0.4", "-0.1", "0.6"])
        self.assertEqual(float(target[3]), 0.0)
        self.assertEqual(float(target[4]), 0.0)
        self.assertAlmostEqual(float(target[5]), 90.0)

    def test_joint_positions_rad_to_studio_seed_outputs_degrees_as_strings(self):
        utils = load_utils()

        seed = utils.joint_positions_rad_to_studio_seed([0.0, math.pi / 2.0, -math.pi / 2.0, 0, 0, 0, 0])

        self.assertEqual(seed, ["0", "90", "-90", "0", "0", "0", "0"])

    def test_cal_reachability_request_encodes_expected_fields(self):
        utils = load_utils()

        request = utils.encode_cal_reachability_request(
            target_pose=["0.4", "0", "0.3", "0", "0", "0"],
            seed_jnt_pos=["0", "1", "2", "3", "4", "5", "6"],
        )

        self.assertEqual(request.count(b"\x0a"), 6)
        self.assertEqual(request.count(b"\x12"), 7)
        self.assertTrue(request.endswith(b"\x38\x01"))

    def test_cal_reachability_response_parses_solved_joint_positions(self):
        utils = load_utils()
        response = b"".join(utils.encode_length_delimited(2, str(i).encode("utf-8")) for i in range(7))

        solved = utils.parse_cal_reachability_response(response)

        self.assertEqual(solved, [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_rdk_cartesian_command_defaults_motion_terms_to_zero(self):
        utils = load_utils()

        command = utils.RdkCartesianCommand.from_pose([0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])

        self.assertEqual(command.pose_d, [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(command.twist_d, [0.0] * 6)
        self.assertEqual(command.wrench_d, [0.0] * 6)
        self.assertEqual(command.acc_d, [0.0] * 6)

    def test_rdk_cartesian_streamer_sends_latest_pose_to_joint_group(self):
        utils = load_utils()

        class FakeCmd:
            pass

        class FakeRdk:
            ARM_1 = "arm_one"
            RT_CARTESIAN_MOTION_FORCE = "rt_cartesian"

            @staticmethod
            def RtCartesianCmd():
                return FakeCmd()

        class FakeRobot:
            def __init__(self):
                self.calls = []

            def StreamCartesianMotionForce(self, commands):
                self.calls.append(commands)

        robot = FakeRobot()
        streamer = utils.RdkCartesianStreamer(FakeRdk, robot)

        streamer.send(
            "ARM_1",
            utils.RdkCartesianCommand.from_pose([0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0]),
        )

        self.assertEqual(len(robot.calls), 1)
        command = robot.calls[0]["arm_one"]
        self.assertEqual(command.pose_d, [0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(command.twist_d, [0.0] * 6)
        self.assertEqual(command.wrench_d, [0.0] * 6)
        self.assertEqual(command.acc_d, [0.0] * 6)

    def test_rdk_cartesian_streamer_uses_rdk_1_9_cartesian_motion_force_api(self):
        utils = load_utils()

        class FakeRdk19:
            ARM_1 = "arm_one"

        class FakeRobot:
            def __init__(self):
                self.calls = []

            def SendCartesianMotionForce(
                self,
                pose_d,
                wrench_d,
                twist_d,
                max_linear_vel,
                max_angular_vel,
                max_linear_acc,
                max_angular_acc,
            ):
                self.calls.append(
                    (
                        pose_d,
                        wrench_d,
                        twist_d,
                        max_linear_vel,
                        max_angular_vel,
                        max_linear_acc,
                        max_angular_acc,
                    )
                )

        robot = FakeRobot()
        streamer = utils.RdkCartesianStreamer(FakeRdk19, robot)

        streamer.send(
            "ARM_1",
            utils.RdkCartesianCommand.from_pose([0.4, -0.1, 0.6, 1.0, 0.0, 0.0, 0.0]),
        )

        self.assertEqual(
            robot.calls,
            [
                (
                    [0.4, -0.1, 0.6, 1.0, 0.0, 0.0, 0.0],
                    [0.0] * 6,
                    [0.0] * 6,
                    1.5,
                    3.0,
                    5.0,
                    10.0,
                )
            ],
        )

    def test_connect_rdk_cartesian_streamer_owns_robot_setup_and_mode_switch(self):
        utils = load_utils()

        class FakeRobot:
            def __init__(self, serial_number, verbose):
                self.serial_number = serial_number
                self.verbose = verbose
                self.switched_to = None

            def fault(self):
                return False

            def operational(self):
                return True

            def mode(self):
                return "idle"

            def SwitchMode(self, mode):
                self.switched_to = mode

            def StreamCartesianMotionForce(self, _commands):
                pass

        class FakeRdk:
            ARM_1 = "arm_one"
            RT_CARTESIAN_MOTION_FORCE = "rt_cartesian"
            created = []

            @classmethod
            def Robot(cls, serial_number, verbose):
                robot = FakeRobot(serial_number, verbose)
                cls.created.append(robot)
                return robot

            @staticmethod
            def RtCartesianCmd():
                return object()

        streamer = utils.connect_rdk_cartesian_streamer(
            "Rizon4-I0LIRN",
            flexivrdk=FakeRdk,
            verbose=True,
        )

        self.assertIsInstance(streamer, utils.RdkCartesianStreamer)
        self.assertEqual(FakeRdk.created[0].serial_number, "Rizon4-I0LIRN")
        self.assertTrue(FakeRdk.created[0].verbose)
        self.assertEqual(FakeRdk.created[0].switched_to, "rt_cartesian")

    def test_connect_rdk_cartesian_streamer_uses_whitelist_and_enable_without_changing_stream_api(self):
        utils = load_utils()

        class FakeMode:
            RT_CARTESIAN_MOTION_FORCE = "rt_cartesian"

        class FakeRobot:
            def __init__(self, serial_number, whitelist, verbose):
                self.serial_number = serial_number
                self.whitelist = whitelist
                self.verbose = verbose
                self.enabled = False
                self.switched_to = None
                self.sent = []

            def fault(self):
                return False

            def operational(self):
                return self.enabled

            def Enable(self):
                self.enabled = True

            def mode(self):
                return "idle"

            def SwitchMode(self, mode):
                self.switched_to = mode

            def StreamCartesianMotionForce(self, commands):
                self.sent.append(commands)

        class FakeCmd:
            pass

        class FakeRdk:
            Mode = FakeMode
            ARM_1 = "arm_one"
            created = []

            @classmethod
            def Robot(cls, serial_number, whitelist, verbose):
                robot = FakeRobot(serial_number, whitelist, verbose)
                cls.created.append(robot)
                return robot

            @staticmethod
            def RtCartesianCmd():
                return FakeCmd()

        streamer = utils.connect_rdk_cartesian_streamer(
            "Rizon4-I0LIRN",
            flexivrdk=FakeRdk,
            network_interface_whitelist=["127.0.0.1"],
            verbose=True,
        )
        streamer.send(
            "ARM_1",
            utils.RdkCartesianCommand.from_pose([0.4, -0.1, 0.6, 1.0, 0.0, 0.0, 0.0]),
        )

        robot = FakeRdk.created[0]
        self.assertEqual(robot.serial_number, "Rizon4-I0LIRN")
        self.assertEqual(robot.whitelist, ["127.0.0.1"])
        self.assertTrue(robot.verbose)
        self.assertTrue(robot.enabled)
        self.assertEqual(robot.switched_to, "rt_cartesian")
        command = robot.sent[0]["arm_one"]
        self.assertEqual(command.pose_d, [0.4, -0.1, 0.6, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(command.twist_d, [0.0] * 6)
        self.assertEqual(command.wrench_d, [0.0] * 6)

    def test_connect_rdk_cartesian_streamer_switches_mode_even_if_operational_state_lags_after_enable(self):
        utils = load_utils()

        class FakeMode:
            NRT_CARTESIAN_MOTION_FORCE = "nrt_cartesian"

        class FakeRobot:
            def __init__(self, serial_number, whitelist, verbose):
                self.serial_number = serial_number
                self.whitelist = whitelist
                self.verbose = verbose
                self.enabled = False
                self.switched_to = None

            def fault(self):
                return False

            def operational(self):
                return False

            def Enable(self):
                self.enabled = True

            def mode(self):
                return "idle"

            def SwitchMode(self, mode):
                self.switched_to = mode

            def SendCartesianMotionForce(self, *_args):
                pass

        class FakeRdk:
            Mode = FakeMode
            created = []

            @classmethod
            def Robot(cls, serial_number, whitelist, verbose):
                robot = FakeRobot(serial_number, whitelist, verbose)
                cls.created.append(robot)
                return robot

        streamer = utils.connect_rdk_cartesian_streamer(
            "Rizon4-I0LIRN",
            flexivrdk=FakeRdk,
        )

        robot = FakeRdk.created[0]
        self.assertIsInstance(streamer, utils.RdkCartesianStreamer)
        self.assertTrue(robot.enabled)
        self.assertEqual(robot.switched_to, "nrt_cartesian")

    def test_rdk_runtime_controller_wraps_pose_as_runtime_stream_command(self):
        utils = load_utils()

        class FakeMode:
            RT_CARTESIAN_MOTION_FORCE = "rt_cartesian"

        class FakeRobot:
            def __init__(self, serial_number, whitelist, verbose):
                self.serial_number = serial_number
                self.whitelist = whitelist
                self.verbose = verbose
                self.enabled = False
                self.switched_to = None
                self.sent = []

            def fault(self):
                return False

            def operational(self):
                return self.enabled

            def Enable(self):
                self.enabled = True

            def mode(self):
                return "idle"

            def SwitchMode(self, mode):
                self.switched_to = mode

            def StreamCartesianMotionForce(self, commands):
                self.sent.append(commands)

        class FakeCmd:
            pass

        class FakeRdk:
            Mode = FakeMode
            ARM_1 = "arm_one"
            created = []

            @classmethod
            def Robot(cls, serial_number, whitelist, verbose):
                robot = FakeRobot(serial_number, whitelist, verbose)
                cls.created.append(robot)
                return robot

            @staticmethod
            def RtCartesianCmd():
                return FakeCmd()

        settings = utils.RdkRuntimeSettings(
            serial_number="Rizon4-I0LIRN",
            joint_group="ARM_1",
            network_interface_whitelist="192.168.32.10",
            verbose=True,
        )
        controller = utils.RdkRuntimeController(settings, flexivrdk=FakeRdk)

        controller.send_pose([0.45, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0])

        robot = FakeRdk.created[0]
        self.assertEqual(robot.serial_number, "Rizon4-I0LIRN")
        self.assertEqual(robot.whitelist, ["192.168.32.10"])
        self.assertTrue(robot.enabled)
        self.assertEqual(robot.switched_to, "rt_cartesian")
        command = robot.sent[0]["arm_one"]
        self.assertEqual(command.pose_d, [0.45, 0.0, 0.35, 1.0, 0.0, 0.0, 0.0])
        self.assertEqual(command.twist_d, [0.0] * 6)
        self.assertEqual(command.wrench_d, [0.0] * 6)
        self.assertEqual(command.acc_d, [0.0] * 6)


if __name__ == "__main__":
    unittest.main()
