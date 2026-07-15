import importlib.util
import math
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
UTILS = ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators" / "elements_studio_utils.py"


def load_utils():
    spec = importlib.util.spec_from_file_location("elements_studio_utils", UTILS)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ElementsStudioUtilsTests(unittest.TestCase):
    def test_rdk_streamer_reads_current_tcp_pose_for_coordinate_reference(self):
        utils = load_utils()

        class FakeStates:
            tcp_pose = [0.3, -0.1, 0.5, 1.0, 0.0, 0.0, 0.0]

        class FakeRobot:
            def states(self):
                return FakeStates()

        class FakeRdk:
            pass

        streamer = utils.RdkCartesianStreamer(FakeRdk, FakeRobot())

        self.assertEqual(streamer.current_tcp_pose(), FakeStates.tcp_pose)

    def test_target_drive_limits_check_vector_norm_and_per_joint_magnitude(self):
        utils = load_utils()

        accepted, norm = utils.valid_target_drives_or_none([3.0, 4.0], max_norm=6.0, max_abs=5.0)
        rejected_abs, _ = utils.valid_target_drives_or_none([5.1, 0.0], max_norm=6.0, max_abs=5.0)
        rejected_norm, _ = utils.valid_target_drives_or_none([3.0, 4.0], max_norm=4.9, max_abs=5.0)

        self.assertEqual(accepted, [3.0, 4.0])
        self.assertAlmostEqual(norm, 5.0)
        self.assertIsNone(rejected_abs)
        self.assertIsNone(rejected_norm)

    def test_joint_speed_limit_checks_each_joint(self):
        utils = load_utils()

        self.assertFalse(utils.joint_speed_limit_exceeded([0.2, -1.1], max_abs_rad_s=1.2))
        self.assertTrue(utils.joint_speed_limit_exceeded([0.2, -1.21], max_abs_rad_s=1.2))
        self.assertFalse(utils.joint_speed_limit_exceeded([100.0], max_abs_rad_s=0.0))

    def test_zero_joint_velocities_uses_full_articulation_dof_count(self):
        utils = load_utils()

        class FakeRobot:
            num_dof = 13

            def __init__(self):
                self.zeros = None

            def get_joint_velocities(self):
                return [1.0] * 7

            def set_joint_velocities(self, values):
                self.zeros = list(values)

        robot = FakeRobot()
        count = utils.zero_articulation_joint_velocities(robot)

        self.assertEqual(count, 13)
        self.assertEqual(robot.zeros, [0.0] * 13)

    def test_zero_joint_velocities_falls_back_to_runtime_velocity_shape(self):
        utils = load_utils()

        class FakeRobot:
            def __init__(self):
                self.zeros = None

            def get_joint_velocities(self):
                return [1.0, -2.0, 3.0]

            def set_joint_velocities(self, values):
                self.zeros = list(values)

        robot = FakeRobot()
        count = utils.zero_articulation_joint_velocities(robot)

        self.assertEqual(count, 3)
        self.assertEqual(robot.zeros, [0.0, 0.0, 0.0])

    def test_hold_robot_joint_positions_synchronizes_drive_target_and_pose(self):
        utils = load_utils()

        class FakeRobot:
            q = [9.0, 9.0]
            num_dof = 3

            def __init__(self):
                self.calls = []

            def switch_control_mode(self, mode):
                self.calls.append(("mode", mode))

            def teleport_to(self, values):
                self.calls.append(("pose", list(values)))

            def set_joint_position_targets(self, values, *, joint_indices):
                self.calls.append(("targets", list(values), list(joint_indices)))

            def set_joint_velocities(self, values):
                self.calls.append(("velocities", list(values)))

        robot = FakeRobot()
        count = utils.hold_robot_joint_positions(robot, [0.1, -0.2])

        self.assertEqual(count, 3)
        self.assertEqual(
            robot.calls,
            [
                ("mode", "position"),
                ("pose", [0.1, -0.2]),
                ("targets", [0.1, -0.2], [0, 1]),
                ("velocities", [0.0, 0.0, 0.0]),
            ],
        )

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

    def test_rdk_cartesian_streamer_rejects_faulted_or_non_operational_robot(self):
        utils = load_utils()

        class FakeRdk:
            ARM_1 = "arm_one"

        class FakeRobot:
            def __init__(self):
                self.is_faulted = True
                self.is_operational = True

            def fault(self):
                return self.is_faulted

            def operational(self):
                return self.is_operational

            def SendCartesianMotionForce(self, *_args):
                raise AssertionError("a bad robot must not receive a command")

        robot = FakeRobot()
        streamer = utils.RdkCartesianStreamer(FakeRdk, robot)
        command = utils.RdkCartesianCommand.from_pose([0.4, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0])

        with self.assertRaisesRegex(RuntimeError, "fault"):
            streamer.send("ARM_1", command)
        robot.is_faulted = False
        robot.is_operational = False
        with self.assertRaisesRegex(RuntimeError, "operational"):
            streamer.send("ARM_1", command)

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

    def test_connect_rdk_cartesian_streamer_waits_for_operational_before_switching_mode(self):
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
                self.operational_checks = 0

            def fault(self):
                return False

            def operational(self):
                self.operational_checks += 1
                return self.enabled and self.operational_checks >= 3

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

        with mock.patch.object(utils.time, "sleep") as sleep:
            streamer = utils.connect_rdk_cartesian_streamer(
                "Rizon4-I0LIRN",
                flexivrdk=FakeRdk,
            )

        robot = FakeRdk.created[0]
        self.assertIsInstance(streamer, utils.RdkCartesianStreamer)
        self.assertTrue(robot.enabled)
        sleep.assert_called_once_with(0.1)
        self.assertEqual(robot.switched_to, "nrt_cartesian")

    def test_connect_rdk_cartesian_streamer_does_not_switch_mode_after_enable_timeout(self):
        utils = load_utils()

        class FakeRobot:
            def __init__(self, *_args):
                self.switched_to = None

            def fault(self):
                return False

            def operational(self):
                return False

            def Enable(self):
                pass

            def mode(self):
                return "idle"

            def SwitchMode(self, mode):
                self.switched_to = mode

        class FakeRdk:
            NRT_CARTESIAN_MOTION_FORCE = "nrt_cartesian"
            robot = FakeRobot()

            @classmethod
            def Robot(cls, *_args):
                return cls.robot

        with self.assertRaisesRegex(TimeoutError, "did not become operational"):
            utils.connect_rdk_cartesian_streamer(
                "Rizon4-I0LIRN",
                flexivrdk=FakeRdk,
                enable_timeout_sec=0.0,
            )

        self.assertIsNone(FakeRdk.robot.switched_to)

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
