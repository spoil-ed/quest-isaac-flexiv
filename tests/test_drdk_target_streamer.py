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
    def test_contact_guard_freezes_one_arm_and_rebases_after_release(self):
        class State:
            def __init__(self, pose, wrench):
                self.tcp_pose = list(pose)
                self.tcp_wrench = list(wrench)

        identity = [1.0, 0.0, 0.0, 0.0]
        left_pose = [0.5, 0.1, 0.2, *identity]
        right_pose = [0.5, -0.1, 0.2, *identity]
        high = [9.5, 0.0, 0.0, 0.0, 0.0, 0.0]
        low = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        guard = mod.ContactWrenchGuard(
            ([10.0] * 6, [10.0] * 6),
            trigger_ratio=0.9,
            release_ratio=0.5,
            trigger_samples=2,
            release_dwell_sec=0.3,
        )
        latest = {
            "left": [0.8, 0.1, 0.2, *identity],
            "right": [0.8, -0.1, 0.2, *identity],
        }

        self.assertEqual(
            guard.update((State(left_pose, high), State(right_pose, low)), latest, now=0.0),
            [],
        )
        self.assertEqual(
            guard.update((State(left_pose, high), State(right_pose, low)), latest, now=0.1),
            [("left", "frozen")],
        )
        self.assertEqual(guard.command_pose("left", latest["left"]), left_pose)
        self.assertEqual(guard.command_pose("right", latest["right"]), latest["right"])

        latest["left"] = [1.0, 0.1, 0.2, *identity]
        guard.update((State(left_pose, low), State(right_pose, low)), latest, now=0.2)
        self.assertEqual(
            guard.update((State(left_pose, low), State(right_pose, low)), latest, now=0.5),
            [("left", "released")],
        )
        self.assertEqual(guard.command_pose("left", latest["left"]), left_pose)
        moved = guard.command_pose("left", [1.1, 0.1, 0.2, *identity])
        self.assertAlmostEqual(moved[0], 0.6)
        self.assertEqual(moved[1:], left_pose[1:])

    def test_joint_torque_guard_rolls_back_and_rebases_after_release(self):
        class State:
            def __init__(self, pose, tau, tau_dot=None, tau_ext=None):
                self.tcp_pose = list(pose)
                self.tau = list(tau)
                self.tau_dot = [0.0] * 7 if tau_dot is None else list(tau_dot)
                self.tau_ext = [0.0] * 7 if tau_ext is None else list(tau_ext)

        identity = [1.0, 0.0, 0.0, 0.0]
        left_tcp = [0.7, 0.1, 0.2, *identity]
        right_tcp = [0.7, -0.1, 0.2, *identity]
        guard = mod.JointTorqueGuard(
            ([100.0] * 7, [100.0] * 7),
            trigger_ratio=0.85,
            release_ratio=0.70,
            trigger_samples=1,
            release_dwell_sec=0.3,
            prediction_horizon_sec=0.02,
            rollback_sec=0.1,
        )
        initial = {"left": left_tcp, "right": right_tcp}
        guard.reset(initial, now=0.0)
        safe_left = [0.75, 0.1, 0.2, *identity]
        guard.record_command("left", safe_left, now=0.1)
        guard.record_command("left", [0.85, 0.1, 0.2, *identity], now=0.25)
        latest = {
            "left": [0.9, 0.1, 0.2, *identity],
            "right": [0.9, -0.1, 0.2, *identity],
        }

        events = guard.update(
            (State(left_tcp, [86.0] + [0.0] * 6), State(right_tcp, [0.0] * 7)),
            latest,
            now=0.3,
        )

        self.assertEqual(events, [("left", "frozen")])
        rebased = guard.command_pose("left", latest["left"])
        self.assertAlmostEqual(rebased[0], safe_left[0])
        self.assertEqual(rebased[1:], safe_left[1:])
        self.assertEqual(guard.command_pose("right", latest["right"]), latest["right"])

        low_states = (State(left_tcp, [60.0] + [0.0] * 6), State(right_tcp, [0.0] * 7))
        guard.update(low_states, latest, now=0.4)
        self.assertEqual(guard.update(low_states, latest, now=0.71), [("left", "released")])
        rebased = guard.command_pose("left", latest["left"])
        self.assertAlmostEqual(rebased[0], safe_left[0])
        self.assertEqual(rebased[1:], safe_left[1:])
        moved = guard.command_pose("left", [1.0, 0.1, 0.2, *identity])
        self.assertAlmostEqual(moved[0], 0.85)

    def test_joint_torque_guard_uses_tau_dot_prediction_and_tau_ext(self):
        class State:
            tcp_pose = [0.7, 0.0, 0.2, 1.0, 0.0, 0.0, 0.0]

            def __init__(self, tau, tau_dot, tau_ext):
                self.tau = tau
                self.tau_dot = tau_dot
                self.tau_ext = tau_ext

        guard = mod.JointTorqueGuard(
            ([100.0] * 7, [100.0] * 7),
            trigger_ratio=0.85,
            release_ratio=0.70,
            trigger_samples=1,
            release_dwell_sec=0.3,
            prediction_horizon_sec=0.02,
            rollback_sec=0.1,
        )
        latest = {"left": list(State.tcp_pose), "right": list(State.tcp_pose)}
        events = guard.update(
            (
                State([80.0] + [0.0] * 6, [300.0] + [0.0] * 6, [0.0] * 7),
                State([0.0] * 7, [0.0] * 7, [90.0] + [0.0] * 6),
            ),
            latest,
            now=1.0,
        )

        self.assertEqual(events, [("left", "frozen"), ("right", "frozen")])
        self.assertAlmostEqual(guard.latest_ratios["left"][0], 0.86)
        self.assertAlmostEqual(guard.latest_ratios["right"][0], 0.90)

    def test_reads_joint_torque_limits_from_robot_pair_info(self):
        class JointGroup:
            ARM_1 = "arm-one"

        class Rdk:
            pass

        Rdk.JointGroup = JointGroup

        class Info:
            def __init__(self, limit):
                self.tau_max = {JointGroup.ARM_1: [limit] * 7}

        class Pair:
            @staticmethod
            def info():
                return Info(64.0), Info(72.0)

        self.assertEqual(
            mod.joint_torque_limits(Pair(), Rdk, "ARM_1"),
            ([64.0] * 7, [72.0] * 7),
        )

    def test_reads_joint_torque_limits_from_rdk_1_9_list_layout(self):
        class Rdk:
            pass

        class Info:
            def __init__(self, limit):
                self.tau_max = [limit] * 7

        class Pair:
            @staticmethod
            def info():
                return Info(64.0), Info(72.0)

        self.assertEqual(
            mod.joint_torque_limits(Pair(), Rdk, "ARM_1"),
            ([64.0] * 7, [72.0] * 7),
        )

    def test_relative_pose_rebase_preserves_orientation_delta(self):
        half = 2.0**-0.5
        rebased = mod.rebase_relative_pose(
            [0.0, 0.0, 0.0, half, 0.0, 0.0, half],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        )

        self.assertAlmostEqual(rebased[0], 0.5)
        self.assertAlmostEqual(abs(rebased[3]), half)
        self.assertAlmostEqual(abs(rebased[6]), half)

    def test_starts_official_self_collision_monitor_with_configured_safety_margin(self):
        events = []

        class Monitor:
            def __init__(self, robot_pair, skipped_links):
                events.append(("init", robot_pair, skipped_links))

            def SetMinDistance(self, distance):
                events.append(("distance", distance))

            def Start(self, interval_ms):
                events.append(("start", interval_ms))

            def Stop(self):
                events.append(("stop",))

        class Drdk:
            SelfCollisionMonitor = Monitor

        pair = object()
        args = mod.parse_args(
            [
                "--left-nullspace-posture", "0,0,0,0,0,0,0",
                "--right-nullspace-posture", "0,0,0,0,0,0,0",
                "--self-collision-monitor",
                "--self-collision-min-distance-m", "0.08",
                "--self-collision-loop-interval-ms", "20",
                "--self-collision-skip-link", "left_base",
                "--self-collision-skip-link", "right_base",
            ]
        )

        monitor = mod.start_self_collision_monitor(args, robot_pair=pair, flexivdrdk=Drdk)
        mod.stop_self_collision_monitor(monitor)

        self.assertEqual(
            events,
            [
                ("init", pair, ["left_base", "right_base"]),
                ("distance", 0.08),
                ("start", 20),
                ("stop",),
            ],
        )

    def test_self_collision_monitor_is_disabled_in_low_level_streamer_by_default(self):
        args = mod.parse_args(
            [
                "--left-nullspace-posture", "0,0,0,0,0,0,0",
                "--right-nullspace-posture", "0,0,0,0,0,0,0",
            ]
        )

        self.assertIsNone(
            mod.start_self_collision_monitor(args, robot_pair=object(), flexivdrdk=object())
        )

    def test_parses_matching_reset_sequence_from_target_packet(self):
        packet = {
            "schema": "flexiv_target_pose.v1",
            "serial": "Rizon4-L",
            "joint_group": "ARM_1",
            "reset_seq": 7,
        }

        self.assertEqual(
            mod.parse_reset_request_seq(
                packet,
                serial_number="Rizon4-L",
                joint_group="ARM_1",
            ),
            7,
        )
        self.assertIsNone(
            mod.parse_reset_request_seq(
                {**packet, "serial": "Rizon4-R"},
                serial_number="Rizon4-L",
                joint_group="ARM_1",
            )
        )

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

            def SetMaxContactWrench(self, wrenches):
                self.max_contact_wrenches = wrenches

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
                "--left-startup-waypoint",
                "1,1,1,1,1,1,1",
                "--right-startup-waypoint",
                "2,2,2,2,2,2,2",
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
        self.assertEqual(pair.joint_commands[1][0], ([1.0] * 7, [2.0] * 7))
        self.assertEqual(pair.joint_commands[2][0], expected)
        self.assertEqual(pair.postures, [expected])
        self.assertEqual(pair.objectives[0]["ref_positions_tracking"], (0.5, 0.5))
        self.assertEqual(pair.max_contact_wrenches, ([20.0] * 3 + [3.0] * 3,) * 2)

    def test_reset_stops_clears_fault_and_recovers_with_send_joint_position(self):
        class State:
            def __init__(self, q):
                self.q = list(q)
                self.dq = [0.0] * 7
                self.tcp_pose = [0.3, 0.0, -0.6, 1.0, 0.0, 0.0, 0.0]

        class Pair:
            def __init__(self):
                self.events = []
                self.current_q = ([0.1] * 7, [0.2] * 7)
                self.current_mode = ("IDLE", "IDLE")
                self.has_fault = True
                self.is_operational = False

            def Stop(self):
                self.events.append("stop")

            def fault(self):
                return self.has_fault

            def ClearFault(self):
                self.events.append("clear_fault")
                self.has_fault = False
                return True, True

            def operational(self):
                return self.is_operational

            def connected(self):
                return True

            def Enable(self):
                self.events.append("enable")
                self.is_operational = True

            def mode(self):
                return self.current_mode

            def SwitchMode(self, mode):
                self.events.append(f"mode:{mode}")
                self.current_mode = (mode, mode)

            def states(self):
                return State(self.current_q[0]), State(self.current_q[1])

            def SendJointPosition(self, positions, _velocities, _max_vel, _max_acc):
                self.events.append(f"send:{positions}")
                self.current_q = tuple(list(position) for position in positions)

            def SetNullSpacePosture(self, _postures):
                self.events.append("set_nullspace")

            def SetNullSpaceObjectives(self, **_objectives):
                self.events.append("set_nullspace_objectives")

            def SetMaxContactWrench(self, _wrenches):
                self.events.append("set_max_contact_wrench")

        class Mode:
            NRT_JOINT_POSITION = "NRT_JOINT_POSITION"
            NRT_CARTESIAN_MOTION_FORCE = "NRT_CARTESIAN_MOTION_FORCE"

        class Rdk:
            pass

        Rdk.Mode = Mode
        pair = Pair()
        phases = []
        args = mod.parse_args(
            [
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

        postures = mod.recover_connected_robot_pair_to_initial_q(
            args,
            robot_pair=pair,
            flexivrdk=Rdk,
            phase_callback=phases.append,
        )

        self.assertEqual(phases, ["reset_stopping", "reset_clearing_fault"])
        self.assertEqual(
            postures,
            ([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0], [6.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.0]),
        )
        self.assertLess(pair.events.index("stop"), pair.events.index("clear_fault"))
        self.assertLess(pair.events.index("clear_fault"), pair.events.index("enable"))
        self.assertLess(pair.events.index("enable"), pair.events.index("mode:NRT_JOINT_POSITION"))
        self.assertTrue(pair.events[4].startswith("send:"))
        self.assertTrue(pair.events[5].startswith("send:"))
        self.assertLess(
            pair.events.index("mode:NRT_JOINT_POSITION"),
            pair.events.index("mode:NRT_CARTESIAN_MOTION_FORCE"),
        )
        self.assertLess(pair.events.index("mode:NRT_CARTESIAN_MOTION_FORCE"), pair.events.index("set_nullspace"))

    def test_reset_enables_even_when_stop_does_not_clear_operational_flag(self):
        class State:
            q = [0.0] * 7
            dq = [0.0] * 7
            tcp_pose = [0.3, 0.0, -0.6, 1.0, 0.0, 0.0, 0.0]

        class Pair:
            def __init__(self):
                self.enable_count = 0
                self.current_mode = ("CARTESIAN", "CARTESIAN")

            def Stop(self):
                self.current_mode = ("IDLE", "IDLE")

            def fault(self):
                return False

            def operational(self):
                return True

            def connected(self):
                return True

            def Enable(self):
                self.enable_count += 1

            def mode(self):
                return self.current_mode

            def SwitchMode(self, mode):
                self.current_mode = (mode, mode)

            def states(self):
                return State(), State()

            def SendJointPosition(self, *_args):
                return None

            def SetNullSpacePosture(self, *_args):
                return None

            def SetNullSpaceObjectives(self, **_kwargs):
                return None

            def SetMaxContactWrench(self, _wrenches):
                return None

        class Mode:
            NRT_JOINT_POSITION = "NRT_JOINT_POSITION"
            NRT_CARTESIAN_MOTION_FORCE = "NRT_CARTESIAN_MOTION_FORCE"

        class Rdk:
            pass

        Rdk.Mode = Mode
        pair = Pair()
        args = mod.parse_args(
            [
                "--left-nullspace-posture",
                "0,0,0,0,0,0,0",
                "--right-nullspace-posture",
                "0,0,0,0,0,0,0",
                "--initial-joint-settle-sec",
                "0.001",
                "--initial-joint-handoff-sec",
                "0",
            ]
        )

        mod.recover_connected_robot_pair_to_initial_q(args, robot_pair=pair, flexivrdk=Rdk)

        self.assertEqual(pair.enable_count, 1)

    def test_reset_retries_mid_motion_fault_from_current_q_at_recovery_speed(self):
        class State:
            def __init__(self, q):
                self.q = list(q)
                self.dq = [0.0] * 7
                self.tcp_pose = [0.3, 0.0, -0.6, 1.0, 0.0, 0.0, 0.0]

        class Pair:
            def __init__(self):
                self.current_q = ([0.1] * 7, [0.2] * 7)
                self.current_mode = ("IDLE", "IDLE")
                self.has_fault = True
                self.target_command_count = 0
                self.stop_count = 0
                self.clear_count = 0
                self.command_velocities = []

            def Stop(self):
                self.stop_count += 1
                self.current_mode = ("IDLE", "IDLE")

            def fault(self):
                return self.has_fault

            def ClearFault(self):
                self.clear_count += 1
                self.has_fault = False
                return True, True

            def operational(self):
                return not self.has_fault

            def connected(self):
                return True

            def Enable(self):
                return None

            def mode(self):
                return self.current_mode

            def SwitchMode(self, mode):
                self.current_mode = (mode, mode)

            def states(self):
                return State(self.current_q[0]), State(self.current_q[1])

            def SendJointPosition(self, positions, _velocities, max_vel, _max_acc):
                self.command_velocities.append(tuple(max_vel[0]))
                is_target = list(positions[0]) != list(self.current_q[0])
                if is_target:
                    self.target_command_count += 1
                    if self.target_command_count == 1:
                        self.has_fault = True
                        return
                self.current_q = tuple(list(position) for position in positions)

            def SetNullSpacePosture(self, _postures):
                return None

            def SetNullSpaceObjectives(self, **_objectives):
                return None

            def SetMaxContactWrench(self, _wrenches):
                return None

        class Mode:
            NRT_JOINT_POSITION = "NRT_JOINT_POSITION"
            NRT_CARTESIAN_MOTION_FORCE = "NRT_CARTESIAN_MOTION_FORCE"

        class Rdk:
            pass

        Rdk.Mode = Mode
        pair = Pair()
        args = mod.parse_args(
            [
                "--left-nullspace-posture",
                "0,1,2,3,4,5,6",
                "--right-nullspace-posture",
                "6,5,4,3,2,1,0",
                "--initial-joint-settle-sec",
                "0.001",
                "--initial-joint-handoff-sec",
                "0",
                "--reset-retry-delay-sec",
                "0",
            ]
        )

        mod.recover_connected_robot_pair_to_initial_q(args, robot_pair=pair, flexivrdk=Rdk)

        self.assertEqual(pair.stop_count, 2)
        self.assertEqual(pair.clear_count, 2)
        self.assertEqual(pair.target_command_count, 2)
        self.assertTrue(pair.command_velocities)
        self.assertTrue(all(max(values) == 0.2 for values in pair.command_velocities))

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

    def test_parses_paired_startup_waypoints(self):
        args = mod.parse_args(
            [
                "--left-nullspace-posture", "0,1,2,3,4,5,6",
                "--right-nullspace-posture", "6,5,4,3,2,1,0",
                "--left-startup-waypoint", "1,1,1,1,1,1,1",
                "--right-startup-waypoint", "2,2,2,2,2,2,2",
            ]
        )

        self.assertEqual(args.left_startup_waypoint, [[1.0] * 7])
        self.assertEqual(args.right_startup_waypoint, [[2.0] * 7])

    def test_status_packet_exposes_initialization_phase_and_joint_state(self):
        packet = mod._status_packet(
            serial="Rizon4-L",
            ready=True,
            phase="joint_initializing",
            reference_pose=None,
            current_pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            current_q=[0.1] * 7,
            tcp_wrench=[1.0, 2.0, 3.0, 0.1, 0.2, 0.3],
            contact_frozen=True,
            joint_tau=[10.0] * 7,
            joint_tau_dot=[1.0] * 7,
            joint_tau_ext=[2.0] * 7,
            joint_tau_max=[64.0] * 7,
            joint_torque_ratio=[0.5] * 7,
            joint_torque_frozen=True,
        )

        self.assertEqual(packet["phase"], "joint_initializing")
        self.assertEqual(packet["current_q"], [0.1] * 7)
        self.assertEqual(packet["reset_seq"], 0)
        self.assertTrue(packet["contact_frozen"])
        self.assertEqual(packet["tcp_wrench"], [1.0, 2.0, 3.0, 0.1, 0.2, 0.3])
        self.assertTrue(packet["joint_torque_frozen"])
        self.assertEqual(packet["joint_tau"], [10.0] * 7)
        self.assertEqual(packet["joint_tau_dot"], [1.0] * 7)
        self.assertEqual(packet["joint_tau_ext"], [2.0] * 7)
        self.assertEqual(packet["joint_tau_max"], [64.0] * 7)
        self.assertEqual(packet["joint_torque_ratio"], [0.5] * 7)


if __name__ == "__main__":
    unittest.main()
