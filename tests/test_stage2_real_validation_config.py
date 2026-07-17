import importlib.util
import json
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run_stage2_dual_rizon4_real_validation.py"
DUAL_APP = ROOT / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/dual_follow_with_studio.py"


def load_validation_script():
    spec = importlib.util.spec_from_file_location("run_stage2_dual_rizon4_real_validation", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_dual_app():
    spec = importlib.util.spec_from_file_location("dual_follow_with_studio", DUAL_APP)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Stage2RealValidationConfigTests(unittest.TestCase):
    def test_dual_quest_receiver_keeps_fresh_read_only_input_state(self):
        dual_app = load_dual_app()
        receiver = dual_app.DualQuestTargetUdpReceiver(
            "127.0.0.1",
            0,
            serials={"left": "Rizon4-L", "right": "Rizon4-R"},
            joint_group="ARM_1",
            max_age_sec=1.0,
        )
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sender.sendto(
                json.dumps(
                    {
                        "schema": "rizon4_quest_input.v1",
                        "serial": "Rizon4-L",
                        "joint_group": "ARM_1",
                        "seq": 12,
                        "side": "left",
                        "motion_data_ready": True,
                        "controller_pose_openxr": [0.1, 1.2, -0.3, 1.0, 0.0, 0.0, 0.0],
                        "enable_button": "squeeze",
                        "enable_value": 0.7,
                        "enabled": True,
                        "gripper_button": "trigger",
                        "gripper_value": 0.2,
                        "gripper_closed": False,
                        "monotonic_time": time.monotonic(),
                    }
                ).encode("utf-8"),
                receiver._socket.getsockname(),
            )
            receiver.poll_latest()
            packet = receiver.latest_input("left")

            self.assertIsNotNone(packet)
            self.assertEqual(packet["seq"], 12)
            self.assertTrue(packet["enabled"])
            self.assertEqual(packet["controller_pose_openxr"][:3], [0.1, 1.2, -0.3])
        finally:
            sender.close()
            receiver.close()

    def test_json_config_supplies_dual_serials_paths_ports_and_outputs(self):
        module = load_validation_script()
        with tempfile.TemporaryDirectory(prefix="stage2_real_config_") as tmp:
            tmp_path = Path(tmp)
            env_config = tmp_path / "environment.json"
            scene_config = tmp_path / "scene.json"
            pipeline_config = tmp_path / "pipeline.json"
            env_config.write_text(
                json.dumps(
                    {
                        "rdk_python": str(tmp_path / "rdk/bin/python"),
                        "isaac_python": str(tmp_path / "isaac/python.sh"),
                        "isaacsim_root": str(tmp_path / "isaac"),
                        "record_output_root": str(tmp_path / "records"),
                        "lerobot_output_root": str(tmp_path / "datasets"),
                    }
                ),
                encoding="utf-8",
            )
            scene_config.write_text(
                json.dumps(
                    {
                        "robots": [
                            {
                                "side": "left",
                                "serial_number": "Rizon4-L",
                                "joint_group": "ARM_CUSTOM",
                                "prim_path": "/World/Left",
                                "usd": str(tmp_path / "rizon.usd"),
                                "examples_ext": str(tmp_path / "examples"),
                                "target_pose": {"host": "127.0.0.1", "port": 6102},
                            },
                            {
                                "side": "right",
                                "serial_number": "Rizon4-R",
                                "joint_group": "ARM_CUSTOM",
                                "prim_path": "/World/Right",
                                "usd": str(tmp_path / "rizon.usd"),
                                "examples_ext": str(tmp_path / "examples"),
                                "target_pose": {"host": "127.0.0.1", "port": 6103},
                            },
                        ],
                        "cameras": [{"name": "cam_front"}],
                    }
                ),
                encoding="utf-8",
            )
            pipeline_config.write_text(
                json.dumps(
                    {
                        "environment_config": "environment.json",
                        "scene_config": "scene.json",
                        "gateway": {
                            "sample_endpoint": "tcp://127.0.0.1:6100",
                            "bridge_endpoint": "tcp://127.0.0.1:6101",
                            "camera_keys": ["color_0"],
                        },
                        "quest_target": {"host": "127.0.0.1", "port": 6104},
                        "record": {"max_frames": 9, "fps": 7, "image_size": "320x240"},
                        "convert": {"repo_id": "local/stage2", "action_mode": "full"},
                        "validation": {"min_left_q_delta": 0.01, "min_right_q_delta": 0.02},
                        "fake_sender": {"axis": "y", "right_axis": "z", "frames": 11, "rate_hz": 44},
                    }
                ),
                encoding="utf-8",
            )

            args = module.parse_args(["--config", str(pipeline_config)])

            self.assertEqual(args.left_serial_number, "Rizon4-L")
            self.assertEqual(args.right_serial_number, "Rizon4-R")
            self.assertEqual(args.joint_group, "ARM_CUSTOM")
            self.assertEqual(args.left_robot_prim_path, "/World/Left")
            self.assertEqual(args.right_robot_prim_path, "/World/Right")
            self.assertEqual(args.sample_endpoint, "tcp://127.0.0.1:6100")
            self.assertEqual(args.bridge_endpoint, "tcp://127.0.0.1:6101")
            self.assertEqual(args.left_target_pose_udp_port, 6102)
            self.assertEqual(args.right_target_pose_udp_port, 6103)
            self.assertEqual(args.quest_target_udp_port, 6104)
            self.assertEqual(args.left_rdk_status_udp_port, 57682)
            self.assertEqual(args.right_rdk_status_udp_port, 57683)
            self.assertFalse(args.rdk_clear_fault)
            self.assertEqual(args.scene_camera_names, ["cam_front"])
            self.assertEqual(args.scene_camera_keys, ["color_0"])
            self.assertEqual(args.repo_id_prefix, "local/stage2")
            self.assertEqual(args.action_mode, "full")
            self.assertEqual(args.record_frames, 9)
            self.assertEqual(args.record_fps, 7.0)
            self.assertEqual(args.fake_axis, "y")
            self.assertEqual(args.fake_right_axis, "z")

    def test_prepare_rejects_identical_dual_serials(self):
        module = load_validation_script()
        with tempfile.TemporaryDirectory(prefix="stage2_same_serial_") as tmp:
            args = module.parse_args(
                [
                    "--config",
                    str(Path(tmp) / "missing.json"),
                    "--left-serial-number",
                    "Rizon4-SAME",
                    "--right-serial-number",
                    "Rizon4-SAME",
                ]
            )
            runner = module.RealValidationRunner(args)

            with self.assertRaisesRegex(RuntimeError, "serials must be different"):
                runner.prepare()

    def test_dual_isaac_app_scene_config_supplies_robot_and_camera_config(self):
        dual_app = load_dual_app()
        with tempfile.TemporaryDirectory(prefix="stage2_dual_app_scene_") as tmp:
            tmp_path = Path(tmp)
            scene_config = tmp_path / "scene.json"
            scene_config.write_text(
                json.dumps(
                    {
                        "robots": [
                            {
                                "side": "left",
                                "serial_number": "Rizon4-L",
                                "joint_group": "ARM_SCENE",
                                "usd": "scene.usd",
                                "examples_ext": "examples",
                            },
                            {
                                "side": "right",
                                "serial_number": "Rizon4-R",
                                "joint_group": "ARM_SCENE",
                            },
                        ],
                        "cameras": [{"name": "cam_front"}],
                    }
                ),
                encoding="utf-8",
            )

            args = dual_app.parse_args(["--scene-config", str(scene_config)])

            self.assertEqual(args.left_serial_number, "Rizon4-L")
            self.assertEqual(args.right_serial_number, "Rizon4-R")
            self.assertEqual(args.joint_group, "ARM_SCENE")
            self.assertEqual(args.usd, tmp_path / "scene.usd")
            self.assertEqual(args.examples_ext, tmp_path / "examples")

    def test_dual_isaac_app_manual_frame_mode_does_not_enable_quest_receiver(self):
        dual_app = load_dual_app()

        manual_args = dual_app.parse_args([])
        quest_args = dual_app.parse_args(["--enable-quest-target-udp"])

        self.assertFalse(manual_args.enable_quest_target_udp)
        self.assertTrue(quest_args.enable_quest_target_udp)
        self.assertEqual(manual_args.left_rdk_status_udp_port, 57682)
        self.assertEqual(manual_args.right_rdk_status_udp_port, 57683)

    def test_dual_task_initialization_separates_studio_home_from_task_initq(self):
        dual_app = load_dual_app()
        task_q = [-1.0, 1.0, 0.2, 1.5, -0.2, 1.0, 0.0]
        home_q = [0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0]

        self.assertEqual(
            dual_app._bootstrap_q_config(
                {"bootstrap_q": home_q, "initial_q": task_q},
                initial_q=task_q,
            ),
            home_q,
        )
        self.assertEqual(
            dual_app._bootstrap_q_config({"initial_q": task_q}, initial_q=task_q),
            task_q,
        )

    def test_dual_task_ready_joint_error_wraps_at_two_pi(self):
        dual_app = load_dual_app()

        error = dual_app._max_wrapped_joint_error(
            [2.0 * 3.141592653589793 - 0.01, 0.2],
            [0.0, 0.21],
        )

        self.assertAlmostEqual(error, 0.01, places=9)

    def test_dual_task_ready_checks_joint_tolerance(self):
        dual_app = load_dual_app()
        args = dual_app.parse_args([])

        self.assertEqual(args.startup_joint_tolerance_rad, 0.03)

    def test_dual_app_batches_physics_substeps_and_keeps_raw_studio_torque(self):
        source = DUAL_APP.read_text(encoding="utf-8")

        self.assertIn("world.step(render=True)", source)
        self.assertNotIn("StepRateLimiter", source)
        self.assertNotIn("valid_target_drives_or_none", source)
        self.assertNotIn("target_drive_scale", source)
        self.assertIn("arm.robot.apply_torques(target_drives)", source)
        torque_loop = source.split(
            "def _apply_arm_studio_torque(arm: ArmRuntime) -> None:",
            maxsplit=1,
        )[1].split("target_update_gate =", maxsplit=1)[0]
        self.assertNotIn("control_transport_ready", torque_loop)
        self.assertNotIn("arm.rdk_ready", torque_loop)
        self.assertNotIn("reset_hold_cycles_remaining", torque_loop)

    def test_coordinated_reset_is_forwarded_to_drdk_without_world_reset(self):
        source = DUAL_APP.read_text(encoding="utf-8")
        reset_handler = source.split(
            "if pending_reset_control is not None:",
            maxsplit=1,
        )[1].split("if world.is_stopped()", maxsplit=1)[0]

        self.assertIn("begin_coordinated_reset(control)", reset_handler)
        self.assertNotIn("world.reset", reset_handler)
        self.assertNotIn("initialize_like_startup", reset_handler)
        self.assertIn('packet["reset_seq"]', source)
        self.assertIn('reset_state = "succeeded"', source)
        self.assertIn('reset_state = "restoring_assets"', source)
        self.assertIn("reset_configured_scene_assets()", source)
        self.assertIn("scene assets are at configured initial state", source)
        self.assertIn("set_reset_scene_collisions_suppressed(True)", source)
        self.assertIn("set_reset_scene_collisions_suppressed(False)", source)
        self.assertIn("collision_attr.Set(False)", source)
        self.assertIn("kinematic_attr.Set(True)", source)
        self.assertIn("reset_robot_filter_active", source)
        self.assertIn("UsdPhysics.FilteredPairsAPI.Apply(left_root)", source)
        self.assertIn("filtered_pairs.AddTarget(right_path)", source)
        self.assertIn("time.monotonic() >= reset_signal_after_time", source)

    def test_quest_mode_does_not_author_usd_inside_the_target_update_branch(self):
        source = DUAL_APP.read_text(encoding="utf-8")
        quest_branch = source.split(
            "elif arm.latest_quest_target is not None:",
            maxsplit=1,
        )[1].split("if not reset_hold_active and not arm.target_control_requested:", maxsplit=1)[0]

        self.assertIn("arm.quest_goal_pose_base_tcp", quest_branch)
        self.assertNotIn("sync_target_to_base_tcp_pose", quest_branch)
        self.assertNotIn("arm.target_frame.set_world_pose", quest_branch)

    def test_quest_target_frame_is_updated_on_the_render_loop(self):
        source = DUAL_APP.read_text(encoding="utf-8")
        render_update = source.split(
            "def _update_quest_target_frames() -> None:",
            maxsplit=1,
        )[1].split("def _update_rdk_statuses()", maxsplit=1)[0]
        render_loop = source.split("while simulation_app.is_running():", maxsplit=1)[1]

        self.assertIn("arm.rdk_world_calibration", render_update)
        self.assertIn("calibration.rdk_pose_to_world", render_update)
        self.assertIn("arm.target_frame, arm.quest_goal_pose_base_tcp", render_update)
        self.assertIn("arm.command_frame", render_update)
        self.assertIn("arm.latest_control_pose_base_tcp", render_update)
        self.assertIn("frame.set_world_pose", render_update)
        self.assertIn("world.step(render=True)", render_loop)
        self.assertIn("_update_quest_target_frames()", render_loop)

    def test_quest_relative_anchor_uses_rdk_tcp_not_isaac_world_conversion(self):
        source = DUAL_APP.read_text(encoding="utf-8")
        relative_branch = source.split(
            "elif arm.latest_quest_target is not None:",
            maxsplit=1,
        )[1].split("if not reset_hold_active and not arm.target_control_requested:", maxsplit=1)[0]

        self.assertIn("arm.rdk_current_pose_base_tcp", relative_branch)
        self.assertIn("arm.rdk_reference_pose_base_tcp", relative_branch)
        self.assertIn('if args.quest_target_mode == "relative":', relative_branch)
        self.assertNotIn("_current_pose_base_tcp(arm)", relative_branch)

    def test_quest_release_holds_the_last_raw_goal_and_keeps_limiting_toward_it(self):
        source = DUAL_APP.read_text(encoding="utf-8")

        self.assertIn('arm.target_control_source == "quest"', source)
        self.assertIn("control_pose_base_tcp = list(arm.quest_goal_pose_base_tcp)", source)
        self.assertIn("arm.limiter.limit(\n                arm.quest_goal_pose_base_tcp", source)


if __name__ == "__main__":
    unittest.main()
