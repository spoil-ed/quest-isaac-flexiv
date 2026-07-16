import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"
FLEXIV_QUEST = ROOT / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest"


def load_script(name: str):
    path = SCRIPTS / name
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RepoLayoutTests(unittest.TestCase):
    def test_root_repo_contains_only_project_layer_entries(self):
        allowed = {
            ".deps",
            ".git",
            ".gitignore",
            ".pytest_cache",
            "AGENTS.md",
            "README.md",
            "SETUP.md",
            "configs",
            "datasets",
            "docker",
            "docs",
            "flexiv_data_collection",
            "flexiv_sim_scenes",
            "isaac_sim_ws",
            "local_exts",
            "logs",
            "requirements.txt",
            "record.sh",
            "print.sh",
            "scripts",
            "spec",
            "start.sh",
            "standalone_examples",
            "tests",
            "third_party",
        }
        actual = {path.name for path in ROOT.iterdir()}

        self.assertTrue(actual.issubset(allowed), sorted(actual - allowed))

    def test_root_start_script_starts_dual_stack_without_recorder(self):
        start_script = ROOT / "start.sh"
        text = start_script.read_text(encoding="utf-8")

        self.assertTrue(start_script.stat().st_mode & 0o111)
        self.assertIn('REPO_ROOT="$(cd --', text)
        self.assertIn("docker/flexiv-studio/compose.yaml", text)
        self.assertIn("start_elements_studio_ui.py", text)
        self.assertIn("start_robot_control_app.py", text)
        self.assertIn("start_flexiv_simulation.py", text)
        self.assertIn("start_data_gateway.py", text)
        self.assertIn("start_drdk_target_streamer.py", text)
        self.assertIn("start_dual_isaac_follow.py", text)
        self.assertIn("rizon4_quest_target_publisher.py", text)
        self.assertNotIn("record_unitree_json.py", text)
        self.assertIn("stop_flexiv_stack.py", text)
        self.assertIn("down --remove-orphans", text)
        self.assertIn("clearing stale host shared memory", text)
        self.assertNotIn("/home/", text)

    def test_root_record_script_wraps_interactive_recorder(self):
        record_script = ROOT / "record.sh"
        text = record_script.read_text(encoding="utf-8")

        self.assertTrue(record_script.stat().st_mode & 0o111)
        self.assertIn('REPO_ROOT="$(cd --', text)
        self.assertIn("scripts/record_unitree_json.py", text)
        self.assertIn("--gateway-endpoint", text)
        self.assertIn("--reset-on-save", text)
        self.assertIn('exec "${COMMAND[@]}"', text)
        self.assertNotIn("/home/", text)

    def test_root_print_script_wraps_dual_arm_state_monitor(self):
        print_script = ROOT / "print.sh"
        text = print_script.read_text(encoding="utf-8")

        self.assertTrue(print_script.stat().st_mode & 0o111)
        self.assertIn('REPO_ROOT="$(cd --', text)
        self.assertIn("scripts/print_dual_arm_state.py", text)
        self.assertIn('exec "$PYTHON"', text)
        self.assertNotIn("/home/", text)

    def test_root_repo_does_not_keep_environment_links_or_generated_dirs(self):
        for name in ("isaacsim", "exts", "recordings", ".venv-grpc"):
            self.assertFalse((ROOT / name).exists(), name)

    def test_dual_studio_guide_keeps_action_and_isaac_on_host(self):
        guide = (ROOT / "docs/dual_arm_teleop_docker_guide_zh.md").read_text(encoding="utf-8")

        self.assertIn('LEFT_ROBOT_SERIAL="Rizon4-qSaFLh"', guide)
        self.assertIn('RIGHT_ROBOT_SERIAL="Rizon4-I0LIRN"', guide)
        self.assertIn("--source-simulator simulator1", guide)
        self.assertIn("Isaac Sim、双臂 Bridge、RDK streamer、gateway、Quest/fake sender 和 recorder 都运行在宿主机", guide)
        self.assertIn("<enable>1</enable>", guide)
        self.assertIn("externalEthernetConfig.xml", guide)

    def test_dual_studio_container_does_not_run_host_rdk_streamer(self):
        docker_root = ROOT / "docker/flexiv-studio"
        dockerfile = (docker_root / "Dockerfile").read_text(encoding="utf-8")
        compose = (docker_root / "compose.yaml").read_text(encoding="utf-8")
        entrypoint = (docker_root / "entrypoint.sh").read_text(encoding="utf-8")
        healthcheck = (docker_root / "healthcheck.sh").read_text(encoding="utf-8")
        prepare_runtime = (docker_root / "prepare-runtime.sh").read_text(encoding="utf-8")

        self.assertIn("127.0.0.1:${FLEXIV_STUDIO_VNC_PORT:-5902}:5900", compose)
        self.assertNotIn("rdk_target_streamer", compose)
        self.assertNotIn("rdk_target_streamer", entrypoint)
        self.assertNotIn("rdk_target_streamer", healthcheck)
        self.assertIn("--addamb=cap_sys_nice", entrypoint)
        self.assertIn("exec -a flexiv-docker-robot-control", entrypoint)
        self.assertIn("exec -a flexiv-docker-simulation", entrypoint)
        self.assertIn("^flexiv-docker-robot-control", healthcheck)
        self.assertIn("^flexiv-docker-simulation", healthcheck)
        self.assertIn("setcap cap_sys_nice=ep /sbin/capsh", dockerfile)
        self.assertIn("RobotControlApp exited; restarting the container runtime", entrypoint)
        self.assertIn("FlexivSimulation exited; restarting the container runtime", entrypoint)
        self.assertIn('"<enable>1</enable>"', prepare_runtime)
        self.assertIn("externalEthernetConfig.xml", prepare_runtime)
        self.assertIn("externalEthernetConfig.xml", healthcheck)

    def test_runtime_scripts_are_split_and_point_to_flexiv_quest_assets(self):
        expected = {
            "flexiv_runtime.py",
            "flexiv_stack_status.py",
            "flexiv_studio_teleop.py",
            "convert_unitree_json_to_lerobot.py",
            "fake_rizon4_quest_sender.py",
            "drdk_target_streamer.py",
            "rdk_target_streamer.py",
            "record_unitree_json.py",
            "print_dual_arm_state.py",
            "rizon4_quest_target_publisher.py",
            "run_stage1_data_collection_smoke.py",
            "run_stage1_single_rizon4_real_validation.py",
            "run_stage2_dual_data_collection_smoke.py",
            "run_stage2_dual_rizon4_real_validation.py",
            "run_stage3_sim_scene_validation.py",
            "start_data_gateway.py",
            "start_dual_isaac_follow.py",
            "start_drdk_target_streamer.py",
            "start_elements_studio_ui.py",
            "start_robot_control_app.py",
            "start_flexiv_simulation.py",
            "start_isaac_follow.py",
            "start_isaac_follow_hydra.py",
            "start_rdk_target_streamer.py",
            "stop_flexiv_stack.py",
            "teleop_sdg.py",
            "validate_data_artifacts.py",
        }

        self.assertEqual({path.name for path in SCRIPTS.glob("*.py")}, expected)
        follow = load_script("start_isaac_follow.py")
        args = follow.parse_args([])
        command = follow.build_command(args)

        self.assertIn("flexiv_quest/follow_ball_with_studio.py", str(command[1]))
        self.assertIn("studio-bridge", command)
        self.assertIn("--quest-target-mode", command)
        self.assertIn("relative", command)
        self.assertEqual(command[command.index("--quest-relative-orientation-mode") + 1], "relative")
        self.assertIn("--quest-position-scale", command)
        self.assertIn("1.0", command)
        self.assertNotIn("rdk-cartesian", command)
        self.assertNotIn("--disable-target-pose-udp", command)
        self.assertNotIn("flexiv_test", " ".join(command))
        self.assertIn("--coordinated-reset", command)
        self.assertEqual(command[command.index("--reset-settle-sec") + 1], "2.0")

        dual_follow = load_script("start_dual_isaac_follow.py")
        dual_command = dual_follow.build_command(
            dual_follow.parse_args(
                [
                    "--scene-config",
                    "/tmp/dual_scene.yaml",
                    "--left-serial-number",
                    "Rizon4-L",
                    "--right-serial-number",
                    "Rizon4-R",
                    "--left-target-pose-udp-port",
                    "57680",
                    "--right-target-pose-udp-port",
                    "57681",
                    "--gateway-endpoint",
                    "tcp://127.0.0.1:5791",
                    "--gpu-dynamics",
                ]
            )
        )
        self.assertIn("flexiv_quest/dual_follow_with_studio.py", str(dual_command[1]))
        self.assertIn("--left-serial-number", dual_command)
        self.assertIn("Rizon4-L", dual_command)
        self.assertIn("--right-serial-number", dual_command)
        self.assertIn("Rizon4-R", dual_command)
        self.assertIn("--gpu-dynamics", dual_command)
        self.assertEqual(dual_command[dual_command.index("--quest-relative-orientation-mode") + 1], "relative")
        self.assertEqual(dual_command[dual_command.index("--reset-timeout-sec") + 1], "90.0")

        monitor_command = dual_follow.build_command(
            dual_follow.parse_args(
                [
                    "--state-monitor-udp-host",
                    "127.0.0.1",
                    "--state-monitor-udp-port",
                    "57684",
                    "--state-monitor-hz",
                    "10",
                ]
            )
        )
        self.assertEqual(monitor_command[monitor_command.index("--state-monitor-udp-port") + 1], "57684")
        self.assertEqual(monitor_command[monitor_command.index("--state-monitor-hz") + 1], "10.0")

    def test_external_rdk_target_streamer_uses_compatible_rdk_client(self):
        streamer = load_script("start_rdk_target_streamer.py")
        compat_path = ROOT
        streamer.RDK_COMPAT_PATH = compat_path

        env = streamer.build_env({"PYTHONPATH": "existing"})
        command = streamer.build_command(streamer.parse_args([]))

        self.assertEqual(env["PYTHONPATH"].split(":")[:2], [str(compat_path), "existing"])
        self.assertIn("rdk_target_streamer.py", command[1])
        self.assertNotIn("--network-interface-whitelist", command)
        self.assertIn("--no-clear-fault", command)
        self.assertIn("--no-reconnect-on-error", command)

    def test_external_drdk_target_streamer_uses_compatible_rdk_client(self):
        streamer = load_script("start_drdk_target_streamer.py")
        compat_path = ROOT
        streamer.RDK_COMPAT_PATH = compat_path

        env = streamer.build_env({"PYTHONPATH": "existing"})
        command = streamer.build_command(
            streamer.parse_args(
                ["--scene-config", str(ROOT / "configs/scenes/pick_place_redblock_flexiv_dual.yaml")]
            )
        )

        self.assertEqual(env["PYTHONPATH"].split(":")[:2], [str(compat_path), "existing"])
        self.assertIn("drdk_target_streamer.py", command[1])
        self.assertIn("--left-serial-number", command)
        self.assertIn("--right-serial-number", command)
        self.assertIn(
            "--left-nullspace-posture=0.0,-0.6981317,0.0,1.57079632679,0.0,0.6981317,0.0",
            command,
        )
        self.assertIn("--nullspace-tracking-weight", command)
        self.assertIn("--initial-joint-max-vel-rad-s", command)
        self.assertIn("--initial-joint-max-acc-rad-s2", command)
        self.assertIn("--initial-joint-handoff-sec", command)
        self.assertIn("--no-clear-fault", command)

    def test_isaac_follow_startup_does_not_embed_rdk_client(self):
        follow = load_script("start_isaac_follow.py")
        command = follow.build_command(follow.parse_args([]))

        self.assertFalse(hasattr(follow, "build_env"))
        self.assertNotIn("--rdk-target-hz", command)

    def test_stack_stop_covers_single_dual_and_quest_processes(self):
        stop = load_script("stop_flexiv_stack.py")

        self.assertNotIn("record_unitree_json.py", stop.DEFAULT_NEEDLES)
        self.assertIn("start_data_gateway.py", stop.DEFAULT_NEEDLES)
        self.assertIn("follow_ball_with_studio.py", stop.DEFAULT_NEEDLES)
        self.assertIn("dual_follow_with_studio.py", stop.DEFAULT_NEEDLES)
        self.assertIn("rizon4_quest_target_publisher.py", stop.DEFAULT_NEEDLES)

    def test_isaac_follow_startup_can_set_rdk_target_frequency(self):
        follow = load_script("start_isaac_follow.py")
        command = follow.build_command(follow.parse_args(["--rdk-target-hz", "60"]))

        self.assertIn("--rdk-target-hz", command)
        self.assertIn("60.0", command)

    def test_empty_python_argument_uses_active_interpreter(self):
        follow = load_script("start_isaac_follow.py")
        streamer = load_script("start_rdk_target_streamer.py")

        follow_command = follow.build_command(follow.parse_args(["--isaac-python", ""]))
        streamer_command = streamer.build_command(streamer.parse_args(["--python", ""]))

        self.assertEqual(follow_command[0], sys.executable)
        self.assertEqual(streamer_command[0], sys.executable)

    def test_isaac_follow_startup_can_override_stage1_runtime_paths_and_ports(self):
        follow = load_script("start_isaac_follow.py")
        command = follow.build_command(
            follow.parse_args(
                [
                    "--serial-number",
                    "Rizon4-CUSTOM",
                    "--rdk-serial-number",
                    "Rizon4-CUSTOM",
                    "--joint-group",
                    "ARM_CUSTOM",
                    "--scene-config",
                    "/tmp/scene.yaml",
                    "--robot-prim-path",
                    "/World/Flexiv/Custom",
                    "--usd",
                    "/tmp/Rizon4.usd",
                    "--examples-ext",
                    "/tmp/examples_ext",
                    "--quest-target-udp-port",
                    "55679",
                    "--target-pose-udp-port",
                    "55678",
                    "--command-timeout-ms",
                    "1",
                ]
            )
        )

        self.assertIn("--rdk-serial-number", command)
        self.assertIn("Rizon4-CUSTOM", command)
        self.assertIn("--joint-group", command)
        self.assertIn("ARM_CUSTOM", command)
        self.assertIn("--scene-config", command)
        self.assertIn("/tmp/scene.yaml", command)
        self.assertIn("--robot-prim-path", command)
        self.assertIn("/World/Flexiv/Custom", command)
        self.assertIn("--usd", command)
        self.assertIn("/tmp/Rizon4.usd", command)
        self.assertIn("--examples-ext", command)
        self.assertIn("/tmp/examples_ext", command)
        self.assertIn("--quest-target-udp-port", command)
        self.assertIn("55679", command)
        self.assertIn("--target-pose-udp-port", command)
        self.assertIn("55678", command)
        self.assertIn("--command-timeout-ms", command)
        self.assertIn("1", command)

    def test_scripts_do_not_reference_removed_flexiv_test_path(self):
        offenders = []
        for path in SCRIPTS.glob("*.py"):
            if "flexiv_test" in path.read_text(encoding="utf-8"):
                offenders.append(path.name)

        self.assertEqual(offenders, [])

    def test_maintained_code_does_not_use_studio_jogging(self):
        offenders = []
        search_roots = [SCRIPTS, ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators"]
        banned = ("studio-jog", "StudioJogging", "CartJog", "CartesianJogging", "SetCartJoggingCmd")
        for root in search_roots:
            for path in root.rglob("*.py"):
                text = path.read_text(encoding="utf-8")
                if any(term in text for term in banned):
                    offenders.append(str(path.relative_to(ROOT)))

        self.assertEqual(offenders, [])

    def test_flexiv_quest_contains_only_maintained_assets(self):
        allowed = {
            "README.md",
            "app_config.yaml",
            "dual_follow_with_studio.py",
            "follow_ball_with_studio.py",
        }
        actual = {path.name for path in FLEXIV_QUEST.iterdir() if path.is_file()}

        self.assertEqual(actual, allowed)

    def test_vendored_televuer_requests_controller_button_events(self):
        text = (ROOT / "third_party" / "televuer" / "src" / "televuer" / "televuer.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('eventType=["trigger", "squeeze"]', text)
        self.assertIn("fps=60", text)
        self.assertFalse(FLEXIV_QUEST.is_symlink())

    def test_vendored_televuer_tolerates_missing_controller_pose(self):
        text = (ROOT / "third_party" / "televuer" / "src" / "televuer" / "televuer.py").read_text(
            encoding="utf-8"
        )

        self.assertIn("def _copy_controller_pose", text)
        self.assertIn("len(pose) != 16", text)


if __name__ == "__main__":
    unittest.main()
