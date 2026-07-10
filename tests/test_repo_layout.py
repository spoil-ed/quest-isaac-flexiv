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
            "AGENTS.md",
            "README.md",
            "configs",
            "datasets",
            "isaac_sim_ws",
            "local_exts",
            "logs",
            "scripts",
            "spec",
            "standalone_examples",
            "tests",
        }
        actual = {path.name for path in ROOT.iterdir()}

        self.assertTrue(actual.issubset(allowed), sorted(actual - allowed))

    def test_root_repo_does_not_keep_environment_links_or_generated_dirs(self):
        for name in ("isaacsim", "exts", "recordings", ".venv-grpc"):
            self.assertFalse((ROOT / name).exists(), name)

    def test_runtime_scripts_are_split_and_point_to_flexiv_quest_assets(self):
        expected = {
            "flexiv_runtime.py",
            "flexiv_stack_status.py",
            "flexiv_studio_teleop.py",
            "install_studio_ball_jog_project.py",
            "start_elements_studio_ui.py",
            "start_robot_control_app.py",
            "start_flexiv_simulation.py",
            "start_isaac_follow.py",
            "studio_plan_control.py",
            "stop_flexiv_stack.py",
            "teleop_sdg.py",
        }

        self.assertEqual({path.name for path in SCRIPTS.glob("*.py")}, expected)
        follow = load_script("start_isaac_follow.py")
        args = follow.parse_args([])
        command = follow.build_command(args)

        self.assertIn("flexiv_quest/follow_ball_with_studio.py", str(command[1]))
        self.assertIn("rdk-cartesian", command)
        self.assertNotIn("flexiv_test", " ".join(command))

    def test_isaac_follow_startup_prepends_compatible_rdk_client(self):
        follow = load_script("start_isaac_follow.py")
        compat_path = ROOT
        follow.RDK_COMPAT_PATH = compat_path

        env = follow.build_env({"PYTHONPATH": "existing"})

        self.assertEqual(env["PYTHONPATH"].split(":")[:2], [str(compat_path), "existing"])

    def test_scripts_do_not_reference_removed_flexiv_test_path(self):
        offenders = []
        for path in SCRIPTS.glob("*.py"):
            if "flexiv_test" in path.read_text(encoding="utf-8"):
                offenders.append(path.name)

        self.assertEqual(offenders, [])

    def test_flexiv_quest_contains_only_maintained_assets(self):
        allowed = {
            "README.md",
            "app_config.yaml",
            "follow_ball_with_studio.py",
        }
        actual = {path.name for path in FLEXIV_QUEST.iterdir() if path.is_file()}

        self.assertEqual(actual, allowed)
        self.assertFalse(FLEXIV_QUEST.is_symlink())


if __name__ == "__main__":
    unittest.main()
