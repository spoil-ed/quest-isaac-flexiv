import importlib.util
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "flexiv_runtime.py"


def load_runtime():
    spec = importlib.util.spec_from_file_location("flexiv_runtime_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FlexivRuntimeTests(unittest.TestCase):
    def test_studio_env_resolves_relative_root_before_building_library_paths(self):
        runtime = load_runtime()

        env = runtime.studio_env(Path("../elements_studio/FlexivElementsStudio"))

        expected_root = (ROOT / "../elements_studio/FlexivElementsStudio").resolve()
        self.assertEqual(env["LD_LIBRARY_PATH"].split(":", 1)[0], str(expected_root / "lib"))
        self.assertEqual(env["QT_QPA_PLATFORM_PLUGIN_PATH"], str(expected_root / "plugins"))

    def test_find_process_by_executable_matches_argv0_basename(self):
        runtime = load_runtime()

        with mock.patch.object(
            runtime,
            "pgrep_commands",
            return_value=[
                (10, "python scripts/start_flexiv_simulation.py"),
                (11, "./FlexivSimulation --group_state home"),
            ],
        ):
            self.assertEqual(runtime.find_process_by_executable("FlexivSimulation"), 11)

    def test_find_process_by_executable_does_not_match_argument_text(self):
        runtime = load_runtime()

        with mock.patch.object(
            runtime,
            "pgrep_commands",
            return_value=[(10, "python monitor.py FlexivSimulation")],
        ):
            self.assertIsNone(runtime.find_process_by_executable("FlexivSimulation"))

    def test_find_process_by_executable_ignores_container_pid(self):
        runtime = load_runtime()

        with (
            mock.patch.object(
                runtime,
                "pgrep_commands",
                return_value=[(11, "./FlexivSimulation --group_state home")],
            ),
            mock.patch.object(runtime, "process_is_containerized", return_value=True),
        ):
            self.assertIsNone(runtime.find_process_by_executable("FlexivSimulation"))


if __name__ == "__main__":
    unittest.main()
