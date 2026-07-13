import importlib.util
import sys
import unittest
from pathlib import Path

from hydra import compose, initialize_config_dir


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/start_isaac_follow_hydra.py"
CONFIG_DIR = ROOT / "configs/control"


def load_launcher():
    spec = importlib.util.spec_from_file_location("start_isaac_follow_hydra", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class HydraControlConfigTests(unittest.TestCase):
    def test_conservative_preset_builds_all_safety_arguments(self):
        module = load_launcher()
        with initialize_config_dir(version_base=None, config_dir=str(CONFIG_DIR)):
            cfg = compose(
                config_name="quest_teleop",
                overrides=[
                    "robot.serial_number=Rizon4-TEST",
                    "runtime.isaac_python=python-test",
                    "launch.dry_run=true",
                ],
            )

        command = module.build_command(cfg)
        rdk_command = module.build_rdk_command(cfg)

        self.assertIn("Rizon4-TEST", command)
        self.assertEqual(command[command.index("--max-linear-speed-m-s") + 1], "0.08")
        self.assertEqual(command[command.index("--max-angular-speed-rad-s") + 1], "0.6")
        self.assertEqual(command[command.index("--max-joint-speed-rad-s") + 1], "1.2")
        self.assertEqual(command[command.index("--max-target-drive-abs") + 1], "80.0")
        self.assertIn("--enable-quest-target-udp", command)
        self.assertIn("--coordinated-reset", command)
        self.assertEqual(command[command.index("--reset-settle-sec") + 1], "2.0")
        self.assertEqual(rdk_command[rdk_command.index("--max-age-sec") + 1], "0.25")

    def test_normal_preset_can_be_selected_and_values_overridden(self):
        module = load_launcher()
        with initialize_config_dir(version_base=None, config_dir=str(CONFIG_DIR)):
            cfg = compose(
                config_name="quest_teleop",
                overrides=[
                    "safety=normal",
                    "robot.serial_number=Rizon4-TEST",
                    "safety.max_linear_speed_m_s=0.12",
                ],
            )

        command = module.build_command(cfg)

        self.assertEqual(command[command.index("--max-linear-speed-m-s") + 1], "0.12")
        self.assertEqual(command[command.index("--max-target-drive-norm") + 1], "200.0")


if __name__ == "__main__":
    unittest.main()
