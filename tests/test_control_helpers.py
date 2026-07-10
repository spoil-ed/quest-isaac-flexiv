import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
UTILS_DIR = ROOT / "standalone_examples" / "api" / "isaacsim.robot.manipulators"
CONTROL_HELPERS = UTILS_DIR / "control_helpers.py"
ELEMENTS_UTILS = UTILS_DIR / "elements_studio_utils.py"


def load_control_helpers():
    elements_spec = importlib.util.spec_from_file_location("elements_studio_utils", ELEMENTS_UTILS)
    elements = importlib.util.module_from_spec(elements_spec)
    sys.modules[elements_spec.name] = elements
    elements_spec.loader.exec_module(elements)

    spec = importlib.util.spec_from_file_location("control_helpers", CONTROL_HELPERS)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ControlHelpersTests(unittest.TestCase):
    def test_simplugin_target_drives_wait_for_runtime_target(self):
        helpers = load_control_helpers()

        self.assertFalse(
            helpers.should_poll_simplugin_target_drives(
                connected=True,
                runtime_target_active=False,
            )
        )
        self.assertTrue(
            helpers.should_poll_simplugin_target_drives(
                connected=True,
                runtime_target_active=True,
            )
        )


if __name__ == "__main__":
    unittest.main()
