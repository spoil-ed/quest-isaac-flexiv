import importlib.util
import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/plot_dual_arm_torque.py"
SPEC = importlib.util.spec_from_file_location("plot_dual_arm_torque", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TorquePlotTests(unittest.TestCase):
    def test_parses_dual_torque_packet(self):
        arm = {
            "q": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            "dq": [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7],
            "torque": {
                "tau": [1, 2, 3, 4, 5, 6, 7],
                "tau_ext": [0] * 7,
                "tau_max": [123, 123, 64, 64, 39, 39, 39],
                "ratio": [0.1] * 7,
                "frozen": True,
            }
        }
        packet = {
            "schema": MODULE.SCHEMA,
            "monotonic_time": 10.0,
            "arms": {"left": arm, "right": arm},
        }

        parsed = MODULE.parse_state_packet(json.dumps(packet).encode("utf-8"))

        self.assertEqual(parsed["arms"]["left"]["q"][3], 0.4)
        self.assertEqual(parsed["arms"]["right"]["dq"][3], -0.4)
        self.assertEqual(parsed["arms"]["left"]["tau"][3], 4.0)
        self.assertTrue(parsed["arms"]["right"]["frozen"])

    def test_rejects_missing_torque_vector(self):
        packet = {
            "schema": MODULE.SCHEMA,
            "monotonic_time": 10.0,
            "arms": {"left": {"torque": {}}, "right": {"torque": {}}},
        }

        with self.assertRaises(ValueError):
            MODULE.parse_state_packet(json.dumps(packet).encode("utf-8"))


if __name__ == "__main__":
    unittest.main()
