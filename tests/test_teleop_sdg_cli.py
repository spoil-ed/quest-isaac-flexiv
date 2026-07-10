import importlib.util
import os
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "teleop_sdg.py"


def load_module():
    spec = importlib.util.spec_from_file_location("teleop_sdg", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TeleopSdgCliTests(unittest.TestCase):
    def test_record_defaults_to_manual_timeline_control(self):
        teleop_sdg = load_module()

        args = teleop_sdg.parse_args(["record"])
        env = teleop_sdg.workflow_env(args, ROOT)

        self.assertEqual(env["SIMATE_TELEOP_WORKFLOW"], "record")
        self.assertEqual(env["SIMATE_TELEOP_RECORD_OPEN_SESSION"], "1")
        self.assertEqual(env["SIMATE_TELEOP_RECORD_AUTO_PLAY"], "0")
        self.assertEqual(env["SIMATE_TELEOP_RECORD_AUTO_START_ON_PLAY"], "1")

    def test_replay_requires_hdf5_and_defaults_to_autostart(self):
        teleop_sdg = load_module()
        hdf5 = ROOT / "recordings" / "demo.hdf5"

        args = teleop_sdg.parse_args(["replay", "--hdf5", str(hdf5)])
        env = teleop_sdg.workflow_env(args, ROOT)

        self.assertEqual(env["SIMATE_TELEOP_WORKFLOW"], "replay")
        self.assertEqual(env["SIMATE_TELEOP_HDF5"], str(hdf5.resolve()))
        self.assertEqual(env["SIMATE_TELEOP_REPLAY_EPISODE"], "0")
        self.assertEqual(env["SIMATE_TELEOP_REPLAY_AUTOSTART"], "1")

    def test_build_isaac_command_enables_workflow_extension(self):
        teleop_sdg = load_module()

        command = teleop_sdg.build_isaac_command(ROOT)

        self.assertIn("--ext-folder", command)
        self.assertIn(str(ROOT / "local_exts"), command)
        self.assertIn("--enable", command)
        self.assertIn("simate.teleop_demo_loader", command)

    def test_load_cloudxr_env_ignores_missing_file(self):
        teleop_sdg = load_module()

        env = teleop_sdg.load_export_file(Path("/tmp/definitely_missing_cloudxr.env"))

        self.assertEqual(env, {})

    def test_load_export_file_parses_simple_exports(self):
        teleop_sdg = load_module()
        path = ROOT / "tests" / "_tmp_export.env"
        path.write_text(
            "export XR_RUNTIME_JSON=/tmp/runtime.json\n"
            "export NV_CXR_RUNTIME_DIR='/tmp/cxr run'\n"
            "IGNORED=value\n"
        )
        try:
            env = teleop_sdg.load_export_file(path)
        finally:
            path.unlink(missing_ok=True)

        self.assertEqual(env["XR_RUNTIME_JSON"], "/tmp/runtime.json")
        self.assertEqual(env["NV_CXR_RUNTIME_DIR"], "/tmp/cxr run")
        self.assertNotIn("IGNORED", env)


if __name__ == "__main__":
    unittest.main()
