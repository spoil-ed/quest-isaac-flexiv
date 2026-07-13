import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/run_stage1_single_rizon4_real_validation.py"
FOLLOW_APP = ROOT / "standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/follow_ball_with_studio.py"


def load_validation_script():
    spec = importlib.util.spec_from_file_location("run_stage1_single_rizon4_real_validation", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def load_follow_app():
    spec = importlib.util.spec_from_file_location("follow_ball_with_studio", FOLLOW_APP)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Stage1RealValidationConfigTests(unittest.TestCase):
    def test_isaac_app_scene_config_supplies_robot_and_camera_config(self):
        follow = load_follow_app()
        with tempfile.TemporaryDirectory(prefix="stage1_follow_scene_") as tmp:
            tmp_path = Path(tmp)
            scene_config = tmp_path / "scene.yaml"
            scene_config.write_text(
                f"""
robot:
  serial_number: Rizon4-SCENE
  joint_group: ARM_SCENE
  prim_path: /World/Flexiv/SceneRizon4
  usd: "{tmp_path}/scene.usd"
  examples_ext: "{tmp_path}/examples"
cameras:
  - name: cam_front
    position: {{x: 1, y: 0, z: 1}}
    orientation: {{w: 1, x: 0, y: 0, z: 0}}
""",
                encoding="utf-8",
            )

            args = follow.parse_args(["--scene-config", str(scene_config)])

            self.assertEqual(args.serial_number, "Rizon4-SCENE")
            self.assertEqual(args.joint_group, "ARM_SCENE")
            self.assertEqual(args.robot_prim_path, "/World/Flexiv/SceneRizon4")
            self.assertEqual(args.usd, tmp_path / "scene.usd")
            self.assertEqual(args.examples_ext, tmp_path / "examples")
            self.assertEqual(args.camera_config, scene_config)

    def test_yaml_config_supplies_serial_paths_ports_and_outputs(self):
        module = load_validation_script()
        with tempfile.TemporaryDirectory(prefix="stage1_real_config_") as tmp:
            tmp_path = Path(tmp)
            env_config = tmp_path / "environment.yaml"
            scene_config = tmp_path / "scene.yaml"
            pipeline_config = tmp_path / "pipeline.yaml"
            env_config.write_text(
                f"""
rdk_python: "{tmp_path}/rdk/bin/python"
isaac_python: "{tmp_path}/isaac/python.sh"
isaacsim_root: "{tmp_path}/isaac"
isaac_sim_ws: "{tmp_path}/ws"
record_output_root: "{tmp_path}/records"
lerobot_output_root: "{tmp_path}/datasets"
""",
                encoding="utf-8",
            )
            scene_config.write_text(
                f"""
robot:
  serial_number: Rizon4-CUSTOM
  joint_group: ARM_CUSTOM
  prim_path: /World/Flexiv/Custom
  usd: "{tmp_path}/custom.usd"
  examples_ext: "{tmp_path}/examples_ext"
cameras:
  - name: cam_front
    resolution: [320, 240]
    fps: 22
    focal_length: 2.5
    position: {{x: 1.0, y: 0.0, z: 1.0}}
    orientation: {{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}
""",
                encoding="utf-8",
            )
            pipeline_config.write_text(
                """
environment_config: environment.yaml
scene_config: scene.yaml
gateway:
  sample_endpoint: "tcp://127.0.0.1:5700"
  bridge_endpoint: "tcp://127.0.0.1:5701"
  fps: 22
  jpeg_quality: 81
target_pose:
  host: "127.0.0.1"
  port: 5702
quest_target:
  host: "127.0.0.1"
  port: 5703
record:
  task_name: "custom_task"
  fps: 7
  image_size: "320x240"
  max_frames: 9
convert:
  repo_id: "local/custom_stage1"
  action_mode: "torque"
fake_sender:
  host: "127.0.0.1"
  side: "left"
  axis: "y"
  amplitude_m: 0.03
  frames: 11
  rate_hz: 44
  quat_wxyz: "1,0,0,0"
""",
                encoding="utf-8",
            )

            args = module.parse_args(["--config", str(pipeline_config)])

            self.assertEqual(args.config, pipeline_config.resolve())
            self.assertEqual(args.environment_config, env_config.resolve())
            self.assertEqual(args.scene_config, scene_config.resolve())
            self.assertEqual(args.serial_number, "Rizon4-CUSTOM")
            self.assertEqual(args.joint_group, "ARM_CUSTOM")
            self.assertEqual(args.rdk_python, tmp_path / "rdk/bin/python")
            self.assertEqual(args.isaac_python, tmp_path / "isaac/python.sh")
            self.assertEqual(args.isaacsim_root, tmp_path / "isaac")
            self.assertEqual(args.usd, tmp_path / "custom.usd")
            self.assertEqual(args.examples_ext, tmp_path / "examples_ext")
            self.assertEqual(args.camera_config, scene_config.resolve())
            self.assertEqual(args.robot_prim_path, "/World/Flexiv/Custom")
            self.assertEqual(args.scene_camera_names, ["cam_front"])
            self.assertEqual(args.sample_endpoint, "tcp://127.0.0.1:5700")
            self.assertEqual(args.bridge_endpoint, "tcp://127.0.0.1:5701")
            self.assertEqual(args.target_pose_udp_port, 5702)
            self.assertEqual(args.quest_target_udp_port, 5703)
            self.assertEqual(args.output_root, tmp_path / "records")
            self.assertEqual(args.lerobot_output_root, tmp_path / "datasets")
            self.assertEqual(args.repo_id_prefix, "local/custom_stage1")
            self.assertEqual(args.action_mode, "torque")
            self.assertEqual(args.record_frames, 9)
            self.assertEqual(args.record_fps, 7.0)
            self.assertEqual(args.image_size, "320x240")
            self.assertEqual(args.fake_side, "left")
            self.assertEqual(args.fake_axis, "y")
            self.assertEqual(args.fake_amplitude_m, 0.03)
            self.assertEqual(args.fake_frames, 11)
            self.assertEqual(args.fake_rate_hz, 44.0)
            self.assertEqual(args.fake_quat_wxyz, "1,0,0,0")

    def test_cli_overrides_config_and_config_overrides_environment_variables(self):
        module = load_validation_script()
        with tempfile.TemporaryDirectory(prefix="stage1_real_precedence_") as tmp:
            tmp_path = Path(tmp)
            env_config = tmp_path / "environment.yaml"
            scene_config = tmp_path / "scene.yaml"
            pipeline_config = tmp_path / "pipeline.yaml"
            env_config.write_text(
                f"""
rdk_python: "{tmp_path}/config_rdk/python"
isaac_python: "{tmp_path}/config_isaac/python.sh"
isaacsim_root: "{tmp_path}/config_isaac"
record_output_root: "{tmp_path}/records"
""",
                encoding="utf-8",
            )
            scene_config.write_text(
                f"""
robot:
  serial_number: Rizon4-CONFIG
  joint_group: ARM_CONFIG
  usd: "{tmp_path}/config.usd"
  examples_ext: "{tmp_path}/examples"
cameras:
  - name: cam_front
    position: {{x: 1, y: 0, z: 1}}
    orientation: {{w: 1, x: 0, y: 0, z: 0}}
""",
                encoding="utf-8",
            )
            pipeline_config.write_text("environment_config: environment.yaml\nscene_config: scene.yaml\n", encoding="utf-8")
            old_env = os.environ.copy()
            os.environ["FLEXIV_RDK_PYTHON"] = str(tmp_path / "env_rdk/python")
            try:
                args = module.parse_args(["--config", str(pipeline_config), "--serial-number", "Rizon4-CLI"])
            finally:
                os.environ.clear()
                os.environ.update(old_env)

            self.assertEqual(args.serial_number, "Rizon4-CLI")
            self.assertEqual(args.rdk_python, tmp_path / "config_rdk/python")
            self.assertEqual(args.joint_group, "ARM_CONFIG")

    def test_prepare_reports_missing_required_runtime_path(self):
        module = load_validation_script()
        with tempfile.TemporaryDirectory(prefix="stage1_real_missing_") as tmp:
            tmp_path = Path(tmp)
            scene_config = tmp_path / "scene.yaml"
            pipeline_config = tmp_path / "pipeline.yaml"
            scene_config.write_text(
                """
robot:
  serial_number: Rizon4-CONFIG
cameras:
  - name: cam_front
    position: {x: 1, y: 0, z: 1}
    orientation: {w: 1, x: 0, y: 0, z: 0}
""",
                encoding="utf-8",
            )
            pipeline_config.write_text("scene_config: scene.yaml\n", encoding="utf-8")
            runner = module.RealValidationRunner(module.parse_args(["--config", str(pipeline_config)]))

            with self.assertRaisesRegex(RuntimeError, "rdk_python is not configured"):
                runner.prepare()

    def test_cli_serial_overrides_default_without_stage1_rejection(self):
        module = load_validation_script()
        args = module.parse_args(["--serial-number", "Rizon4-USER"])
        runner = module.RealValidationRunner(args)

        self.assertEqual(args.serial_number, "Rizon4-USER")
        self.assertEqual(runner.sample_endpoint, args.sample_endpoint)


if __name__ == "__main__":
    unittest.main()
