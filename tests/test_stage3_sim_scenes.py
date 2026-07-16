import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

from flexiv_sim_scenes.config import load_scene_config, parse_scene_objects, scene_task_metadata


ROOT = Path(__file__).resolve().parents[1]
STAGE2_RUNNER = ROOT / "scripts/run_stage2_dual_rizon4_real_validation.py"
STAGE3_SCENES = {
    "pick_place_redblock_flexiv_dual": ROOT / "configs/scenes/pick_place_redblock_flexiv_dual.yaml",
    "pick_redblock_into_drawer_flexiv_dual": ROOT / "configs/scenes/pick_redblock_into_drawer_flexiv_dual.yaml",
    "stack_rgyblock_flexiv_dual": ROOT / "configs/scenes/stack_rgyblock_flexiv_dual.yaml",
    "move_cylinder_flexiv_dual": ROOT / "configs/scenes/move_cylinder_flexiv_dual.yaml",
}
STUDIO_HOME_Q = [0.0, -0.6981317, 0.0, 1.57079632679, 0.0, 0.6981317, 0.0]


def load_stage2_runner():
    spec = importlib.util.spec_from_file_location("run_stage2_dual_rizon4_real_validation", STAGE2_RUNNER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Stage3SimSceneConfigTests(unittest.TestCase):
    def test_stage3_task_scene_configs_parse_expected_objects(self):
        expected_objects = {
            "pick_place_redblock_flexiv_dual": {"work_table_top", "red_block", "place_pad"},
            "pick_redblock_into_drawer_flexiv_dual": {"work_table_top", "red_block", "drawer_cabinet"},
            "stack_rgyblock_flexiv_dual": {"work_table_top", "red_block", "yellow_block", "green_block"},
            "move_cylinder_flexiv_dual": {"work_table_top", "dark_cylinder", "cylinder_goal_band"},
        }
        for task_name, scene_path in STAGE3_SCENES.items():
            with self.subTest(task_name=task_name):
                data = load_scene_config(scene_path)
                self.assertEqual(scene_task_metadata(data)["name"], task_name)
                self.assertEqual([camera["name"] for camera in data["cameras"]], ["cam_front"])
                camera = data["cameras"][0]
                self.assertEqual(camera["position"]["y"], 0.0)
                self.assertGreater(camera["position"]["z"], 2.0)
                self.assertEqual(camera["up"], {"x": 1.0, "y": 0.0, "z": 0.0})
                self.assertEqual(len(data["robots"]), 2)
                for robot in data["robots"]:
                    self.assertEqual(robot["initial_q"], STUDIO_HOME_Q)
                    self.assertTrue(str(robot["usd"]).endswith("/Rizon4_with_Grav.usd"))
                    expected_y = 0.20 if robot["side"] == "left" else -0.20
                    self.assertEqual(robot["position"], {"x": -0.06, "y": expected_y, "z": 1.08})
                    self.assertEqual(
                        robot["orientation"],
                        {"w": 0.70710678, "x": 0.0, "y": 0.70710678, "z": 0.0},
                    )
                specs = parse_scene_objects(data, config_path=scene_path, validate_assets=True)
                self.assertEqual({spec.name for spec in specs}, expected_objects[task_name])
                table = next(spec for spec in specs if spec.name == "work_table_top")
                self.assertGreaterEqual(table.position[2] + table.size[2] / 2.0, 0.38)
                self.assertGreaterEqual(table.size[1] / table.size[0], 2.5)

    def test_missing_usd_asset_fails_clearly(self):
        with tempfile.TemporaryDirectory(prefix="stage3_missing_asset_") as tmp:
            scene_path = Path(tmp) / "scene.json"
            scene_path.write_text(
                json.dumps(
                    {
                        "scene_objects": [
                            {
                                "name": "broken_asset",
                                "type": "usd",
                                "prim_path": "/World/Broken",
                                "usd": "missing.usd",
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            data = load_scene_config(scene_path)

            with self.assertRaisesRegex(FileNotFoundError, "broken_asset"):
                parse_scene_objects(data, config_path=scene_path, validate_assets=True)

    def test_stage3_pipeline_config_supplies_task_profile_and_scene_objects(self):
        runner = load_stage2_runner()
        args = runner.parse_args(["--config", str(ROOT / "configs/pipelines/stage3_pick_place_redblock_dual.yaml")])

        self.assertEqual(args.fake_trajectory_profile, "pick_place_redblock_dual")
        self.assertEqual(args.physics_hz, 2000.0)
        self.assertEqual(args.render_hz, 30.0)
        self.assertFalse(args.gpu_dynamics)
        self.assertEqual(args.target_pose_publish_hz, 30.0)
        self.assertEqual(args.isaac_max_frames, 900)
        self.assertEqual(args.scene_task_metadata["name"], "pick_place_redblock_flexiv_dual")
        self.assertIn("red_block", {item["name"] for item in args.scene_object_summary})
        self.assertEqual(args.scene_camera_names, ["cam_front"])
        self.assertEqual(args.scene_camera_keys, ["color_0"])
        self.assertEqual(args.left_rdk_status_udp_port, 58682)
        self.assertEqual(args.right_rdk_status_udp_port, 58683)
        self.assertFalse(args.rdk_clear_fault)


if __name__ == "__main__":
    unittest.main()
