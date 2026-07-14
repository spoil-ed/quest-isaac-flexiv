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


def load_stage2_runner():
    spec = importlib.util.spec_from_file_location("run_stage2_dual_rizon4_real_validation", STAGE2_RUNNER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Stage3SimSceneConfigTests(unittest.TestCase):
    def test_stage3_task_scene_configs_parse_expected_objects(self):
        expected_objects = {
            "pick_place_redblock_flexiv_dual": {"back_wall", "work_table_top", "red_block", "place_pad"},
            "pick_redblock_into_drawer_flexiv_dual": {"back_wall", "work_table_top", "red_block", "drawer_cabinet"},
            "stack_rgyblock_flexiv_dual": {"back_wall", "work_table_top", "red_block", "yellow_block", "green_block"},
            "move_cylinder_flexiv_dual": {"back_wall", "work_table_top", "dark_cylinder", "cylinder_goal_band"},
        }
        for task_name, scene_path in STAGE3_SCENES.items():
            with self.subTest(task_name=task_name):
                data = load_scene_config(scene_path)
                self.assertEqual(scene_task_metadata(data)["name"], task_name)
                self.assertEqual([camera["name"] for camera in data["cameras"]], ["cam_front"])
                specs = parse_scene_objects(data, config_path=scene_path, validate_assets=True)
                self.assertEqual({spec.name for spec in specs}, expected_objects[task_name])

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
        self.assertEqual(args.scene_task_metadata["name"], "pick_place_redblock_flexiv_dual")
        self.assertIn("red_block", {item["name"] for item in args.scene_object_summary})
        self.assertEqual(args.scene_camera_names, ["cam_front"])
        self.assertEqual(args.scene_camera_keys, ["color_0"])


if __name__ == "__main__":
    unittest.main()
