import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path

from flexiv_sim_scenes.config import load_scene_config, parse_scene_objects, scene_task_metadata
from flexiv_sim_scenes.isaac import _configured_xform_scale


ROOT = Path(__file__).resolve().parents[1]
STAGE2_RUNNER = ROOT / "scripts/run_stage2_dual_rizon4_real_validation.py"
STAGE3_SCENES = {
    "pick_place_redblock_flexiv_dual": ROOT / "configs/scenes/pick_place_redblock_flexiv_dual.yaml",
    "pick_redblock_into_drawer_flexiv_dual": ROOT / "configs/scenes/pick_redblock_into_drawer_flexiv_dual.yaml",
    "stack_rgyblock_flexiv_dual": ROOT / "configs/scenes/stack_rgyblock_flexiv_dual.yaml",
    "move_cylinder_flexiv_dual": ROOT / "configs/scenes/move_cylinder_flexiv_dual.yaml",
}
STUDIO_HOME_Q = [0.0, -0.698132, 0.0, 1.5708, 0.0, 0.698132, 0.0]
TASK_INITIAL_Q = {
    "left": [-1.64741995, 1.54859907, 0.76728963, 1.87202577, 2.14803061, 1.47715997, 0.63020754],
    "right": [-1.41133392, -1.47221452, -0.84874996, -1.69582841, -0.32155028, 1.58311903, 0.66627366],
}
ORIGINAL_INITIAL_Q = {
    "left": [-1.8879, 1.7997, 0.5862, 1.9189, 2.1874, 1.8322, -0.1244],
    "right": [-1.18, -1.7187, -0.6799, -1.7503, -0.1607, 1.9371, -0.0858],
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
                    self.assertEqual(robot["bootstrap_q"], STUDIO_HOME_Q)
                    self.assertEqual(robot["initial_q_waypoints"], [])
                    self.assertEqual(robot["initial_q"], TASK_INITIAL_Q[robot["side"]])
                    self.assertTrue(str(robot["usd"]).endswith("/Rizon4_with_Grav.usd"))
                    expected_y = 0.20 if robot["side"] == "left" else -0.20
                    self.assertEqual(robot["position"], {"x": -0.06, "y": expected_y, "z": 1.08})
                    self.assertEqual(
                        robot["orientation"],
                        {"w": 0.70710678, "x": 0.0, "y": 0.70710678, "z": 0.0},
                    )
                    workspace = robot["workspace"]
                    self.assertTrue(workspace["enabled"])
                    self.assertEqual(workspace["frame"], "world")
                    self.assertTrue(workspace["visualize"])
                    expected_min = (
                        {"x": 0.20, "y": -0.05, "z": 0.25}
                        if robot["side"] == "left"
                        else {"x": 0.20, "y": -0.65, "z": 0.25}
                    )
                    expected_max = (
                        {"x": 0.85, "y": 0.65, "z": 1.05}
                        if robot["side"] == "left"
                        else {"x": 0.85, "y": 0.05, "z": 1.05}
                    )
                    self.assertEqual(workspace["min"], expected_min)
                    self.assertEqual(workspace["max"], expected_max)
                    for axis in ("x", "y", "z"):
                        self.assertLessEqual(workspace["min"][axis], robot["target"]["position"][axis])
                        self.assertGreaterEqual(workspace["max"][axis], robot["target"]["position"][axis])
                left_q = next(robot["initial_q"] for robot in data["robots"] if robot["side"] == "left")
                right_q = next(robot["initial_q"] for robot in data["robots"] if robot["side"] == "right")
                symmetry_residual = [
                    left_q[0] + right_q[0] + 3.141592653589793,
                    left_q[1] + right_q[1],
                    left_q[2] + right_q[2],
                ]
                self.assertLess(sum(value * value for value in symmetry_residual) ** 0.5, 0.37)
                joint_delta = [
                    value - reference
                    for side in ("left", "right")
                    for value, reference in zip(
                        next(robot["initial_q"] for robot in data["robots"] if robot["side"] == side),
                        ORIGINAL_INITIAL_Q[side],
                    )
                ]
                self.assertLess(sum(value * value for value in joint_delta) ** 0.5, 2.50)
                self.assertLess(max(abs(value) for value in joint_delta), 1.50)
                for robot in data["robots"]:
                    expected_euler = (
                        {"x": 142.74600614, "y": 27.68557028, "z": 94.99443205}
                        if robot["side"] == "left"
                        else {"x": -142.74600614, "y": 27.68557028, "z": -94.99443205}
                    )
                    self.assertEqual(robot["target"]["euler_deg"], expected_euler)
                self.assertEqual(
                    next(robot["target"]["position"] for robot in data["robots"] if robot["side"] == "left"),
                    {"x": 0.61510761, "y": 0.30584618, "z": 0.80234595},
                )
                self.assertEqual(
                    next(robot["target"]["position"] for robot in data["robots"] if robot["side"] == "right"),
                    {"x": 0.61510761, "y": -0.30584618, "z": 0.80234595},
                )
                specs = parse_scene_objects(data, config_path=scene_path, validate_assets=True)
                self.assertEqual({spec.name for spec in specs}, expected_objects[task_name])
                table = next(spec for spec in specs if spec.name == "work_table_top")
                self.assertAlmostEqual(table.position[2] + table.size[2] / 2.0, 0.16)
                self.assertGreaterEqual(table.size[1] / table.size[0], 2.5)
                self.assertIsNotNone(table.physics_material)
                self.assertEqual(table.physics_material.restitution, 0.0)
                self.assertEqual(table.physics_material.compliant_contact_stiffness, 20000.0)
                self.assertEqual(table.physics_material.compliant_contact_damping, 250.0)
                self.assertFalse(table.physics_material.compliant_contact_acceleration_spring)
                self.assertEqual(
                    _configured_xform_scale(table),
                    tuple(size * scale for size, scale in zip(table.size, table.scale)),
                )

    def test_scene_reset_restores_transforms_velocities_and_joint_state(self):
        source = (ROOT / "flexiv_sim_scenes/isaac.py").read_text(encoding="utf-8")

        self.assertIn("def reset_scene_objects(", source)
        self.assertIn("_set_xform(stage, spec, scale=_configured_xform_scale(spec))", source)
        self.assertIn("CreateVelocityAttr", source)
        self.assertIn("CreateAngularVelocityAttr", source)
        self.assertIn("_reset_joint_positions_and_velocities(stage, spec)", source)

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

    def test_scene_rejects_damping_without_compliant_contact_stiffness(self):
        data = {
            "scene_objects": [
                {
                    "name": "invalid_table",
                    "type": "cuboid",
                    "prim_path": "/World/InvalidTable",
                    "physics_material": {
                        "compliant_contact": {"stiffness": 0.0, "damping": 250.0}
                    },
                }
            ]
        }

        with self.assertRaisesRegex(ValueError, "damping requires positive stiffness"):
            parse_scene_objects(data, validate_assets=False)

    def test_unified_pipeline_config_supplies_task_profile_and_scene_objects(self):
        runner = load_stage2_runner()
        args = runner.parse_args(["--config", str(ROOT / "configs/pipelines/dual_arm_data_collection.yaml")])

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
        self.assertEqual(args.left_rdk_status_udp_port, 57682)
        self.assertEqual(args.right_rdk_status_udp_port, 57683)
        self.assertFalse(args.rdk_clear_fault)


if __name__ == "__main__":
    unittest.main()
