import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from flexiv_data_collection.converter import convert_unitree_json_to_lerobot
from flexiv_data_collection.gateway import FakeBackend, LatestBridgeData
from flexiv_data_collection.recorder import FlexivEpisodeWriter
from flexiv_data_collection.validators import validate_lerobot_dataset, validate_raw_frame_diffs, validate_unitree_json


def _has_runtime_deps() -> bool:
    try:
        import cv2  # noqa: F401
        import numpy  # noqa: F401
    except ModuleNotFoundError:
        return False
    return shutil.which("ffmpeg") is not None and shutil.which("ffprobe") is not None


@unittest.skipUnless(_has_runtime_deps(), "Stage1 converter test requires cv2, numpy, ffmpeg and ffprobe")
class Stage1ConverterTests(unittest.TestCase):
    def test_fake_sample_records_and_converts_to_h264_video(self):
        with tempfile.TemporaryDirectory(prefix="stage1_converter_test_") as tmp:
            root = Path(tmp)
            raw_dir = root / "raw"
            writer = FlexivEpisodeWriter(
                raw_dir,
                fps=10,
                image_size=(160, 120),
                task_goal="stage1 test",
                task_desc="stage1 converter test",
                task_steps="fake; record; convert",
            )
            first_episode = writer.create_episode()
            self.assertEqual(first_episode.name, "episode_001")
            backend = FakeBackend(10, (160, 120), ["color_0"])
            latest = LatestBridgeData()
            for _ in range(3):
                writer.add_sample(backend.sample(latest))
            writer.save_episode()

            second_episode = writer.create_episode()
            self.assertEqual(second_episode.name, "episode_002")
            writer.discard_episode()
            self.assertEqual(writer.create_episode().name, "episode_002")
            writer.discard_episode()

            raw_result = validate_unitree_json(raw_dir)
            self.assertEqual(raw_result["unitree_json"]["total_frames"], 3)

            dataset_root = convert_unitree_json_to_lerobot(
                raw_dir,
                "local/stage1_converter_test",
                output_root=root / "lerobot",
            )
            result = validate_lerobot_dataset(dataset_root, strict_single_arm=True)
            videos = result["lerobot_dataset"]["videos"]
            self.assertEqual(len(videos), 1)
            self.assertTrue(all(item["codec"] == "h264" for item in videos))

    def test_frame_diff_validator_rejects_duplicate_raw_frames(self):
        import cv2
        import numpy as np

        with tempfile.TemporaryDirectory(prefix="stage1_frame_diff_test_") as tmp:
            colors = Path(tmp) / "episode_001" / "colors"
            colors.mkdir(parents=True)
            image = np.zeros((24, 32, 3), dtype=np.uint8)
            cv2.imwrite(str(colors / "000000_color_0.jpg"), image)
            cv2.imwrite(str(colors / "000001_color_0.jpg"), image)

            with self.assertRaisesRegex(ValueError, "duplicate_ratio"):
                validate_raw_frame_diffs(
                    Path(tmp),
                    camera_keys=("color_0",),
                    max_duplicate_ratio=0.0,
                )


if __name__ == "__main__":
    unittest.main()
