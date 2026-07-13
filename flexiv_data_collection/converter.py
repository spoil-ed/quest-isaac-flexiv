#!/usr/bin/env python3
"""Convert Stage1 Unitree JSON episodes into a minimal LeRobot-style dataset."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

from flexiv_data_collection.schema import (
    FLEXIV_CAMERA_NAMES,
    FLEXIV_CAMERA_TO_IMAGE_KEY,
    FLEXIV_JSON_DATA_NAMES,
    FLEXIV_MOTOR_NAMES,
    FLEXIV_ROBOT_TYPE,
    unitree_parts_to_full_vector,
    unitree_parts_to_vector,
)
from flexiv_data_collection.video import make_h264_video


@dataclass(frozen=True)
class EpisodeData:
    path: Path
    payload: dict[str, Any]

    @property
    def frames(self) -> list[dict[str, Any]]:
        return list(self.payload.get("data") or [])

    @property
    def task(self) -> str:
        text = self.payload.get("text") or {}
        return str(text.get("goal") or text.get("desc") or "Flexiv Stage1 data collection")

    @property
    def fps(self) -> float:
        info = self.payload.get("info") or {}
        image_info = info.get("image") or {}
        return float(image_info.get("fps", 30.0))


def discover_episodes(raw_dir: Path) -> list[EpisodeData]:
    files = sorted(Path(raw_dir).glob("**/data.json"))
    if not files:
        raise FileNotFoundError(f"No data.json files found under {raw_dir}")
    episodes = []
    for path in files:
        with path.open("r", encoding="utf-8") as file_obj:
            episodes.append(EpisodeData(path=path, payload=json.load(file_obj)))
    return episodes


def _extract_action(frame: dict[str, Any], action_mode: Literal["qpos", "full"]) -> list[float]:
    actions = frame["actions"]
    if action_mode == "qpos":
        return unitree_parts_to_vector(actions)
    return [
        *unitree_parts_to_full_vector(actions, "qpos"),
        *unitree_parts_to_full_vector(actions, "qvel"),
        *unitree_parts_to_full_vector(actions, "torque"),
    ]


def _action_names(action_mode: Literal["qpos", "full"]) -> list[str]:
    if action_mode == "qpos":
        return list(FLEXIV_MOTOR_NAMES)
    return [f"{name}.{field}" for field in ("qpos", "qvel", "torque") for name in FLEXIV_MOTOR_NAMES]


def _write_table(rows: list[dict[str, Any]], data_dir: Path, episode_index: int) -> Path:
    data_dir.mkdir(parents=True, exist_ok=True)
    parquet_path = data_dir / f"episode_{episode_index:06d}.parquet"
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except ModuleNotFoundError:
        fallback = parquet_path.with_suffix(".jsonl")
        with fallback.open("w", encoding="utf-8") as file_obj:
            for row in rows:
                file_obj.write(json.dumps(row, ensure_ascii=False) + "\n")
        return fallback
    table = pa.Table.from_pylist(rows)
    pq.write_table(table, parquet_path)
    return parquet_path


def _image_paths_for_camera(episodes: list[EpisodeData], color_key: str) -> list[Path]:
    result = []
    for episode in episodes:
        episode_dir = episode.path.parent
        for frame in episode.frames:
            rel = (frame.get("colors") or {}).get(color_key)
            if rel:
                result.append(episode_dir / rel)
    return result


def convert_unitree_json_to_lerobot(
    raw_dir: Path,
    repo_id: str,
    *,
    output_root: Path,
    action_mode: Literal["qpos", "full"] = "qpos",
    fps: float | None = None,
) -> Path:
    episodes = discover_episodes(raw_dir)
    dataset_root = output_root.expanduser() / repo_id
    if dataset_root.exists():
        shutil.rmtree(dataset_root)
    data_dir = dataset_root / "data" / "chunk-000"
    meta_dir = dataset_root / "meta"
    videos_root = dataset_root / "videos"
    meta_dir.mkdir(parents=True, exist_ok=True)

    total_frames = 0
    episode_meta = []
    for episode_index, episode in enumerate(episodes):
        rows = []
        episode_fps = fps if fps is not None else episode.fps
        for frame_index, frame in enumerate(episode.frames):
            rows.append(
                {
                    "observation.state": unitree_parts_to_vector(frame["states"]),
                    "action": _extract_action(frame, action_mode),
                    "episode_index": episode_index,
                    "frame_index": frame_index,
                    "timestamp": frame_index / max(episode_fps, 1e-6),
                    "task": episode.task,
                }
            )
        data_path = _write_table(rows, data_dir, episode_index)
        episode_meta.append(
            {
                "episode_index": episode_index,
                "length": len(rows),
                "dataset_from_index": total_frames,
                "dataset_to_index": total_frames + len(rows),
                "task": episode.task,
                "data_path": str(data_path.relative_to(dataset_root)),
            }
        )
        total_frames += len(rows)

    first_fps = fps if fps is not None else episodes[0].fps
    video_meta = {}
    for color_key, camera_name in FLEXIV_CAMERA_TO_IMAGE_KEY.items():
        image_paths = _image_paths_for_camera(episodes, color_key)
        if not image_paths:
            continue
        video_path = videos_root / f"observation.images.{camera_name}" / "chunk-000" / "file-000.mp4"
        make_h264_video(image_paths, video_path, fps=first_fps)
        video_meta[camera_name] = str(video_path.relative_to(dataset_root))

    info = {
        "repo_id": repo_id,
        "robot_type": FLEXIV_ROBOT_TYPE,
        "fps": first_fps,
        "total_episodes": len(episodes),
        "total_frames": total_frames,
        "features": {
            "observation.state": {
                "shape": [len(FLEXIV_MOTOR_NAMES)],
                "names": [list(FLEXIV_MOTOR_NAMES)],
            },
            "action": {
                "shape": [len(_action_names(action_mode))],
                "names": [_action_names(action_mode)],
            },
            **{
                f"observation.images.{camera}": {
                    "dtype": "video",
                    "names": ["height", "width", "channel"],
                }
                for camera in FLEXIV_CAMERA_NAMES
                if camera in video_meta
            },
        },
        "videos": video_meta,
        "minimal_quest_isaac_flexiv_stage1": True,
    }
    with (meta_dir / "info.json").open("w", encoding="utf-8") as file_obj:
        json.dump(info, file_obj, indent=2, ensure_ascii=False)
    with (meta_dir / "episodes.jsonl").open("w", encoding="utf-8") as file_obj:
        for item in episode_meta:
            file_obj.write(json.dumps(item, ensure_ascii=False) + "\n")
    return dataset_root


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--output-root", type=Path, default=Path(os.environ.get("LEROBOT_OUTPUT_ROOT", "datasets/lerobot")))
    parser.add_argument("--action-mode", choices=["qpos", "full"], default="qpos")
    parser.add_argument("--fps", type=float, default=None)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    dataset_root = convert_unitree_json_to_lerobot(
        args.raw_dir,
        args.repo_id,
        output_root=args.output_root,
        action_mode=args.action_mode,
        fps=args.fps,
    )
    print(f"Created Stage1 LeRobot-style dataset: {dataset_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
