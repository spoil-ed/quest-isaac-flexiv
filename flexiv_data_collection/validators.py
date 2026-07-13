#!/usr/bin/env python3
"""Validation helpers for Stage1 Unitree JSON and LeRobot-style outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from flexiv_data_collection.real_validation import (
    EXPECTED_STAGE1_SERIAL,
    STAGE1_CAMERA_KEYS,
    STAGE1_CAMERA_NAMES,
    summarize_stage1_single_arm_frames,
)
from flexiv_data_collection.schema import FLEXIV_VECTOR_DIM, validate_unitree_sample
from flexiv_data_collection.video import probe_video_codec


def discover_data_json(path: Path) -> list[Path]:
    if path.is_file():
        if path.name != "data.json":
            raise ValueError(f"Expected data.json, got {path}")
        return [path]
    files = sorted(path.glob("**/data.json"))
    if not files:
        raise FileNotFoundError(f"No data.json files found under {path}")
    return files


def validate_unitree_json(
    path: Path,
    *,
    strict_single_arm: bool = False,
    expected_serial: str = EXPECTED_STAGE1_SERIAL,
    min_left_q_delta: float = 0.0,
    min_left_torque_norm: float = 0.0,
    min_servo_cycle_delta: int = 0,
) -> dict[str, Any]:
    total_files = 0
    total_frames = 0
    files = []
    strict_reports = []
    for data_json in discover_data_json(path):
        payload = json.loads(data_json.read_text(encoding="utf-8"))
        frames = payload.get("data", [])
        if not frames:
            raise ValueError(f"{data_json} has no data frames")
        for idx, frame in enumerate(frames):
            try:
                validate_unitree_sample(frame)
            except Exception as exc:
                raise ValueError(f"{data_json} frame {idx}: {exc}") from exc
        if strict_single_arm:
            strict_reports.append(
                {
                    "path": str(data_json),
                    **summarize_stage1_single_arm_frames(
                        frames,
                        expected_serial=expected_serial,
                        required_camera_keys=STAGE1_CAMERA_KEYS,
                        exact_camera_keys=True,
                        min_left_q_delta=min_left_q_delta,
                        min_left_torque_norm=min_left_torque_norm,
                        min_servo_cycle_delta=min_servo_cycle_delta,
                    ),
                }
            )
        total_files += 1
        total_frames += len(frames)
        files.append({"path": str(data_json), "frames": len(frames)})
    result = {
        "unitree_json": {
            "files": files,
            "total_files": total_files,
            "total_frames": total_frames,
            "vector_dim": FLEXIV_VECTOR_DIM,
        }
    }
    if strict_single_arm:
        result["stage1_single_arm"] = {
            "valid": True,
            "reports": strict_reports,
        }
    return result


def validate_lerobot_dataset(
    dataset_root: Path,
    *,
    strict_single_arm: bool = False,
    required_camera_names: tuple[str, ...] = STAGE1_CAMERA_NAMES,
) -> dict[str, Any]:
    info_path = dataset_root / "meta" / "info.json"
    if not info_path.exists():
        raise FileNotFoundError(info_path)
    info = json.loads(info_path.read_text(encoding="utf-8"))
    info_videos = info.get("videos") or {}
    if strict_single_arm:
        if set(info_videos) != set(required_camera_names):
            raise ValueError(
                f"Stage1 single-arm dataset must contain exactly videos for {list(required_camera_names)}, "
                f"got {sorted(info_videos)}"
            )
    videos = []
    for video_path in sorted(dataset_root.glob("videos/**/*.mp4")):
        codec = probe_video_codec(video_path)
        if codec != "h264":
            raise ValueError(f"{video_path} codec must be h264, got {codec}")
        videos.append({"path": str(video_path), "codec": codec})
    if not videos:
        raise FileNotFoundError(f"No MP4 videos found under {dataset_root / 'videos'}")
    if strict_single_arm and len(videos) != len(required_camera_names):
        raise ValueError(f"Stage1 single-arm dataset must contain {len(required_camera_names)} MP4 file(s), got {len(videos)}")
    return {
        "lerobot_dataset": {
            "path": str(dataset_root),
            "info": info,
            "videos": videos,
        }
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--strict-single-arm", action="store_true")
    parser.add_argument("--expected-serial", default=EXPECTED_STAGE1_SERIAL)
    parser.add_argument("--min-left-q-delta", type=float, default=0.0)
    parser.add_argument("--min-left-torque-norm", type=float, default=0.0)
    parser.add_argument("--min-servo-cycle-delta", type=int, default=0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result: dict[str, Any] = {}
    result.update(
        validate_unitree_json(
            args.raw_dir,
            strict_single_arm=bool(args.strict_single_arm),
            expected_serial=args.expected_serial,
            min_left_q_delta=float(args.min_left_q_delta),
            min_left_torque_norm=float(args.min_left_torque_norm),
            min_servo_cycle_delta=int(args.min_servo_cycle_delta),
        )
    )
    result.update(validate_lerobot_dataset(args.dataset_root, strict_single_arm=bool(args.strict_single_arm)))
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
