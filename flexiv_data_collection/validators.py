#!/usr/bin/env python3
"""Validation helpers for Stage1 Unitree JSON and LeRobot-style outputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from flexiv_data_collection.dual_validation import (
    DEFAULT_STAGE2_LEFT_SERIAL,
    DEFAULT_STAGE2_RIGHT_SERIAL,
    STAGE2_DEFAULT_CAMERA_NAMES,
    summarize_stage2_dual_arm_frames,
)
from flexiv_data_collection.real_validation import (
    EXPECTED_STAGE1_SERIAL,
    STAGE1_CAMERA_KEYS,
    STAGE1_CAMERA_NAMES,
    summarize_stage1_single_arm_frames,
)
from flexiv_data_collection.schema import FLEXIV_VECTOR_DIM, validate_unitree_sample
from flexiv_data_collection.video import probe_video_stream_info


def nonempty_serial(value: str) -> str:
    serial = str(value).strip()
    if not serial:
        raise argparse.ArgumentTypeError(
            "must not be empty; set ROBOT_SERIAL, e.g. export ROBOT_SERIAL=Rizon4-YOUR-SERIAL"
        )
    return serial


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
    strict_dual_arm: bool = False,
    expected_serial: str = EXPECTED_STAGE1_SERIAL,
    expected_left_serial: str = DEFAULT_STAGE2_LEFT_SERIAL,
    expected_right_serial: str = DEFAULT_STAGE2_RIGHT_SERIAL,
    required_camera_keys: tuple[str, ...] = STAGE1_CAMERA_KEYS,
    min_left_q_delta: float = 0.0,
    min_right_q_delta: float = 0.0,
    min_left_torque_norm: float = 0.0,
    min_right_torque_norm: float = 0.0,
    min_target_frame_delta: float = 0.0,
    min_left_target_frame_delta: float = 0.0,
    min_right_target_frame_delta: float = 0.0,
    min_servo_cycle_delta: int = 0,
) -> dict[str, Any]:
    if strict_single_arm and strict_dual_arm:
        raise ValueError("Choose only one strict validation mode")
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
            try:
                strict_summary = summarize_stage1_single_arm_frames(
                    frames,
                    expected_serial=expected_serial,
                    required_camera_keys=STAGE1_CAMERA_KEYS,
                    exact_camera_keys=True,
                    min_left_q_delta=min_left_q_delta,
                    min_left_torque_norm=min_left_torque_norm,
                    min_target_frame_delta=min_target_frame_delta,
                    min_servo_cycle_delta=min_servo_cycle_delta,
                )
            except Exception as exc:
                raise ValueError(f"{data_json}: {exc}") from exc
            strict_reports.append({"path": str(data_json), **strict_summary})
        if strict_dual_arm:
            try:
                strict_summary = summarize_stage2_dual_arm_frames(
                    frames,
                    expected_left_serial=expected_left_serial,
                    expected_right_serial=expected_right_serial,
                    required_camera_keys=required_camera_keys,
                    exact_camera_keys=True,
                    min_left_q_delta=min_left_q_delta,
                    min_right_q_delta=min_right_q_delta,
                    min_left_torque_norm=min_left_torque_norm,
                    min_right_torque_norm=min_right_torque_norm,
                    min_left_target_frame_delta=min_left_target_frame_delta,
                    min_right_target_frame_delta=min_right_target_frame_delta,
                    min_servo_cycle_delta=min_servo_cycle_delta,
                )
            except Exception as exc:
                raise ValueError(f"{data_json}: {exc}") from exc
            strict_reports.append({"path": str(data_json), **strict_summary})
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
    if strict_dual_arm:
        result["stage2_dual_arm"] = {
            "valid": True,
            "reports": strict_reports,
        }
    return result


def validate_lerobot_dataset(
    dataset_root: Path,
    *,
    strict_single_arm: bool = False,
    strict_dual_arm: bool = False,
    required_camera_names: tuple[str, ...] = STAGE1_CAMERA_NAMES,
    expected_video_fps: float | None = None,
    fps_tolerance: float = 0.01,
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
    if strict_dual_arm:
        missing = sorted(set(required_camera_names) - set(info_videos))
        if missing:
            raise ValueError(f"Stage2 dual-arm dataset is missing videos for {missing}; got {sorted(info_videos)}")
    videos = []
    for video_path in sorted(dataset_root.glob("videos/**/*.mp4")):
        stream_info = probe_video_stream_info(video_path)
        codec = str(stream_info.get("codec_name", ""))
        if codec != "h264":
            raise ValueError(f"{video_path} codec must be h264, got {codec}")
        if expected_video_fps is not None:
            fps = stream_info.get("avg_fps") or stream_info.get("r_fps")
            if fps is None or abs(float(fps) - float(expected_video_fps)) > float(fps_tolerance):
                raise ValueError(f"{video_path} fps must be {expected_video_fps}, got {fps}")
        videos.append({"path": str(video_path), "codec": codec, "stream": stream_info})
    if not videos:
        raise FileNotFoundError(f"No MP4 videos found under {dataset_root / 'videos'}")
    if strict_single_arm and len(videos) != len(required_camera_names):
        raise ValueError(f"Stage1 single-arm dataset must contain {len(required_camera_names)} MP4 file(s), got {len(videos)}")
    if strict_dual_arm and len(videos) < len(required_camera_names):
        raise ValueError(f"Stage2 dual-arm dataset must contain at least {len(required_camera_names)} MP4 file(s), got {len(videos)}")
    return {
        "lerobot_dataset": {
            "path": str(dataset_root),
            "info": info,
            "videos": videos,
        }
    }


def _camera_image_paths(raw_dir: Path, camera_key: str) -> list[Path]:
    return sorted(raw_dir.glob(f"episode_*/colors/*_{camera_key}.jpg"))


def frame_diff_stats(
    raw_dir: Path,
    *,
    camera_keys: tuple[str, ...],
    duplicate_threshold: float = 0.2,
) -> dict[str, Any]:
    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("opencv-python and numpy are required for raw frame-diff validation") from exc

    cameras: dict[str, Any] = {}
    for camera_key in camera_keys:
        paths = _camera_image_paths(raw_dir, camera_key)
        if not paths:
            raise FileNotFoundError(f"No raw JPG frames found for camera key {camera_key!r} under {raw_dir}")
        diffs: list[float] = []
        previous = None
        for path in paths:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"Failed to read raw frame {path}")
            if previous is not None:
                if previous.shape != image.shape:
                    raise ValueError(f"Frame shape changed for {camera_key}: {previous.shape} -> {image.shape}")
                diffs.append(float(np.mean(np.abs(image.astype(np.float32) - previous.astype(np.float32)))))
            previous = image
        duplicate_pairs = sum(1 for value in diffs if value < float(duplicate_threshold))
        pair_count = len(diffs)
        cameras[camera_key] = {
            "frames": len(paths),
            "pairs": pair_count,
            "mean_absdiff": float(sum(diffs) / pair_count) if pair_count else 0.0,
            "max_absdiff": float(max(diffs)) if diffs else 0.0,
            "min_absdiff": float(min(diffs)) if diffs else 0.0,
            "duplicate_pairs": duplicate_pairs,
            "duplicate_ratio": float(duplicate_pairs / pair_count) if pair_count else 0.0,
            "duplicate_threshold": float(duplicate_threshold),
        }
    return {"raw_frame_diffs": {"path": str(raw_dir), "cameras": cameras}}


def validate_raw_frame_diffs(
    raw_dir: Path,
    *,
    camera_keys: tuple[str, ...],
    duplicate_threshold: float = 0.2,
    max_duplicate_ratio: float | None = None,
    min_mean_frame_diff: float | None = None,
) -> dict[str, Any]:
    result = frame_diff_stats(raw_dir, camera_keys=camera_keys, duplicate_threshold=duplicate_threshold)
    for camera_key, stats in result["raw_frame_diffs"]["cameras"].items():
        if max_duplicate_ratio is not None and float(stats["duplicate_ratio"]) > float(max_duplicate_ratio):
            raise ValueError(
                f"{camera_key} duplicate_ratio {stats['duplicate_ratio']:.6g} > allowed {float(max_duplicate_ratio):.6g}"
            )
        if min_mean_frame_diff is not None and float(stats["mean_absdiff"]) < float(min_mean_frame_diff):
            raise ValueError(
                f"{camera_key} mean_absdiff {stats['mean_absdiff']:.6g} < required {float(min_mean_frame_diff):.6g}"
            )
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--strict-single-arm", action="store_true")
    parser.add_argument("--strict-dual-arm", action="store_true")
    parser.add_argument(
        "--expected-serial",
        default=None,
        type=nonempty_serial,
    )
    parser.add_argument("--min-left-q-delta", type=float, default=0.0)
    parser.add_argument("--min-right-q-delta", type=float, default=0.0)
    parser.add_argument("--min-left-torque-norm", type=float, default=0.0)
    parser.add_argument("--min-right-torque-norm", type=float, default=0.0)
    parser.add_argument("--min-target-frame-delta", type=float, default=0.0)
    parser.add_argument("--min-left-target-frame-delta", type=float, default=0.0)
    parser.add_argument("--min-right-target-frame-delta", type=float, default=0.0)
    parser.add_argument("--min-servo-cycle-delta", type=int, default=0)
    parser.add_argument("--expected-video-fps", type=float, default=None)
    parser.add_argument("--video-fps-tolerance", type=float, default=0.01)
    parser.add_argument("--frame-diff-threshold", type=float, default=0.2)
    parser.add_argument("--max-duplicate-frame-ratio", type=float, default=None)
    parser.add_argument("--min-mean-frame-diff", type=float, default=None)
    parser.add_argument("--expected-left-serial", default=None, type=nonempty_serial)
    parser.add_argument("--expected-right-serial", default=None, type=nonempty_serial)
    parser.add_argument("--required-camera-names", default=",".join(STAGE2_DEFAULT_CAMERA_NAMES))
    parser.add_argument("--required-camera-keys", default=None)
    args = parser.parse_args(argv)
    if args.strict_single_arm and args.strict_dual_arm:
        parser.error("--strict-single-arm and --strict-dual-arm are mutually exclusive")
    if args.strict_single_arm and args.expected_serial is None:
        parser.error("--expected-serial is required with --strict-single-arm; set ROBOT_SERIAL first")
    if args.strict_dual_arm:
        if args.expected_left_serial is None:
            parser.error("--expected-left-serial is required with --strict-dual-arm")
        if args.expected_right_serial is None:
            parser.error("--expected-right-serial is required with --strict-dual-arm")
    args.required_camera_names_tuple = tuple(
        item.strip() for item in str(args.required_camera_names).split(",") if item.strip()
    )
    if args.required_camera_keys:
        args.required_camera_keys_tuple = tuple(
            item.strip() for item in str(args.required_camera_keys).split(",") if item.strip()
        )
    else:
        args.required_camera_keys_tuple = tuple(f"color_{idx}" for idx, _name in enumerate(args.required_camera_names_tuple))
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    result: dict[str, Any] = {}
    result.update(
        validate_unitree_json(
            args.raw_dir,
            strict_single_arm=bool(args.strict_single_arm),
            strict_dual_arm=bool(args.strict_dual_arm),
            expected_serial=args.expected_serial,
            expected_left_serial=args.expected_left_serial or DEFAULT_STAGE2_LEFT_SERIAL,
            expected_right_serial=args.expected_right_serial or DEFAULT_STAGE2_RIGHT_SERIAL,
            required_camera_keys=args.required_camera_keys_tuple,
            min_left_q_delta=float(args.min_left_q_delta),
            min_right_q_delta=float(args.min_right_q_delta),
            min_left_torque_norm=float(args.min_left_torque_norm),
            min_right_torque_norm=float(args.min_right_torque_norm),
            min_target_frame_delta=float(args.min_target_frame_delta),
            min_left_target_frame_delta=float(args.min_left_target_frame_delta),
            min_right_target_frame_delta=float(args.min_right_target_frame_delta),
            min_servo_cycle_delta=int(args.min_servo_cycle_delta),
        )
    )
    result.update(
        validate_lerobot_dataset(
            args.dataset_root,
            strict_single_arm=bool(args.strict_single_arm),
            strict_dual_arm=bool(args.strict_dual_arm),
            required_camera_names=args.required_camera_names_tuple if args.strict_dual_arm else STAGE1_CAMERA_NAMES,
            expected_video_fps=args.expected_video_fps,
            fps_tolerance=float(args.video_fps_tolerance),
        )
    )
    if args.max_duplicate_frame_ratio is not None or args.min_mean_frame_diff is not None:
        result.update(
            validate_raw_frame_diffs(
                args.raw_dir,
                camera_keys=args.required_camera_keys_tuple,
                duplicate_threshold=float(args.frame_diff_threshold),
                max_duplicate_ratio=args.max_duplicate_frame_ratio,
                min_mean_frame_diff=args.min_mean_frame_diff,
            )
        )
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.out is not None:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
