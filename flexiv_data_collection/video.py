"""H264 MP4 creation and validation helpers."""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from pathlib import Path


def require_ffmpeg() -> str:
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg is required to create H264 MP4 files")
    return ffmpeg


def require_ffprobe() -> str:
    ffprobe = shutil.which("ffprobe")
    if not ffprobe:
        raise RuntimeError("ffprobe is required to validate MP4 files")
    return ffprobe


def make_h264_video(image_paths: list[Path], output_path: Path, *, fps: float = 30.0) -> Path:
    if not image_paths:
        raise ValueError("No image paths provided")
    ffmpeg = require_ffmpeg()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="flexiv_h264_frames_") as tmp:
        tmp_dir = Path(tmp)
        for idx, image_path in enumerate(image_paths):
            if not image_path.exists():
                raise FileNotFoundError(image_path)
            suffix = image_path.suffix.lower() or ".jpg"
            link_path = tmp_dir / f"frame_{idx:06d}{suffix}"
            try:
                link_path.symlink_to(image_path.resolve())
            except OSError:
                shutil.copy2(image_path, link_path)
        first_suffix = image_paths[0].suffix.lower() or ".jpg"
        pattern = str(tmp_dir / f"frame_%06d{first_suffix}")
        command = [
            ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-framerate",
            str(float(fps)),
            "-i",
            pattern,
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(output_path),
        ]
        subprocess.run(command, check=True)
    codec = probe_video_codec(output_path)
    if codec != "h264":
        raise RuntimeError(f"Expected H264 video codec, got {codec!r} for {output_path}")
    return output_path


def probe_video_codec(path: Path) -> str:
    ffprobe = require_ffprobe()
    result = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=codec_name",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    payload = json.loads(result.stdout)
    streams = payload.get("streams") or []
    if not streams:
        raise RuntimeError(f"No video stream found in {path}")
    return str(streams[0].get("codec_name", ""))
