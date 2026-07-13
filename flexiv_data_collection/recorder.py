#!/usr/bin/env python3
"""Record Stage1 gateway samples into Unitree JSON episodes."""

from __future__ import annotations

import argparse
import json
import select
import shutil
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Any

from flexiv_data_collection.protocol import GATEWAY_SAMPLE_TYPE, JsonLineReqClient, decode_sample_images
from flexiv_data_collection.schema import (
    BODY,
    FLEXIV_CAMERA_TO_IMAGE_KEY,
    FLEXIV_MOTOR_NAMES,
    LEFT_ARM,
    LEFT_EE,
    RIGHT_ARM,
    RIGHT_EE,
    validate_unitree_sample,
)


class FlexivEpisodeWriter:
    def __init__(
        self,
        task_dir: Path,
        *,
        fps: float,
        image_size: tuple[int, int],
        task_goal: str,
        task_desc: str,
        task_steps: str,
    ) -> None:
        self.task_dir = task_dir
        self.task_name = task_dir.name
        self.fps = fps
        self.image_size = image_size
        self.task_goal = task_goal
        self.task_desc = task_desc
        self.task_steps = task_steps
        self.episode_dir: Path | None = None
        self.json_path: Path | None = None
        self.first_item = True
        self.item_id = -1

    def _next_episode_id(self) -> int:
        self.task_dir.mkdir(parents=True, exist_ok=True)
        ids = []
        for path in self.task_dir.iterdir():
            if path.is_dir() and path.name.startswith("episode_"):
                try:
                    ids.append(int(path.name.split("_")[-1]))
                except ValueError:
                    pass
        return (max(ids) + 1) if ids else 1

    def create_episode(self) -> Path:
        self.item_id = -1
        episode_id = self._next_episode_id()
        self.episode_dir = self.task_dir / f"episode_{episode_id:03d}"
        self.json_path = self.episode_dir / "data.json"
        (self.episode_dir / "colors").mkdir(parents=True, exist_ok=True)
        (self.episode_dir / "depths").mkdir(parents=True, exist_ok=True)
        (self.episode_dir / "audios").mkdir(parents=True, exist_ok=True)
        self.first_item = True
        with self.json_path.open("w", encoding="utf-8") as file_obj:
            file_obj.write("{\n")
            file_obj.write('"info": ' + json.dumps(self._info(), ensure_ascii=False, indent=4) + ",\n")
            file_obj.write('"text": ' + json.dumps(self._text(), ensure_ascii=False, indent=4) + ",\n")
            file_obj.write('"data": [\n')
        return self.episode_dir

    def add_sample(self, sample: dict[str, Any]) -> None:
        import cv2

        if self.episode_dir is None or self.json_path is None:
            raise RuntimeError("No active episode")
        if sample.get("type") != GATEWAY_SAMPLE_TYPE:
            raise ValueError(f"Unexpected gateway sample type: {sample.get('type')!r}")
        validate_unitree_sample(sample)
        self.item_id += 1
        colors = decode_sample_images(sample)
        color_paths = {}
        for color_key, image in colors.items():
            filename = f"{self.item_id:06d}_{color_key}.jpg"
            rel = Path("colors") / filename
            out = self.episode_dir / rel
            if not cv2.imwrite(str(out), image):
                raise RuntimeError(f"Failed to write image {out}")
            color_paths[color_key] = rel.as_posix()

        item = {
            "idx": self.item_id,
            "colors": color_paths,
            "depths": sample.get("depths") or {},
            "states": sample["states"],
            "actions": sample["actions"],
            "tactiles": sample.get("tactiles"),
            "audios": sample.get("audios"),
            "sim_state": sample.get("sim_state") or {},
        }
        item["sim_state"].setdefault("gateway", {})
        item["sim_state"]["gateway"].update(
            {
                "seq": sample.get("seq"),
                "backend": sample.get("backend"),
                "stamp_ns": sample.get("stamp_ns"),
            }
        )
        with self.json_path.open("a", encoding="utf-8") as file_obj:
            if not self.first_item:
                file_obj.write(",\n")
            file_obj.write(json.dumps(item, ensure_ascii=False, indent=4))
            self.first_item = False

    def save_episode(self) -> Path:
        if self.json_path is None:
            raise RuntimeError("No active episode")
        with self.json_path.open("a", encoding="utf-8") as file_obj:
            file_obj.write("\n]\n}\n")
        saved = self.json_path
        self.episode_dir = None
        self.json_path = None
        return saved

    def discard_episode(self) -> None:
        if self.episode_dir is not None and self.episode_dir.exists():
            shutil.rmtree(self.episode_dir)
        self.episode_dir = None
        self.json_path = None

    def _info(self) -> dict[str, Any]:
        width, height = self.image_size
        return {
            "version": "1.0.0",
            "date": time.strftime("%Y-%m-%d"),
            "author": "quest-isaac-flexiv",
            "task_name": self.task_name,
            "image": {"width": width, "height": height, "fps": self.fps},
            "depth": {"width": width, "height": height, "fps": self.fps},
            "audio": {"sample_rate": 16000, "channels": 1, "format": "PCM", "bits": 16},
            "joint_names": {
                LEFT_ARM: list(FLEXIV_MOTOR_NAMES[0:7]),
                LEFT_EE: [FLEXIV_MOTOR_NAMES[7]],
                RIGHT_ARM: list(FLEXIV_MOTOR_NAMES[8:15]),
                RIGHT_EE: [FLEXIV_MOTOR_NAMES[15]],
                BODY: [],
            },
            "camera_names": dict(FLEXIV_CAMERA_TO_IMAGE_KEY),
            "tactile_names": {LEFT_EE: [], RIGHT_EE: []},
            "sim_state": "",
        }

    def _text(self) -> dict[str, str]:
        return {
            "goal": self.task_goal,
            "desc": self.task_desc,
            "steps": self.task_steps,
        }


def parse_image_size(value: str) -> tuple[int, int]:
    width_s, height_s = value.lower().split("x", 1)
    return int(width_s), int(height_s)


def task_name_arg(value: str) -> str:
    task_name = str(value).strip()
    if not task_name or task_name in (".", "..") or Path(task_name).name != task_name or "\\" in task_name:
        raise argparse.ArgumentTypeError("task name must be one non-empty folder name without path separators")
    return task_name


def resolve_task_dir(args: argparse.Namespace) -> Path:
    if args.task_dir is not None:
        return Path(args.task_dir)
    return Path(args.output_root) / str(args.task_name)


class StdinKeyReader:
    """Non-blocking, single-keystroke terminal reader that restores TTY state."""

    def __init__(self) -> None:
        self.fd: int | None = None
        self.original_attributes = None
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.original_attributes = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def poll(self) -> str | None:
        if self.fd is None:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        key = sys.stdin.read(1)
        return key if key not in ("", "\r", "\n") else None

    def close(self) -> None:
        if self.fd is not None and self.original_attributes is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_attributes)
            self.fd = None


def request_sample(client: JsonLineReqClient) -> dict[str, Any] | None:
    client.send_json({"type": "sample_request", "stamp_ns": time.time_ns()})
    reply = client.recv_json(timeout=5.0)
    if reply.get("type") == "error":
        print(f"[recorder] gateway error: {reply.get('error')}", flush=True)
        return None
    return reply


def request_reset(client: JsonLineReqClient, reason: str) -> None:
    client.send_json({"type": "reset_request", "reason": reason, "stamp_ns": time.time_ns()})
    reply = client.recv_json(timeout=5.0)
    if reply.get("type") == "error":
        print(f"[recorder] reset request rejected: {reply.get('error')}", flush=True)
    else:
        control = reply.get("control") or {}
        print(f"[recorder] reset requested seq={control.get('seq')} reason={reason}", flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gateway-endpoint", default="tcp://127.0.0.1:5590")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--episodes", type=int, default=1)
    task_group = parser.add_mutually_exclusive_group(required=True)
    task_group.add_argument(
        "--task-name",
        type=task_name_arg,
        help="Task folder name under --output-root; recommended for new recordings.",
    )
    task_group.add_argument(
        "--task-dir",
        type=Path,
        help="Legacy direct task folder path; use --task-name for new recordings.",
    )
    parser.add_argument("--output-root", type=Path, default=Path("datasets/stage1_records"))
    parser.add_argument("--image-size", default="640x480")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--reset-on-save", action="store_true")
    parser.add_argument("--start-key", default="s")
    parser.add_argument("--stop-key", default="e")
    parser.add_argument("--discard-key", default="d")
    parser.add_argument("--reset-key", default="r")
    parser.add_argument("--quit-key", default="q")
    parser.add_argument("--auto-start", action="store_true")
    parser.add_argument("--task-goal", default="Flexiv Stage1 data collection")
    parser.add_argument("--task-desc", default="Record Flexiv controller actions and observations")
    parser.add_argument("--task-steps", default="teleoperate; record; save; convert")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    task_dir = resolve_task_dir(args)
    writer = FlexivEpisodeWriter(
        task_dir,
        fps=args.fps,
        image_size=parse_image_size(args.image_size),
        task_goal=args.task_goal,
        task_desc=args.task_desc,
        task_steps=args.task_steps,
    )
    client = JsonLineReqClient(args.gateway_endpoint)
    period = 1.0 / max(args.fps, 1e-6)
    auto = args.auto_start or not sys.stdin.isatty()
    episodes_done = 0
    active = False
    paused = False
    frames_this_episode = 0
    last_tick = 0.0
    key_reader = StdinKeyReader()

    try:
        print(
            "[recorder] connected. Commands: "
            f"{args.start_key}=start/resume, {args.stop_key}=pause/save, "
            f"{args.discard_key}=discard, {args.reset_key}=reset, {args.quit_key}=quit",
            flush=True,
        )
        while episodes_done < args.episodes:
            key = key_reader.poll()
            if key == args.quit_key:
                break
            if key == args.reset_key:
                request_reset(client, "keyboard")
                last_tick = time.monotonic()
                continue
            if key == args.discard_key and active:
                writer.discard_episode()
                if args.reset_on_save:
                    request_reset(client, "discard")
                active = False
                paused = False
                frames_this_episode = 0
                print("[recorder] discarded active episode", flush=True)
                continue
            if key == args.start_key and active and paused:
                paused = False
                last_tick = time.monotonic()
                print("[recorder] resumed", flush=True)
                continue
            if (key == args.start_key or (auto and not active)) and not active:
                episode_dir = writer.create_episode()
                active = True
                paused = False
                frames_this_episode = 0
                print(f"[recorder] started {episode_dir}", flush=True)
            if key == args.stop_key and active:
                if not paused:
                    paused = True
                    print("[recorder] paused", flush=True)
                else:
                    saved = writer.save_episode()
                    if args.reset_on_save:
                        request_reset(client, "save")
                    active = False
                    paused = False
                    episodes_done += 1
                    frames_this_episode = 0
                    print(f"[recorder] saved {saved}", flush=True)
                continue
            if not active or paused:
                time.sleep(0.05)
                continue

            sleep_time = last_tick + period - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            last_tick = time.monotonic()

            sample = request_sample(client)
            if sample is None:
                continue
            writer.add_sample(sample)
            frames_this_episode += 1
            if frames_this_episode % max(1, int(args.fps)) == 0:
                print(f"[recorder] episode_frame={frames_this_episode}", flush=True)
            if args.max_frames > 0 and frames_this_episode >= args.max_frames:
                saved = writer.save_episode()
                if args.reset_on_save:
                    request_reset(client, "max_frames")
                active = False
                paused = False
                episodes_done += 1
                frames_this_episode = 0
                print(f"[recorder] saved {saved}", flush=True)
    except KeyboardInterrupt:
        print("[recorder] interrupted", flush=True)
    finally:
        key_reader.close()
        if active and frames_this_episode > 0:
            saved = writer.save_episode()
            if args.reset_on_save:
                request_reset(client, "interrupt_save")
            print(f"[recorder] saved interrupted episode {saved}", flush=True)
        elif active:
            writer.discard_episode()
            print("[recorder] discarded empty interrupted episode", flush=True)
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
