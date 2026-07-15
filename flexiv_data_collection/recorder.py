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

    def saved_task_summary(self) -> tuple[int, int, float]:
        """Return saved episode count, frame count, and duration without loading full JSON."""

        if not self.task_dir.exists():
            return 0, 0, 0.0
        episode_count = 0
        total_frames = 0
        total_duration_sec = 0.0
        for episode_dir in sorted(self.task_dir.glob("episode_*")):
            json_path = episode_dir / "data.json"
            if not episode_dir.is_dir() or not json_path.is_file():
                continue
            frame_count = 0
            episode_fps = max(float(self.fps), 1e-6)
            found_fps = False
            try:
                with json_path.open("r", encoding="utf-8") as file_obj:
                    for line in file_obj:
                        stripped = line.lstrip()
                        if stripped.startswith('"idx":'):
                            frame_count += 1
                        elif not found_fps and stripped.startswith('"fps":'):
                            try:
                                episode_fps = max(
                                    float(stripped.split(":", 1)[1].rstrip(" ,\n")),
                                    1e-6,
                                )
                                found_fps = True
                            except ValueError:
                                pass
            except OSError:
                continue
            episode_count += 1
            total_frames += frame_count
            total_duration_sec += frame_count / episode_fps
        return episode_count, total_frames, total_duration_sec

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


def format_duration(duration_sec: float) -> str:
    total_seconds = max(0, int(round(float(duration_sec))))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


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


def reset_status_from_sample(sample: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(sample, dict):
        return None
    sim_state = sample.get("sim_state") or {}
    bridge = sim_state.get("bridge") or {}
    status = bridge.get("reset")
    return status if isinstance(status, dict) else None


def wait_for_reset(
    client: JsonLineReqClient,
    *,
    expected_seq: int | None,
    timeout_sec: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + max(0.1, float(timeout_sec))
    last_status = None
    while time.monotonic() < deadline:
        sample = request_sample(client)
        status = reset_status_from_sample(sample)
        if status is None:
            if expected_seq is None and sample is not None:
                return {}
            time.sleep(0.1)
            continue
        last_status = status
        seq = int(status.get("last_seq", 0))
        if expected_seq is not None and seq < int(expected_seq):
            time.sleep(0.1)
            continue
        state = str(status.get("state", ""))
        if state in ("idle", "succeeded") and bool(status.get("ready", True)):
            return status
        # Backward compatibility for the original dual-arm bridge status,
        # which exposed only whether it was still holding the start pose.
        if not state and "holding_start_pose" in status and not bool(status["holding_start_pose"]):
            return status
        if state == "failed" and expected_seq is not None:
            raise RuntimeError(str(status.get("error") or f"reset seq={seq} failed"))
        if state == "failed":
            # This is a historical terminal status observed during recorder
            # startup. Return it so the caller can request a fresh recovery
            # reset instead of treating the old failure as a new exception.
            return status
        time.sleep(0.1)
    raise TimeoutError(
        f"reset did not become ready within {float(timeout_sec):.3f}s; last_status={last_status}"
    )


def request_reset(client: JsonLineReqClient, reason: str, *, timeout_sec: float = 25.0) -> dict[str, Any]:
    client.send_json({"type": "reset_request", "reason": reason, "stamp_ns": time.time_ns()})
    reply = client.recv_json(timeout=5.0)
    if reply.get("type") == "error":
        raise RuntimeError(f"reset request rejected: {reply.get('error')}")
    control = reply.get("control") or {}
    seq = int(control.get("seq", 0))
    print(f"[recorder] reset requested seq={seq} reason={reason}; waiting for RDK landing", flush=True)
    status = wait_for_reset(client, expected_seq=seq, timeout_sec=timeout_sec)
    print(f"[recorder] reset succeeded seq={seq}", flush=True)
    return status


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
    parser.add_argument(
        "--reset-timeout-sec",
        type=float,
        default=25.0,
        help="Maximum time to wait for Isaac/RDK reset completion before failing.",
    )
    parser.add_argument("--start-key", default="s")
    parser.add_argument("--stop-key", default="e")
    parser.add_argument("--discard-key", default="d")
    parser.add_argument("--reset-key", default="r")
    parser.add_argument(
        "--reset-key-cooldown-sec",
        type=float,
        default=2.5,
        help="Ignore repeated reset-key events during this cooldown (terminal key repeat protection).",
    )
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
    last_keyboard_reset = float("-inf")
    reset_failed = False
    key_reader = StdinKeyReader()
    saved_task_episodes, saved_task_frames, saved_task_duration_sec = writer.saved_task_summary()

    def register_saved_episode(frame_count: int) -> None:
        nonlocal saved_task_episodes, saved_task_frames, saved_task_duration_sec
        saved_task_episodes += 1
        saved_task_frames += int(frame_count)
        saved_task_duration_sec += int(frame_count) / max(float(args.fps), 1e-6)

    def print_recording_status(*, event: str) -> None:
        current_duration_sec = frames_this_episode / max(float(args.fps), 1e-6)
        print(
            f"[recorder] 录制信息 event={event}: "
            f"本次完成={episodes_done}/{args.episodes}条，"
            f"当前={frames_this_episode}帧/{format_duration(current_duration_sec)}，"
            f"任务已保存={saved_task_episodes}条/{saved_task_frames}帧/"
            f"{format_duration(saved_task_duration_sec)}，"
            f"含当前总时长={format_duration(saved_task_duration_sec + current_duration_sec)}",
            flush=True,
        )

    def perform_reset(reason: str) -> dict[str, Any]:
        nonlocal reset_failed
        try:
            return request_reset(client, reason, timeout_sec=args.reset_timeout_sec)
        except Exception:
            reset_failed = True
            raise

    try:
        print(
            "[recorder] connected. Commands: "
            f"{args.start_key}=start/resume, {args.stop_key}=pause/save, "
            f"{args.discard_key}=discard, {args.reset_key}=reset, {args.quit_key}=quit",
            flush=True,
        )
        print_recording_status(event="启动待机")
        while episodes_done < args.episodes:
            key = key_reader.poll()
            if key == args.quit_key:
                break
            if key == args.reset_key:
                now = time.monotonic()
                if now - last_keyboard_reset >= max(0.0, float(args.reset_key_cooldown_sec)):
                    perform_reset("keyboard")
                    last_keyboard_reset = now
                else:
                    print("[recorder] ignored repeated reset key", flush=True)
                last_tick = time.monotonic()
                continue
            if key == args.discard_key and active:
                writer.discard_episode()
                active = False
                paused = False
                frames_this_episode = 0
                if args.reset_on_save:
                    perform_reset("discard")
                print("[recorder] discarded active episode", flush=True)
                print_recording_status(event="已丢弃")
                continue
            if key == args.start_key and active and paused:
                paused = False
                last_tick = time.monotonic()
                print("[recorder] resumed", flush=True)
                continue
            if (key == args.start_key or (auto and not active)) and not active:
                reset_status = wait_for_reset(
                    client,
                    expected_seq=None,
                    timeout_sec=args.reset_timeout_sec,
                )
                if str(reset_status.get("state", "")) == "failed":
                    print(
                        "[recorder] previous reset failed; requesting a fresh recovery reset",
                        flush=True,
                    )
                    perform_reset("startup_recovery")
                episode_dir = writer.create_episode()
                active = True
                paused = False
                frames_this_episode = 0
                print(f"[recorder] started {episode_dir}", flush=True)
                print_recording_status(event="开始录制")
            if key == args.stop_key and active:
                if not paused:
                    paused = True
                    print("[recorder] paused", flush=True)
                    print_recording_status(event="已暂停")
                else:
                    completed_frames = frames_this_episode
                    saved = writer.save_episode()
                    register_saved_episode(completed_frames)
                    active = False
                    paused = False
                    episodes_done += 1
                    frames_this_episode = 0
                    if args.reset_on_save:
                        perform_reset("save")
                    print(f"[recorder] saved {saved}", flush=True)
                    print_recording_status(event="已保存")
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
                print_recording_status(event="录制中")
            if args.max_frames > 0 and frames_this_episode >= args.max_frames:
                completed_frames = frames_this_episode
                saved = writer.save_episode()
                register_saved_episode(completed_frames)
                active = False
                paused = False
                episodes_done += 1
                frames_this_episode = 0
                if args.reset_on_save:
                    perform_reset("max_frames")
                print(f"[recorder] saved {saved}", flush=True)
                print_recording_status(event="达到帧数上限并保存")
    except KeyboardInterrupt:
        print("[recorder] interrupted", flush=True)
    finally:
        key_reader.close()
        try:
            if active and frames_this_episode > 0:
                completed_frames = frames_this_episode
                saved = writer.save_episode()
                register_saved_episode(completed_frames)
                active = False
                frames_this_episode = 0
                if args.reset_on_save and not reset_failed:
                    perform_reset("interrupt_save")
                print(f"[recorder] saved interrupted episode {saved}", flush=True)
                print_recording_status(event="中断保存")
            elif active:
                writer.discard_episode()
                print("[recorder] discarded empty interrupted episode", flush=True)
        finally:
            client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
