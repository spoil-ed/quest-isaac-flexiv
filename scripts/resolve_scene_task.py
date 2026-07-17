#!/usr/bin/env python3
"""Resolve a scene YAML by its task.name metadata."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE_DIR = REPO_ROOT / "configs" / "scenes"


def scene_task_name(scene_path: Path) -> str | None:
    data: Any = yaml.safe_load(scene_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        return None
    task = data.get("task")
    if not isinstance(task, dict):
        return None
    name = task.get("name")
    if not isinstance(name, str) or not name.strip():
        return None
    return name.strip()


def discover_scene_tasks(scene_dir: Path = DEFAULT_SCENE_DIR) -> dict[str, Path]:
    tasks: dict[str, Path] = {}
    for scene_path in sorted(scene_dir.glob("*.yaml")):
        task_name = scene_task_name(scene_path)
        if task_name is None:
            continue
        if task_name in tasks:
            raise ValueError(
                f"duplicate task.name {task_name!r}: {tasks[task_name].name}, {scene_path.name}"
            )
        tasks[task_name] = scene_path.resolve()
    return tasks


def resolve_scene_task(task_name: str, scene_dir: Path = DEFAULT_SCENE_DIR) -> Path:
    if not task_name or "/" in task_name or "\\" in task_name or task_name in {".", ".."}:
        raise ValueError("task name must not be empty or contain path separators")
    tasks = discover_scene_tasks(scene_dir)
    try:
        return tasks[task_name]
    except KeyError as exc:
        available = ", ".join(sorted(tasks)) or "<none>"
        raise ValueError(f"unknown task {task_name!r}; available tasks: {available}") from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("task_name", help="Exact task.name value from a scene YAML")
    parser.add_argument("--scene-dir", type=Path, default=DEFAULT_SCENE_DIR)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        scene_path = resolve_scene_task(args.task_name, args.scene_dir)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise SystemExit(f"[scene-task] ERROR: {exc}") from exc
    print(scene_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
