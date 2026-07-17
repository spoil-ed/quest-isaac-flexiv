#!/usr/bin/env python3
"""Run Stage3 config-driven dual-arm simulation scene validation."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "configs/pipelines/dual_arm_data_collection.yaml"

if str(REPO_ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "scripts"))

import run_stage2_dual_rizon4_real_validation as stage2_runner  # noqa: E402


def _with_default_config(argv: list[str] | None) -> list[str]:
    values = list(argv or [])
    if "--config" not in values and not any(value.startswith("--config=") for value in values):
        return ["--config", str(DEFAULT_CONFIG), *values]
    return values


def main(argv: list[str] | None = None) -> int:
    args = stage2_runner.parse_args(_with_default_config(argv))
    runner = stage2_runner.RealValidationRunner(args)
    runner.report_path = runner.run_root / "stage3_sim_scene_validation.json"
    runner.summary_path = runner.run_root / "stage3_sim_scene_summary.json"
    try:
        runner.run()
    except Exception as exc:
        stage2_runner.json_print({"event": "error", "stage": "stage3", "error": str(exc)})
        for name, path in runner.logs.items():
            stage2_runner.json_print(
                {"event": "log_tail", "name": name, "path": path, "tail": stage2_runner.tail_log(path)}
            )
        if not args.keep_running_on_failure:
            runner.cleanup()
        raise
    finally:
        if not args.keep_running_on_failure:
            runner.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
