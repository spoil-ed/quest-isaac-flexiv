#!/usr/bin/env python3
"""Start Quest-controlled Isaac/Flexiv using the unified Hydra control config."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import hydra
from omegaconf import DictConfig, OmegaConf


REPO_ROOT = Path(__file__).resolve().parents[1]


def csv(values) -> str:
    return ",".join(str(float(value)) for value in values)


def resolve_repo_path(value: str) -> Path:
    path = Path(str(value)).expanduser()
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def validate_control_config(cfg: DictConfig) -> None:
    OmegaConf.to_container(cfg, resolve=True, throw_on_missing=True)
    if float(cfg.safety.max_linear_speed_m_s) <= 0.0:
        raise ValueError("safety.max_linear_speed_m_s must be > 0")
    if float(cfg.safety.max_angular_speed_rad_s) <= 0.0:
        raise ValueError("safety.max_angular_speed_rad_s must be > 0")
    if float(cfg.safety.max_joint_speed_rad_s) <= 0.0:
        raise ValueError("safety.max_joint_speed_rad_s must be > 0")
    if float(cfg.safety.max_target_drive_abs) <= 0.0 or float(cfg.safety.max_target_drive_norm) <= 0.0:
        raise ValueError("target drive limits must be > 0")
    if float(cfg.quest.position_scale) <= 0.0:
        raise ValueError("quest.position_scale must be > 0")
    if float(cfg.reset.settle_sec) < 0.0:
        raise ValueError("reset.settle_sec must be >= 0")
    if float(cfg.reset.timeout_sec) <= 0.0:
        raise ValueError("reset.timeout_sec must be > 0")
    if min(
        float(cfg.reset.position_tolerance_m),
        float(cfg.reset.angular_tolerance_rad),
        float(cfg.reset.joint_speed_tolerance_rad_s),
    ) < 0.0:
        raise ValueError("reset tolerances must be >= 0")
    if any(float(lo) > float(hi) for lo, hi in zip(cfg.quest.workspace_min, cfg.quest.workspace_max)):
        raise ValueError("quest.workspace_min must be <= quest.workspace_max on every axis")


def build_command(cfg: DictConfig) -> list[str]:
    validate_control_config(cfg)
    command = [
        sys.executable,
        str(REPO_ROOT / "scripts/start_isaac_follow.py"),
        "--isaac-python",
        str(cfg.runtime.isaac_python),
        "--serial-number",
        str(cfg.robot.serial_number),
        "--joint-group",
        str(cfg.robot.joint_group),
        "--scene-config",
        str(resolve_repo_path(cfg.scene.config)),
        "--physics-hz",
        str(float(cfg.control.physics_hz)),
        "--render-hz",
        str(float(cfg.control.render_hz)),
        "--rdk-target-hz",
        str(float(cfg.control.rdk_target_hz)),
        "--target-pose-publish-hz",
        str(float(cfg.control.target_pose_publish_hz)),
        "--command-timeout-ms",
        str(int(cfg.control.command_timeout_ms)),
        "--quest-target-udp-host",
        str(cfg.network.quest_target_host),
        "--quest-target-udp-port",
        str(int(cfg.network.quest_target_port)),
        "--quest-target-max-age-sec",
        str(float(cfg.quest.packet_max_age_sec)),
        "--quest-target-mode",
        str(cfg.quest.target_mode),
        "--quest-axis-map",
        str(cfg.quest.axis_map),
        "--quest-position-scale",
        str(float(cfg.quest.position_scale)),
        "--quest-position-deadband-m",
        str(float(cfg.quest.position_deadband_m)),
        "--quest-workspace-min",
        csv(cfg.quest.workspace_min),
        "--quest-workspace-max",
        csv(cfg.quest.workspace_max),
        "--target-pose-udp-host",
        str(cfg.network.target_pose_host),
        "--target-pose-udp-port",
        str(int(cfg.network.target_pose_port)),
        "--max-linear-speed-m-s",
        str(float(cfg.safety.max_linear_speed_m_s)),
        "--max-angular-speed-rad-s",
        str(float(cfg.safety.max_angular_speed_rad_s)),
        "--max-joint-speed-rad-s",
        str(float(cfg.safety.max_joint_speed_rad_s)),
        "--max-target-drive-abs",
        str(float(cfg.safety.max_target_drive_abs)),
        "--max-target-drive-norm",
        str(float(cfg.safety.max_target_drive_norm)),
        "--reset-settle-sec",
        str(float(cfg.reset.settle_sec)),
        "--reset-timeout-sec",
        str(float(cfg.reset.timeout_sec)),
        "--reset-position-tolerance-m",
        str(float(cfg.reset.position_tolerance_m)),
        "--reset-angular-tolerance-rad",
        str(float(cfg.reset.angular_tolerance_rad)),
        "--reset-joint-speed-tolerance-rad-s",
        str(float(cfg.reset.joint_speed_tolerance_rad_s)),
    ]
    command.append("--coordinated-reset" if bool(cfg.reset.coordinated) else "--no-coordinated-reset")
    if bool(cfg.quest.enabled):
        command.append("--enable-quest-target-udp")
    command.append("--manual-play" if bool(cfg.launch.manual_play) else "--no-manual-play")
    if bool(cfg.launch.headless):
        command.append("--headless")
    if str(cfg.gateway.endpoint):
        command.extend(
            [
                "--gateway-endpoint",
                str(cfg.gateway.endpoint),
                "--gateway-fps",
                str(float(cfg.gateway.fps)),
                "--gateway-jpeg-quality",
                str(int(cfg.gateway.jpeg_quality)),
            ]
        )
    return command


def build_rdk_command(cfg: DictConfig) -> list[str]:
    validate_control_config(cfg)
    return [
        sys.executable,
        str(REPO_ROOT / "scripts/start_rdk_target_streamer.py"),
        "--python",
        str(cfg.runtime.rdk_python),
        "--host",
        str(cfg.network.target_pose_host),
        "--port",
        str(int(cfg.network.target_pose_port)),
        "--serial-number",
        str(cfg.robot.serial_number),
        "--joint-group",
        str(cfg.robot.joint_group),
        "--max-age-sec",
        str(float(cfg.control.rdk_packet_max_age_sec)),
        "--log-hz",
        str(float(cfg.control.rdk_log_hz)),
    ]


@hydra.main(version_base=None, config_path="../configs/control", config_name="quest_teleop")
def main(cfg: DictConfig) -> None:
    command = build_command(cfg)
    rdk_command = build_rdk_command(cfg) if bool(cfg.launch.rdk_streamer) else None
    print("[hydra-control] resolved config:", flush=True)
    print(OmegaConf.to_yaml(cfg, resolve=True), flush=True)
    print("[hydra-control] command:", flush=True)
    print(json.dumps(command, ensure_ascii=False), flush=True)
    if rdk_command is not None:
        print("[hydra-control] RDK streamer command:", flush=True)
        print(json.dumps(rdk_command, ensure_ascii=False), flush=True)
    if bool(cfg.launch.dry_run):
        return
    if rdk_command is not None:
        subprocess.run(rdk_command, cwd=REPO_ROOT, check=True)
    subprocess.run(command, cwd=REPO_ROOT, check=True)


if __name__ == "__main__":
    main()
