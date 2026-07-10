#!/usr/bin/env python3
"""Install an Elements Studio project that stays in CartesianJogging for Isaac ball following."""

from __future__ import annotations

import argparse
from pathlib import Path


DEFAULT_STUDIO_USER_DATA_DIR = (
    "/home/simate/workspace/elements_studio/FlexivElementsStudio/"
    "user_data_ui/simDir/simulator0/user_data"
)
DEFAULT_PROJECT_NAME = "isaac_ball_jog"
DEFAULT_PLAN_NAME = "isaac_ball_jog"
DEFAULT_ARM_SERIAL = "Rizon4-I0LIRN"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--user-data-dir", default=DEFAULT_STUDIO_USER_DATA_DIR, help="Elements Studio user_data dir.")
    parser.add_argument("--project-name", default=DEFAULT_PROJECT_NAME, help="Project directory and .proj name.")
    parser.add_argument("--plan-name", default=DEFAULT_PLAN_NAME, help="Studio plan name.")
    parser.add_argument("--arm-serial", default=DEFAULT_ARM_SERIAL, help="Robot serial saved in the .proj file.")
    parser.add_argument(
        "--no-default",
        action="store_true",
        help="Install the project but do not update settings/defaultProjectCfg.xml.",
    )
    return parser.parse_args(argv)


def build_plan_text(plan_name: str) -> str:
    return f'''plan_name: "{plan_name}"
config {{
  node_name: "rootNode"
  pt_name: "Plan"
  pt_type: "PLAN"
  switch_tcp_param {{
    switch_tcp: true
    tool_name: ""
    tcp_index: 0
    tcp_var_name: ""
  }}
  is_breakpoint: false
  lock_external_axes: true
  enable_sync_motion: false
}}
transit_list {{
  start_node_name: "startNode"
  end_node_name: "cart_jogging"
  transit_period: 0
  trigger_condition {{
    condition_type: "NO_CHECK"
  }}
  transit_desc: ""
  transit_name: "Condition0"
}}
node_list {{
  node_name: "cart_jogging"
  pt_name: "cartesian_jogging"
  pt_type: "CARTESIAN_JOGGING"
  switch_tcp_param {{
    switch_tcp: true
    tool_name: ""
    tcp_index: 0
    tcp_var_name: ""
  }}
  param_assignment {{
    lhs_param {{
      name: "enableSixJointAxesCtrl"
      module_name: "cart_jogging"
      type: "BOOL"
      category: "PT_INPUT"
    }}
    rhs_param {{
      type: "BOOL"
      category: "CONST"
      data: "0"
    }}
  }}
  is_breakpoint: false
  lock_external_axes: true
}}
plan_log {{
  enable_log: false
  time_interval: 5
  max_duration: 30
}}
plan_desc: "CartesianJogging primitive receives SetCartJoggingCmd from Isaac and returns target_drives through flexiv_sim_plugin."
sw_ver: VER_3_11
disable_pause_and_resume: false
enable_cart_constraint: true
'''


def build_project_text(project_name: str, plan_name: str, arm_serial: str) -> str:
    return f'''project_name: "{project_name}"
version: "31100"
plan_file_name: "{plan_name}.plan"
scene_width: 4812
scene_height: 4988
plan_config {{
  plan_name: "{plan_name}"
  node_config {{
    node_name: "rootNode"
  }}
  block_list {{
    node_name: "startNode"
    x: 77
    y: 231
  }}
  block_list {{
    node_name: "cart_jogging"
    x: 420
    y: 231
  }}
}}
project_desc {{
  desc: "Isaac ball following through Studio CartesianJogging and flexiv_sim_plugin target_drives."
}}
robot_info {{
  arm_serial_number: "{arm_serial}"
}}
'''


def build_default_project_cfg(project_name: str) -> str:
    return f'''<?xml version="1.0"?>
<fvr>
   <default_project_name>{project_name}</default_project_name>
</fvr>
'''


def install_project(user_data_dir: Path, project_name: str, plan_name: str, arm_serial: str, *, make_default: bool) -> Path:
    projects_dir = user_data_dir / "projects"
    project_dir = projects_dir / project_name
    project_dir.mkdir(parents=True, exist_ok=True)
    (project_dir / ".project").touch()
    (project_dir / f"{plan_name}.plan").write_text(build_plan_text(plan_name), encoding="utf-8")
    (project_dir / f"{project_name}.proj").write_text(
        build_project_text(project_name, plan_name, arm_serial),
        encoding="utf-8",
    )
    if make_default:
        settings_dir = user_data_dir / "settings"
        settings_dir.mkdir(parents=True, exist_ok=True)
        (settings_dir / "defaultProjectCfg.xml").write_text(build_default_project_cfg(project_name), encoding="utf-8")
    return project_dir


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    project_dir = install_project(
        Path(args.user_data_dir).expanduser(),
        args.project_name,
        args.plan_name,
        args.arm_serial,
        make_default=not args.no_default,
    )
    print(f"[StudioBallJogProject] installed {project_dir}", flush=True)
    if not args.no_default:
        print(f"[StudioBallJogProject] default project set to {args.project_name}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
