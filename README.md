# Isaac Sim Flexiv Minimal Workspace

This repository is the thin project layer on top of the original Isaac Sim and
Flexiv workspaces. It keeps only the assets, local extensions, scripts, tests,
and specs needed for the current Flexiv Studio / Quest teleoperation work.

## Layout

- `scripts/`: all runnable entry points and small runtime helpers.
- `standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/`: maintained
  Flexiv Isaac asset/demo code for the new Quest/Studio workflow, including
  the editable bridge app.
- `local_exts/`: local Isaac Sim extensions for teleop loading and Flexiv Studio
  teleop adaptation.
- `configs/`: project configuration files.
- `spec/`: planning/spec documents maintained by the project owner.
- `tests/`: fast regression tests for repository layout and pure-Python helpers.
- `isaac_sim_ws/`: upstream Flexiv Isaac Sim workspace clone, kept as an external
  reference and ignored by this outer repository.
- Original Flexiv examples are treated as backup/reference. New workflow changes
  should land in `flexiv_quest/` and `scripts/`.

Generated logs, recordings, datasets, Python caches, and local dependency
folders are ignored.

## Runtime Entry Points

Start the Flexiv stack as separate processes:

```bash
cd /home/simate/workspace/isaacsim-flexiv

python scripts/install_studio_ball_jog_project.py
python scripts/start_robot_control_app.py
python scripts/start_flexiv_simulation.py
python scripts/start_elements_studio_ui.py
python scripts/studio_plan_control.py start
python scripts/start_flexiv_bridge.py
python scripts/start_isaac_follow.py
```

Check or stop the processes:

```bash
python scripts/flexiv_stack_status.py
python scripts/stop_flexiv_stack.py
```

Teleop / SDG workflows:

```bash
scripts/teleop_sdg teleop
scripts/teleop_sdg record
scripts/teleop_sdg replay --hdf5 recordings/teleop_hdf5/<session>.hdf5
scripts/flexiv_studio_teleop
```

## Test

```bash
python -m unittest discover -s tests -p 'test_*.py'
```
