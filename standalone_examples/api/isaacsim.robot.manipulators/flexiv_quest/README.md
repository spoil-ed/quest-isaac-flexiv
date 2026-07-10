# Flexiv Quest Assets

This directory keeps only the Isaac-side assets that differ from the original
Isaac Sim Flexiv examples.

Maintained files:

- `follow_ball_with_studio.py`: Isaac visual target ball plus Elements Studio
  jogging and SimPlugin torque bridge.
- `flexiv_isaac_bridge_app.py`: editable bridge app for the new workflow.
- `app_config.yaml`: single-arm Rizon4 bridge configuration used by local
  experiments.

Runtime entry points live in the repository `scripts/` directory:

```bash
scripts/start_elements_studio_ui.py
scripts/start_robot_control_app.py
scripts/start_flexiv_simulation.py
scripts/start_flexiv_bridge.py
scripts/start_isaac_follow.py
scripts/flexiv_stack_status.py
scripts/stop_flexiv_stack.py
```

The stack is intentionally started as separate processes. Avoid relying on the
Elements Studio UI to launch or restart `RobotControlApp`, `FlexivSimulation`,
or Isaac.
