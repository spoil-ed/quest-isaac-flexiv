# Stage2 双臂控制与录制开发报告

## 摘要

Stage2 在 Stage1 单 Rizon4 数据采集闭环之上，新增旧仓库原生双臂主线：

```text
fake/Quest dual target -> Isaac dual TargetFrame
  -> 2x RDK target streamer -> 2x Studio/FlexivSimulation
  -> 2x SimPlugin target_drives -> Isaac
  -> Stage2 gateway -> recorder -> Unitree JSON -> LeRobot-style dataset -> H264 MP4
```

单臂 Stage1 入口、配置、严格右臂零占位验证和 recorder 按键语义保持不变。

## 主要成果

- 新增双臂 scene/pipeline 配置：`dual_rizon4_cam_front.yaml` 和 `stage2_dual_rizon4_data_collection.yaml`。
- 新增双臂 Isaac app：独立于单臂 `follow_ball_with_studio.py`，通过一个 Quest UDP endpoint 按 `side=left/right` 分流，向左右两个 RDK streamer 发布 target pose。
- 新增双臂验收编排脚本：`run_stage2_dual_rizon4_real_validation.py`，启动本仓库 gateway、双 RDK、dual Isaac、fake dual sender、recorder、converter 和 strict dual validator。
- 新增双臂严格校验：要求 Stage2 backend、左右 serial 匹配、左右 q delta 达标、左右 torque 非零、相机帧数完整、LeRobot-style 视频为 H264。
- 新增无 Isaac smoke：`run_stage2_dual_data_collection_smoke.py`，用于验证 gateway/recorder/converter/MP4 数据工具链。

## 配置与边界

- Stage2 默认本机样例 serial 为 left `Rizon4-VIHhZM`、right `Rizon4-WE7ssd`；可通过 CLI 或 scene config 覆盖。
- Stage2 默认只验收 `cam_front/color_0`，后续可通过 scene config 增加腕部相机。
- Stage2 不依赖 `/data/qiming/unitree_lerobot` 或 `/data/qiming/xr_teleoperate`。
- Stage2 不修改 Stage1 单臂主流程；双臂能力通过新 app、新配置和新脚本进入。

## 验收方式

无 Isaac smoke：

```bash
python scripts/run_stage2_dual_data_collection_smoke.py
```

真实闭环：

```bash
python scripts/run_stage2_dual_rizon4_real_validation.py \
  --config configs/pipelines/stage2_dual_rizon4_data_collection.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL"
```

成功产物包括：

- `raw/episode_001/data.json`
- LeRobot-style dataset
- `videos/observation.images.cam_front/chunk-000/file-000.mp4`
- `stage2_dual_rizon4_real_validation.json`
- `stage2_dual_rizon4_real_summary.json`

## 回归要求

- `python -m unittest discover -s tests -p 'test_*.py'` 应通过。
- Stage1 严格单臂 validator 仍要求右臂全零占位。
- 不使用 Stage2 配置时，旧单臂 Quest/TargetFrame/RDK/Studio 行为保持不变。
