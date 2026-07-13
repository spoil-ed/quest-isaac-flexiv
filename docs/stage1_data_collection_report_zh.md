# Stage1 数据采集与转换开发报告

## 目标

Stage1 将新管线中的 gateway、recorder、Unitree JSON 录制、LeRobot 风格转换和 H264 MP4 验收能力并入旧 `quest-isaac-flexiv` 仓库。纠偏后，Stage1 严格限定为单 Rizon4、单相机、旧仓库自启动 gateway 的真实闭环验收。旧控制闭环保持不变：

```text
Quest/fake -> Isaac TargetFrame -> RDK -> Studio/RobotControlApp -> SimPlugin -> Isaac
```

本阶段不抽出 Quest/Isaac 坐标映射层；该层作为后续 stage。

## 开发内容

- 新增 `flexiv_data_collection/` 自包含包，不依赖 `/data/qiming/unitree_lerobot` 或 `/data/qiming/xr_teleoperate`。
- 新增 Stage1 gateway，可接收 Isaac app 推送的 `flexiv_bridge_sample`，并向 recorder 提供 `sample_request`。
- 新增 recorder，将 gateway sample 写为 Unitree JSON episode 和 `colors/*.jpg`。
- 新增 converter，将 Unitree JSON 转为最小 LeRobot-style 目录，并用 `ffmpeg libx264` 生成 H264 MP4。
- 新增 validators，用 `ffprobe` 检查 MP4 codec 是否为 `h264`。
- 新增严格单臂 validator：右臂 16D 占位必须全零，只允许 `cam_front/color_0`，并校验 Stage1 backend、serial、servo cycle 和左臂运动阈值。
- 修改旧 Isaac app，只有传 `--gateway-endpoint` 时才创建 scene config 中声明的相机并发布数据 sample。
- 新增 fake Quest sender，向旧 Isaac Quest target UDP 口发送 `rizon4_quest_target.v1` 包；严格验收脚本默认使用专用端口 `55679`，也可通过配置覆盖。
- 新增三层配置：environment config 管本机运行时路径，scene config 管 Rizon4/相机，pipeline config 管 gateway/record/convert/validation。默认 scene serial 使用 `Rizon4-VIHhZM`；该 serial 可通过 YAML/CLI 覆盖，脚本会校验 sample serial 与当前配置一致。

## 交付入口

- `scripts/start_data_gateway.py`
- `scripts/record_unitree_json.py`
- `scripts/convert_unitree_json_to_lerobot.py`
- `scripts/fake_rizon4_quest_sender.py`
- `scripts/validate_data_artifacts.py`
- `scripts/run_stage1_data_collection_smoke.py`
- `scripts/run_stage1_single_rizon4_real_validation.py`

## 验收路径

无 Isaac 快速验收只验证数据工具链，不作为真实闭环验收：

```bash
/data/conda/env/flexiv/bin/python scripts/run_stage1_data_collection_smoke.py
```

严格本机闭环验收：

```bash
/data/conda/env/flexiv/bin/python scripts/run_stage1_single_rizon4_real_validation.py
```

该脚本默认读取 `configs/pipelines/stage1_single_rizon4_data_collection.yaml`，再引用 `configs/environments/local_flexiv_runtime.yaml` 和 `configs/scenes/single_rizon4_cam_front.yaml`。默认端口为 `5690/5691/55678/55679`，若端口已被占用会直接失败，避免误用已有外部 gateway；端口、路径、serial、scene camera 均可通过 YAML 或 CLI 覆盖。recorder 启动前会探测 sample，必须满足：

- `sim_state.backend == quest_isaac_flexiv_stage1`
- serial 等于当前配置 serial
- `servo_cycle` 持续增长
- 存在 `color_0`
- 右臂占位为零
- 左臂已有非 stale 运动和非零 target torque

录制后严格验收：

- Unitree JSON frames 非空，且右臂 `qpos/qvel/torque` 全零。
- LeRobot-style dataset 只包含 `cam_front` 一个 H264 MP4。
- `left_q_delta_norm >= 0.005`，确保视频中机械臂有可见运动。
- 验收报告写入单机器人、单相机、未复用外部 gateway，并记录 pipeline/environment/scene config 路径及解析后的 serial/USD/camera。

## 当前边界

- Stage1 仍是单 Rizon4 验收，右臂在 16D Flexiv schema 中为零占位。
- Stage1 真实验收只允许一个 Isaac 机器人 prim 和一个 `cam_front` 相机；双臂和腕部相机留到后续 stage。
- 禁止使用已有 `unitree_lerobot` 或 `flexiv_studio_pipeline` gateway 作为 Stage1 验收输入。上一轮 `tcp://127.0.0.1:5790` 双臂产物已判定无效。
- 不传 `--gateway-endpoint` 时旧 Isaac app 行为保持不变。
- converter 生成最小 LeRobot-style 目录和 H264 MP4；后续 stage 可接入官方 LeRobot API。
- 采集保存默认不向 bridge 发送 reset，避免 episode 保存时影响旧控制闭环。
