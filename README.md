# Quest Isaac Flexiv

使用 Meta Quest/fake sender 控制 Isaac Sim 中的 Flexiv Rizon4，并将相机、机器人状态和控制量录制为 Unitree JSON，再转换为 LeRobot 数据集。Stage1 保留单臂主线，Stage2 新增双臂主线，Stage3 新增配置驱动的仿真任务场景。

```text
Quest -> Isaac TargetFrame -> Flexiv RDK -> Elements Studio/FlexivSimulation
      -> SimPlugin target_drives -> Isaac -> Stage1 recorder

Dual Quest/fake -> Isaac Dual TargetFrame -> 2x Flexiv RDK/Studio
      -> 2x SimPlugin target_drives -> Isaac -> Stage2 recorder

Stage3 scene config -> Isaac task objects/cameras + Stage2 dual control
      -> Unitree JSON -> LeRobot-style dataset -> H264 task video
```

## 仓库分布

| 路径 | 内容 |
| --- | --- |
| `SETUP.md` | 环境安装、外部运行时、证书和启动前检查 |
| `docs/SCRIPT_PARAMETERS.md` | 主流程脚本全部 CLI 参数、默认值和作用 |
| `configs/` | 场景、相机、Hydra 控制、安全限制和数据采集配置 |
| `scripts/` | Studio、Isaac、Quest、gateway、recorder、转换和验证入口 |
| `flexiv_data_collection/` | gateway、Unitree JSON、LeRobot 转换和严格验证实现 |
| `flexiv_sim_scenes/` | Stage3 任务场景 YAML 解析和 Isaac 场景物体加载 |
| `standalone_examples/.../flexiv_quest/` | Isaac Rizon4 场景、Quest 输入和 Studio bridge |
| `datasets/stage1_records/` | 按 task name 保存的原始 episode |
| `datasets/lerobot/` | 转换后的 LeRobot 数据集 |
| `logs/` | 后台进程 stdout/stderr |
| `tests/` | 不启动 Isaac/Studio 的快速回归测试 |

首次使用先完成 [SETUP.md](SETUP.md)。以下命令均在仓库根目录执行：

```bash
export ROBOT_SERIAL="Rizon4-YOUR-SERIAL"
export HOST_IP="192.168.32.11"
export ISAAC_PYTHON="/path/to/isaacsim/bin/python"
export STUDIO_ROOT="/path/to/FlexivElementsStudio"
export TASK_NAME="pick_cube"
```

## 脚本执行顺序

`start_robot_control_app.py`、`start_flexiv_simulation.py`、`start_elements_studio_ui.py`、`start_isaac_follow.py` 和 `start_rdk_target_streamer.py` 会转入后台；gateway、Quest publisher 和 recorder 分别在独立终端前台运行。

以下只展示推荐参数组合；各参数的默认值、作用和注意事项见 [脚本参数说明](docs/SCRIPT_PARAMETERS.md)。

关键参数速查：

| 参数 | 作用 |
| --- | --- |
| `--studio-root` | Elements Studio 安装根目录。 |
| `--isaac-python` / `--python` | Isaac 和 RDK streamer 使用的 Python。 |
| `--serial-number` | RDK 别名序列号；Studio、Isaac、Quest 和验证必须一致。 |
| `--left-serial-number` / `--right-serial-number` | Stage2 双臂左右 RDK 别名序列号。 |
| `--scene-config` | 机器人、USD 和 `cam_front` 相机配置。 |
| `--sample-endpoint` | recorder 从 gateway 获取样本及发送 reset 的地址。 |
| `--bridge-endpoint` / `--gateway-endpoint` | Isaac 向 gateway 推送数据的同一地址。 |
| `--quest-target-udp-port` / `--udp-port` | Quest publisher 与 Isaac 必须使用同一端口。 |
| `--target-pose-udp-port` / RDK `--port` | Isaac 与 RDK streamer 必须使用同一端口。 |
| `--task-name` | 原始数据的 task 文件夹名称。 |
| `--episodes` | 本次 recorder 需要保存的 episode 数量。 |
| `--fps` | gateway、recorder 或转换阶段各自的采样/视频频率，具体作用见参数文档。 |
| `--reset-on-save` | episode 保存、丢弃或自动结束后请求协调 reset。 |

### 1. 启动 Elements Studio runtime

```bash
python scripts/start_elements_studio_ui.py --studio-root "$STUDIO_ROOT"
python scripts/start_robot_control_app.py --studio-root "$STUDIO_ROOT"
python scripts/start_flexiv_simulation.py --studio-root "$STUDIO_ROOT"
```

冷启动时先等待 Studio UI 显示机器人，再执行后两条命令。Studio UI 可能会自动启动 RobotControlApp 和 FlexivSimulation；启动脚本会检测已有进程，不会重复启动。

### 2. 启动数据 gateway

```bash
python scripts/start_data_gateway.py \
  --backend bridge \
  --sample-endpoint tcp://127.0.0.1:5690 \
  --bridge-endpoint tcp://127.0.0.1:5691 \
  --fps 30 \
  --camera-keys color_0
```

### 3. 启动 RDK target streamer

```bash
python scripts/start_rdk_target_streamer.py \
  --python "$ISAAC_PYTHON" \
  --serial-number "$ROBOT_SERIAL" \
  --port 55678
```

### 4. 启动 Isaac、相机和 SimPlugin bridge

```bash
python scripts/start_isaac_follow.py \
  --isaac-python "$ISAAC_PYTHON" \
  --serial-number "$ROBOT_SERIAL" \
  --scene-config configs/scenes/single_rizon4_cam_front.yaml \
  --no-manual-play \
  --enable-quest-target-udp \
  --quest-target-udp-port 55679 \
  --target-pose-udp-port 55678 \
  --rdk-target-hz 60 \
  --gateway-endpoint tcp://127.0.0.1:5691 \
  --gateway-fps 30
```

### 5. 连接 Quest

```bash
.venv-quest/bin/python scripts/rizon4_quest_target_publisher.py \
  --host-ip "$HOST_IP" \
  --serial-number "$ROBOT_SERIAL" \
  --udp-host 127.0.0.1 \
  --udp-port 55679 \
  --side right \
  --enable-button squeeze \
  --axis-map=-z,-x,y \
  --position-delta-scale 1.0 \
  --position-deadband 0.0 \
  --rate-hz 60
```

在 Quest 浏览器打开 publisher 输出的 `https://<HOST_IP>:8012`，进入 VR 后按住右手柄 `squeeze` 控制机器人。

### 6. 按 task name 录制 episode

```bash
python scripts/record_unitree_json.py \
  --gateway-endpoint tcp://127.0.0.1:5690 \
  --task-name "$TASK_NAME" \
  --output-root datasets/stage1_records \
  --fps 10 \
  --episodes 10 \
  --image-size 640x480 \
  --reset-on-save
```

目录自动分配为：

```text
datasets/stage1_records/<task_name>/
├── episode_001/
├── episode_002/
└── episode_003/
```

编号从 `001` 开始，已有 episode 后自动续号；丢弃未保存的 episode 不占用编号。旧 `--task-dir <完整路径>` 仍兼容，但新录制推荐使用 `--task-name`。

recorder 快捷键：

- `s`：开始或继续。
- `e`：暂停；暂停后再按一次保存。
- `d`：丢弃当前 episode。
- `r`：一键 reset Isaac + Studio 到启动状态。
- `q`：退出。

手动录制命令不要添加自动开始参数。通过 `--episodes N` 指定轨迹数量；保存一条后 recorder 会继续等待 `s`，直到累计保存 N 条轨迹才退出。

recorder 启动、录制中、暂停、保存和丢弃时都会输出录制统计，包括本次完成条数、当前 episode 帧数/时长、任务目录已保存总条数/总帧数/总时长。

### 7. 转换为 LeRobot 数据集

```bash
python scripts/convert_unitree_json_to_lerobot.py \
  --raw-dir "datasets/stage1_records/$TASK_NAME" \
  --repo-id "qiming/$TASK_NAME" \
  --output-root datasets/lerobot
```

### 8. 严格验证

```bash
python scripts/validate_data_artifacts.py \
  --raw-dir "datasets/stage1_records/$TASK_NAME" \
  --dataset-root "datasets/lerobot/qiming/$TASK_NAME" \
  --strict-single-arm \
  --expected-serial "$ROBOT_SERIAL" \
  --min-left-q-delta 0.005 \
  --min-left-torque-norm 1e-8 \
  --min-servo-cycle-delta 5
```

### 9. 检查与停止

```bash
python scripts/flexiv_stack_status.py
tail -f logs/*.stderr.log
python scripts/stop_flexiv_stack.py
```

## Stage2 双臂流程

Stage2 使用独立配置，不改变上面的 Stage1 单臂流程：

- scene config：`configs/scenes/dual_rizon4_cam_front.yaml`
- pipeline config：`configs/pipelines/stage2_dual_rizon4_data_collection.yaml`
- 默认本机样例 serial：left `Rizon4-VIHhZM`，right `Rizon4-WE7ssd`，可用 CLI 或 scene config 覆盖。
- 默认端口：gateway `5790/5791`，Quest target `57679`，left/right target pose `57680/57681`。

无 Isaac 的双臂数据工具链 smoke：

```bash
python scripts/run_stage2_dual_data_collection_smoke.py
```

真实双臂闭环验收：

```bash
python scripts/run_stage2_dual_rizon4_real_validation.py \
  --config configs/pipelines/stage2_dual_rizon4_data_collection.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL"
```

验收会启动本仓库 gateway、两个 RDK target streamer、双臂 Isaac app、fake dual sender、recorder、converter 和 strict dual validator。成功后输出：

```text
datasets/stage1_records/quest_isaac_flexiv_stage2_dual_rizon4_real_<stamp>/
├── raw/episode_001/data.json
├── logs/
├── stage2_dual_rizon4_real_validation.json
└── stage2_dual_rizon4_real_summary.json

datasets/lerobot/qiming/quest_isaac_flexiv_stage2_dual_rizon4_<stamp>/
└── videos/observation.images.cam_front/chunk-000/file-000.mp4
```

手动拆分启动时使用：

```bash
python scripts/start_dual_isaac_follow.py \
  --isaac-python "$ISAAC_PYTHON" \
  --scene-config configs/scenes/dual_rizon4_cam_front.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --enable-quest-target-udp \
  --quest-target-udp-port 57679 \
  --left-target-pose-udp-port 57680 \
  --right-target-pose-udp-port 57681 \
  --gateway-endpoint tcp://127.0.0.1:5791
```

双臂严格验证示例：

```bash
python scripts/validate_data_artifacts.py \
  --raw-dir "$STAGE2_RAW_DIR" \
  --dataset-root "$STAGE2_DATASET_ROOT" \
  --strict-dual-arm \
  --expected-left-serial "$LEFT_ROBOT_SERIAL" \
  --expected-right-serial "$RIGHT_ROBOT_SERIAL" \
  --required-camera-names cam_front \
  --required-camera-keys color_0 \
  --min-left-q-delta 0.005 \
  --min-right-q-delta 0.005 \
  --min-left-torque-norm 1e-8 \
  --min-right-torque-norm 1e-8 \
  --min-servo-cycle-delta 5
```

## Stage3 仿真任务场景

Stage3 在 Stage2 双臂闭环上增加 config-driven scene kit，不改变原始平地双臂场景。原始平地配置仍是 `configs/scenes/dual_rizon4_cam_front.yaml`；墙挂桌面任务通过以下 scene/pipeline 显式启用：

| 任务 | Scene config | Pipeline config |
| --- | --- | --- |
| pick/place red block | `configs/scenes/pick_place_redblock_flexiv_dual.yaml` | `configs/pipelines/stage3_pick_place_redblock_dual.yaml` |
| red block into drawer | `configs/scenes/pick_redblock_into_drawer_flexiv_dual.yaml` | `configs/pipelines/stage3_pick_redblock_into_drawer_dual.yaml` |
| stack R/G/Y blocks | `configs/scenes/stack_rgyblock_flexiv_dual.yaml` | `configs/pipelines/stage3_stack_rgyblock_dual.yaml` |
| move cylinder | `configs/scenes/move_cylinder_flexiv_dual.yaml` | `configs/pipelines/stage3_move_cylinder_dual.yaml` |

运行任一 Stage3 真实闭环：

```bash
python scripts/run_stage3_sim_scene_validation.py \
  --config configs/pipelines/stage3_pick_place_redblock_dual.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL"
```

不传 `--config` 时默认运行 pick/place red block。成功产物包括：

```text
datasets/stage1_records/quest_isaac_flexiv_stage3_<task>_real_<stamp>/
├── raw/episode_001/data.json
├── logs/
├── stage3_sim_scene_validation.json
└── stage3_sim_scene_summary.json

datasets/lerobot/qiming/quest_isaac_flexiv_stage3_<task>_<stamp>/
└── videos/observation.images.cam_front/chunk-000/file-000.mp4
```

Stage3 scene config 中的 `scene_objects` 支持 `usd`、`articulation`、`cuboid` 和 `cylinder`。Unitree/IsaacLab 资产通过 `${UNITREE_ASSET_ROOT}` 引用；代码会优先使用 `UNITREE_SIM_ISAACLAB_ASSETS`，否则查找相邻 workspace 的 `../unitree/unitree_sim_isaaclab/assets`。

Stage3 fake sender 使用任务 waypoint profile，例如：

```bash
python scripts/fake_rizon4_quest_sender.py \
  --dual \
  --trajectory-profile pick_place_redblock_dual \
  --amplitude-m 0.075 \
  --cycles 18 \
  --quat-wxyz 1.0,0.0,0.0,0.0
```

Stage3 验收重点是“任务场景 + fake 遥操完整闭环视频”：要求双臂、夹爪区域、桌面和任务物体清晰可见；本阶段不把真实抓取/堆叠/抽屉放置成功作为硬门槛。

## 产物说明

```text
datasets/stage1_records/<task_name>/episode_001/
├── data.json
└── colors/
    ├── 000000_color_0.jpg
    └── ...

datasets/lerobot/qiming/<task_name>/
├── data/chunk-000/*.parquet
├── meta/info.json
├── meta/episodes.jsonl
└── videos/observation.images.cam_front/chunk-000/*.mp4
```

- `data.json`：逐帧 states、actions、相机路径、时间和 Isaac/Studio 状态。
- `colors/*.jpg`：`cam_front` 原始 RGB 帧。
- `*.parquet`：LeRobot episode 表格数据。
- `*.mp4`：H264 相机视频。
- `meta/`：数据集特征、episode 索引和统计信息。
- 严格验证会检查 serial、单臂占位、相机完整性、运动量、力矩、servo cycle 和 H264 codec。
- Stage2 双臂严格验证会要求左右臂都有运动和非零力矩，不允许把右臂当作 Stage1 零占位。

## 其他说明

- 相机配置位于 `configs/scenes/single_rizon4_cam_front.yaml`。修改 `position`、`look_at` 和 `focal_length` 后重启 Isaac。
- Isaac 中可打开第二个 Viewport，并选择 `/World/cam_front` 实时查看录制画面。
- 控制与安全参数统一位于 `configs/control/quest_teleop.yaml`；Hydra 入口是 `scripts/start_isaac_follow_hydra.py`。
- Quest publisher 的平移死区默认是 `0.0`，统一由 Isaac/Hydra 的 `quest.position_deadband_m` 控制，避免两层死区叠加。
- 所有主流程 CLI 参数见 [docs/SCRIPT_PARAMETERS.md](docs/SCRIPT_PARAMETERS.md)，单个脚本也可执行 `--help`。
- `reset.coordinated=true` 时，首次启动会记录初始 TCP；recorder 的 `r` 通过 RDK 回到该 `target_pose`，只有位置、姿态和速度连续稳定后才允许继续录制，超时会报错。
- `ROBOT_SERIAL` 不能为空，并且 Studio、Isaac、Quest、RDK 和 validator 必须一致。
- 安装或缺包问题见 [SETUP.md 的环境故障排查](SETUP.md#环境故障排查)。
- 快速测试：`python -m pytest -q`。
