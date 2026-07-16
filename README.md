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
| `docker/flexiv-studio/` | 宿主机 + Docker 双 Studio 的镜像、Compose 和 runtime 准备脚本 |
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

在 Quest 浏览器打开 publisher 输出的 `https://<HOST_IP>:8012`，进入 VR 后按住右手柄 `squeeze` 控制机器人；按下对应手柄的 `trigger` 直接闭合 Isaac Sim 夹爪，松开 `trigger` 张开夹爪。夹爪控制独立于 `squeeze`，不经过 Studio/RDK。

### 6. 按 task name 录制 episode

双臂正式采集可在独立终端直接运行包装脚本：

```bash
./record.sh
```

默认任务为 `pick_place_redblock_dual`，采集 `10` 条、`30 FPS`、分辨率 `640x480`，保存到 `datasets/stage1_records`，每次保存/丢弃后请求协调 reset。recorder 保持前台运行以接收单键操作。常用覆盖方式：

```bash
TASK_NAME=another_task EPISODES=20 ./record.sh
```

也可设置 `TASK_DIR`、`OUTPUT_ROOT`、`FPS`、`IMAGE_SIZE`、`MAX_FRAMES`、`RESET_TIMEOUT_SEC`、`RESET_ON_SAVE`、`GATEWAY_ENDPOINT` 或 `ISAAC_PYTHON`。脚本检测到已有 recorder 时会拒绝重复启动。

底层等价命令如下，仍可用于单臂或自定义参数：

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
- `r`：协调 reset 两套 Studio：先停止当前控制并清除 fault，再通过 `SendJointPosition(init_q)` 平滑恢复任务初始关节位姿；两臂重新 ready 后才允许继续录制。
- `q`：退出。

手动录制命令不要添加自动开始参数。通过 `--episodes N` 指定轨迹数量；保存一条后 recorder 会继续等待 `s`，直到累计保存 N 条轨迹才退出。

recorder 启动、录制中、暂停、保存和丢弃时都会输出录制统计，包括本次完成条数、当前 episode 帧数/时长、任务目录已保存总条数/总帧数/总时长。

### 7. 实时查看双臂关节与末端位姿

双臂 Isaac 进程默认以 `10 Hz` 向本机 UDP `57684` 发布只读状态。该通道独立于 gateway/recorder，不占用采集连接，也不会向机器人发送命令。另开终端运行：

```bash
./scripts/print.sh
```

默认以 `5 Hz` 刷新，分别显示左右臂的 serial、READY 状态、7 个关节角（rad/deg）、关节速度（rad/s），以及 TCP 在机器人基座坐标系和 Isaac 世界坐标系中的位置与四元数。每一侧还会显示原始 Quest OpenXR 手柄位姿、squeeze/trigger 模拟量与按键状态、映射后的基座坐标位移和最终 TCP 目标；这些输入由独立的 `rizon4_quest_input.v1` 观测包持续发布，松开 squeeze 后仍可查看手柄状态。常用选项：

```bash
./scripts/print.sh --rate-hz 10     # 以 10 Hz 刷新
./scripts/print.sh --once           # 收到一帧后退出
./scripts/print.sh --no-clear       # 保留历史输出，不清屏
```

如果 Isaac 是在加入该功能前启动的，需要在 recorder 退出后用 `./start.sh` 冷启动一次控制栈；`start.sh` 本身不会停止正在运行的 recorder。

### 8. 转换为 LeRobot 数据集

```bash
python scripts/convert_unitree_json_to_lerobot.py \
  --raw-dir "datasets/stage1_records/$TASK_NAME" \
  --repo-id "qiming/$TASK_NAME" \
  --output-root datasets/lerobot
```

### 9. 严格验证

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

### 10. 检查与停止

```bash
python scripts/flexiv_stack_status.py
tail -f logs/*.stderr.log
python scripts/stop_flexiv_stack.py
```

## Stage2 双臂流程

Stage2 不改变 Stage1 的单臂控制方式，而是复制出两套互相独立的
`TargetFrame -> RDK -> Studio -> SimPlugin -> Isaac` 闭环。当前已经实机验证的版本为：

```text
                                      宿主机
                         ┌─────────────────────────────────┐
TargetFrameLeft  --30Hz->│ left RDK streamer               │
                         │     │                           │
                         │     └── Docker bridge ──────────┼──> Docker Studio 左臂 qSaFLh
                         │                                  │       │
TargetFrameRight --30Hz->│ right RDK streamer ─────────────┼──> 宿主机 Studio 右臂 I0LIRN
                         │                                  │       │
                         │ Isaac 双臂 Bridge <──2000Hz──────┼───────┘
                         │ gateway / recorder / Quest       │
                         └─────────────────────────────────┘
```

- 左臂：Docker Elements Studio，alias `Rizon4-qSaFLh`。
- 右臂：宿主机 Elements Studio，alias `Rizon4-I0LIRN`。
- Docker 只隔离左臂的 Studio、RobotControlApp、FlexivSimulation 和 GUI；Isaac、两个 RDK streamer、action、gateway、Quest/fake sender 和 recorder 都在宿主机。
- 不是两个 Docker Studio，也不需要把 RDK streamer 放进容器。
- 两侧 Studio 都使用 External Interface + `externalEthernetConfig.xml`，连接方法相同。
- 当前验证场景是 `standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/app_config.yaml`；Stage2 平地配置仍为 `configs/scenes/dual_rizon4_cam_front.yaml`，Stage3 任务场景见下一节。
- 场景文件里的 serial 是可移植样例。实际 alias 必须通过 CLI 或环境变量传入，不要把本机值写回共享配置。

Docker 的首次安装、镜像准备、GUI 和网络细节见
[双臂 Studio 配置](docs/dual_arm_teleop_docker_guide_zh.md)。下面给出当前版本从冷启动到手动跟随的完整最短流程。

### 一键启动双臂遥操（不含 recorder）

完成首次 Docker runtime 准备和 Quest 证书配置后，可以直接启动当前验证过的整套双臂链路：

```bash
./start.sh
```

脚本每次都会先停止上一轮 Quest、Isaac、DRDK、gateway、宿主机 Studio runtime 和 Docker 左臂容器，清理已经停止的宿主机右臂对应的旧 Flexiv 共享内存，再依次冷启动 Docker 左臂 Studio、宿主机右臂 Studio、RobotControlApp、FlexivSimulation、gateway、DRDK RobotPair、Isaac GUI 和双手 Quest publisher。已经运行的 recorder 始终保持原状态，脚本既不停止它，也不创建新的 recorder；因此 recorder 所在终端和键盘控制权不会被 `start.sh` 改动。

默认外部 Studio 使用与仓库同级的 `../elements_studio/FlexivElementsStudio`，Isaac 使用 conda 环境 `isaacsim`，Quest 使用仓库 `.venv-quest` 或 conda 环境 `tv`。其他机器通过环境变量覆盖，不在脚本中写本机绝对路径：

```bash
export STUDIO_ROOT="/path/to/FlexivElementsStudio"
export ISAAC_PYTHON="/path/to/isaacsim/python"
export QUEST_PYTHON="/path/to/quest/python"
export HOST_IP="192.168.x.x"
./start.sh
```

默认左右 alias 分别为 `Rizon4-qSaFLh` 和 `Rizon4-I0LIRN`，可用 `LEFT_ROBOT_SERIAL`、`RIGHT_ROBOT_SERIAL` 覆盖；`SCENE_CONFIG` 可选择另一份仓库相对场景配置。Docker Studio GUI 位于 `127.0.0.1:5902`。不需要手工执行停止脚本；`./start.sh` 自身就是完整冷启动入口，完成后再按任务单独启动 recorder。

### 双臂端口

| 数据 | 发送端 -> 接收端 | 默认端口 |
| --- | --- | --- |
| 左臂目标位姿 | Isaac -> 左 RDK streamer | UDP `57680` |
| 右臂目标位姿 | Isaac -> 右 RDK streamer | UDP `57681` |
| 左臂 RDK 状态/参考 TCP | 左 RDK streamer -> Isaac | UDP `57682` |
| 右臂 RDK 状态/参考 TCP | 右 RDK streamer -> Isaac | UDP `57683` |
| Quest 双手目标 | Quest publisher -> Isaac | UDP `57679` |
| gateway sample/bridge | recorder / Isaac -> gateway | TCP `5790/5791` |

同一台宿主机上不要重复启动 streamer 或 Isaac；否则 UDP 端口会被旧进程占用。

### 1. 设置环境

以下命令均在仓库根目录执行。外部安装位置只通过环境变量传入：

```bash
conda activate isaacsim
export STUDIO_ROOT="/path/to/FlexivElementsStudio"
export ISAAC_PYTHON="/path/to/isaacsim/python"
export LEFT_ROBOT_SERIAL="Rizon4-qSaFLh"
export RIGHT_ROBOT_SERIAL="Rizon4-I0LIRN"
```

先清理上一轮仓库进程；这条命令不会停止 Docker 容器：

```bash
python scripts/stop_flexiv_stack.py
```

### 2. 首次准备并启动 Docker 左臂

首次使用或宿主机 Studio 更新后执行准备脚本。它只把指定 simulator 复制到已忽略的 `.deps/`，不会修改宿主机 Studio：

```bash
docker/flexiv-studio/prepare-runtime.sh \
  --studio-root "$STUDIO_ROOT" \
  --source-simulator simulator1 \
  --target-suffix "${LEFT_ROBOT_SERIAL#Rizon4-}" \
  --force

export FLEXIV_ARM_SERIAL="A02L-00-M6-${LEFT_ROBOT_SERIAL#Rizon4-}"
docker compose -f docker/flexiv-studio/compose.yaml build
docker compose -f docker/flexiv-studio/compose.yaml up -d
docker compose -f docker/flexiv-studio/compose.yaml ps
```

容器会自动启动左臂 Studio、RobotControlApp 和 FlexivSimulation。Docker GUI 只监听本机回环地址：

```bash
vncviewer 127.0.0.1:5902
```

VNC 无密码，不要把 `5902` 发布到公网。容器健康状态应为 `healthy`；如果只需要进入 GUI 手工修改，可在启动前设置 `FLEXIV_AUTO_START_RUNTIME=0`。

### 3. 启动宿主机右臂 Studio

```bash
python scripts/start_elements_studio_ui.py --studio-root "$STUDIO_ROOT"
python scripts/start_robot_control_app.py --studio-root "$STUDIO_ROOT"
python scripts/start_flexiv_simulation.py --studio-root "$STUDIO_ROOT"
```

冷启动时当前约定是宿主机 `simulator0` 对应 `I0LIRN`，Docker `simulator1` 对应 `qSaFLh`。在继续前确认两个 Studio 都无 fault，并检查 External Interface：

```bash
rg -n '<enable>|<interface_config_file>' \
  "$STUDIO_ROOT/user_data_ui/simDir/simulator0/user_data/settings/robotExtInterfaceCfg.xml" \
  .deps/docker_studio/FlexivElementsStudio/user_data_ui/simDir/simulator1/user_data/settings/robotExtInterfaceCfg.xml
```

两侧都应显示 `<enable>1</enable>` 和 `externalEthernetConfig.xml`。

### 4A. 启动两个独立 RDK streamer（Stage1 兼容 backend）

```bash
python scripts/start_rdk_target_streamer.py \
  --python "$ISAAC_PYTHON" \
  --serial-number "$LEFT_ROBOT_SERIAL" \
  --port 57680 \
  --status-port 57682

python scripts/start_rdk_target_streamer.py \
  --python "$ISAAC_PYTHON" \
  --serial-number "$RIGHT_ROBOT_SERIAL" \
  --port 57681 \
  --status-port 57683
```

streamer 默认进入 `NRT_CARTESIAN_MOTION_FORCE`，通过 `SendCartesianMotionForce()` 把 30 Hz 离散目标交给 runtime。它不做 IK 或力矩解算。streamer 默认不自动清 fault、不在异常后自动重连，以避免错误坐标或残留进程导致机械臂反复重新上力。

### 4B. 启动 DRDK RobotPair（双臂同步 backend）

DRDK 与两个独立 RDK streamer 是互斥 backend：两者会争用相同 UDP 端口和 runtime 控制权，不能同时启动。首次使用时，在 Isaac Python 环境安装与仓库 RDK 1.9.1 兼容的固定版本；`--no-deps` 防止 pip 把 RDK 升级到不兼容版本：

```bash
"$ISAAC_PYTHON" -m pip install --no-deps flexivdrdk==1.2.1
```

然后以一个宿主机进程连接 Docker 左 runtime 和宿主机右 runtime：

```bash
python scripts/start_drdk_target_streamer.py \
  --python "$ISAAC_PYTHON" \
  --scene-config configs/scenes/pick_place_redblock_flexiv_dual.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --left-port 57680 \
  --right-port 57681 \
  --left-status-port 57682 \
  --right-status-port 57683 \
  --max-linear-speed-m-s 0.5 \
  --max-angular-speed-rad-s 0.75
```

DRDK backend 固定使用 `NRT_CARTESIAN_MOTION_FORCE`。它只在收到同一 `servo_cycle` 的左右目标后调用一次 `RobotPair.SendCartesianMotionForce()`；目标的轨迹生成、Cartesian/关节控制和力矩解算仍由两套 Flexiv runtime 完成。

streamer 从 `--scene-config` 读取左右 `robots[].initial_q`。两套 runtime 先以 scene 的 `bootstrap_q`（必须匹配 Studio home）完成 SimPlugin/DRDK 握手；随后 DRDK 进入 `NRT_JOINT_POSITION`，先用切换后的当前 q 建立无跳变指令基准，再调用 `SendJointPosition(init_q)`，由 runtime 内部运动生成器平滑移动到两侧 initq。关节位置和速度连续稳定后，DRDK 再切换 `NRT_CARTESIAN_MOTION_FORCE`，重新调用 `SetNullSpacePosture((init_q_left, init_q_right))` 并锁存当前 TCP。DRDK 与 Isaac 必须使用同一份 scene config；缺少任一侧七轴 `initial_q` 时拒绝启动。

DRDK RobotPair 的 readiness 是成对的：任一 runtime 断连、fault 或不再 operational 时，streamer 会同时向 Isaac 发布两侧 not-ready、停止发送 Cartesian 目标并等待显式 reset，不会自行清 fault。recorder 按 `r` 后，Isaac 先撤销用户控制，临时关闭 scene config 中桌面、工件等场景物体的碰撞，并把刚体工件暂时固定；机械臂 articulation 的碰撞形状始终保持不变。DRDK 随后依次执行 `RobotPair.Stop()`、`ClearFault()`、`Enable()`，再进入 `NRT_JOINT_POSITION`，先发送当前 q 建立无跳变基准，然后通过 `SendJointPosition(init_q)` 平滑回到两侧任务初始位姿。双臂 ready 后，Isaac 把全部 `scene_objects` 恢复到 scene config 的初始位置、姿态和关节状态，将刚体线速度/角速度清零，等待一个目标周期后再恢复碰撞与动力学。这样即使机械臂已经挤压桌面，清 fault 后的回程也不会立刻被同一接触再次打断，红块等任务资产也会回到下一条 episode 的初始状态。该流程不会调用 Isaac `world.reset()`，也不会瞬时写 articulation。其日志位于 `logs/drdk_target_streamer_dual*.stdout.log`。

Isaac 与两套 runtime 的 2 kHz SimPlugin 力矩闭环从各侧 SimPlugin connected 起立即运行，不等待 DRDK ready；DRDK ready 只开放 Cartesian/Quest/TargetFrame 目标。否则 runtime 在 DRDK 仍进行成对发现时收不到自身力矩响应，可能把该瞬态误判为负载超限。

### 5. 启动当前验证过的 Isaac GUI 场景

下面的命令自动播放 Timeline，并允许直接拖动左右 TargetFrame。手工拖 Frame 时不要添加 `--enable-quest-target-udp`，否则目标源会切换成 Quest packet。

```bash
python scripts/start_dual_isaac_follow.py \
  --isaac-python "$ISAAC_PYTHON" \
  --scene-config standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/app_config.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --no-manual-play \
  --no-gpu-dynamics \
  --physics-hz 2000 \
  --render-hz 30 \
  --target-pose-publish-hz 30 \
  --left-target-pose-udp-port 57680 \
  --right-target-pose-udp-port 57681 \
  --left-rdk-status-udp-port 57682 \
  --right-rdk-status-udp-port 57683
```

打开日志：

```bash
tail -f logs/rdk_target_streamer*.stdout.log \
  logs/drdk_target_streamer_dual*.stdout.log \
  logs/isaac_dual_follow_target*.stdout.log
```

正常启动顺序应包含：

1. 两个独立 RDK streamer 或一个 DRDK RobotPair streamer 监听目标端口并连接两侧 alias。
2. 两个 `SimPlugin connected`。
3. 左右 TargetFrame 初始化到各自机械臂在 `bootstrap_q` 的末端。
4. 两个 streamer 输出 `latched current TCP reference`。
5. Isaac 输出左右 `calibrated ... RDK TCP reference`。
6. 两个 RDK 状态变为 `operational`，左右 Isaac articulation 进入 effort mode。
7. DRDK 输出 `initial NRT joint trajectory started`、进度和 `initial_q reached and settled`；Isaac 在此阶段显示 `joint_initializing`，但不允许用户目标接管。
8. 输出 `READY: both task initial poses reached` 后才能开始任务；拖动某一侧 Frame 超过激活阈值后，该侧输出 `startup hold released` 并开始跟随。

在 Isaac Stage 窗口选择 `/World/TargetFrameLeft` 或 `/World/TargetFrameRight`，用移动/旋转工具拖动。不要移动机器人 articulation、墙面或 base prim。左、右 Frame 各自只控制对应机械臂。

### 切换为 Quest 控制

当前双臂 Isaac 接收端支持 Quest packet，并按 `side=left/right` 分流。停止手工模式的 Isaac app 后，用与上节相同的启动命令额外加入：

```bash
  --enable-quest-target-udp \
  --quest-target-udp-port 57679 \
  --quest-relative-orientation-mode relative \
  --quest-position-scale 1.0 \
  --quest-position-deadband-m 0.0
```

然后选择一侧启动 Quest publisher。例如右臂：

```bash
.venv-quest/bin/python scripts/rizon4_quest_target_publisher.py \
  --host-ip "$HOST_IP" \
  --serial-number "$RIGHT_ROBOT_SERIAL" \
  --udp-host 127.0.0.1 \
  --udp-port 57679 \
  --side right \
  --enable-button squeeze \
  --axis-map=-z,-x,y \
  --position-delta-scale 1.0 \
  --position-deadband 0.0 \
  --rate-hz 30
```

左臂把 serial 改为 `$LEFT_ROBOT_SERIAL`、`--side` 改为 `left`。在 Quest 浏览器打开 publisher 打印的 `https://<HOST_IP>:8012`，进入 VR 后按住对应手柄的 `squeeze` 才会发送目标；按下瞬间同时锁存当前手柄姿态与当前 RDK TCP 姿态，之后位置和方向都只跟随相对增量，首次 packet 不会造成姿态跳变。松开后保持最后的位置和方向目标。左右手柄的 `trigger` 独立控制对应 Isaac Sim 夹爪：按下闭合，松开张开。

目前 `rizon4_quest_target_publisher.py` 的一个 TeleVuer 会话只发布一侧控制器。左右臂分别都能用 Quest 调试，但不要同时启动两个 publisher：它们会争用固定的 HTTPS/WSS `8012` 端口。Isaac 接收端已经支持同一 UDP 端口上的双侧 packet；真正的同步双手 Quest 遥操还需要把 publisher 改为在一个 TeleVuer 会话中同时发布 left/right。

### 坐标对齐原理

墙挂场景中，USD articulation root 的世界变换不一定等于 FlexivSimulation/RDK 实际使用的运动学基坐标。直接用场景 root pose 做逆变换会把一个正常的 Frame 目标变成约 1 m 的 TCP 跳变，随后触发关节速度或力矩超限。

当前实现不再猜测轴映射。每个 streamer 连接后读取 Studio/RDK 的实际当前 TCP `T_base_tcp_0`；Isaac 同时读取同一时刻的末端世界位姿 `T_world_tcp_0`，建立一次参考配对：

```text
T_world_base_effective = T_world_tcp_0 * inverse(T_base_tcp_0)
T_base_tcp_target      = inverse(T_world_base_effective) * T_world_target
```

因此 TargetFrame 初始位置始终与机械臂当前末端对齐，Docker 左臂和宿主机右臂分别标定，不共享偏移。用户真正移动 Frame 前，streamer 只保持它锁存的当前 RDK TCP；不会把配置文件中的可视化初值直接发送给 Studio。

墙装 base pose 是场景物理定义，不属于标定参数，必须保持：

| 机械臂 | position `(x, y, z)` | quaternion `(w, x, y, z)` |
| --- | --- | --- |
| left | `(-0.06, 0.42, 1.08)` | `(0.70710678, 0, 0.70710678, 0)` |
| right | `(-0.06, -0.42, 1.08)` | `(0.70710678, 0, 0.70710678, 0)` |

需要调整画面或操作空间时修改相机、任务物体或 TargetFrame，不要移动墙挂机械臂 base。

### 2000 Hz 与 30 Hz 的关系

- `physics_hz=2000`：SimPlugin/FlexivSimulation 的物理闭环，步长 `physics_dt=0.0005 s`。
- `render_hz=30`：Isaac GUI/RTX 渲染频率。
- `target_pose_publish_hz=30`：TargetFrame/Quest 目标发送频率。
- gateway、recorder 和视频通常也是 30 Hz。

一次 30 Hz 的 `world.step(render=True)` 内部会执行约 66–67 个 0.5 ms 物理子步。每个物理子步都向两套 SimPlugin 发送关节状态，再接收并原样施加 Studio 返回的 `target_drives`。力矩不抽帧、不平均、不缩放，也没有额外的双臂关节速度保护。

`--no-gpu-dynamics` 只让小型刚体场景沿用 Stage1 的 CPU PhysX，避免 2000 Hz 下每个子步发生 CPU/GPU 同步；Isaac 窗口和 RTX 渲染仍在 NVIDIA GPU 上。卡顿时应减少渲染、相机或其他进程负载，不要把 2000 Hz 物理闭环直接降到 30/200 Hz。

### 常见问题

| 现象 | 原因与处理 |
| --- | --- |
| Docker Studio 没有 GUI | 先确认容器为 `healthy`，安装 VNC viewer 后连接 `127.0.0.1:5902`；同时检查 `docker compose ... logs -f studio-left`。 |
| `robot not found` | 检查 alias 是否严格为 `qSaFLh`/`I0LIRN` 对应值、两侧 External Interface 是否启用、Docker bridge 是否在线，以及 RobotControlApp/FlexivSimulation 是否都在运行。 |
| Frame 能拖但机械臂不跟随 | 手工模式不能传 `--enable-quest-target-udp`；检查是否依次出现 calibration、`operational`、effort 和 `target control armed`。还要确认没有旧 streamer/Isaac 占用端口。 |
| 启动后机械臂瞬间跳动 | 立即停止，不要继续拖动。冷启动两套 runtime，确认 `bootstrap_q` 与 Studio home 一致，并检查 DRDK 是否先进入 `joint_initializing`。非 home `initial_q` 应由 `SendJointPosition()` 的 runtime 轨迹生成器连续到达，不能瞬时写 articulation。 |
| `joint torque exceeds the limit` | 立即停止本轮；streamer 会锁存退出且不会循环清 fault。先检查坐标标定和启动顺序，再在 Studio 中人工确认状态后冷启动。以前的超限来自墙挂 root 坐标误当 RDK base 导致的大目标跳变，不是本项目新增的力矩限制。 |
| `Estimated payload ... > 10 kg` | 通常表示 Studio 在缺少或收到陈旧 SimPlugin torque 时估计出异常负载。确认两侧 SimPlugin 都连接、两臂同时进入 effort，然后冷启动，不要只反复清 fault。 |
| Isaac 很卡 | 保持 `physics_hz=2000`、`render_hz=30` 和 `--no-gpu-dynamics`；关闭多余 Isaac/Studio/streamer 进程，降低额外 viewport/相机负载。RTX 渲染使用 GPU，但 2000 Hz PhysX/SimPlugin 闭环仍会占用 CPU。 |

状态与停止命令：

```bash
python scripts/flexiv_stack_status.py
docker compose -f docker/flexiv-studio/compose.yaml ps
python scripts/stop_flexiv_stack.py
docker compose -f docker/flexiv-studio/compose.yaml down
```

发生 fault 后不要在进程仍运行时删除共享内存。只有确认宿主机右臂 RobotControlApp 和 FlexivSimulation 已停止后，才检查并清理由它们遗留的 `/dev/shm` 条目；Docker 左臂通过重启容器清理容器内部状态。

### Stage2 数据链路验收

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
  --no-gpu-dynamics \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --enable-quest-target-udp \
  --quest-target-udp-port 57679 \
  --left-target-pose-udp-port 57680 \
  --right-target-pose-udp-port 57681 \
  --left-rdk-status-udp-port 57682 \
  --right-rdk-status-udp-port 57683 \
  --gateway-endpoint tcp://127.0.0.1:5791
```

Isaac 的窗口和 RTX 渲染仍在 NVIDIA GPU 上；`--no-gpu-dynamics` 仅让小型刚体场景沿用 Stage1 的 CPU PhysX，避免 2000 Hz 时每步发生 CPU/GPU 同步。手工拖动 Frame 时不要传 `--enable-quest-target-udp`。

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

Stage3 两臂继续使用本仓库 Isaac extension 中的 `Rizon4_with_Grav.usd`。`bootstrap_q` 必须与两套 Studio 启动参数 `--group_state home` 一致，采用 `[0, -0.698132, 0, 1.5708, 0, 0.698132, 0]`；最终任务 `initial_q` 分别为左 `[-1.84, 1.839, 0.555, 2.03, 2.033, 1.777, 0]`、右 `[-1.301593, -1.71, -0.646, -1.835, -0.132, 1.924, 0]`，同时作为进入 Cartesian 模式后的零空间参考。两臂墙装 base pose 固定为左 `(-0.06, 0.20, 1.08)`、右 `(-0.06, -0.20, 1.08)`、共同绕 Y 轴 `+90°`。启动时 bridge 先以 `bootstrap_q` 完成握手，DRDK 再通过 `NRT_JOINT_POSITION` 平滑到 `initial_q`；切换 Cartesian 后锁存到位 TCP，最后才允许 Quest/TargetFrame 接管。

Flexiv SimPlugin 的物理闭环使用 2000 Hz，`render_hz`、TargetFrame/RDK 目标、gateway 和 recorder 仍为 30 Hz。Isaac `World` 会把 2000 Hz 物理子步批量放进 30 Hz 渲染步中，所以降频的是目标采样、GUI 和数据流，不是 Studio 力矩闭环。不要把 `physics_hz` 降到录像帧率；低频直接应用 Studio effort 会改变积分步长并造成关节振荡和超力矩。

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

Stage3 scene config 中的 `scene_objects` 支持 `usd`、`articulation`、`cuboid` 和 `cylinder`。Unitree/IsaacLab 资产通过 `${UNITREE_ASSET_ROOT}` 引用；代码会优先使用 `UNITREE_ASSET_ROOT` 或 `UNITREE_SIM_ISAACLAB_ASSETS`，否则查找相邻 workspace 的 `../unitree/unitree_sim_isaaclab/assets`。

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
- 双臂 DRDK 流程中，recorder 的 `r` 会触发 `Stop → ClearFault → Enable → NRT SendJointPosition(init_q) → Cartesian/null-space ready`；默认最多等待 90 秒。失败或超时会暂停当前 episode、保持 recorder 存活并报告原因；排除持续碰撞后可再次按 `r`，成功后按 `s` 继续录制。
- `ROBOT_SERIAL` 不能为空，并且 Studio、Isaac、Quest、RDK 和 validator 必须一致。
- 安装或缺包问题见 [SETUP.md 的环境故障排查](SETUP.md#环境故障排查)。
- 快速测试：`python -m pytest -q`。
