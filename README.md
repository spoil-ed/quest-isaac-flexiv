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

切换 NRT 模式后，streamer 默认读取两臂当前关节位置并调用 `SetNullSpacePosture((q_left, q_right))`，避免进入 Cartesian 控制时零空间参考发生跳变。任务确有固定零空间姿态时可传 `--left-nullspace-posture` 和 `--right-nullspace-posture`，格式均为七个逗号分隔的弧度值。

DRDK RobotPair 的 readiness 是成对的：任一 runtime 断连、fault 或不再 operational 时，streamer 会同时向 Isaac 发布两侧 not-ready 并锁存退出，不自动清 fault 或重连。其日志位于 `logs/drdk_target_streamer_dual*.stdout.log`。

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
3. 左右 TargetFrame 初始化到各自机械臂末端。
4. 两个 streamer 输出 `latched current TCP reference`。
5. Isaac 输出左右 `calibrated ... RDK TCP reference`。
6. 两个 RDK 状态变为 `operational`，左右 Isaac articulation 进入 effort mode。
7. 拖动某一侧 Frame 超过激活阈值后，该侧输出 `target control armed` 并开始跟随。

在 Isaac Stage 窗口选择 `/World/TargetFrameLeft` 或 `/World/TargetFrameRight`，用移动/旋转工具拖动。不要移动机器人 articulation、墙面或 base prim。左、右 Frame 各自只控制对应机械臂。

### 切换为 Quest 控制

当前双臂 Isaac 接收端支持 Quest packet，并按 `side=left/right` 分流。停止手工模式的 Isaac app 后，用与上节相同的启动命令额外加入：

```bash
  --enable-quest-target-udp \
  --quest-target-udp-port 57679 \
  --quest-relative-orientation-mode packet \
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

左臂把 serial 改为 `$LEFT_ROBOT_SERIAL`、`--side` 改为 `left`。在 Quest 浏览器打开 publisher 打印的 `https://<HOST_IP>:8012`，进入 VR 后按住对应手柄的 `squeeze` 才会发送目标；松开后停止更新用户目标。左右手柄的 `trigger` 独立控制对应 Isaac Sim 夹爪：按下闭合，松开张开。

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
| 启动后机械臂自己动 | 立即停止，不要继续拖动。通常是旧目标、错误坐标变换或 Studio/Isaac 初始姿态不一致；冷启动两套 runtime，确认 `initial_q` 都是 Studio 的 `home`，并等待当前 TCP 标定完成。 |
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

Stage3 两臂继续使用本仓库 Isaac extension 中的 `Rizon4_with_Grav.usd`。场景 `initial_q` 必须与两套 Studio 启动参数 `--group_state home` 一致，即 `[0, -0.6981317, 0, 1.57079632679, 0, 0.6981317, 0]`。两臂墙装 base pose 固定为左 `(-0.06, 0.42, 1.08)`、右 `(-0.06, -0.42, 1.08)`、共同绕 Y 轴 `+90°`，不得为了调整画面或避障修改；应调整相机或任务物体。启动时 bridge 会同步 Isaac 关节位置和 USD position-drive target；完成两侧 RDK 当前 TCP 标定且状态 operational 后，两臂同时切入 Studio effort 闭环。没有新 Quest/fake/Frame 目标时，streamer 保持各自锁存的当前 RDK TCP。

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
