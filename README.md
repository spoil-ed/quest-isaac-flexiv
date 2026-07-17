# Isaac Sim Flexiv 双臂遥操与采集

本仓库连接 Quest、Isaac Sim 和两套 Flexiv Elements Studio，实现双臂遥操、场景 reset 和数据采集。

当前双臂拓扑：

```text
Quest ──30 Hz──> Isaac TargetFrame ──30 Hz──> DRDK RobotPair
                                                   │
                           ┌───────────────────────┴───────────────────────┐
                           │                                               │
                 Docker Studio 左臂                              宿主机 Studio 右臂
                  Rizon4-qSaFLh                                  Rizon4-I0LIRN
                           │                                               │
                           └──────── SimPlugin 2 kHz 力矩闭环 ─────────────┘
                                                   │
                                                Isaac Sim
```

控制逻辑和坐标映射见 [控制架构](spec/architecture/control_architecture_zh.md)，双 Studio 配置见 [Docker 双臂指南](docs/dual_arm_teleop_docker_guide_zh.md)。

## 准备工作

要求：

- Ubuntu 22.04、NVIDIA GPU 和可用驱动；
- Docker Engine 与 Compose 插件；
- Conda 环境 `isaacsim`；
- Flexiv Elements Studio；
- Quest 与宿主机位于可互通网络，仅采集或 TargetFrame 调试时可不连接 Quest。

首次安装按 [SETUP.md](SETUP.md) 操作。完成后检查：

```bash
conda activate isaacsim
python -c 'import omni, yaml, numpy; print("isaacsim env OK")'
docker info >/dev/null && docker compose version
nvidia-smi
```

两套 Studio 必须使用不同 alias：

| 侧别 | 运行位置 | alias |
| --- | --- | --- |
| 左臂 | Docker | `Rizon4-qSaFLh` |
| 右臂 | 宿主机 | `Rizon4-I0LIRN` |

默认外部 Studio 位于仓库同级目录 `../elements_studio/FlexivElementsStudio`。不同安装位置通过环境变量传入，不要把本机绝对路径写入代码或配置：

```bash
export STUDIO_ROOT=/path/to/FlexivElementsStudio
export ISAAC_CONDA_ENV=isaacsim
export HOST_IP=<Quest可访问的宿主机IP>
```

## 启动方案

### 1. 启动控制栈

在仓库根目录执行：

```bash
./scripts/start.sh
```

脚本会停止旧控制进程并重新启动 Docker 左 Studio、宿主机右 Studio、gateway、Isaac GUI、DRDK 和 Quest publisher；已经运行的 recorder 不会被停止。

DRDK 接触、关节力矩与碰撞参数由 `configs/pipelines/dual_arm_data_collection.yaml` 的 `control.drdk` 统一控制。双臂几何自碰撞监视默认关闭；按需通过布尔环境变量临时覆盖：

```bash
SELF_COLLISION_MONITOR=true ./scripts/start.sh
SELF_COLLISION_MONITOR=false ./scripts/start.sh
```

该开关只覆盖 DRDK `SelfCollisionMonitor`，不影响 pipeline 中单独开启的 TCP 接触 wrench 限制和关节力矩前置保护。关节保护比较 `tau`、`tau_ext` 及由 `tau_dot` 外推的短期力矩与 runtime `tau_max`：达到 85% 时回退到最近安全 target；降到 70% 以下稳定 0.30 s 后重建相对零点并恢复。

按任务名选择 `configs/scenes` 中的场景：

```bash
./scripts/start.sh --task pick_place_redblock_flexiv_dual
```

也可显式传入场景：

```bash
SCENE_CONFIG=configs/scenes/move_cylinder_flexiv_dual.yaml ./scripts/start.sh
```

启动后确认两套 Studio 的模拟机器人均已启动。Docker Studio GUI 默认地址为 `127.0.0.1:5902`：

```bash
vncviewer 127.0.0.1:5902
```

### 2. 连接并标定 Quest

Quest 浏览器打开启动日志中的：

```text
https://<HOST_IP>:8012/?ws=wss://<HOST_IP>:8012
```

另开终端检查输入：

```bash
./scripts/print.sh
```

该命令会同时打开双臂实时状态窗口，按左右臂两列显示 `q`、`dq`、实测力矩和保护比例；终端状态和图形窗口通过独立 UDP 端口接收数据，退出 `print.sh` 时图形窗口也会关闭。默认使用 `isaacsim` conda 环境，可通过 `PRINT_PYTHON` 覆盖。

将左右手柄间距保持在 `0.40±0.01 m`，并使两只手柄在水平面内同向且垂直于左右连线。`SPACING`、`DIRECTION` 均为绿色 `PASS` 后，同时按住双手 `squeeze` 固定 Quest 到机器人坐标系。

- 按住单侧 `squeeze`：该侧机械臂按相对位置和相对姿态跟随；
- 松开 `squeeze`：保持最后目标，不回初始位置；
- 按住 `trigger`：对应 Isaac 夹爪闭合；松开后张开。

### 3. 开始采集

控制栈 READY 后，在独立终端运行：

```bash
./scripts/record.sh
```

常用覆盖：

```bash
TASK_NAME=pick_place_redblock_dual EPISODES=20 FPS=30 ./scripts/record.sh
```

按键：

- `s`：开始或继续；
- `e`：暂停，再按一次保存；
- `d`：丢弃当前 episode；
- `r`：协调 reset；
- `q`：退出。

数据保存到 `datasets/stage1_records/<task_name>/`，后台日志位于 `logs/`。

## 遇到问题怎么办

先看实时状态和最新日志：

```bash
./scripts/print.sh --verbose
tail -f "$(ls -t logs/isaac_dual_follow_target*.stdout.log | head -n1)"
tail -f "$(ls -t logs/drdk_target_streamer_dual*.stdout.log | head -n1)"
```

状态窗口依次显示左右臂 J1–J7 的 `q`、`dq`、实测 `tau` 和保护实际采用的最大风险比例；红色虚线 `0.85` 是关节保护触发线。

| 现象 | 处理 |
| --- | --- |
| Quest 页面无信息或手柄无输入 | 在 Quest 中确认 Wi-Fi 已连接到与宿主机相同且可互通的局域网；随后刷新浏览器中的 `https://<HOST_IP>:8012` 页面并重新进入会话。 |
| 左臂或右臂不动 | 确认两套 Studio 模拟机器人均已启动，alias 与 scene 一致；检查日志中左右两侧是否均为 `READY`。 |
| Quest 有数据但不跟随 | 先让 `print.sh` 两行均为 `PASS`，双 squeeze 完成坐标标定，再按住对应侧 squeeze。 |
| 连接被拒绝或进程状态混乱 | 再次执行 `./scripts/start.sh --task <task>`，它会清理旧控制栈后冷启动。 |
| Docker GUI 不显示 | 检查 `docker ps` 和 `docker logs flexiv-studio-left`；本机缺少客户端时安装 `tigervnc-viewer`。 |
| 关节力矩保护触发 | 松开 squeeze，等待状态中的 `joint_torque_frozen` 自动解除；保护会回退安全 target，不需要 reset。 |
| 已发生 Studio 力矩 fault | 先松开 squeeze，再按 recorder 的 `r`。若 `ClearFault` 持续失败，说明仍有接触或穿插，需冷重启恢复；不要瞬移关节或调用 `world.reset()`。 |
| reset 第一次未完成 | 保持机械臂周围无新障碍，再按一次 `r`；recorder 会保持运行。查看 DRDK 日志中的具体关节和恢复阶段。 |
| Isaac 很卡 | 确认使用 NVIDIA GPU，并关闭无关 Isaac/Studio 进程。保持 `physics_hz=2000`，只降低渲染、目标和采集频率。 |

协调 reset 的完成条件严格为：两臂停止并清除 fault，以 NRT 关节轨迹回到 `initial_q`，切回 Cartesian 并重装零空间，随后复位非机械臂资产、清零刚体速度并恢复碰撞。回程再次触发保护时会以更低速度从实际关节位置继续，单个 reset 最多尝试 3 次；超过次数才报告失败，不会把半恢复视为成功。

## 参数调节方法

优先修改任务 scene，不要直接改 USD 或在代码中写绝对路径：

```text
configs/scenes/<task>.yaml
```

全仓库只保留一份运行管线 `configs/pipelines/dual_arm_data_collection.yaml`；任务差异通过 scene config 或 `start.sh --task` 传入，不再维护 Stage1/2/3 pipeline 副本。

| 目标 | 参数或位置 | 默认值/说明 |
| --- | --- | --- |
| 选择任务 | `./scripts/start.sh --task <task.name>` | 按 YAML 中的 `task.name` 精确匹配 |
| 外部 Studio | `STUDIO_ROOT` | 默认 `../elements_studio/FlexivElementsStudio` |
| 左右 alias | `LEFT_ROBOT_SERIAL`、`RIGHT_ROBOT_SERIAL` | `Rizon4-qSaFLh`、`Rizon4-I0LIRN` |
| 机器人初始姿态 | scene 的 `robots[].initial_q` | DRDK 到位后也作为零空间参考 |
| 冷启动中间点 | scene 的 `robots[].initial_q_waypoints` | 只用于冷启动 |
| 机器人握手姿态 | scene 的 `robots[].bootstrap_q` | 必须与 Studio home 一致 |
| 场景资产初始状态 | scene 的 `scene_objects` | reset 时恢复位置、姿态和关节状态 |
| Quest/TargetFrame 线速度 | `--max-linear-speed-m-s` | 当前启动方案为 `1.0 m/s` |
| 角速度 | `--max-angular-speed-rad-s` | 当前为 `0.75 rad/s` |
| 初始化关节速度/加速度 | `--initial-joint-max-vel-rad-s` / `--initial-joint-max-acc-rad-s2` | `0.5 rad/s` / `1.0 rad/s²` |
| reset 关节速度/加速度 | `--reset-joint-max-vel-rad-s` / `--reset-joint-max-acc-rad-s2` | `0.2 rad/s` / `0.4 rad/s²` |
| reset 重试 | `--reset-max-attempts` / `--reset-retry-delay-sec` | `3` 次 / `0.5 s` |
| Isaac 关节力矩上限 | pipeline 的 `control.joint_effort_limits_nm` | 匹配 Studio 已有配置：`150,150,80,80,49,49,49 Nm` |
| TCP 接触 wrench | pipeline 的 `control.drdk.contact_wrench` | 每侧 `20 N / 3 Nm`，独立于关节硬上限 |
| 物理/渲染频率 | `--physics-hz` / `--render-hz` | `2000 Hz` / `30 Hz` |
| 目标/采集频率 | `--target-pose-publish-hz` / `FPS` | `30 Hz` / `30 FPS` |

DRDK 参数由 `scripts/start_drdk_target_streamer.py` 传给 streamer；Isaac 参数由 `scripts/start_dual_isaac_follow.py` 传入。修改启动参数后必须重新执行 `scripts/start.sh`。

调参原则：

1. 先调整 scene 中的资产与 `initial_q`，确认无碰撞；
2. 再小幅调整速度和加速度；
3. 始终保持 Studio/SimPlugin 的 `2000 Hz` 物理闭环；
4. 用 `print.sh` 和日志验证左右臂均 READY 后再采集。

完整 CLI 参数见 [脚本参数说明](docs/SCRIPT_PARAMETERS.md)。
