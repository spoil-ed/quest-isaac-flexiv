# Isaac Sim Flexiv 双臂遥操与采集

本仓库连接 Quest、Isaac Sim 和两套 Flexiv Elements Studio，实现双臂遥操、场景 reset 和数据采集。

当前双臂拓扑：

```text
Quest ──90 Hz──> Isaac TargetFrame ──90 Hz──> DRDK RobotPair
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

在仓库根目录执行统一入口：

```bash
./scripts/start_all.sh
```

该命令依次启动三部分：双 Studio/Isaac/DRDK/Quest 控制栈、Web 状态监视和等待网页命令的 recorder。浏览器打开启动日志中的 `http://<HOST_IP>:8080`，即可在一个页面查看左右臂 `q/dq/τ/τ_ext`、力矩保护比例、TCP、Quest tracking/squeeze/trigger 与录制状态。`RESET 双臂 + 环境` 在健康、保护停止或 fault 状态下都可主动点击：它停止控制，以低速关节轨迹恢复双臂到 `initial_q`，随后复位 scene config 中的非机械臂资产；完成后双手同时按住 squeeze 重新标定 Quest 坐标系。

只需重启控制栈并保留已有前台 recorder 时，仍可使用 `./scripts/start.sh`；旧的 `print.sh` 和 `record.sh` 保留为诊断兼容入口，不要与 Web dashboard 同时占用状态端口。

DRDK 接触、关节力矩与碰撞参数由 `configs/pipelines/dual_arm_data_collection.yaml` 的 `control.drdk` 统一控制。双臂几何自碰撞监视默认开启，在连杆距离小于 5 cm 时先停止 RobotPair，避免互撞演变为关节力矩硬故障；按需可通过布尔环境变量临时覆盖：

启动前还需把 Elements 安全密码放入环境变量（不要写入仓库）：

```bash
export FLEXIV_SAFETY_PASSWORD='<Elements 安全密码>'
```

也可以仅在本机创建不会提交的 `.deps/runtime.env`：

```bash
FLEXIV_SAFETY_PASSWORD='<Elements 安全密码>'
```

`scripts/start.sh` 会自动加载该文件；若力矩调节器已启用但密码缺失，启动器会直接报错，不再进入只能手动 Reset 的半初始化状态。

启动器在两臂 IDLE 时把官方输出力矩调节因子设为 `0.85`，使 A3/A4 的指令饱和值从 `83.2 Nm` 降为 `54.4 Nm`；Studio 的最终 `64 Nm` 安全上限保持不变。

```bash
SELF_COLLISION_MONITOR=true ./scripts/start_all.sh
SELF_COLLISION_MONITOR=false ./scripts/start_all.sh
```

该开关只覆盖 DRDK `SelfCollisionMonitor`，不影响 pipeline 中单独开启的 TCP 接触 wrench 限制和关节力矩前置保护。关节保护比较 `tau`、`tau_ext` 及短期预测力矩与 runtime `tau_max`；runtime 未提供有效 `tau_dot` 时会根据连续 `tau` 样本估算变化率。达到 72% 时回退到最近安全 target；降到 55% 以下稳定 0.15 s 后重建相对零点并恢复。

按任务名选择 `configs/scenes` 中的场景：

```bash
./scripts/start_all.sh --task pick_place_redblock_flexiv_dual
```

也可显式传入场景：

```bash
SCENE_CONFIG=configs/scenes/move_cylinder_flexiv_dual.yaml ./scripts/start_all.sh
```

每条机械臂的矩形 TCP 工作区直接在 scene YAML 的 `robots[].workspace` 中设置，坐标单位为米、坐标系为 Isaac world。启动后 Isaac 视口会显示对应颜色的 12 边线框，Quest 原始目标越界时，实际命令目标会被裁剪到边界：

```yaml
workspace:
  enabled: true
  frame: world
  min: {x: 0.20, y: -0.05, z: 0.25}
  max: {x: 0.85, y: 0.65, z: 1.05}
  visualize: true
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

Quest tracking、双手按键、共享坐标系状态和录制状态直接显示在 Web 页面的“采集门控”区域。

两只手柄均出现 tracking 数据后，先把双手距离调整到标定当下两条机械臂实际 TCP 间距的 ±3 cm；姿态不检查。Isaac 仅在双臂 READY 时实时提供该距离，距离显示 `PASS` 后同时按住双手 `squeeze` 0.25 s 即可锁定共享坐标系。系统在锁定坐标系的同一帧分别建立手柄和当前 TCP 零点，第一帧增量严格为零，随后无需松手即可直接相对跟随。

- 按住单侧 `squeeze`：该侧机械臂按相对位置和相对姿态跟随；
- 松开 `squeeze`：立即 hold 松开时实测 TCP，不继续追旧目标，也不回初始位置；
- 按住 `trigger`：对应 Isaac 夹爪闭合；松开后张开。

### 3. 开始采集

启动前可用环境变量设置录制任务：

```bash
TASK_NAME=pick_place_redblock_dual EPISODES=20 FPS=30 ./scripts/start_all.sh
```

Web 录制按钮：

- `开始 / 继续`：新建 episode 或继续暂停中的 episode；
- `暂停`：停止写帧但保留当前 episode；
- `保存本条`：直接完成并保存当前 episode；
- `丢弃本条`：删除当前 episode；
- `Reset`：执行协调 reset；
- `停止录制器`：安全退出；非空的录制中 episode 会按原 recorder 规则保存。

数据保存到 `datasets/stage1_records/<task_name>/`，后台日志位于 `logs/`。

## 遇到问题怎么办

先看 `http://<HOST_IP>:8080` 的在线状态、保护曲线和错误栏；需要底层日志时：

```bash
tail -f "$(ls -t logs/isaac_dual_follow_target*.stdout.log | head -n1)"
tail -f "$(ls -t logs/drdk_target_streamer_dual*.stdout.log | head -n1)"
tail -f "$(ls -t logs/web_recorder*.stdout.log | head -n1)"
```

Web 页面依次显示左右臂 J1–J7 的 `q`、`dq`、实测 `tau` 和保护实际采用的最大风险比例；实时曲线中的红色虚线 `0.72` 是关节保护触发线。

| 现象 | 处理 |
| --- | --- |
| Quest 页面无信息或手柄无输入 | 在 Quest 中确认 Wi-Fi 已连接到与宿主机相同且可互通的局域网；随后刷新浏览器中的 `https://<HOST_IP>:8012` 页面并重新进入会话。 |
| 左臂或右臂不动 | 确认两套 Studio 模拟机器人均已启动，alias 与 scene 一致；检查日志中左右两侧是否均为 `READY`。 |
| Quest 有数据但不跟随 | 确认 Quest tracking 为 `PASS`，把双手距离调整到目标 ±3 cm；距离 `PASS` 后同时按住双手 squeeze 0.25 s。相对姿态不参与门控。 |
| Web 页面打不开 | 检查 `ss -ltnp | grep :8080` 和最新 `logs/web_control_dashboard*.stderr.log`；页面默认只应在可信局域网内使用。 |
| 连接被拒绝或进程状态混乱 | 再次执行 `./scripts/start_all.sh --task <task>`，它会清理旧控制栈、旧 Web recorder 和 dashboard 后冷启动。 |
| Docker GUI 不显示 | 检查 `docker ps` 和 `docker logs flexiv-studio-left`；本机缺少客户端时安装 `tigervnc-viewer`。 |
| 关节力矩保护触发 | 松开 squeeze，等待状态中的 `joint_torque_frozen` 自动解除；保护会回退安全 target，不需要 reset。 |
| 页面显示 `self_collision_stopped` | 松开双手 squeeze，点击 Web `Reset`；恢复过程临时停止几何监视、用 NRT 关节轨迹把两臂分开并回到 `initial_q`，落位后自动重新启用监视。 |
| 已发生 Studio 力矩 fault | 先松开 squeeze，再按 recorder 的 `r`。若 `ClearFault` 持续失败，说明仍有接触或穿插，需冷重启恢复；不要瞬移关节或调用 `world.reset()`。 |
| reset 第一次未完成 | 保持机械臂周围无新障碍，再按一次 `r`；recorder 会保持运行。查看 DRDK 日志中的具体关节和恢复阶段。 |
| Isaac 很卡 | 确认使用 NVIDIA GPU，并关闭无关 Isaac/Studio 进程。保持 `physics_hz=2000`，只降低渲染、目标和采集频率。 |

协调 reset 的完成条件严格为：两臂停止并清除 fault，以 NRT 关节轨迹回到 `initial_q`，切回 Cartesian 并重装零空间，随后复位非机械臂资产、清零刚体速度并恢复碰撞。回程再次触发保护时会以更低速度从实际关节位置继续，单个 reset 最多尝试 3 次；超过次数才报告失败，不会把半恢复视为成功。

## 参数调节方法

优先修改任务 scene，不要直接改 USD 或在代码中写绝对路径：

```text
configs/scenes/<task>.yaml
```

全仓库只保留一份运行管线 `configs/pipelines/dual_arm_data_collection.yaml`；任务差异通过 scene config 或 `start_all.sh --task` 传入，不再维护 Stage1/2/3 pipeline 副本。

| 目标 | 参数或位置 | 默认值/说明 |
| --- | --- | --- |
| 选择任务 | `./scripts/start_all.sh --task <task.name>` | 按 YAML 中的 `task.name` 精确匹配 |
| 外部 Studio | `STUDIO_ROOT` | 默认 `../elements_studio/FlexivElementsStudio` |
| 左右 alias | `LEFT_ROBOT_SERIAL`、`RIGHT_ROBOT_SERIAL` | `Rizon4-qSaFLh`、`Rizon4-I0LIRN` |
| 机器人初始姿态 | scene 的 `robots[].initial_q` | DRDK 到位后也作为零空间参考 |
| 冷启动中间点 | scene 的 `robots[].initial_q_waypoints` | 只用于冷启动 |
| 机器人握手姿态 | scene 的 `robots[].bootstrap_q` | 必须与 Studio home 一致 |
| 场景资产初始状态 | scene 的 `scene_objects` | reset 时恢复位置、姿态和关节状态 |
| Quest/TargetFrame 线速度 | pipeline 的 `control.max_linear_speed_m_s` | Studio normal safety 上限 `3.0 m/s` |
| 角速度 | pipeline 的 `control.max_angular_speed_rad_s` | Studio normal safety 上限 `12.0 rad/s` |
| 线/角加速度 | pipeline 的 `control.max_*_acc_*` | `12.0 m/s²` / `45 rad/s²` |
| 目标重采样 | pipeline 的 `control.drdk.target_resampling` | 90 Hz 输入、1000 Hz NRT 输出、10 ms 预测、全量有界前馈 |
| 初始化关节速度/加速度 | pipeline 的 `control.initial_joint_*` | 统一速度 `2.0944 rad/s` / `2.0 rad/s²` |
| reset 关节速度/加速度 | pipeline 的 `control.reset_joint_*` | 统一速度 `2.0944 rad/s` / `3.0 rad/s²` |
| reset 重试 | `--reset-max-attempts` / `--reset-retry-delay-sec` | `3` 次 / `0.5 s` |
| Isaac 关节力矩上限 | pipeline 的 `control.joint_effort_limits_nm` | 匹配 Studio 已有配置：`150,150,80,80,49,49,49 Nm` |
| TCP 接触 wrench | pipeline 的 `control.drdk.contact_wrench` | 每侧 `20 N / 3 Nm`，独立于关节硬上限 |
| 物理/渲染频率 | `--physics-hz` / `--render-hz` | `2000 Hz` / `30 Hz` |
| 目标/采集频率 | `--target-pose-publish-hz` / `FPS` | `90 Hz` / `30 FPS` |

DRDK 参数由 `scripts/start_drdk_target_streamer.py` 传给 streamer；Isaac 参数由 `scripts/start_dual_isaac_follow.py` 传入。修改启动参数后必须重新执行 `scripts/start.sh`。

调参原则：

1. 先调整 scene 中的资产与 `initial_q`，确认无碰撞；
2. 再小幅调整速度和加速度；
3. 始终保持 Studio/SimPlugin 的 `2000 Hz` 物理闭环；
4. 用 `print.sh` 和日志验证左右臂均 READY 后再采集。

完整 CLI 参数见 [脚本参数说明](docs/SCRIPT_PARAMETERS.md)。
