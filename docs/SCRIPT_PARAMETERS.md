# 脚本参数说明

本文档解释 README 主流程中脚本暴露的 CLI 参数。所有脚本都可执行 `python <script> --help` 查看当前参数；命令行参数优先于环境变量和配置文件。

## Studio runtime

### `start_robot_control_app.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--studio-root` | `STUDIO_ROOT` 或 `FlexivElementsStudio` | Elements Studio 安装根目录。 |
| `--serial` | 自动发现 | Studio 内部机器人序列号，例如 `A02L-00-M6-I0LIRN`，不是 RDK 使用的 `Rizon4-*` 别名。 |
| `--control-box` | `CX01-02-P1-00034` | 模拟控制箱序列号；通常不修改。 |
| `--user-data` | simulator0 的 user_data | RobotControlApp 用户数据目录。 |
| `--config` | 从 `specs/robots` 自动发现 | `flexivCfg.xml` 路径；自动发现失败时显式指定。 |

### `start_flexiv_simulation.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--studio-root` | `STUDIO_ROOT` | Elements Studio 安装根目录。 |
| `--robot-urdf` | 自动发现 | Studio 生成的机器人 URDF。 |
| `--robot-srdf` | 自动发现 | Studio 生成的机器人 SRDF。 |
| `--scene-urdf` | 自动发现 | Studio 生成的场景 URDF。 |
| `--param` | 自动发现 | `arm_driver_param.xml`。 |
| `--group-state` | `home` | SRDF 初始 group state。 |

### `start_elements_studio_ui.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--studio-root` | `STUDIO_ROOT` | Elements Studio 安装根目录。 |

## 数据 gateway

### `start_data_gateway.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--sample-endpoint` | `tcp://0.0.0.0:5590` | recorder 请求样本、保存后发送 reset 的 TCP 地址。主流程使用 `5690`。 |
| `--bridge-endpoint` | `tcp://0.0.0.0:5591` | Isaac 推送相机和机器人状态的 TCP 地址。主流程使用 `5691`。 |
| `--backend` | `bridge` | `bridge` 接真实 Isaac；`fake` 只用于数据工具链 smoke。 |
| `--fps` | `30` | fake backend 生成频率及其时间基准；真实 bridge 的相机频率由 Isaac 控制。 |
| `--image-size` | `640x480` | fake backend 图像尺寸。 |
| `--camera-keys` | schema 中全部相机键 | 逗号分隔的颜色键；单相机流程使用 `color_0`。 |
| `--fake-sim-backend` | `fake` | fake backend 写入 `sim_state.backend`；Stage2 smoke 使用 `quest_isaac_flexiv_stage2_dual`。 |
| `--fake-left-serial/right-serial` | 空 | fake backend 写入左右 serial，用于 Stage2 strict dual smoke。 |

## RDK target streamer

### `start_rdk_target_streamer.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--python` | `ISAAC_PYTHON` | 运行 RDK streamer 的 Python；空字符串自动回退到当前解释器。 |
| `--host` | `127.0.0.1` | 接收 Isaac 目标位姿的本机 UDP 地址。 |
| `--port` | `45678` | 目标位姿 UDP 端口；README 主流程使用 `55678`。 |
| `--serial-number` | 兼容默认 serial | RDK/Studio 模拟机器人序列号，推荐始终传 `$ROBOT_SERIAL`。 |
| `--joint-group` | `ARM_1` | Studio 关节组。 |
| `--network-interface-whitelist` | 空 | 限定 RDK 发现使用的本机 IPv4，多个地址用逗号分隔。 |
| `--max-age-sec` | `0.5` | 丢弃超过该时长的 Isaac 目标包。 |
| `--log-hz` | `2` | 目标位姿日志频率；小于等于 0 关闭周期日志。 |
| `--status-host/port` | 空/`0` | 向双臂 Isaac 回报 RDK operational；Stage2 左右使用 `57682/57683`。 |
| `--clear-fault/--no-clear-fault` | 不清故障 | 默认保留故障现场，不自动清除 Studio/RDK fault。 |
| `--reconnect-on-error/--no-reconnect-on-error` | 不重连 | 默认故障锁存退出；仅诊断时显式允许重连。 |

该脚本固定使用 `NRT_CARTESIAN_MOTION_FORCE` 和 `SendCartesianMotionForce()`；30 Hz 客户端不使用 RT streaming API，轨迹生成与控制解算由 Flexiv runtime 完成。

### `start_drdk_target_streamer.py`

双臂 DRDK backend，与两个 `start_rdk_target_streamer.py` 进程互斥。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--python` | `ISAAC_PYTHON` | 运行 DRDK streamer 的 Python。 |
| `--scene-config` | 必填 | 与 Isaac 相同的双臂 scene YAML；左右 `robots[].initial_q` 是 `SetNullSpacePosture()` 的任务 initq。 |
| `--left/right-serial-number` | Stage2 左右 alias | DRDK `RobotPair` 连接的两个不同 runtime alias。 |
| `--left/right-port` | `57680/57681` | 接收 Isaac 左右目标的 UDP 端口。 |
| `--left/right-status-port` | `57682/57683` | 向 Isaac 返回左右 ready、参考 TCP 和当前 TCP。 |
| `--left/right-translation-in-world` | `0,0,0` | DRDK RobotPair 世界坐标中的 base 平移；当前 base-frame 目标链保持零值。 |
| `--nullspace-tracking-weight` | `0.5` | 两臂参考关节姿态跟踪权重，范围 `[0.1, 1.0]`。 |
| `--max-linear-speed-m-s` | `0.5` | NRT runtime 轨迹生成器的最大线速度。 |
| `--max-angular-speed-rad-s` | `0.75` | NRT runtime 轨迹生成器的最大角速度。 |
| `--max-linear-acc-m-s2` | `2.0` | 最大线加速度。 |
| `--max-angular-acc-rad-s2` | `5.0` | 最大角加速度。 |
| `--network-interface-whitelist` | 空 | DRDK 发现允许使用的本机 IPv4，逗号分隔。 |
| `--connect-timeout-sec` | `30` | 仅启动阶段等待 SimPlugin 使两套 runtime 可发现；连接成功后不用于故障重连。 |
| `--initial-joint-timeout-sec` | `45` | NRT joint-position 初始化总超时。 |
| `--initial-joint-handoff-sec` | `0.5` | 切换 NRT joint mode 后，以切换后的当前 q 建立无跳变指令基准并保持的时间。 |
| `--initial-joint-settle-sec` | `0.5` | 关节位置和速度进入容差后必须连续稳定的时间。 |
| `--initial-joint-tolerance-rad` | `0.02` | DRDK 判断两臂到达 `initial_q` 的最大关节位置误差。 |
| `--initial-joint-speed-tolerance-rad-s` | `0.03` | 初始化完成的最大关节速度。 |
| `--initial-joint-max-vel-rad-s` | `0.5` | `SendJointPosition()` 初始化轨迹的各关节最大速度。 |
| `--initial-joint-max-acc-rad-s2` | `1.0` | 初始化轨迹的各关节最大加速度。 |
| `--clear-fault/--no-clear-fault` | 不清故障 | 默认保留故障现场。 |

该脚本启动时短暂使用 `NRT_JOINT_POSITION + SendJointPosition()` 平滑到 scene config 的左右 `initial_q`；到位后固定使用 `NRT_CARTESIAN_MOTION_FORCE`，并重新把 `initial_q` 设置为零空间参考。关节轨迹、IK、动力学和力矩仍由 runtime 处理；任一侧故障会使 RobotPair 两侧同时 not-ready 并退出。

## Isaac 控制入口

### `start_isaac_follow.py`

运行时和场景参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--isaac-python` | `ISAAC_PYTHON` | Isaac Sim Python；空字符串回退到当前解释器。 |
| `--serial-number` | 兼容默认 serial | SimPlugin 节点序列号，必须与 Studio/RDK/Quest 一致。 |
| `--rdk-serial-number` | 同 `--serial-number` | 仅直接 RDK 模式使用的序列号。 |
| `--joint-group` | scene/app 默认 | Studio 关节组。 |
| `--scene-config` | 无 | 机器人、USD、extension 和相机 YAML；推荐使用单 Rizon4 scene。 |
| `--robot-prim-path` | scene 默认 | Rizon4 USD prim 路径。 |
| `--robot-name` | scene 默认 | Isaac scene object 名称。 |
| `--end-effector-prim-name` | scene 默认 | 末端 link/prim 名称。 |
| `--usd` | scene 默认 | 覆盖 Rizon4 USD 文件。 |
| `--examples-ext` | scene 默认 | 覆盖 Flexiv Isaac examples extension。 |
| `--manual-play/--no-manual-play` | manual play | 是否等待用户点击 Isaac Timeline Play。自动启动使用 `--no-manual-play`。 |
| `--headless` | 关闭 | 无 GUI 运行。 |
| `--physics-hz` | app 默认 | 物理回调频率。 |
| `--render-hz` | app 默认 | 渲染频率。 |

Quest 输入参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--enable-quest-target-udp` | 关闭 | 开启 Quest 目标 UDP 监听。 |
| `--quest-target-udp-host` | app 默认 | Quest UDP 绑定地址。 |
| `--quest-target-udp-port` | app 默认 `45679` | Quest UDP 端口；README 使用 `55679`。 |
| `--quest-target-max-age-sec` | app 默认 | Quest 包最大年龄。 |
| `--quest-target-mode` | `relative` | `relative` 按压时锚定当前 TCP；`absolute` 直接使用包内位姿。 |
| `--quest-axis-map` | app 默认 | OpenXR 到机器人基座轴映射，例如 `-z,-x,y`。 |
| `--quest-position-scale` | `1.0` | Quest 位移缩放。 |
| `--quest-position-deadband-m` | app 默认 | Isaac 端平移死区，单位米。 |
| `--quest-workspace-min/max` | app 默认 | 基座坐标系 TCP 工作空间下限/上限，格式 `x,y,z`。 |

输出、控制与安全参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--target-pose-udp-host/port` | app 默认 | Isaac 发给 RDK streamer 的目标位姿地址。 |
| `--target-pose-publish-hz` | app 默认 | 目标位姿 UDP 发布频率，不等于机器人速度。 |
| `--rdk-target-hz` | app 默认 | RDK 目标更新频率。 |
| `--command-timeout-ms` | app 默认 | 等待 SimPlugin 命令的单周期超时。 |
| `--max-linear-speed-m-s` | app 默认 | TCP 平移命令速度上限。 |
| `--max-angular-speed-rad-s` | app 默认 | TCP 旋转命令速度上限。 |
| `--max-target-drive-abs` | app 默认 | 单关节 target drive 绝对值上限。 |
| `--max-target-drive-norm` | app 默认 | target drive 向量范数上限。 |
| `--gateway-endpoint` | 空 | 非空时创建 scene 相机并向 gateway 推送数据。 |
| `--gateway-fps` | app 默认 | gateway 图像/状态推送频率。 |
| `--gateway-jpeg-quality` | app 默认 | JPEG 质量，范围通常为 0–100。 |
| `--camera-config` | scene config | 旧相机配置兼容入口；新流程使用 `--scene-config`。 |
| `--coordinated-reset/--no-coordinated-reset` | 开启 | 是否接收 recorder/gateway reset 并复用启动初始化。 |
| `--reset-settle-sec` | `2.0` | TCP 落入容差后必须连续稳定的时间。 |
| `--reset-timeout-sec` | `20.0` | RDK 未能在该时间内落位则 reset 失败。 |
| `--reset-position-tolerance-m` | `0.01` | reset 完成的 TCP 位置误差阈值。 |
| `--reset-angular-tolerance-rad` | `0.10` | reset 完成的 TCP 姿态误差阈值。 |
| `--reset-joint-speed-tolerance-rad-s` | `0.05` | reset 完成的最大关节速度阈值。 |

### `start_isaac_follow_hydra.py`

Hydra 参数来自 `configs/control/quest_teleop.yaml`，使用 `键=值` 覆盖：

| 配置组 | 说明 |
| --- | --- |
| `robot.*` | serial 和 joint group。 |
| `runtime.*` | Isaac/RDK Python。 |
| `scene.config` | scene YAML。 |
| `control.*` | physics、render、RDK/UDP 频率和超时。 |
| `quest.*` | 模式、轴映射、缩放、死区、工作空间和包年龄。 |
| `safety.*` | TCP/关节速度及 target drive 上限。 |
| `reset.*` | 协调 reset 和稳定时间。 |
| `network.*` | Quest 与目标位姿 UDP 地址。 |
| `gateway.*` | 数据 bridge 地址、fps 和 JPEG 质量。 |
| `launch.*` | headless、manual play、是否启动 RDK、dry run。 |

示例：`python scripts/start_isaac_follow_hydra.py robot.serial_number="$ROBOT_SERIAL" safety.max_linear_speed_m_s=0.05`。

### `start_dual_isaac_follow.py`

Stage2 双臂 Isaac 启动入口，参数语义与单臂入口一致，差异如下：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--scene-config` | 无 | 推荐使用 `configs/scenes/dual_rizon4_cam_front.yaml`。 |
| `--left-serial-number/right-serial-number` | scene 或本机样例 | 左右 SimPlugin/RDK serial。 |
| `--left-target-pose-udp-host/port` | `127.0.0.1:57680` | 左臂发给 RDK streamer 的目标位姿地址。 |
| `--right-target-pose-udp-host/port` | `127.0.0.1:57681` | 右臂发给 RDK streamer 的目标位姿地址。 |
| `--left-rdk-status-udp-host/port` | `127.0.0.1:57682` | 左臂 streamer operational/fault 回报地址。 |
| `--right-rdk-status-udp-host/port` | `127.0.0.1:57683` | 右臂 streamer operational/fault 回报地址。 |
| `--rdk-status-max-age-sec` | `1.0` | status 超过该时间即视为掉线并退回 position hold。 |
| `--target-activation-position-tolerance-m` | `1e-3` | 手动 Frame 世界坐标平移超过此值后才请求启用对应 RDK 控制。 |
| `--target-activation-orientation-tolerance-rad` | `0.00873` | 手动 Frame 世界坐标旋转超过约 `0.5°` 后才请求启用对应 RDK 控制。 |
| `--startup-joint-tolerance-rad` | `0.03` | DRDK 报告到位后，Isaac 对自身与 RDK 实测 `q` 的二次 READY 校验阈值。 |
| `--quest-target-udp-port` | `57679` | 一个 Quest/fake UDP endpoint，通过 packet `side` 字段分流左右臂。 |
| `--gateway-endpoint` | 空 | 非空时发布 Stage2 dual gateway sample。 |

双臂入口固定采用多速率执行：`--physics-hz 2000` 是 Studio/SimPlugin 状态—力矩闭环，`--render-hz 30` 和 `--target-pose-publish-hz 30` 是 GUI/目标更新。每侧 scene 用 `bootstrap_q` 对齐 Studio home，用 `initial_q` 定义任务初始关节姿态。DRDK 的 `joint_initializing` 阶段允许 Studio 力矩闭环驱动 NRT 关节轨迹，但禁止 Quest/Frame 接管；切换 Cartesian 并锁存 initq TCP 后才进入任务 `ready`。

## Quest publisher

### `rizon4_quest_target_publisher.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--host-ip` | `192.168.32.10` | Quest 可访问的主机 IP，同时用于 HTTPS/WSS 地址。 |
| `--udp-host/--udp-port` | `127.0.0.1:45679` | Isaac Quest UDP 目标。README 使用端口 `55679`。 |
| `--serial-number` | 兼容默认 serial | 写入 Quest 包的机器人序列号。 |
| `--joint-group` | `ARM_1` | Studio 关节组。 |
| `--side` | `right` | 使用左手或右手控制器。 |
| `--enable-button` | `squeeze` | 控制使能按键。 |
| `--axis-map` | `-z,-x,y` | OpenXR 平移轴到机器人基座轴映射。 |
| `--position-delta-scale` | `1.0` | publisher 侧位移缩放；推荐保持 1，由 Isaac/Hydra 统一缩放。 |
| `--position-deadband` | `0.0` | publisher 侧平移死区；默认不重复过滤，由 Isaac/Hydra 的 `quest.position_deadband_m` 统一处理。 |
| `--engage-settle-sec` | `0.25` | 按下使能后建立参考点的等待时间。 |
| `--right-tcp-rot-offset` | 固定 wxyz | 右手控制器到 TCP 的姿态偏移。 |
| `--enable-threshold` | `0.5` | squeeze/trigger 等模拟量使能阈值。 |
| `--televuer-root` | 仓库 `third_party/televuer` | TeleVuer 源码目录。 |
| `--cert-file/--key-file` | `configs/xr_teleoperate` | HTTPS 证书和私钥。 |
| `--rate-hz` | `30` | Quest 包发布频率。 |
| `--log-hz` | `2` | 状态日志频率。 |

### `fake_rizon4_quest_sender.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--dual` | 关闭 | 同一 UDP endpoint 发送 left/right 两个 Quest target packet。 |
| `--left-serial-number/right-serial-number` | Stage2 样例 | `--dual` 模式下左右 serial。 |
| `--axis/right-axis` | `x` | 左右 fake 位移轴。 |
| `--same-direction` | 关闭 | 默认左右相反方向运动；开启后同方向运动。 |

## Recorder

### `record_unitree_json.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--gateway-endpoint` | `tcp://127.0.0.1:5590` | gateway sample 地址；README 使用 `5690`。 |
| `--fps` | `30` | 录制采样频率及 data.json 图像 fps。 |
| `--episodes` | `1` | 本次进程需要保存的 episode 数。 |
| `--task-name` | 与 `--task-dir` 二选一 | `--output-root` 下的安全文件夹名。 |
| `--task-dir` | 与 `--task-name` 二选一 | 旧版完整 task 目录。 |
| `--output-root` | `datasets/stage1_records` | task 文件夹根目录。 |
| `--image-size` | `640x480` | data.json 中声明的图像尺寸。 |
| `--max-frames` | `0` | 单个 episode 最大帧数；0 表示不限制。达到上限自动保存。 |
| `--reset-on-save` | 关闭 | 保存、丢弃或自动结束后请求协调 reset。 |
| `--reset-key-cooldown-sec` | `2.5` | reset 快捷键防连发时间，避免终端按键自动重复导致连续初始化。 |
| `--reset-timeout-sec` | `25.0` | recorder 等待 Isaac/RDK reset 落位的最长时间；失败时停止录制并返回错误。 |
| `--start-key` | `s` | 开始/继续快捷键。 |
| `--stop-key` | `e` | 第一次暂停、第二次保存。 |
| `--discard-key` | `d` | 丢弃当前 episode。 |
| `--reset-key` | `r` | 立即请求 reset。 |
| `--quit-key` | `q` | 退出。 |
| `--auto-start` | 关闭 | 自动开始；非 TTY 输入时也自动启用。手动录制不使用此参数。 |
| `--task-goal/--task-desc/--task-steps` | Stage1 默认文本 | 写入 data.json 的任务语义描述。 |

## 转换与验证

### `convert_unitree_json_to_lerobot.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--raw-dir` | 必填 | 含 `episode_*/data.json` 的 task 目录。 |
| `--repo-id` | 必填 | 输出数据集相对 ID，例如 `qiming/pick_cube`。 |
| `--output-root` | `LEROBOT_OUTPUT_ROOT` 或 `datasets/lerobot` | LeRobot 数据集根目录。 |
| `--action-mode` | `qpos` | `qpos` 输出 16D 位置；`full` 输出 qpos/qvel/torque。 |
| `--fps` | 原始 episode fps | 覆盖视频和时间戳 fps。 |

### `validate_data_artifacts.py`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--raw-dir` | 必填 | 原始 task 目录。 |
| `--dataset-root` | 必填 | 转换后的 LeRobot 数据集目录。 |
| `--out` | 无 | 可选 JSON 验证报告路径。 |
| `--strict-single-arm` | 关闭 | 开启 Stage1 单 Rizon4 严格检查。 |
| `--strict-dual-arm` | 关闭 | 开启 Stage2 双 Rizon4 严格检查。 |
| `--expected-serial` | 严格模式必填 | 必须与数据中的 serial 一致，不能是空字符串。 |
| `--expected-left-serial/right-serial` | 双臂严格模式必填 | 必须与 Stage2 数据中的左右 serial 一致。 |
| `--required-camera-names` | `cam_front` | 双臂严格模式下要求 LeRobot dataset 至少包含这些视频。 |
| `--required-camera-keys` | 按 camera names 推导 | 双臂严格模式下要求 Unitree JSON 包含这些 color key。 |
| `--min-left-q-delta` | `0` | episode 相对首帧的最小关节位移范数。 |
| `--min-right-q-delta` | `0` | Stage2 右臂相对首帧的最小关节位移范数。 |
| `--min-left-torque-norm` | `0` | 最大左臂力矩范数必须严格大于该值。 |
| `--min-right-torque-norm` | `0` | Stage2 最大右臂力矩范数必须严格大于该值。 |
| `--min-servo-cycle-delta` | `0` | episode 内最小 servo cycle 跨度。 |

## Stage2 验收脚本

### `run_stage2_dual_data_collection_smoke.py`

无 Isaac/Studio 的数据工具链 smoke：fake gateway -> recorder -> Unitree JSON -> LeRobot-style dataset -> H264 MP4 -> strict dual validator。

### `run_stage2_dual_rizon4_real_validation.py`

真实双臂闭环验收：读取 `configs/pipelines/stage2_dual_rizon4_data_collection.yaml`，启动本仓库 gateway、两个 RDK streamer、dual Isaac app、fake dual sender、recorder、converter 和 validator。常用覆盖参数包括 `--left-serial-number`、`--right-serial-number`、`--config`、`--record-frames`、`--keep-running-on-failure`。

## 状态与停止

- `flexiv_stack_status.py`：无参数，显示已知后台进程状态。
- `stop_flexiv_stack.py --timeout 8`：停止项目控制栈；`--timeout` 是发送 SIGTERM 后等待秒数，超时才强制结束。
