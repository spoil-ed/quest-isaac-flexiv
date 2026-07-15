# 2026-07-14 双臂 Flexiv + Isaac Sim 系统问题复盘

本文记录双臂系统的主要架构问题、解决方案及当前遗留项，不包含一般代码缺陷、Git 操作、进程清理和单项测试问题。

## 系统基线

```text
Docker Studio 左臂 qSaFLh ── SimPlugin ─┐
                                         ├── 宿主机 Isaac 双臂 Bridge
宿主机 Studio 右臂 I0LIRN ─ SimPlugin ─┘

TargetFrameLeft  ── 宿主机 left RDK streamer  ── Docker 左臂 Studio
TargetFrameRight ── 宿主机 right RDK streamer ── 宿主机右臂 Studio
```

- 左臂使用 Docker Elements Studio，型号为 `Rizon4-qSaFLh`。
- 右臂使用宿主机 Elements Studio，型号为 `Rizon4-I0LIRN`。
- Docker 仅隔离左臂 Studio、RobotControlApp、FlexivSimulation 和 GUI；Isaac、RDK streamer、action、gateway、recorder 及 Quest/fake sender 均运行于宿主机。
- Studio、SimPlugin 与 Isaac 的物理闭环频率为 `2000 Hz`；TargetFrame、Quest、GUI、相机及数据流约为 `30 Hz`。

## 1. 双 Studio 运行隔离

### 问题

两套 Elements Studio 在同一宿主机运行时会共享端口、用户数据、进程名、共享内存和显示环境，无法保证 RobotControlApp、FlexivSimulation 与 GUI 长期稳定共存。官方多机器人方案依赖两台 Ubuntu 主机，当前环境需在单机上提供等效隔离。

### 方案

- 右臂 Studio 保留在宿主机，左臂 Studio 运行于 Docker。
- Docker 使用独立 bridge 网络、Studio runtime 和用户数据副本，不使用 host 网络。
- 左臂 GUI 通过本机 VNC `127.0.0.1:5902` 访问。
- 原厂 Studio 二进制及用户数据仅复制到 `.deps/docker_studio/`，不纳入版本控制。

该结构保留现有单臂环境，仅为新增左臂提供隔离，与官方双机拓扑一致。

## 2. 保持原有控制链路

### 问题

Docker 仅用于替代第二台 Ubuntu 主机，不应引入新的控制层，也不应改变 Stage1 已验证的 RDK/Studio 协议。

### 方案

- 两个 RDK streamer 均运行于宿主机。
- 两套 Studio 均采用 External Interface + Ethernet，并启用 `externalEthernetConfig.xml`。
- Docker 仅承载左臂 Studio runtime。
- 左右臂 alias 在 Studio、RDK、Isaac、Quest 数据包及验证流程中保持一致。

| 功能 | left | right |
| --- | --- | --- |
| Studio alias | `Rizon4-qSaFLh` | `Rizon4-I0LIRN` |
| Target pose UDP | `57680` | `57681` |
| RDK status/reference UDP | `57682` | `57683` |

Quest target 统一发送至 UDP `57679`，接收端按数据包中的 `side=left/right` 分流。

## 3. 复用 Stage1 单臂控制

### 问题

双臂联调曾引入 position hold、fake 控制、力矩缩放、关节速度拒绝及 target-drive 阈值等额外逻辑，导致链路偏离 Stage1，且无法明确验证目标是否经过 TargetFrame。

### 方案

双臂系统由两套独立的 Stage1 链路组成：

```text
TargetFrameLeft  -> left RDK  -> left Studio  -> left SimPlugin  -> left articulation
TargetFrameRight -> right RDK -> right Studio -> right SimPlugin -> right articulation
```

- 左右臂分别维护 TargetFrame、RDK streamer、RDK status 和 SimPlugin 节点。
- fake 与 Quest 仅更新 TargetFrame，不绕过主控制链路。
- Studio 返回的 `target_drives` 在对应物理子步直接施加。
- 不增加双臂专用 torque scale、joint speed reject 或 target-drive 阈值。

Docker 对控制算法透明，双臂仅扩展控制通道数量。

## 4. 墙挂场景等效重力力矩平衡

### 问题

墙挂安装将机械臂基座相对世界坐标系旋转约 `90°`，但 Isaac 的世界重力方向保持不变。因此，重力在机械臂基坐标系中的等效方向由下式确定：

```text
g_base = R_world_base^T * g_world
```

若 Studio/FlexivSimulation 仍按原安装方向计算重力补偿，则其输出的补偿力矩与 Isaac 中墙挂机械臂的实际重力力矩不一致。静止状态下的关节净力矩为：

```text
tau_net(q) = tau_studio(q) - tau_gravity_isaac(q)
```

当 `tau_net(q) != 0` 时，即使目标关节位置不变，机械臂也会产生加速度、漂移或振荡，并可能触发关节速度或力矩超限。该问题的本质是墙挂姿态下的等效重力力矩未平衡，而非 TCP 目标坐标变换误差。

### 方案

- 根据每只机械臂的 `R_world_base` 计算 `g_base`，确保 Isaac 与 Studio/FlexivSimulation 使用一致的安装姿态和重力方向。
- 任务物体仍需承受世界坐标系 `-Z` 方向重力，不应为补偿机械臂而直接旋转整个场景的全局重力。
- 通过控制器安装姿态配置或机械臂级重力补偿，使墙挂机械臂的重力力矩与 Studio 输出力矩一致。
- 左右臂分别校验；安装姿态不同时不得共享重力补偿参数。
- 在 home 姿态且无目标输入时，确认关节速度保持接近零、输出力矩与 Isaac 重力力矩平衡后，方可启用正式 effort 闭环。
- 不通过移动 TargetFrame、墙挂 base 或任务物体修正重力力矩失配。

TCP 坐标映射仅负责目标位姿转换，不能解决动力学中的重力力矩失配。

## 5. 控制频率与交互性能

### 问题

Studio/SimPlugin 的力矩闭环依赖 `2000 Hz`（物理步长 `0.5 ms`）。降低 physics frequency 会改变力矩离散积分过程，可能引发振荡、关节速度异常或力矩超限；渲染与交互则无需同频运行。

### 方案

- `physics_hz = 2000`：Studio/SimPlugin/Isaac 状态—力矩闭环。
- `render_hz = 30`：Isaac GUI 与 RTX 渲染。
- `target_pose_publish_hz = 30`：TargetFrame/Quest 目标发布。
- gateway、recorder 与视频约为 `30 Hz`。

每次 `world.step(render=True)` 约执行 66–67 个 `0.5 ms` 物理子步；每个子步均完成状态发送、Studio 力矩接收与力矩施加。

当前硬件为 24 核 Intel Core Ultra 7 270K Plus 和 RTX 5080 16 GB。系统采用 GPU 执行 RTX/Viewport 渲染，PhysX 使用 `--no-gpu-dynamics` 保持 Stage1 的 CPU dynamics，避免小场景在高频子步中产生额外 CPU/GPU 同步开销。性能优化应降低渲染、相机和数据频率，不应降低物理闭环频率。

## 6. 墙挂场景状态一致性

### 问题

Studio/FlexivSimulation 的 `home`、Isaac 的 `initial_q`、墙挂 base pose、TargetFrame 初始位姿及任务物体高度必须一致。任一配置不匹配均可能导致启动漂移、桌面碰撞或目标跳变。

### 方案

两臂 `initial_q` 与 Studio SRDF `home` 保持一致：

```text
[0, -0.6981317, 0, 1.57079632679, 0, 0.6981317, 0]
```

墙挂 base 固定为：

| side | position `(x, y, z)` | quaternion `(w, x, y, z)` |
| --- | --- | --- |
| left | `(-0.06, 0.42, 1.08)` | `(0.70710678, 0, 0.70710678, 0)` |
| right | `(-0.06, -0.42, 1.08)` | `(0.70710678, 0, 0.70710678, 0)` |

- 调整桌面及红块、圆柱、抽屉和目标区域高度，确保 home pose 与桌面碰撞体之间存在安全间隙。
- 启动时同步 joint position 与 USD position-drive target，再按实际末端位姿初始化 TargetFrame。
- 构图调整使用相机，任务空间调整使用任务物体；墙挂 base 不参与补偿。

## 7. 路径与资产可移植性

### 问题

项目依赖仓库内容、相邻 Isaac Sim workspace、Unitree/IsaacLab 任务资产以及本机 Elements Studio 和 Isaac runtime。提交开发机绝对路径会限制部署环境，并可能将路径解析问题误判为资产缺失。

### 方案

- 仓库资产路径在配置文件中使用相对路径，并相对配置文件解析。
- 代码自有资产从 `__file__` 推导仓库根目录。
- 外部 runtime 和 workspace 通过环境变量或 CLI 参数提供。
- Unitree 资产通过 `UNITREE_ASSET_ROOT` 或相应 workspace 环境变量解析。
- Stage3 增加资产存在性、双臂 base、`initial_q` 和桌面安全高度检查。

## 8. Quest 双手输入

### 当前状态

- 仅启动一个 TeleVuer/WebXR 会话，避免两个 publisher 争用固定 HTTPS/WSS `8012` 端口。
- 同一进程读取左右控制器位姿与按键，并在同一 UDP endpoint 上按 `side=left/right` 分流。
- 左右手独立维护位置零点、squeeze 接管状态和 TCP 映射；松开 squeeze 后对应机械臂保持上一目标。
- XYZ 使用按下 squeeze 时建立的相对位移，方向使用映射后的手柄绝对姿态。
- 双臂墙装路径不复用 Stage1 直立安装的正 Z 工作空间裁剪，只保留 Cartesian 目标速度限制。
- 左右 trigger 独立控制对应 Isaac Grav 夹爪；按下闭合、松开张开，不经过 Studio/RDK。

## 结论

双臂系统沿用两套 Stage1 TargetFrame 闭环。核心边界如下：

1. Docker 将左臂 Studio 隔离为等效的第二台 Ubuntu 主机。
2. Docker 仅改变 Studio 的运行位置，不改变控制协议。
3. 墙挂安装通过统一 Isaac 与 Studio/FlexivSimulation 的等效重力方向，实现关节重力力矩平衡。
4. `2000 Hz` 物理闭环与约 `30 Hz` 的交互、渲染和数据链路分频运行。

墙挂场景、Docker 和 Quest 均作为边界适配层，不改变 TargetFrame 主控制链路。
