# 双臂 Flexiv + Isaac Sim 系统主要卡点

## 系统基线

```text
Docker Studio 左臂 ── SimPlugin ─┐
                                 ├── 宿主机 Isaac 双臂 Bridge
宿主机 Studio 右臂   ─ SimPlugin ─┘

TargetFrameLeft  ── 宿主机 left RDK streamer  ── Docker 左臂 Studio
TargetFrameRight ── 宿主机 right RDK streamer ── 宿主机右臂 Studio
```

- 左臂使用 Docker Elements Studio。
- 右臂使用宿主机 Elements Studio。
- Docker 仅隔离左臂 Studio、RobotControlApp、FlexivSimulation 和 GUI；Isaac、RDK streamer、action、gateway、recorder 及 Quest/fake sender 均运行于宿主机。
- Studio、SimPlugin 与 Isaac 的物理闭环频率为 `2000 Hz`；TargetFrame 与 Quest 目标层约为 `90 Hz`，GUI、相机及数据流约为 `30 Hz`。

## 1. 双 Studio 运行隔离

### 问题

两套 Elements Studio 在同一宿主机运行时会共享端口、用户数据、进程名、共享内存和显示环境，无法保证 RobotControlApp、FlexivSimulation 与 GUI 长期稳定共存。官方多机器人方案依赖两台 Ubuntu 主机，当前环境需在单机上提供等效隔离。

### 方案

- 右臂 Studio 保留在宿主机，左臂 Studio 运行于 Docker。
- Docker 使用独立 bridge 网络、Studio runtime 和用户数据副本，不使用 host 网络。
- 左臂 GUI 通过本机 VNC `127.0.0.1:5902` 访问。
- 原厂 Studio 二进制及用户数据仅复制到 `.deps/docker_studio/`，不纳入版本控制。

该结构保留现有单臂环境，仅为新增左臂提供隔离，与官方双机拓扑一致。

## 2. 墙挂场景等效重力力矩平衡

### 问题

墙挂安装将机械臂基座相对世界坐标系旋转约 `90°`，但 Isaac 的世界重力方向保持不变。

### 方案

- 在 Flexiv Elements Studio 中修改摆放位置，自动补偿重力。

## 3. 控制频率与交互性能

### 问题

Studio/SimPlugin 的力矩闭环依赖 `2000 Hz`（物理步长 `0.5 ms`）。降低 physics frequency 会改变力矩离散积分过程，可能引发振荡、关节速度异常或力矩超限；渲染与交互则无需同频运行，同频在 Isaac Sim 上会导致卡顿。

### 方案

- `physics_hz = 2000`：Studio/SimPlugin/Isaac 状态—力矩闭环。
- `render_hz = 30`：Isaac GUI 与 RTX 渲染。
- `target_pose_publish_hz = 30`：TargetFrame/Quest 目标发布。
- gateway、recorder 与视频约为 `30 Hz`。

每次 `world.step(render=True)` 约执行 66–67 个 `0.5 ms` 物理子步；每个子步均完成状态发送、Studio 力矩接收与力矩施加。

当前硬件为 24 核 Intel Core Ultra 7 270K Plus 和 RTX 5080 16 GB。系统采用 GPU 执行 RTX/Viewport 渲染，PhysX 使用 `--no-gpu-dynamics` 保持 Stage1 的 CPU dynamics，避免小场景在高频子步中产生额外 CPU/GPU 同步开销。性能优化应降低渲染、相机和数据频率，不应降低物理闭环频率。

## 4. Quest 双手输入

### 当前状态

- Isaac 双臂接收端可在同一 UDP endpoint 上按 `side=left/right` 分流。
- Quest 目标经 TargetFrame 和在线坐标校准进入控制链路。
- 单侧 Quest publisher 可独立控制任一机械臂。
