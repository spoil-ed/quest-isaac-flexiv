# Flexiv 双臂 Studio：宿主机 + Docker

Stage2/Stage3 需要两套相互独立的 Flexiv Elements Studio runtime。同一宿主机直接启动两套 Studio 会共享固定端口和用户数据，因此本项目采用以下拓扑：

```text
宿主机 action ─┬─ RDK 左臂 ─ Docker Studio（qSaFLh）─┐
               └─ RDK 右臂 ─ 宿主机 Studio（I0LIRN）─┴─ SimPlugin ─ 宿主机双臂 Isaac Bridge
```

这与 Flexiv 官方多机器人方案一致：Docker 容器替代官方步骤中的第二台 Ubuntu 电脑，Docker bridge 替代两机之间的有线网络。Isaac Sim、双臂 Bridge、RDK streamer、gateway、Quest/fake sender 和 recorder 都运行在宿主机；Docker 只隔离左臂 Studio、RobotControlApp、FlexivSimulation 和显示会话。不要再启动两个 Studio 容器。

数据是双向流动的：Isaac Bridge 经 SimPlugin 向两套控制器发布机器人状态，两套控制器再把 `target_drives` 返回给同一个 Bridge；宿主机 action 可经两个独立 RDK streamer，或经一个 DRDK `RobotPair` streamer，送入左右控制器。两种 target backend 互斥，均固定使用 NRT；控制解算仍在 runtime 内。

Docker 模板位于 `docker/flexiv-studio/`。Flexiv 原厂程序受许可证约束，不提交到仓库；准备脚本会将用户已有的 Studio 复制到已忽略的 `.deps/docker_studio/`。

## 1. 前置条件

- Linux x86_64 和可用的 NVIDIA 驱动。
- Docker Engine、Compose v2 和 NVIDIA Container Toolkit。
- 一套已能在宿主机运行的 Flexiv Elements Studio。
- 在仓库根目录执行本文命令。
- 项目 Python 默认使用 `conda activate isaacsim` 后的环境。

Ubuntu 可以安装发行版 Docker 包：

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-buildx docker-compose-v2
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

重新登录或执行 `newgrp docker` 使用户组生效。配置 NVIDIA runtime：

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

验证：

```bash
docker compose version
docker info --format '{{json .Runtimes}}'
nvidia-ctk cdi list
```

输出应包含 `nvidia` runtime 和至少一个 `nvidia.com/gpu=...` 设备。

## 2. 设置本机参数

外部安装路径只通过环境变量或 CLI 传入，不写入仓库配置：

```bash
conda activate isaacsim
export STUDIO_ROOT="/path/to/FlexivElementsStudio"
export ISAAC_PYTHON="/path/to/isaacsim/python"
export LEFT_ROBOT_SERIAL="Rizon4-qSaFLh"
export RIGHT_ROBOT_SERIAL="Rizon4-I0LIRN"
```

RDK alias 是 `Rizon4-` 加机械臂硬件序列号最后六位。可用下面的命令读取宿主机模拟器硬件序列号：

```bash
find "$STUDIO_ROOT/user_data_ui/simDir/simulator0" \
  -mindepth 2 -maxdepth 2 -name arm_driver_param.xml -printf '%h\n'
```

宿主机和容器必须使用不同的六位后缀。

## 3. 准备容器侧 Studio 数据

下面的命令只复制宿主机 `simulator1` 的 qSaFLh 数据到隔离副本；副本中的其他 simulator 会被删除：

```bash
docker/flexiv-studio/prepare-runtime.sh \
  --studio-root "$STUDIO_ROOT" \
  --source-simulator simulator1 \
  --target-suffix "${LEFT_ROBOT_SERIAL#Rizon4-}" \
  --force
```

默认输出目录是：

```text
.deps/docker_studio/FlexivElementsStudio/
```

脚本不会修改宿主机 Studio。结束时会同时打印宿主机 alias 和容器 alias；先确认它们不同。

## 4. 构建并启动左臂容器

```bash
export FLEXIV_ARM_SERIAL="A02L-00-M6-${LEFT_ROBOT_SERIAL#Rizon4-}"
docker compose -f docker/flexiv-studio/compose.yaml build
docker compose -f docker/flexiv-studio/compose.yaml up -d
docker compose -f docker/flexiv-studio/compose.yaml ps
```

容器使用独立的 `172.28.0.0/24` bridge，避免和宿主机 Studio 的固定端口冲突。VNC 只发布到宿主机回环地址 `127.0.0.1:5902`，没有密码，不应改成公网监听。

查看界面和日志：

```bash
sudo apt-get install -y tigervnc-viewer
vncviewer 127.0.0.1:5902
docker compose -f docker/flexiv-studio/compose.yaml logs -f studio-left
docker inspect --format '{{.State.Health.Status}}' flexiv-studio-left
```

entrypoint 默认启动以下进程：

- Xvfb、openbox 和 x11vnc；
- FlexivElementsStudio；
- RobotControlApp；
- FlexivSimulation。

如需只打开 GUI 并手工配置，启动前设置 `FLEXIV_AUTO_START_RUNTIME=0`。

如果 Docker Hub 不可达，可以从当前 Ubuntu apt 镜像制作本地 `ubuntu:24.04` 基础镜像，再重新 build：

```bash
sudo apt-get install -y debootstrap
sudo debootstrap --variant=minbase noble /tmp/flexiv-ubuntu-rootfs \
  http://mirrors.tuna.tsinghua.edu.cn/ubuntu
sudo tar -C /tmp/flexiv-ubuntu-rootfs -cf /tmp/flexiv-ubuntu-rootfs.tar .
sudo chown "$USER:$USER" /tmp/flexiv-ubuntu-rootfs.tar
docker import /tmp/flexiv-ubuntu-rootfs.tar ubuntu:24.04
```

## 5. 启动宿主机右臂 Studio

```bash
python scripts/start_elements_studio_ui.py --studio-root "$STUDIO_ROOT"
python scripts/start_robot_control_app.py --studio-root "$STUDIO_ROOT"
python scripts/start_flexiv_simulation.py --studio-root "$STUDIO_ROOT"
```

冷启动时宿主机使用 `simulator0`（I0LIRN），Docker 使用 `simulator1`（qSaFLh）。脚本会检测已经存在的同名进程。

## 6. 验证两套 runtime

首先检查进程和容器：

```bash
python scripts/flexiv_stack_status.py
docker compose -f docker/flexiv-studio/compose.yaml ps
docker exec flexiv-studio-left sh -lc \
  "pgrep -af 'FlexivElementsStudio|RobotControlApp|FlexivSimulation'"
```

`prepare-runtime.sh` 会把容器 simulator 对齐为与宿主机相同的 External/Ethernet 连接方式。启动 RDK 前检查两侧配置：

```bash
rg -n '<enable>|<interface_config_file>' \
  "$STUDIO_ROOT/user_data_ui/simDir/simulator0/user_data/settings/robotExtInterfaceCfg.xml" \
  .deps/docker_studio/FlexivElementsStudio/user_data_ui/simDir/simulator1/user_data/settings/robotExtInterfaceCfg.xml
```

两侧都必须是 `<enable>1</enable>` 和 `externalEthernetConfig.xml`。旧版准备脚本生成的 runtime 如果仍是 External IO disabled，重新运行第 3 节的 `prepare-runtime.sh --force`，然后重建容器；不需要把 RDK streamer 放入容器。

手工在 Isaac GUI 中拖动左右 Target Frame 时，不要给 dual app 传 `--enable-quest-target-udp`；此时 Frame 世界位姿会转换为左右 `pose_base_tcp_des`，经宿主机两个 RDK streamer 发送。启用 Quest UDP 后，控制源切换为 Quest packet，程序会在没有新 packet 时暂停用户目标发布。

双臂控制就是两套独立的 Stage1 TargetFrame 回路，外置 streamer 是进程边界，Docker 只隔离左臂 Studio。两个 Studio runtime 就绪后，先启动两个 RDK streamer，再启动 Isaac。手动模式下，程序先把两个可见 TargetFrame 对齐到各自末端；streamer 随即锁存 Studio/RDK 实际 TCP，并用“Isaac 末端世界位姿 ↔ RDK TCP”这一对参考位姿直接标定坐标变换。收到 operational 回执后才在 2000 Hz 原样施加对应 SimPlugin torque；用户实际移动 Frame 后才释放标定后的目标。

Isaac 外循环也按 30 Hz 渲染，但一次 `world.step(render=True)` 会在内部执行约 66–67 个 0.5 ms 物理子步。每个物理子步都先向两个 SimPlugin 节点发送同一周期的关节状态，再分别等待并原样施加两个 Studio 返回的 `target_drives`。力矩命令不做抽帧、合并、平均、缩放或双臂专属阈值拒绝；因此渲染和目标更新降频不会改变 Studio 控制器使用的物理积分步长。

```bash
python scripts/start_dual_isaac_follow.py \
  --isaac-python "$ISAAC_PYTHON" \
  --scene-config configs/scenes/dual_rizon4_cam_front.yaml \
  --no-gpu-dynamics \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --no-manual-play \
  --physics-hz 2000 \
  --render-hz 30 \
  --target-pose-publish-hz 30 \
  --left-target-pose-udp-port 57680 \
  --right-target-pose-udp-port 57681 \
  --left-rdk-status-udp-port 57682 \
  --right-rdk-status-udp-port 57683
```

Isaac 的 GUI/RTX 渲染仍使用 NVIDIA GPU；`--no-gpu-dynamics` 仅让 PhysX 沿用 Stage1 的 CPU 求解，避免这个小型 2000 Hz 场景承担逐步 CPU/GPU 同步开销。不要为了提高 GUI 帧率降低 `--physics-hz`，应保持 2000 Hz 并调整 `--render-hz` 和 `--target-pose-publish-hz`。

分别启动两个 RDK target streamer。两者都应进入 operational 状态，不能持续打印“robot not found”：

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

tail -f logs/rdk_target_streamer*.stderr.log
```

需要双臂同步提交和零空间初始化时，用一个 DRDK streamer 替代上面的两个 RDK streamer，不能同时运行：

```bash
"$ISAAC_PYTHON" -m pip install --no-deps flexivdrdk==1.2.1
python scripts/start_drdk_target_streamer.py \
  --python "$ISAAC_PYTHON" \
  --scene-config configs/scenes/pick_place_redblock_flexiv_dual.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --left-port 57680 --right-port 57681 \
  --left-status-port 57682 --right-status-port 57683
```

DRDK 在两个 runtime operational 后先进入 `NRT_JOINT_POSITION`，通过 `SendJointPosition()` 平滑到同一 scene config 的左右 `initial_q`。到位并稳定后再切换 `NRT_CARTESIAN_MOTION_FORCE`、调用 `SetNullSpacePosture((init_q_left, init_q_right))`，并锁存两臂当前 TCP。scene 的 `bootstrap_q` 必须匹配 Studio home；`joint_initializing` 阶段保持 2 kHz SimPlugin 力矩闭环，但不开放 Quest/Frame。任一侧 fault 会使双侧一起 not-ready；streamer 保持存活等待 recorder 的显式 reset。按 `r` 后，Isaac 临时关闭桌面和工件等场景碰撞并固定刚体工件，DRDK 执行 `Stop → ClearFault → Enable → SendJointPosition(init_q)`。双臂重新 ready 后，全部 `scene_objects` 恢复配置中的初始变换和关节状态，刚体速度清零；等待一个目标周期后再恢复场景碰撞和工件动力学。机械臂 articulation 的 collision shape 不会在运行中改动；流程也不会重置 Isaac world 或直接写 articulation。

正常日志顺序是：两个 streamer 开始监听、两个 `SimPlugin connected`、TargetFrame 初始化到末端、两个 `latched current TCP reference`、两个 `calibrated ... RDK TCP reference`、RDK operational、两侧进入 effort。Studio FlexivSimulation 需要 Isaac SimPlugin 状态流才能推进，因此 DRDK 可以先进入发现重试，但 Isaac 必须随后启动，不能等 DRDK ready 后才启动 Isaac。此时只保持当前 TCP；拖动某个 Frame 后才出现该侧 `target control armed` 并释放用户目标。若出现 `joint torque exceeds the limit`，streamer 会发出 not-ready 并等待操作员按 `r`；不要在脚本外循环清故障。reset 失败时 recorder 会暂停但不会退出；先确认没有持续的几何穿插，再按一次 `r` 重试。仅当显式 reset 也无法恢复、需要完整冷启动时，宿主机右臂才可能残留 `/dev/shm/*I0LIRN*`；确认宿主机 RobotControlApp 和 FlexivSimulation 已停止后再删除这些临时共享内存。

确认两侧 External/Ethernet 配置相同后，如果宿主机 RDK 仍无法发现容器 simulator，再检查网络：

```bash
ip address show flexiv-studio-bridge
ping -c 1 172.28.0.2
docker logs flexiv-studio-left
```

发现协议可能依赖网卡广播。此时不要改用 `network_mode: host`，否则两套固定端口会再次冲突；应确认 RDK 未绑定单一物理网卡，并允许 Docker bridge 上的 UDP 流量。

## 7. 运行 Stage3

两套 Studio 都在线且 RDK 能识别两个 alias 后，停止上一步独立测试的 streamer，避免端口被重复占用：

```bash
python scripts/stop_flexiv_stack.py
```

这不会停止 Docker 容器。随后运行 README 主线中的首个 Stage3 场景：

```bash
conda activate isaacsim
python scripts/run_stage3_sim_scene_validation.py \
  --config configs/pipelines/dual_arm_data_collection.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL"
```

成功标准包括：左右 RDK 都 operational、两臂状态都有更新、力矩不是全零、录制和 LeRobot 转换完成。仅能加载 Isaac 场景但 RDK 找不到 robot，不算 Stage3 完成。

## 8. 停止与重建

停止项目进程和右臂容器：

```bash
python scripts/stop_flexiv_stack.py
docker compose -f docker/flexiv-studio/compose.yaml down
```

保留 Studio runtime、只重建镜像：

```bash
docker compose -f docker/flexiv-studio/compose.yaml build --no-cache
```

宿主机 Studio 更新后，重新执行 `prepare-runtime.sh --force`。不要将 `.deps/docker_studio/`、Flexiv 许可证、原厂二进制或用户绝对路径提交到 Git。
