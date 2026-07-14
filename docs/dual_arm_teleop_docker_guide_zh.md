# Quest Isaac Flexiv 双臂遥操 Docker 外置说明

本文档放在 `quest-isaac-flexiv` 仓库外，用于指导用户配置双臂遥操所需的 Flexiv Elements Studio Docker 会话，并和旧仓库 Stage2 双臂控制、录制、转换流程配合使用。

适用主仓库：

```bash
/data/qiming/quest_flexiv/quest-isaac-flexiv
```

## 1. 总体边界

Docker 在本流程中只负责隔离两套 Flexiv Elements Studio GUI 会话：

```text
flexiv-studio-left  -> VNC 127.0.0.1:5900 -> simulator0 / left serial
flexiv-studio-right -> VNC 127.0.0.1:5902 -> simulator1 / right serial
```

以下组件仍建议跑在宿主机：

- Isaac Sim 6.0.1 和 `dual_follow_with_studio.py`
- 本仓库 Stage2 gateway、recorder、converter、validator
- 两个 `rdk_target_streamer.py`
- Quest publisher 或 fake Quest sender

这样做的原因是 Isaac Sim 对 GPU、驱动、扩展路径和 `flexivsimplugin` 版本较敏感，而 Docker 的主要收益是把两套 Studio 用户数据、显示会话和 VNC 窗口隔离开。

当前旧仓库 Stage2 已具备双臂 fake 闭环和双臂录制验证能力；真实 Quest 双手遥操还处在脚本组合阶段。`rizon4_quest_target_publisher.py` 每个进程只发布一个 `side`，而它内部的 TeleVuer 服务默认使用固定 `8012` 端口，因此双 publisher 同机并行需要实际端口验证或后续改造成单进程双手 publisher。本文档会分别标明“已稳定验证路径”和“真实 Quest 使用注意事项”。

网络上有一个硬限制：不要把两套 Studio 容器改成 `network_mode: host`。两套 Studio 会在各自命名空间内使用相同的内部端口，例如 `127.0.0.1:17002` 一类服务；共享 host network 时第二套会话很容易因为端口占用直接退出。默认 compose 只把 VNC 映射到宿主机，这是最稳的隔离形态。

如果宿主机上的 RDK streamer 无法通过 serial 找到容器内 Studio，请优先按第 11 节排查 Remote Mode 和 serial；仍然失败时，有三条路线：

- 临时使用两套宿主机/虚拟机 Studio 会话做验收。
- 识别 RobotControlApp/RDK 实际需要的端口后，在 compose 中显式映射左右容器端口，不要切到 host network。
- 把对应侧的 RDK streamer 也放进同一个 Studio 容器内运行，但这需要额外挂载本仓库、Python/RDK 环境和脚本，不属于当前旧仓库默认路径。

## 2. 前置条件

宿主机需要具备：

- Docker Engine 和 Docker Compose v2
- 可运行的 Flexiv Elements Studio 安装目录
- 已配置好的 `simulator0` 和 `simulator1`
- `quest-isaac-flexiv` 的 Stage2 依赖已安装
- Meta Quest 与宿主机在同一局域网

建议先定义这些变量：

```bash
export QIF_ROOT=/data/qiming/quest_flexiv/quest-isaac-flexiv
export SOURCE_STUDIO=/data/qiming/FlexivElementsStudio
export SESSION_ROOT=/data/qiming/flexiv_studio_docker
export DOCKER_ROOT=/data/qiming/quest_flexiv/flexiv-studio-docker

export LEFT_ROBOT_SERIAL=Rizon4-VIHhZM
export RIGHT_ROBOT_SERIAL=Rizon4-WE7ssd
export HOST_IP=192.168.32.11
export ISAAC_PYTHON=/path/to/isaacsim/bin/python
export ISAACSIM_ROOT=/path/to/isaacsim
export FLEXIV_RDK_PYTHON=/path/to/rdk_or_isaac_python
```

`SOURCE_STUDIO` 必须指向含有这些文件的目录：

```text
FlexivElementsStudio
RobotControlApp
FlexivSimulation
user_data_ui/simDir/simulator0/
user_data_ui/simDir/simulator1/
```

如果当前 Studio 还没有 `simulator1`，先在宿主机 Studio 中创建第二台 Rizon4 模拟机器人，并确认两台机器人都能进入 Remote Mode。

## 3. 准备 Docker 模板

当前 `quest-isaac-flexiv` 仓库不提交 Dockerfile。可以把已有的 Studio Docker 模板复制到外层目录，作为本项目的外置 Docker 环境：

```bash
mkdir -p "$DOCKER_ROOT"
rsync -a /data/qiming/flexiv_studio_pipeline/docker/flexiv-studio/ "$DOCKER_ROOT/"
```

如果没有 `/data/qiming/flexiv_studio_pipeline`，需要在 `$DOCKER_ROOT` 下准备等价内容：

- `Dockerfile`：基于 Ubuntu 22.04/CUDA，安装 Xvfb、openbox、x11vnc 和 Qt/X11 运行库
- `entrypoint.sh`：启动 Xvfb、openbox、x11vnc，然后执行 `./FlexivElementsStudio -p ubuntu_pc`
- `docker-compose.yml`：定义 `flexiv-studio-left` 和 `flexiv-studio-right` 两个服务

关键 compose 语义如下：

```yaml
services:
  flexiv-studio-left:
    image: flexiv-studio-vnc:local
    volumes:
      - ${SESSION_ROOT}/left/FlexivElementsStudio:/opt/FlexivElementsStudio:rw
    ports:
      - "127.0.0.1:5900:5900"
    environment:
      DISPLAY_NUM: "20"
      VNC_PORT: "5900"
      STUDIO_DIR: /opt/FlexivElementsStudio
      FLEXIV_PHYSICS_ENGINE: external

  flexiv-studio-right:
    image: flexiv-studio-vnc:local
    volumes:
      - ${SESSION_ROOT}/right/FlexivElementsStudio:/opt/FlexivElementsStudio:rw
    ports:
      - "127.0.0.1:5902:5902"
    environment:
      DISPLAY_NUM: "21"
      VNC_PORT: "5902"
      STUDIO_DIR: /opt/FlexivElementsStudio
      FLEXIV_PHYSICS_ENGINE: external
```

`FLEXIV_PHYSICS_ENGINE=external` 表示 Studio 使用 External/Isaac Sim 物理后端。若 Studio 包内有 `bin/FlexivSimulation_external`，entrypoint 会把它切换为当前 `FlexivSimulation`。

## 4. 生成左右 Studio 会话

将一份 Studio 安装复制成左右两个独立会话：

```bash
mkdir -p "$SESSION_ROOT/left" "$SESSION_ROOT/right"

rsync -a --delete --exclude 'log/*' \
  "$SOURCE_STUDIO/" "$SESSION_ROOT/left/FlexivElementsStudio/"

rsync -a --delete --exclude 'log/*' \
  "$SOURCE_STUDIO/" "$SESSION_ROOT/right/FlexivElementsStudio/"

mkdir -p \
  "$SESSION_ROOT/left/FlexivElementsStudio/log" \
  "$SESSION_ROOT/right/FlexivElementsStudio/log"
```

检查左右 simulator 资源：

```bash
find "$SESSION_ROOT/left/FlexivElementsStudio/user_data_ui/simDir/simulator0" \
  -maxdepth 3 -name arm_driver_param.xml -print

find "$SESSION_ROOT/right/FlexivElementsStudio/user_data_ui/simDir/simulator1" \
  -maxdepth 3 -name arm_driver_param.xml -print
```

推荐约定：

- left 容器使用 `simulator0`
- right 容器使用 `simulator1`
- left serial 与 Stage2 scene config 中 left serial 一致
- right serial 与 Stage2 scene config 中 right serial 一致

默认 Stage2 scene 是：

```bash
$QIF_ROOT/configs/scenes/dual_rizon4_cam_front.yaml
```

其中本机样例为：

```text
left  Rizon4-VIHhZM
right Rizon4-WE7ssd
```

实际机器不同的时候，改 scene config 或运行脚本时用 CLI 覆盖 serial，左右两边必须全链路一致。

## 5. 启动 Studio Docker

构建镜像：

```bash
cd "$DOCKER_ROOT"
SESSION_ROOT="$SESSION_ROOT" docker compose --project-name flexiv-dual-studio build
```

启动两个 Studio 容器：

```bash
SESSION_ROOT="$SESSION_ROOT" docker compose --project-name flexiv-dual-studio up -d \
  flexiv-studio-left flexiv-studio-right
```

检查状态：

```bash
docker compose --project-name flexiv-dual-studio ps
docker logs --tail 80 flexiv-studio-left
docker logs --tail 80 flexiv-studio-right
```

用 VNC 客户端连接：

```text
left  127.0.0.1:5900
right 127.0.0.1:5902
```

在两个 VNC 窗口中分别确认：

- 左窗口选择或连接 `simulator0`
- 右窗口选择或连接 `simulator1`
- Physics Engine 是 `External` 或 `Isaac Sim`
- Remote Mode 选择 `Ethernet`
- 左右 serial 与 `LEFT_ROBOT_SERIAL`、`RIGHT_ROBOT_SERIAL` 一致
- 两边没有 fault；如有 fault，先在 Studio 中清除

这一步完成后，Docker 侧只需要保持两个 VNC/Studio 会话在线。

## 6. 配置旧仓库 Stage2

进入主仓库：

```bash
cd "$QIF_ROOT"
```

检查 Stage2 配置：

```bash
sed -n '1,160p' configs/scenes/dual_rizon4_cam_front.yaml
sed -n '1,180p' configs/pipelines/stage2_dual_rizon4_data_collection.yaml
```

重点检查：

- `robots[side=left].serial_number`
- `robots[side=right].serial_number`
- `quest_target.port`，默认 `57679`
- `target_pose.left.port`，默认 `57680`
- `target_pose.right.port`，默认 `57681`
- `gateway.sample_endpoint`，默认 `tcp://127.0.0.1:5790`
- `gateway.bridge_endpoint`，默认 `tcp://127.0.0.1:5791`

若本机 serial 不同，优先通过 CLI 覆盖：

```bash
--left-serial-number "$LEFT_ROBOT_SERIAL" \
--right-serial-number "$RIGHT_ROBOT_SERIAL"
```

## 7. 双臂 fake 闭环验收

在 Studio Docker 已经打开 Remote Mode 后，可以先跑 fake 双臂验收：

```bash
cd "$QIF_ROOT"

python scripts/run_stage2_dual_rizon4_real_validation.py \
  --config configs/pipelines/stage2_dual_rizon4_data_collection.yaml \
  --left-serial-number "$LEFT_ROBOT_SERIAL" \
  --right-serial-number "$RIGHT_ROBOT_SERIAL" \
  --rdk-python "$FLEXIV_RDK_PYTHON" \
  --isaac-python "$ISAAC_PYTHON" \
  --isaacsim-root "$ISAACSIM_ROOT"
```

该命令会启动：

- 本仓库 Stage2 gateway
- 两个 RDK target streamer
- 双臂 Isaac app
- fake dual Quest sender
- recorder
- LeRobot-style converter
- strict dual validator

成功后会输出类似：

```text
datasets/stage1_records/quest_isaac_flexiv_stage2_dual_rizon4_real_<stamp>/
├── raw/episode_001/data.json
├── logs/
├── stage2_dual_rizon4_real_validation.json
└── stage2_dual_rizon4_real_summary.json

datasets/lerobot/qiming/quest_isaac_flexiv_stage2_dual_rizon4_<stamp>/
└── videos/observation.images.cam_front/chunk-000/file-000.mp4
```

fake 验收通过只能说明旧仓库双臂控制、录制和转换链路可用；正式遥操前仍需检查 Quest publisher、证书和局域网访问。

## 8. Quest 双臂遥操使用

当前 `rizon4_quest_target_publisher.py` 是单侧控制器发布器。双臂遥操需要开两个 publisher 进程，分别发送 `side=left` 和 `side=right` 到同一个 Isaac Quest UDP endpoint。

先生成或确认 Quest HTTPS 证书：

```bash
cd "$QIF_ROOT"
mkdir -p configs/xr_teleoperate

openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout configs/xr_teleoperate/key.pem \
  -out configs/xr_teleoperate/cert.pem \
  -subj "/CN=$HOST_IP" \
  -addext "subjectAltName=IP:$HOST_IP"
```

启动 Stage2 控制链路时，可以拆分启动，也可以参考验收脚本。手动拆分的核心 Isaac 启动命令是：

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
  --quest-relative-orientation-mode reference \
  --quest-position-scale 1.0 \
  --quest-position-deadband-m 0.0 \
  --gateway-endpoint tcp://127.0.0.1:5791 \
  --gateway-fps 30
```

左手 publisher：

```bash
.venv-quest/bin/python scripts/rizon4_quest_target_publisher.py \
  --host-ip "$HOST_IP" \
  --serial-number "$LEFT_ROBOT_SERIAL" \
  --udp-host 127.0.0.1 \
  --udp-port 57679 \
  --side left \
  --enable-button squeeze \
  --axis-map=-z,-x,y \
  --position-delta-scale 1.0 \
  --position-deadband 0.0 \
  --rate-hz 30
```

右手 publisher：

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

两个 publisher 都会打印 Quest 浏览器地址。若两个进程都尝试占用同一个 HTTPS/Vuer 端口而失败，可先只运行一个 publisher 做单侧实机遥操，或将第二个 publisher 改造为复用同一个 TeleVuer/WebXR 会话后再用于正式双手遥操。当前旧仓库原生脚本层面的稳定方案是 fake 双臂和单 publisher 单侧控制；双 publisher 同机并行需要实际端口占用验证。

## 9. 录制、转换与视频

启动 gateway：

```bash
python scripts/start_data_gateway.py \
  --backend bridge \
  --sample-endpoint tcp://127.0.0.1:5790 \
  --bridge-endpoint tcp://127.0.0.1:5791 \
  --fps 30 \
  --image-size 640x480 \
  --camera-keys color_0
```

启动 recorder：

```bash
python scripts/record_unitree_json.py \
  --gateway-endpoint tcp://127.0.0.1:5790 \
  --task-name dual_teleop_task \
  --output-root datasets/stage1_records \
  --fps 30 \
  --episodes 10 \
  --image-size 640x480
```

recorder 快捷键：

- `s`：开始或继续
- `e`：暂停；暂停后再按一次保存
- `d`：丢弃当前 episode
- `r`：请求 reset
- `q`：退出

转换为 LeRobot-style dataset：

```bash
python scripts/convert_unitree_json_to_lerobot.py \
  --raw-dir datasets/stage1_records/dual_teleop_task \
  --repo-id qiming/dual_teleop_task \
  --output-root datasets/lerobot \
  --fps 30
```

严格验证双臂数据和 H264 MP4：

```bash
python scripts/validate_data_artifacts.py \
  --raw-dir datasets/stage1_records/dual_teleop_task \
  --dataset-root datasets/lerobot/qiming/dual_teleop_task \
  --strict-dual-arm \
  --expected-left-serial "$LEFT_ROBOT_SERIAL" \
  --expected-right-serial "$RIGHT_ROBOT_SERIAL" \
  --required-camera-names cam_front \
  --required-camera-keys color_0 \
  --min-left-q-delta 0.005 \
  --min-right-q-delta 0.005 \
  --min-left-torque-norm 1e-8 \
  --min-right-torque-norm 1e-8 \
  --min-servo-cycle-delta 5 \
  --expected-video-fps 30
```

MP4 默认位置：

```text
datasets/lerobot/qiming/dual_teleop_task/videos/observation.images.cam_front/chunk-000/file-000.mp4
```

## 10. 停止与清理

停止旧仓库运行进程：

```bash
cd "$QIF_ROOT"
python scripts/stop_flexiv_stack.py
```

停止 Docker Studio：

```bash
cd "$DOCKER_ROOT"
SESSION_ROOT="$SESSION_ROOT" docker compose --project-name flexiv-dual-studio stop
```

完全删除容器但保留左右 Studio 会话数据：

```bash
SESSION_ROOT="$SESSION_ROOT" docker compose --project-name flexiv-dual-studio down
```

会话数据保留在：

```text
/data/qiming/flexiv_studio_docker/left/FlexivElementsStudio
/data/qiming/flexiv_studio_docker/right/FlexivElementsStudio
```

## 11. 常见问题

### VNC 打开但没有机器人

检查左右会话是否复制了完整 `user_data_ui/simDir`，并确认 VNC 中选中了正确 simulator。左侧应使用 `simulator0`，右侧应使用 `simulator1`。

### RDK streamer 连接不到机器人

检查：

- 两个 VNC 中 Remote Mode 是否已经打开
- serial 是否和命令行、scene config 完全一致
- Studio 是否处于 External/Isaac Sim physics engine
- Docker 容器是否仍在运行

### 视频能生成但机械臂不动

检查：

- `dual_follow_with_studio.py` 日志中是否出现 Stage2 gateway connected
- `rdk_target_streamer_left/right` 日志是否持续收到 target pose
- fake 或 Quest packet 的 `side` 是否分别为 `left/right`
- validator 中 `left_q_delta_norm` 和 `right_q_delta_norm` 是否大于阈值

### 双臂运动过大或姿态突变

正式验收建议使用：

```bash
--quest-relative-orientation-mode reference
--quest-position-scale 1.0
--quest-position-deadband-m 0.0
```

这会避免 fake/Quest packet 中固定四元数在 relative 模式下造成不必要的姿态跳变。

### Docker 权限不足

如果当前用户不在 `docker` 组，可以临时使用：

```bash
sg docker -c 'docker compose --project-name flexiv-dual-studio ps'
```

长期使用建议由管理员把当前用户加入 `docker` 组后重新登录。
