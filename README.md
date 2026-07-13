# Quest Isaac Flexiv

使用 Meta Quest 右手柄控制 Isaac Sim 中的 Flexiv Rizon4。Isaac Sim 负责场景、状态读取和力矩执行；目标位姿经 Flexiv RDK 送入 Elements Studio/FlexivSimulation 控制栈，再由 SimPlugin 将 `target_drives` 回传给 Isaac。最终控制不使用 Isaac IK，也不使用 jog/CartesianJogging。

```text
Quest -> TeleVuer -> UDP :45679 -> Isaac TargetFrame
      -> UDP :45678 -> Flexiv RDK -> Elements Studio/FlexivSimulation
      -> SimPlugin target_drives -> Isaac apply_torques
```

## 仓库完整性结论

**完整流程依赖仓库外内容，仅克隆本仓库不能运行。** 仓库提交了项目脚本、本地 Isaac 扩展、Quest 输入适配和测试，但没有提交以下运行时：

| 依赖 | 是否在 Git 中 | 用途 |
| --- | --- | --- |
| NVIDIA Isaac Sim 6.0.1（Python 3.12） | 否 | 仿真和 `isaacsim.*`/`omni.*` API |
| Flexiv Elements Studio 及其本地模拟机器人配置 | 否 | `RobotControlApp`、`FlexivSimulation`、控制器和 UI |
| Flexiv Isaac workspace | 否，`isaac_sim_ws/` 被忽略 | Rizon4 USD、Flexiv Isaac 示例扩展 |
| `flexivsimplugin==1.2.0` | 否 | Isaac 与 FlexivSimulation 的力矩桥接；代码严格校验版本 |
| Flexiv RDK Python 包 | 否 | 将目标位姿送入 Flexiv runtime；当前兼容副本为 1.9.1 |
| Quest publisher Python 包 | 否 | `vuer`、`numpy`、OpenCV |
| HTTPS 证书和私钥 | 否，`configs/xr_teleoperate/*.pem` 被忽略 | Quest 浏览器的 WebXR/WSS |
| `.deps/grpc`、`.deps/flexivrdk_1_9_1` | 否，`.deps/` 被忽略 | 本机兼容依赖缓存 |

此外，现有默认值包含本机绝对路径：

- Isaac Python：`/home/simate/miniconda3/envs/isaacsim/bin/python`
- Elements Studio：`/home/simate/workspace/elements_studio/FlexivElementsStudio`
- Rizon4 USD：仓库下被忽略的 `isaac_sim_ws/.../Rizon4.usd`

启动脚本允许用 `--isaac-python`、`--studio-root` 覆盖前两项；USD 默认值需要修改 `configs/flexiv_studio_teleop.yaml`，或直接运行底层示例并传 `--usd`。本仓库没有锁定依赖的 `requirements.txt`/Conda 文件，也没有自动取得 Elements Studio 的安装器，因此环境安装目前不是完全自动化的。

## 平台和硬件

- Ubuntu 22.04 x86_64（Flexiv Isaac workspace 声明的支持平台）
- 支持 Isaac Sim 6.0.1 的 NVIDIA GPU 和驱动
- Meta Quest 和控制器，与主机处于可互访的局域网
- Elements Studio 安装包及使用权限

以下版本是本仓库当前机器上验证过的基线，而不是可任意替换的最低版本：Python 3.12.13、Isaac Sim 6.0.1、`flexivsimplugin 1.2.0`、`flexivrdk 1.9.1`（独立 streamer 兼容副本）。

## 环境安装

### 1. 安装 Isaac Sim

按照 NVIDIA 的 pip/Conda 方式建立 Python 3.12 环境。示例环境名为 `isaacsim`：

```bash
conda create -n isaacsim python=3.12 -y
conda activate isaacsim
python -m pip install --upgrade pip
# 按 NVIDIA Isaac Sim 6.0.1 官方安装说明配置 NVIDIA PyPI 源后安装：
python -m pip install 'isaacsim[all,extscache]==6.0.1' --extra-index-url https://pypi.nvidia.com
```

确认安装，并记录解释器路径：

```bash
python -c 'import isaacsim; print(isaacsim.__file__)'
ISAAC_PYTHON="$(command -v python)"
```

Isaac Sim 对驱动和 Python 版本有严格要求；如果官方 6.0.1 安装说明与上述命令不同，以对应版本官方说明为准。

### 2. 安装 Flexiv Isaac workspace

取得 Flexiv 官方 Isaac workspace，使其内容位于仓库的 `isaac_sim_ws/`。该目录应至少包含：

```text
isaac_sim_ws/exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd
isaac_sim_ws/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/flexiv/
```

本仓库已有的 `isaac_sim_ws/` 是一个被 Git 忽略的独立 checkout，不会随本仓库克隆。若 workspace 的扩展未被 Isaac 环境发现，使用其安装脚本复制到 Isaac 根目录，或将扩展加入 Isaac 的 extension search path：

```bash
bash isaac_sim_ws/install_ws.sh /path/to/isaac-sim-root
```

### 3. 安装 Flexiv Python 依赖

在 Isaac Python 环境安装 SimPlugin；版本必须为 1.2.0：

```bash
"$ISAAC_PYTHON" -m pip install 'flexivsimplugin==1.2.0' spdlog
"$ISAAC_PYTHON" -c 'import flexivsimplugin; assert flexivsimplugin.__version__ == "1.2.0"'
```

独立 RDK streamer 需要与 Elements Studio/runtime 协议兼容的 Flexiv RDK。当前环境使用 1.9.1，并放在忽略目录以避免 Isaac 环境中 2.x 抢先被导入：

```bash
mkdir -p .deps/flexivrdk_1_9_1
"$ISAAC_PYTHON" -m pip install --target .deps/flexivrdk_1_9_1 'flexivrdk==1.9.1'
```

若所安装的 Elements Studio 明确要求其他 RDK 版本，应同时调整 `.deps` 版本和兼容性测试，不能直接假设 2.x 可替代 1.9.1。

### 4. 安装 Quest publisher 环境

Quest publisher 不应使用当前 Isaac 环境，因为该环境默认不含 `vuer`。创建独立虚拟环境：

```bash
python3 -m venv .venv-quest
.venv-quest/bin/python -m pip install --upgrade pip
.venv-quest/bin/python -m pip install vuer numpy opencv-python
```

`third_party/televuer` 源码已随仓库提交，publisher 会自动把它加入 `sys.path`，不需要单独安装 TeleVuer。

### 5. 安装并配置 Elements Studio

将 Flexiv Elements Studio 解压或安装到本机，并在 UI 中创建 Rizon4 模拟机器人。首次配置必须生成下列内容，启动脚本会自动发现它们：

```text
<studio-root>/RobotControlApp
<studio-root>/FlexivSimulation
<studio-root>/FlexivElementsStudio
<studio-root>/user_data_ui/simDir/simulator0/*/arm_driver_param.xml
<studio-root>/user_data_ui/simDir/simulator0/user_data/settings/generated_robot*_abs_path.urdf
<studio-root>/user_data_ui/simDir/simulator0/user_data/settings/generated_robot*_abs_path.srdf
<studio-root>/user_data_ui/simDir/simulator0/user_data/settings/user_scene_abs_path.urdf
<studio-root>/specs/robots/*/flexivCfg.xml
```

在 Elements Studio 中启用 Remote Mode/Ethernet，并确认模拟机器人序列号。项目默认使用 `Rizon4-I0LIRN`；不同序列号须在各启动命令中一致传入 `--serial-number`。

### 6. 生成 Quest HTTPS 证书

将 `HOST_IP` 换成 Quest 能访问的主机局域网 IPv4：

```bash
HOST_IP=192.168.32.10
mkdir -p configs/xr_teleoperate
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout configs/xr_teleoperate/key.pem \
  -out configs/xr_teleoperate/cert.pem \
  -subj "/CN=$HOST_IP" \
  -addext "subjectAltName=IP:$HOST_IP"
```

Quest 浏览器需要接受该自签名证书。证书和私钥不会被 Git 提交。

## 启动前检查

```bash
export REPO_ROOT="$(pwd)"
export ISAAC_PYTHON=/path/to/isaacsim/bin/python
export STUDIO_ROOT=/path/to/FlexivElementsStudio

test -x "$ISAAC_PYTHON"
test -x "$STUDIO_ROOT/RobotControlApp"
test -x "$STUDIO_ROOT/FlexivSimulation"
test -x "$STUDIO_ROOT/FlexivElementsStudio"
test -f "$REPO_ROOT/isaac_sim_ws/exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd"
test -f "$REPO_ROOT/configs/xr_teleoperate/cert.pem"
test -f "$REPO_ROOT/configs/xr_teleoperate/key.pem"
"$ISAAC_PYTHON" -c 'import flexivsimplugin; print(flexivsimplugin.__version__)'
PYTHONPATH="$REPO_ROOT/.deps/flexivrdk_1_9_1" "$ISAAC_PYTHON" -c 'import flexivrdk; print(flexivrdk.__file__)'
```

先将配置中的 `robots[0].usd` 改为本机 `Rizon4.usd` 的绝对路径。`configs/flexiv_studio_teleop.yaml` 和 `standalone_examples/.../app_config.yaml` 当前仍写有原开发机路径。

## 完整启动流程

在仓库根目录执行。每个 `start_*.py` 都会后台启动进程并把 stdout/stderr 写入 `logs/`。

```bash
# 1. Flexiv runtime 组件
python3 scripts/start_robot_control_app.py --studio-root "$STUDIO_ROOT"
python3 scripts/start_flexiv_simulation.py --studio-root "$STUDIO_ROOT"
python3 scripts/start_elements_studio_ui.py --studio-root "$STUDIO_ROOT"

# 2. Isaac 场景及 SimPlugin bridge
python3 scripts/start_isaac_follow.py \
  --isaac-python "$ISAAC_PYTHON" \
  --serial-number Rizon4-I0LIRN \
  --enable-quest-target-udp \
  --rdk-target-hz 60

# 3. Isaac :45678 -> Flexiv RDK
python3 scripts/start_rdk_target_streamer.py \
  --python "$ISAAC_PYTHON" \
  --serial-number Rizon4-I0LIRN

# 4. Quest -> Isaac :45679（前台运行）
.venv-quest/bin/python scripts/rizon4_quest_target_publisher.py \
  --host-ip "$HOST_IP" \
  --udp-host 127.0.0.1 \
  --udp-port 45679 \
  --side right \
  --enable-button squeeze \
  --axis-map=-z,-x,y \
  --position-delta-scale 1.0 \
  --position-deadband 0.05 \
  --engage-settle-sec 0.25 \
  --rate-hz 60
```

在 Quest 浏览器打开 publisher 打印的地址，例如 `https://192.168.32.10:8012/?ws=wss://192.168.32.10:8012`。接受证书、进入 VR 并允许控制器追踪。按住右手柄 `squeeze` 时，日志应显示 `ready=True enabled=True` 并持续发送。

默认端口：Quest 目标 `UDP 45679`，Isaac 到 RDK 目标 `UDP 45678`，TeleVuer HTTPS/WSS `TCP 8012`。防火墙至少需要允许 Quest 到主机的 TCP 8012。

## 运行检查与停止

```bash
python3 scripts/flexiv_stack_status.py
tail -f logs/*.stderr.log
python3 scripts/stop_flexiv_stack.py
```

若 Isaac 界面等待手动开始，点击 Play；`--no-manual-play` 可让启动参数不要求手动 Play。首次启动 Isaac 可能长时间编译 shader。

## 测试

不启动 Isaac/Studio 的快速测试只需要 Python 标准库及测试代码中用到的已安装包：

```bash
"$ISAAC_PYTHON" -m unittest discover -s tests -p 'test_*.py'
```

这些测试验证纯 Python 逻辑和仓库布局，不等价于完整硬件、Isaac、Elements Studio 端到端验证。

## 常见故障

- `Rizon4.usd` 不存在：`isaac_sim_ws/` 未取得，或 YAML/默认绝对路径仍指向原开发机。
- `flexivsimplugin==1.2.0 is required`：在 Isaac Python 中安装精确版本。
- `No module named vuer`：使用 `.venv-quest` 运行 publisher 并安装 `vuer`。
- 找不到 `arm_driver_param.xml`/URDF/SRDF：先在 Elements Studio 创建并启动过模拟机器人。
- RDK 发现不到机器人：确认 Elements Studio Remote Mode、序列号、网络接口和本机防火墙；必要时给 streamer 传 `--network-interface-whitelist <IPv4>`。
- Quest 页面打不开或 `ready=False`：确认 Quest 可访问 `HOST_IP:8012`，浏览器已接受证书并授权 WebXR 控制器。
- 机器人不动：确认 Isaac timeline 为 Play、SimPlugin 已连接、RDK streamer 正在收到 `:45678` 数据，并依次查看 `logs/` 中五个后台进程的 stderr。

## 目录

- `scripts/`：运行入口、状态和停止脚本。
- `standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/`：Isaac 场景和桥接实现。
- `local_exts/`：Isaac Sim 本地扩展。
- `third_party/televuer/`：仓库内 vendored Quest/Vuer 输入层。
- `configs/`：项目配置；PEM 证书不提交。
- `spec/`：目标和控制链路说明。
- `tests/`：快速回归测试。
