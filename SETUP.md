# 环境安装

本文档负责 Quest Isaac Flexiv 的平台要求、外部运行时安装、Python 依赖、Elements Studio 配置、Quest 证书和启动前检查。运行与录制方式见 [README.md](README.md)。

## 仓库完整性

完整流程依赖仓库外内容，仅克隆本仓库不能运行。

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

仓库内资源和输出目录统一使用相对路径：USD workspace 位于 `isaac_sim_ws/`，Stage1 输出位于 `datasets/`。Isaac Python、Elements Studio 和 RDK 等仓库外运行时不写入配置文件，请通过 `ISAAC_PYTHON`、`ISAACSIM_ROOT`、`STUDIO_ROOT`、`FLEXIV_RDK_PYTHON` 或对应 CLI 参数提供。

## 平台和硬件

- Ubuntu 22.04 x86_64（Flexiv Isaac workspace 声明的支持平台）
- 支持 Isaac Sim 6.0.1 的 NVIDIA GPU 和驱动
- Meta Quest 和控制器，与主机处于可互访的局域网
- Elements Studio 安装包及使用权限

当前验证基线是 Python 3.12.13、Isaac Sim 6.0.1、`flexivsimplugin 1.2.0`、`flexivrdk 1.9.1`。这些是已验证版本，不代表可任意替换的最低版本。

## 1. 安装 Isaac Sim

按照 NVIDIA 的 pip/Conda 方式建立 Python 3.12 环境。示例环境名为 `isaacsim`：

```bash
conda create -n isaacsim python=3.12 -y
conda activate isaacsim
python -m pip install --upgrade pip
python -m pip install 'isaacsim[all,extscache]==6.0.1' --extra-index-url https://pypi.nvidia.com
```

确认安装，并记录解释器路径：

```bash
python -c 'import isaacsim; print(isaacsim.__file__)'
export ISAAC_PYTHON="$(command -v python)"
```

Isaac Sim 对驱动和 Python 版本有严格要求；如果官方 6.0.1 安装说明与上述命令不同，以对应版本官方说明为准。

## 2. 安装 Flexiv Isaac workspace

取得 Flexiv 官方 Isaac workspace，使其内容位于仓库的 `isaac_sim_ws/`。该目录应至少包含：

```text
isaac_sim_ws/exts/isaacsim.robot.manipulators.examples/data/flexiv/Rizon4.usd
isaac_sim_ws/exts/isaacsim.robot.manipulators.examples/isaacsim/robot/manipulators/examples/flexiv/
```

`isaac_sim_ws/` 是被 Git 忽略的独立 checkout，不会随本仓库克隆。若扩展未被 Isaac 环境发现，使用 workspace 安装脚本，或将扩展加入 Isaac extension search path：

```bash
bash isaac_sim_ws/install_ws.sh /path/to/isaac-sim-root
```

## 3. 安装项目和 Flexiv Python 依赖

项目 CLI、Hydra 控制、Stage1 数据采集、转换和测试依赖统一安装：

```bash
python3 -m pip install -r requirements.txt
```

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

若 Elements Studio 明确要求其他 RDK 版本，应同时调整 `.deps` 版本和兼容性测试，不能直接假设 2.x 可替代 1.9.1。

## 4. 安装 Quest publisher 环境

Quest publisher 使用独立虚拟环境：

```bash
python3 -m venv .venv-quest
.venv-quest/bin/python -m pip install --upgrade pip
.venv-quest/bin/python -m pip install vuer numpy opencv-python
```

`third_party/televuer` 源码已随仓库提交，publisher 会自动加入其路径，不需要单独安装 TeleVuer。

## 5. 安装并配置 Elements Studio

将 Flexiv Elements Studio 解压或安装到本机，并在 UI 中创建 Rizon4 模拟机器人。首次配置必须生成下列内容：

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

在 Elements Studio 中启用 Remote Mode/Ethernet，并确认模拟机器人序列号。所有启动命令必须使用同一个序列号：

```bash
# 待填写：替换为 Elements Studio 中显示的真实模拟机器人序列号。
export ROBOT_SERIAL="Rizon4-YOUR-SERIAL"
```

## 6. 生成 Quest HTTPS 证书

将 `HOST_IP` 换成 Quest 能访问的主机局域网 IPv4：

```bash
export HOST_IP=192.168.32.10
mkdir -p configs/xr_teleoperate
openssl req -x509 -newkey rsa:2048 -nodes -days 365 \
  -keyout configs/xr_teleoperate/key.pem \
  -out configs/xr_teleoperate/cert.pem \
  -subj "/CN=$HOST_IP" \
  -addext "subjectAltName=IP:$HOST_IP"
```

Quest 浏览器需要接受该自签名证书。证书和私钥不会被 Git 提交。

## 7. 启动前检查

在仓库根目录设置本机路径：

```bash
export REPO_ROOT="$(pwd)"
export ROBOT_SERIAL="Rizon4-YOUR-SERIAL"  # 待填写
export ISAAC_PYTHON="../path/to/isaacsim/bin/python"
export ISAACSIM_ROOT="../path/to/isaacsim"
export STUDIO_ROOT="../path/to/FlexivElementsStudio"
export FLEXIV_RDK_PYTHON="../path/to/rdk_venv/bin/python"

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

Stage1 scene 配置使用相对于配置文件的仓库路径，不需要填写绝对 USD 路径。旧版 `configs/flexiv_studio_teleop.yaml` 和 `standalone_examples/.../app_config.yaml` 使用时，按其注释填写资源路径。

检查通过后，返回 [README.md 的完整启动流程](README.md#完整启动流程)。

## 环境故障排查

- `Rizon4.usd` 不存在：确认已取得 `isaac_sim_ws/`，并检查 scene YAML 中的相对路径。
- `flexivsimplugin==1.2.0 is required`：使用 `$ISAAC_PYTHON` 安装精确版本，不要装到另一个 Python 环境。
- `No module named vuer`：使用 `.venv-quest/bin/python` 运行 publisher，并确认该环境已安装 `vuer`。
- 找不到 `arm_driver_param.xml`、URDF 或 SRDF：先在 Elements Studio 创建并启动一次模拟机器人，确认 `user_data_ui/simDir` 和 `specs/robots` 已生成。
