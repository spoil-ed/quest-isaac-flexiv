# Quest Isaac Flexiv

本仓库是基于原始 Isaac Sim / Flexiv 工程维护的主线项目层，只保留当前 Quest 遥操、Elements Studio/RDK 控制链路、脚本入口、配置和测试。原始示例只作为参考，新方案代码放在本仓库内维护。

## 目标

用 Quest 手柄位姿控制 Isaac Sim 中的 Flexiv Rizon4。Isaac Sim 负责场景、机器人状态读取和力矩执行；控制逻辑走 Flexiv runtime / Elements Studio 可视化链路，不使用 Isaac 自带 IK，也不使用 jog / CartesianJogging 作为跟随控制方案。

主线闭环：

```text
Quest controller pose
  -> scripts/rizon4_quest_target_publisher.py
  -> UDP target pose
  -> Isaac TargetFrame / follow bridge
  -> Flexiv runtime / RDK control stack
  -> target_drives / torque
  -> Isaac apply_torques
  -> q, dq feedback
  -> next control frame
```

控制语义：

- 按住右手柄 `squeeze` 时，publisher 只发送按下后的相对位移 `controller_delta_base`，Isaac 侧把该 delta 加到按下瞬间的 TCP 位姿上。
- 松开 `squeeze` 时暂停发送目标，不回到固定零点。
- 默认 Rizon4 序列号为 `Rizon4-I0LIRN`。

## 目录

- `scripts/`: 所有可执行入口和 runtime helper。
- `standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/`: Isaac 侧 Flexiv/Quest demo 资产和桥接代码。
- `local_exts/`: Isaac Sim 本地扩展。
- `third_party/televuer/`: 仓库内 vendored Quest/Vuer 输入层。
- `configs/`: 项目配置。`configs/xr_teleoperate/*.pem` 是本地 HTTPS 证书，不提交。
- `spec/`: 简要目标和方案文档。
- `tests/`: 快速回归测试。

## 启动

分别启动各组件：

```bash
cd /home/simate/workspace/isaacsim-flexiv

python scripts/start_robot_control_app.py
python scripts/start_flexiv_simulation.py
python scripts/start_elements_studio_ui.py
python scripts/start_isaac_follow.py --enable-quest-target-udp --rdk-target-hz 60
python scripts/start_rdk_target_streamer.py
```

启动 Quest 输入 publisher：

```bash
python scripts/rizon4_quest_target_publisher.py \
  --host-ip 192.168.32.10 \
  --udp-host 127.0.0.1 \
  --udp-port 45679 \
  --side right \
  --enable-button squeeze \
  --axis-map x,y,z \
  --position-delta-scale 3.0 \
  --rate-hz 60
```

在 Quest 浏览器打开 publisher 打印的 HTTPS 地址，例如：

```text
https://192.168.32.10:8012/?ws=wss://192.168.32.10:8012
```

进入 VR 后，按住右手柄 `squeeze` 并移动手柄。publisher 日志中应从 `ready=False` 变为 `ready=True`，按住时 `enabled=True` 且持续发送 UDP target pose。

## 状态检查

```bash
python scripts/flexiv_stack_status.py
python scripts/stop_flexiv_stack.py
```

## 测试

```bash
python -m unittest discover -s tests -p 'test_*.py'
```
