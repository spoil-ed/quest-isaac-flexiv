# Flexiv 双臂控制架构

本文是本仓库的控制逻辑规范。它说明控制目标怎样产生、怎样进行坐标变换、怎样分发给左右机械臂，以及 Studio、RDK、SimPlugin 和 Isaac Sim 在闭环中的职责。

本文不记录安装方法、进程修复过程或历史故障。运行命令和部署步骤属于 `README.md` 与 `docs/`。

## 1. 控制目标

系统控制两台 Flexiv Rizon4：

- 左臂由 Docker 内的 Elements Studio/FlexivSimulation 提供控制器。
- 右臂由宿主机上的 Elements Studio/FlexivSimulation 提供控制器。
- Isaac Sim、Quest 输入、RDK/DRDK target transport、gateway 和 recorder 都运行在宿主机。
- 双臂控制是两套 Stage1 单臂控制的并列组合。左臂多出的 Docker 层只改变 Studio 的部署位置，不改变控制算法。

最终控制链如下：

```text
左手柄 ─┐                     ┌─> 左目标 ─┐   ┌─> Docker 左 Studio
        ├─> Quest publisher ─>│          ├──>│
右手柄 ─┘                     └─> 右目标 ─┘   └─> 宿主机右 Studio
                                      NRT RDK/DRDK transport
                                               │
                              FlexivSimulation <─ SimPlugin ─> Isaac 双臂 articulation
```

每只机械臂拥有独立的：

- serial alias；
- Quest 接管状态；
- 手柄零点；
- RDK TCP 零点；
- 坐标标定；
- Cartesian target limiter；
- RDK readiness；
- SimPlugin 连接状态；
- 最后一个有效目标。

任何一侧的输入、零点或状态都不得复用于另一侧。

## 2. 各层控制职责

### 2.1 Quest publisher

`scripts/rizon4_quest_target_publisher.py` 只负责读取手柄和生成低频用户目标：

- 一个 TeleVuer 会话同时读取左、右手柄；
- 左手包使用 `side=left` 和左臂 serial；
- 右手包使用 `side=right` 和右臂 serial；
- 双手首次同时按下 `squeeze` 用于确认并锁定共享 Quest-to-RDK 坐标系；
- 坐标系锁定后，每只手的 `squeeze` 独立控制对应机械臂；
- 默认以 30 Hz 发布；
- 不计算关节角、关节速度或关节力矩。

### 2.2 Isaac 双臂应用

`standalone_examples/api/isaacsim.robot.manipulators/flexiv_quest/dual_follow_with_studio.py` 是目标路由器和仿真执行器：

- 根据 packet 的 `side` 和 serial 将输入送到对应机械臂；
- 管理 squeeze 接管状态和相对零点；
- 将 Quest 或 TargetFrame 目标转换到对应机械臂的 RDK base frame；
- 对 Cartesian 目标做速度限制；双臂墙面安装路径默认不做工作空间裁剪；
- 以 30 Hz 向两个 streamer 发布目标；
- 以 2000 Hz 执行两套 SimPlugin 物理闭环；
- 将 Studio 返回的原始关节力矩施加到 Isaac articulation。

Isaac 应用不使用自己的 IK 替代 Studio，也不根据 30 Hz 目标自行计算控制力矩。

### 2.3 RDK/DRDK target transport

目标传输层有两种互斥 backend：

- RDK fallback：每只机械臂运行一个 `scripts/rdk_target_streamer.py`，左右连接和故障相互独立；
- DRDK：运行一个 `scripts/drdk_target_streamer.py`，通过 `RobotPair` 对两个 runtime 原子提交同一 `servo_cycle` 的左右目标。

两种 backend 都必须使用 `NRT_CARTESIAN_MOTION_FORCE` 和 `SendCartesianMotionForce()`。30 Hz 的 Quest/TargetFrame 进程不满足 RT 流式接口的严格周期要求；NRT 使轨迹生成、Cartesian 控制和关节/力矩解算留在 Flexiv runtime 内。

DRDK transport 对低频目标增加一层因果 SE(3) 重采样，但不改变控制模式：每收到一对同步的 30 Hz 左右目标，以相邻位置差分估计 base 系线速度，以 `q_k * inverse(q_{k-1})` 的最短旋转向量估计 base 系角速度；速度先按 Cartesian 上限裁剪，再经过低通、加速度斜率限制和静止死区。两个输入帧之间，NRT 循环按恒速模型预测位置，并通过 `exp(omega * h) * q_k` 预测方向。每个周期调用 `SendCartesianMotionForce(poses, velocities=twists, ...)`；超过预测窗口后保持最后预测 pose 且速度前馈归零，绝不无限外推。该层只用于降低 30 Hz 输入的阶梯感和补偿少量链路延迟，不能制造 Quest 未观测到的信息，也不把 NRT 伪装成 RT。

当前 pipeline 使用 500 Hz NRT 重采样、12 ms 预测窗口和 0.65 速度滤波系数。估计速度全量前馈，并按两套 Studio 实际加载的 normal safety 上限裁剪到 `3.0 m/s`、`12.0 rad/s`；NRT 操作速度使用相同上限，加速度保持 `8.0 m/s²`、`30 rad/s²`，因为 Studio 配置没有公布对应的安全最大加速度。冷启动和 reset 的接口目前使用统一关节速度标量，因此均取生成 Rizon4 模型最慢 J1/J2 的 `dq_max=2.0944 rad/s`；加速度分别保持 `2.0 rad/s²`、`3.0 rad/s²`。实际循环允许因主机调度错过周期，错过后直接推进到下一时刻，不突发补发旧命令。松开 squeeze、控制源切换、协调 reset、接触冻结或关节力矩回退时，必须立即清零该侧速度估计和预测状态，以实际 hold/reference pose 重新建立连续基准。

为避免轻微接触直接演变为 Studio fault，Cartesian 刚度使用额定 80%、阻尼使用 0.8。DRDK 创建 `RobotPair` 后必须在两臂 `IDLE` 时通过其底层 RDK Robot 实例分别调用官方 `Safety.SetJointOutputTorqueRegulator(0.85, 50)`；安全密码只从 pipeline 指定的环境变量读取，不得写入仓库。这样 A3/A4 的控制器输出饱和值由默认 `64×1.3=83.2 Nm` 降为 `64×0.85=54.4 Nm`，在 Studio 的 `tau_max=64 Nm` 前保留余量；该调节器不修改 `tau_max`，也不能抵消碰撞和惯性产生的附加力矩。关节力矩风险比从 0.58 起连续降速，在 0.72 执行 `JointTorqueGuard` 回退/冻结，低于 0.55 并稳定 0.15 s 后自动解除；冻结时预测和速度前馈归零，同时保留 0.20 的正向 NRT 限速使 0.05 s 前的安全回退点可达。接触保护允许 `30 N / 5 Nm`，单样本达到 90% 即保持当前 TCP，低于 55% 并稳定 0.12 s 后自动重建相对零点。满刚度/满前馈实验已实测使右 A3 在 guard 采到 54.5 Nm 后继续冲到 64.39 Nm fault，不能作为采集默认值。

双手共享坐标系默认采用 tracking + 距离 + 双 squeeze 标定：两只控制器均有 OpenXR tracking，双手三维间距与 scene 初始双 TCP 间距相差不超过 0.03 m，并且双手 squeeze 同时达到 0.15、稳定 0.25 s 才锁存。相对姿态不参与默认门控。首次锁存、松开后再次按下都以当下手柄和 TCP 作为相对零点，第一帧位移严格为零；松开保持最后目标，不回初始位姿。reset 后仍要求先释放再双手 squeeze，防止旧按键状态自动重锁存。

传输层负责：

- 连接指定的一对或单个 serial；
- 读取 runtime 当前 TCP，形成各侧启动参考；
- 接收 Isaac 发布的 `pose_base_tcp_des`；
- 由相邻 30 Hz 样本估计 SE(3) twist，在高频循环内做有界短时预测并把 pose/velocity 一同交给 NRT Cartesian motion-force 接口；
- 向 Isaac 返回 `ready`、参考 TCP 和实时 TCP；
- 没有新用户目标时保持最后有效目标。

DRDK 初始化顺序固定为：从与 Isaac 相同的 scene config 读取左右 `robots[].initial_q`、在 Isaac 前启动初始发现循环、SimPlugin 以安全 `bootstrap_q` 使两个 runtime connected、创建 `RobotPair` 并进入 `NRT_JOINT_POSITION`、读取切换后的当前 q 并先发送为保持指令以建立无跳变控制基准、再调用 `SendJointPosition(init_q)` 让 runtime 平滑到两侧 initq、等待关节位置和速度连续稳定、切换 `NRT_CARTESIAN_MOTION_FORCE`、重新调用 `SetNullSpacePosture((init_q_left, init_q_right))`、锁存两臂当前 TCP，最后同时发布任务 ready。初始发现可以在限定超时内重试；连接成功后的断连或 fault 必须锁存为双侧 not-ready 并等待显式 reset，不得自动清 fault 或恢复用户目标。

进入 Cartesian 模式后，streamer 还必须从唯一 pipeline 的 `control.drdk.nullspace_objectives` 调用 `SetNullSpaceObjectives()`。当前两臂统一使用平移可操作度 `0.2`、旋转可操作度 `0.2`、参考姿态跟踪 `0.8`，在保留 `initial_q` 构型偏好的同时给 runtime IK 留出改善可操作度的优化空间。这三个权重只影响冗余关节解选择，不修改 TCP 主任务、速度、阻抗或接触限制；每次重新进入 Cartesian 模式都必须和 `SetNullSpacePosture()` 一起重设。

启动器从唯一 pipeline 的 `control.drdk` 读取 DRDK 安全参数，并默认创建官方 DRDK `SelfCollisionMonitor`；`SELF_COLLISION_MONITOR=true/false` 只用于临时覆盖。它使用 scene `robots[].position` 作为两侧 base 在共同世界坐标中的平移，以 10 ms 周期检查两臂几何、保留 0.15 m 最小距离；默认不跳过任何 link。15 cm 是制动余量而不是碰撞体厚度：实测高负载下 `Stop()` 可耗时约 724 ms，原 5 cm 阈值会在停止完成前让 link7 继续接近并触发 A3/A4 硬力矩 fault。启动时先用确定性的 NRT 轨迹到达已知安全的 `initial_q`，锁存 Cartesian 参考后才启动监视器，避免初始化路径中的短暂近距离把正常系统锁成 `self_collision_stopped`。距离不足时 DRDK 无视当前任务和控制模式停止双臂，streamer 以“RobotPair 已离开预期 Cartesian mode”识别 monitor 的异步 `Stop()`，锁存 `self_collision_stopped` 并停止转发 Quest 目标；不能只用 `RobotPair.stopped()` 判断，因为双臂正常静止保持位姿时它同样为 true。若最后一帧 Cartesian 命令与异步 `Stop()` 竞态并被 runtime 拒绝，该异常必须转换为 `self_collision_stopped` 状态而不能退出 streamer，确保 Web Reset 始终有活着的接收端。协调 reset 必须先停止该监视器，再允许 NRT 关节轨迹把已过近的双臂分开并回到安全 `initial_q`，切回 Cartesian 后重新创建监视器；否则 monitor 会在近距离条件尚未解除时持续 Stop，使 reset 无法起步。仅有 `self_collision_stopped` 而两侧 runtime 未进入 fault 时属于可软恢复状态，Web Reset 应完成上述事务；若 Stop 后的残余碰撞继续触发 Studio 关节力矩 fault，则仍尝试最多 3 次 `ClearFault → Enable`，任一侧 `ClearFault()` 持续失败即属于硬故障，禁止自动重新接管，必须冷重启 Studio/DRDK 控制栈。该模块只覆盖左右机械臂之间的自碰撞，不覆盖桌面、工件等场景接触；监视器是独立生命周期对象，`RobotPair` 因断连重建时必须随之重建，进程退出时必须停止。

环境接触保护使用 `RobotPair.SetMaxContactWrench((left_limit, right_limit))`，默认每侧 `[30,30,30,5,5,5]`（前三项 N，后三项 Nm），且每次重新进入 `NRT_CARTESIAN_MOTION_FORCE` 都必须重设。该接口负责 runtime 内层接触 wrench 调节，不等同于停止目标生成。streamer 因此还读取每侧 `states().tcp_wrench`，独立运行带迟滞的目标冻结器：达到上限 90% 时锁存该侧实际 TCP 为 hold pose，另一侧不受影响；六个分量全部降到上限 55% 以下并稳定 0.12 s 后解除。解除时恢复最新原始目标，由 NRT 内部轨迹生成器按照速度和加速度上限平滑追上；不得重建永久输入/输出 offset，否则 TargetFrame 与真实命令会在多次保护后持续分离。reset 或重新锁存 Cartesian reference 时必须清空两侧冻结和计数状态。`tcp_wrench` 与 `contact_frozen` 随 status 数据发布，供诊断确认实际仿真反馈是否有效。

关节力矩前置保护封装在 `JointTorqueGuard`，不介入 2 kHz 力矩解算。streamer 从 `RobotPair.info().tau_max` 读取左右每关节软件上限，并在每个 target 周期读取 `states().tau`、`tau_dot` 和 `tau_ext`。每个关节还以连续 `tau` 样本计算并滤波有限差分斜率；预测时在 runtime `tau_dot` 和本地斜率之间选择能产生更大绝对预测力矩的一项，避免仿真 runtime 始终返回零 `tau_dot` 时丢失提前量。单关节风险比定义为 `max(|tau|, |tau_ext|, |tau + tau_dot_effective * 0.005|) / tau_max`，并要求连续 3 个危险样本才触发，抑制 Isaac 单周期力矩导数尖峰。触发后停止采用该侧新 Quest 目标，并通过 `SendCartesianMotionForce()` 回退至至少 0.05 s 前发送的最近安全 target；另一侧继续独立工作。全部关节风险比低于 0.55 并持续 0.15 s 后解除，随后恢复最新目标并由 NRT 内部轨迹生成器有界追上，不保留永久 offset。reset 或重新锁存 Cartesian reference 必须清空历史和冻结状态，并以当前 reference pose 建立第一条安全历史。status 必须发布 `joint_tau`、`joint_tau_dot`、`joint_tau_ext`、`joint_tau_max`、`joint_torque_ratio` 与 `joint_torque_frozen`。

唯一 pipeline 的 `control.joint_effort_limits_nm` 把两套 Isaac articulation 的 J1..J7 最大 effort 设置为 `[150,150,80,80,49,49,49] Nm`，并在初始化时回读确认；该值与 Studio 模拟 Rizon4 已有的 safety-function 配置一致。`RobotInfo.tau_max` 仍提供更低的额定保护基准，供 72% 前置回退使用；Isaac 上限不能代替该回退或更低的 TCP 接触 wrench 限制。

协调 reset 是控制协议的一部分。recorder 按 `r` 后经 gateway 生成单调递增的 `reset_seq`；Isaac 立即撤销双臂用户目标、清空 Quest clutch/mapper/limiter 和 TCP 标定，并把同一序号随左右 `flexiv_target_pose.v1` 包重复发送，直到 DRDK 确认。DRDK 对每个新序号只执行一次以下状态机：

1. Isaac 先撤销用户控制并临时关闭配置中场景物体的碰撞；刚体工件同时暂时设为 kinematic，避免失去桌面支撑后掉落。两臂 articulation 的 collision shape 不能在运行中删除、重建或禁用，否则会使 PhysX articulation tensor view 失效。这样已与环境穿插或挤压的机械臂可以脱离接触，避免清 fault 后的回程立即再次触发力矩保护。
2. `reset_stopping`：调用 `RobotPair.Stop()` 对两臂做软件受控停止；这不是实体急停按钮。
3. `reset_clearing_fault`：若存在 fault，则调用一次 `ClearFault()`，且必须确认两侧都成功。
4. 重新 `Enable()` 并等待 RobotPair operational。
5. `joint_initializing`：切换 `NRT_JOINT_POSITION`，先以切换后的当前 q 调用 `SendJointPosition()` 建立连续基准，再调用 `SendJointPosition(init_q_left, init_q_right)`；reset 使用独立的低速/低加速度上限，由 runtime 生成关节轨迹，等待位置和速度连续稳定。
6. 切换 `NRT_CARTESIAN_MOTION_FORCE`，重设 `SetNullSpacePosture(initial_q)`，重新锁存两侧当前 TCP，清空旧 Cartesian packet，并发布带相同 `reset_seq` 的双侧 ready。
7. Isaac 只在两侧 status 都确认该序号、关节状态位于任务 `initial_q` 且 SimPlugin effort 闭环正常时，才把全部 `scene_objects` 恢复到 scene config 声明的初始位置、姿态、缩放和关节状态，并把刚体线速度/角速度清零。资产保持 kinematic 且无碰撞至少一个目标更新周期，再恢复原 collision/kinematic 状态并报告 `succeeded`；recorder 收到该状态后才允许继续录制。

reset 不调用 Isaac `world.reset()`，不瞬时写 articulation，也不使用旧 TCP target pose 代替任务 `initial_q`。如果回程中再次出现 fault，同一 `reset_seq` 会执行有界恢复：重新 `Stop → ClearFault → Enable`，从新的实际 q 建立连续基准并继续回最终 `initial_q`；默认最多 3 次，不能无限重试。只有两臂到位、切回 Cartesian、重装零空间并完成场景资产复位后才发布成功。耗尽次数才发布 `reset_failed` 和最后错误原因；此时保持用户控制关闭及场景物体碰撞暂时禁用，使下一次按 `r` 可以重新执行完整事务。机械臂 articulation 自身的碰撞始终保持启用，recorder 也不因一次 reset 失败退出。

SimPlugin 的 2 kHz 状态—力矩闭环只依赖对应 runtime 已连接，不依赖 RDK/DRDK ready：每个物理周期先发送两臂状态，再等待并原样应用两套 runtime 的 `target_drives`。这是官方多机器人 bridge 的控制顺序，也避免 runtime 在 DRDK 尚未完成成对发现时因力矩未被应用而误判负载。DRDK ready 只门控 Cartesian/Quest/TargetFrame 用户目标，不门控底层 Studio 力矩闭环。

正式采集入口 `start_all.sh` 组合三个生命周期仍相互独立的进程组：控制栈、Web dashboard、recorder。Isaac 以 10 Hz 将 `flexiv_dual_arm_state.v1` 单播给 dashboard；recorder 继续作为 gateway `5790` 的唯一采样消费者，dashboard 禁止连接该端口，避免状态刷新与 30 Hz 图像/动作采集争用请求—应答连接。dashboard 通过本机 UDP `57687` 发送有限集合 `start/pause/save/discard/reset/quit`，recorder 通过 `57688` 回报 `flexiv_recorder_status.v1`。按钮只改变 recorder 状态机；`reset` 仍由 recorder 经 gateway 进入既有协调 reset 协议，网页不得直接向 Isaac、DRDK 或 Studio 发运动命令。控制栈重启但 recorder 按设计保留时，gateway 的旧 TCP 请求—应答连接会失效；recorder 必须在 EOF、连接复位或 broken pipe 后重连并只重发当前请求一次，响应超时则不得自动重发，以免同一个 reset 被重复提交。Dashboard 默认 HTTP `8080` 仅面向可信局域网，不提供任意命令执行接口。

streamer 不进行 OpenXR 坐标变换，不做 IK、动力学或力矩解算。DRDK 的同步发送只耦合传输时序，不把两份 Stage1 用户控制逻辑合成一个控制器。

### 2.4 Elements Studio 与 FlexivSimulation

Studio/FlexivSimulation 是机械臂控制器：

- 接收 RDK Cartesian TCP 目标；
- 在其内部控制周期计算关节控制量；
- 通过 SimPlugin 接收 Isaac 的 `q`、`dq`；
- 通过 SimPlugin 返回七轴 `target_drives`。

左、右 Studio 的算法职责完全相同。Docker 只隔离左侧运行环境。

### 2.5 SimPlugin 与 Isaac 物理

每个 0.5 ms 物理步执行：

```text
Isaac q,dq
  -> SimPlugin.SendRobotStates(...)
  -> Studio/FlexivSimulation controller
  -> SimPlugin.GetCommands()
  -> target_drives
  -> Isaac articulation.apply_torques(target_drives)
```

`target_drives` 必须原样用于对应机械臂：

- 不抽帧；
- 不合并；
- 不平均；
- 不按 30 Hz/200 Hz 比例缩放；
- 不额外加入自定义关节速度保护或位置约束；
- 左右力矩不得交叉。

## 3. 两层频率

系统明确分为低频目标层和高频物理控制层。

| 层 | 默认频率 | 作用 |
| --- | ---: | --- |
| Quest 采样与发布 | 30 Hz | 读取用户手柄输入 |
| TargetFrame/Quest target 更新 | 30 Hz | 更新 Cartesian 目标 |
| RDK/DRDK NRT target 提交 | 30 Hz | 将离散目标送入两套 runtime |
| Isaac 渲染 | 30 Hz | GUI 与相机显示 |
| SimPlugin 物理闭环 | 2000 Hz | `q,dq -> Studio -> torque -> physics` |

目标层的 30 Hz 不改变动力学积分步长。Isaac 物理步长始终为：

```text
physics_dt = 1 / 2000 = 0.0005 s
```

一次约 33.3 ms 的目标周期内会执行约 66 至 67 次物理子步。每个子步都有独立的状态发送、力矩接收和物理积分。

## 4. 统一控制坐标系

### 4.1 规范坐标系

所有发给 Studio 的 Cartesian 目标统一表示为 Flexiv RDK base frame 下的 TCP pose：

```text
pose_base_tcp = [x, y, z, qw, qx, qy, qz]
```

以下内容都必须在 RDK base frame 中处理：

- 当前 TCP；
- 目标 TCP；
- 相对位移；
- 工作空间；
- Cartesian 速度限制；
- squeeze 零点；
- 松开后的保持目标。

Isaac world frame 和 OpenXR frame 只能在输入或显示边界转换，不能成为内部控制状态的混合表示。

### 4.2 Isaac world 与 RDK base 对齐

墙装 USD articulation 的 root frame 不保证等于 Studio/RDK 实际运动学基座。因此不得仅使用 USD root pose 猜测 RDK 坐标。

每只机械臂启动时同时取得：

- Isaac 中的末端世界位姿 `T_world_tcp_0`；
- RDK 读取的当前 TCP 位姿 `T_base_tcp_0`。

由同一机械臂、同一时刻的位姿对计算：

```text
T_world_base = T_world_tcp_0 * inverse(T_base_tcp_0)
```

之后：

```text
T_base_tcp_target = inverse(T_world_base) * T_world_target
T_world_target    = T_world_base * T_base_tcp_target
```

左、右机械臂必须分别计算 `T_world_base`。

### 4.3 OpenXR 平移轴映射

Quest 手柄位置增量先通过 signed permutation 映射到规范控制轴：

```text
delta_base = scale * axis_map * (p_hand - p_hand_zero)
```

当前 Stage1 兼容映射为：

```text
axis_map = -z,-x,y
```

其含义是：

```text
base_x = -openxr_z
base_y = -openxr_x
base_z =  openxr_y
```

轴映射只执行一次。publisher 已经发送 `controller_delta_base` 时，Isaac 不得再次应用 `axis_map`。

### 4.4 双手确认与共享坐标系标定

固定 `axis_map` 只能统一 OpenXR 和 RDK 的轴定义，不能消除用户初始站位相对机器人 base 的水平角度误差。双臂 publisher 因此必须在开放任何 Quest 目标前，用同一时刻的左右手柄建立一个共享标定坐标系。

未标定时，双手 pose 持续发布给状态监视，但不发布 `rizon4_quest_target.v1`。当前启动方案启用独立距离门控：两只手柄都有 tracking、三维间距与 scene 初始双 TCP 间距相差不超过 0.03 m，并同时按住 `squeeze` 0.25 s，才允许锁定共享坐标系。相对位姿不参与默认门控，也不在 Web 面板显示。

坐标系确认和机械臂接管使用同一次双 squeeze。publisher 在确认帧锁定共享坐标系；左右 mapper 同时以该帧手柄 pose 建立零点，Isaac 同时以当前左右 TCP 建立输出锚点，因此第一条目标的平移和旋转增量必须严格为零。用户无需松开或二次按压，继续保持 squeeze 并移动即可相对跟随。

可选的 `--strict-shared-calibration` 调试模式仍可传入 scene TCP 参考并比较下列共同刚体变换下的不变量，但默认 `start.sh` 不启用该模式：

```text
norm(u_right-u_left) ~= norm(r_right-r_left)
inverse(R^q_left)  * (u_right-u_left) ~= inverse(R^r_left)  * (r_right-r_left)
inverse(R^q_right) * (u_left-u_right) ~= inverse(R^r_right) * (r_left-r_right)
inverse(R^q_left)  * R^q_right ~= inverse(R^r_left) * R^r_right
```

通过后，分别由 Quest 手柄对和机器人 TCP 对构造三维正交基：横向轴取左减右连线，前向轴取两侧局部 `+Z` 指向的平均值并投影到横向轴的正交平面，第三轴由叉乘得到。令两组列基分别为 `B_q`、`B_r`，则共享映射为 `C=B_r*transpose(B_q)`。若平均前向与连线近似平行，使用与横向轴不共线的确定性后备轴。两手中点是标定坐标系的概念原点；由于控制只使用每只手的相对增量，平移原点会在相减时抵消。`C` 在确认瞬间锁存，松开 squeeze、暂停或重新接管都不得改变；协调 reset 或重启 publisher 才会使它失效。

协调 reset 开始后，Isaac 通过独立的本机 UDP 标定失效通知，把同一个 `reset_seq` 重复发送给 Quest publisher。publisher 按序号去重后清除共享旋转和左右相对 mapper，且必须先观测到旧的双 squeeze 已释放；随后两手 tracking 有效且再次同时按住 squeeze 即可重新锁定坐标系。在重新标定前只发布原始 Quest 诊断和夹爪输入，不发布机械臂跟随目标。

锁定后，位置和旋转增量必须使用同一 `C`：

```text
delta_base = scale * C * A * (p_hand - p_hand_zero)
R_delta_base = C * R_hand(t) * inverse(R_hand_zero) * inverse(C)
```

因此不是“直接修改 XYZ”：初始水平角度误差会同时作用到平移方向和姿态旋转轴。左右手共享 `C`，但锁定后的手柄零点、RDK TCP 零点、目标保持和 limiter 仍然逐臂独立。

## 5. Squeeze 相对遥操状态机

共享坐标系确认完成后，每只手独立执行以下状态机：

```text
DISENGAGED --squeeze rising--> ALIGNING --> TRACKING
    ^                                      |
    └-------------squeeze released---------┘
```

夹爪不属于该 squeeze 状态机。左右手柄 `trigger` 分别产生独立的夹爪命令：按下时闭合，松开时张开。夹爪命令即使在 squeeze 松开、TCP 不跟随时也持续有效，并由 Isaac Sim 直接调用对应 Grav 夹爪控制器，不经过 Studio 或 RDK。

### 5.1 DISENGAGED

- 手柄可以任意移动；
- 不更新机械臂目标；
- 机械臂保持上一条有效 TCP 目标；
- 绝不退回场景初始化 TargetFrame；
- 绝不因为 packet 超时回 home。

### 5.2 squeeze 上升沿与 ALIGNING

按下 squeeze 时，为该侧记录两个零点：

```text
p_hand_0 = 当前手柄位置
T_base_tcp_0 = 当前机械臂 RDK TCP
R_hand_0 = 当前映射到 RDK base 的手柄方向
```

位置和方向同时建立相对零点。首次有效 packet 的完整位姿目标必须等于当前 RDK TCP，不能因为手柄当前的绝对方向产生姿态跳变。可使用短暂 settle window 持续更新手柄零点，过滤按键按下瞬间的抖动。

### 5.3 TRACKING：相对位置

按住 squeeze 后，目标位置为：

```text
p_des = p_tcp_0 + scale * axis_map * (p_hand - p_hand_0)
```

关键性质：

- 使用手柄位移差，不使用手柄绝对位置；
- 使用按下瞬间的 RDK TCP，不使用场景初始 TargetFrame 作为锚点；
- 左右手各自拥有 `p_hand_0` 和 `p_tcp_0`；
- 手柄移动多少，目标相对移动多少，再由 scale 调整；
- 目标速度限制在得到 `p_des` 后应用。双臂墙面安装路径默认禁止工作空间裁剪，因为其 RDK 基座坐标中的有效 TCP Z 可为负值，不能复用直立安装 Stage1 的正 Z 边界；只有任务明确提供同一 RDK 基座坐标系下的有效边界时，才允许显式开启裁剪。

### 5.4 TRACKING：相对姿态

按住 squeeze 后，末端只跟随手柄相对按下瞬间的旋转增量：

```text
R_delta = R_hand * inverse(R_hand_0)
R_des = R_delta * R_tcp_0
```

关键性质：

- squeeze 同时为 XYZ 和方向建立零点；
- 手柄在按下后的旋转量决定末端旋转量，手柄按下前的绝对方向不影响机械臂；
- 每次重新按下都以实时 RDK TCP 方向为新的 `R_tcp_0`，因此可以像位置一样分段调整手柄；
- OpenXR 姿态必须使用与平移相同的轴变换进行基变换，不能直接复制四元数分量；
- 目标方向经过角速度限制后再发送给 RDK，同时用于 CommandFrame 可视化；
- `relative` 是默认控制策略；`packet` 仅保留为旧的绝对姿态兼容模式，`reference` 用于明确要求锁定按下时末端方向的任务。

### 5.5 squeeze 松开

松开 squeeze 时：

```text
T_hold = 最后一个经过 limiter 的有效 T_base_tcp_des
```

随后：

- 停止根据手柄更新目标；
- 持续保持 `T_hold`，或由 streamer 锁存相同目标；
- 手柄在松开期间的移动完全忽略；
- 不恢复到 home；
- 不恢复到初始 TargetFrame；
- 不把实时手柄位置继续累计到旧零点。

再次按下 squeeze 时，以新的手柄完整位姿和当前 RDK TCP 完整位姿重新建立零点。这样可以通过多次“按住移动/旋转、松开重新摆手”的方式扩展操作范围。

## 6. 左右手分流

Quest UDP 接收端按照以下两个字段共同分流：

```text
side   = left | right
serial = 对应机械臂运行时 alias
```

处理规则：

- `side=left` 只能更新左臂状态；
- `side=right` 只能更新右臂状态；
- serial 与该侧运行时配置不一致的 packet 必须丢弃；
- 一只手未按 squeeze 时，不影响另一只手；
- 独立 RDK backend 中，一侧未 ready 时只暂停该侧目标交接；DRDK backend 中，任一侧未 ready 会使 RobotPair 两侧同时 not-ready；
- 一侧 SimPlugin 断开时，另一侧继续独立运行。

一个 TeleVuer/HTTPS 会话应同时发布左右手，不能通过启动两个争用同一服务端口的 publisher 来实现双手控制。

## 7. TargetFrame 控制

TargetFrame 和 Quest 是两种用户目标输入，但共用同一条 RDK/Studio/SimPlugin 控制主线。

### 7.1 手工 TargetFrame 模式

- 左 Frame 只控制左臂；
- 右 Frame 只控制右臂；
- Frame 世界位姿通过该侧 `T_world_base` 转换为 RDK TCP 目标；
- 启动时 Frame 应与该侧末端当前世界位姿一致；
- 用户真正拖动 Frame 后才激活目标控制。

### 7.2 Quest 模式

- Frame 不是 Quest 相对运动的数值锚点；
- 相对零点来自实时 RDK TCP；
- Quest 的相对 XYZ 与相对方向组成原始目标 `T_base_tcp_goal`；
- 较大的 `/World/TargetFrameLeft`、`/World/TargetFrameRight` 分别显示两侧原始 Quest 目标，不等待速度限制器追上；
- 较小的 `/World/CommandFrameLeft`、`/World/CommandFrameRight` 分别显示 limiter 输出、实际下发给 streamer 的控制目标；
- 原始 Frame 与 CommandFrame 重合时表示 limiter 已追上输入；两者持续分离表示速度或角速度限制正在形成跟随延迟；
- `T_base_tcp_goal` 通过该侧启动时标定得到的 `T_world_base` 转换到 Isaac 世界坐标后显示；
- 发送给 streamer 的 `T_base_tcp_control` 由 limiter 逐步逼近 `T_base_tcp_goal`；
- 2000 Hz callback 只保存待显示的目标快照，绝不在其中修改 USD；
- Isaac 主渲染循环以 30 Hz 更新可视 Frame，避免场景写操作阻塞 SimPlugin 物理闭环；
- 松开 squeeze 后 Frame 保持最后的 `T_base_tcp_goal`，limiter 可继续让机械臂逼近该固定目标；
- 再次按下 squeeze 时，以当前 RDK TCP 和当前手柄建立新的完整位姿零点；
- recorder 同时记录 RDK 控制目标和 Frame 世界位姿，使显示结果与实际控制命令可以交叉验证。

## 8. Cartesian 目标限制

limiter 位于用户目标层，不位于 2000 Hz 力矩层。

推荐处理顺序：

```text
相对手柄输入
  -> RDK base 目标
  -> workspace clamp
  -> 最大线速度限制
  -> 最大角速度限制
  -> 30 Hz target packet
```

第一次接管时，limiter 必须从当前 RDK TCP reset。松开后保存的是 limiter 的最后输出，而不是未经限制的原始手柄目标。

limiter 只平滑 Cartesian 目标，不改变 Studio 返回的关节力矩，也不改变物理积分频率。

## 9. 控制启用条件

某侧用户目标只有在以下条件全部满足时才可交给 streamer：

```text
SimPlugin connected
AND Isaac articulation ready
AND 所选 RDK/DRDK backend ready
AND runtime reference TCP available
AND startup hold completed
AND 该侧已有明确的用户接管请求
```

在条件未满足时：

- Isaac articulation 保持启动关节姿态；
- streamer 保持其锁存的当前 RDK TCP；
- Quest packet 可以被丢弃或等待下一次重新按压；
- 不允许把未标定的 world pose 直接发送到 RDK。

fault clearing 不属于正常控制状态机，不能自动发生。fault 后应停止该侧用户目标，重新建立完整的启动参考。

## 10. 初始化与任务配置

不同任务可以有不同的初始化位置，但配置只描述初值和策略，不绕过控制状态机。

每只机械臂的 scene 配置应包含：

```yaml
bootstrap_q: [q1_home, q2_home, q3_home, q4_home, q5_home, q6_home, q7_home]
initial_q_waypoints:
  - [q1_safe, q2_safe, q3_safe, q4_safe, q5_safe, q6_safe, q7_safe]
initial_q: [q1_task, q2_task, q3_task, q4_task, q5_task, q6_task, q7_task]
```

含义：

- `bootstrap_q`：只用于 Studio/SimPlugin/DRDK 安全握手，必须与两套 Studio 的启动 home 一致；旧场景缺失该字段时回退到 `initial_q`；
- `initial_q_waypoints`：只用于确有必要的成对 NRT 关节中间点，左右数量必须一致；当前墙装任务为空，home 通过一段受限轨迹直接进入已验证任务解。reset 直接回 `initial_q`，途中保护则从新的实际 q 低速重试；
- `initial_q`：任务 initq；首先作为 `SendJointPosition()` 的精确关节目标，到位后再作为 Cartesian 模式的零空间参考；
- `joint_initializing`：DRDK transport 已 operational，Isaac 开放 2 kHz Studio torque，但 Quest/TargetFrame 的 `control_active` 仍为 false；
- 完成判定：DRDK 实测 `q` 和 `dq` 连续满足位置、速度与稳定时间阈值；切换 Cartesian 后，Isaac 再核对自己的 articulation `q` 和 DRDK `q`；
- `T_world_base`：以 initq 到位时的 Isaac 末端世界位姿和 DRDK 锁存 TCP 配准，而不是用 bootstrap/home TCP；
- 用户接管：左右都进入 Cartesian `ready` 后，首次 Quest squeeze 或 TargetFrame 变化沿用 Stage1 相对运动逻辑。

墙装双臂任务的默认 initq 按严格优先级求解，低优先级不得破坏高优先级：

1. 腕部方向与关节限位：两侧 TCP frame 必须镜像，局部 `+Z` 作为夹爪进给轴，共同指向桌面高度附近的虚拟球心；当前约以 `44.8°` 俯角向前、向内，绕进给轴的 roll 也保持镜像。关节不仅要在 USD hard limit 内，还必须位于 Studio URDF 的 soft limit 内并保留配置规定的安全余量；
2. 双腕相对位姿：不再要求 40 cm 或水平；两侧法兰使用相同世界 X/Z、Y 正负镜像，完整相对 SE(3) 由 scene target 保存，并作为 Quest 首次接管的一致性参考；
3. 原始解附近：以任务明确给定的原始参考 initq 为起点，通过小步姿态插值和位置插值连续跟踪同一 IK 支路；在前两项满足的可行解中，同时最小化十四关节欧氏距离和单关节最大变化，禁止从随机 seed 直接跳到另一反解支路；
4. 前三关节对称：只在关节距离相近的合格分支之间，再最小化 `[q1_left+q1_right+π, q2_left+q2_right, q3_left+q3_right]` 的范数，不得为了关节外观完全镜像而牺牲前三项。

修改 initq 时必须使用 USD FK 依次验证方向误差、Studio soft-limit 余量、法兰相对位姿、相对原始解的关节距离和前三关节镜像残差，不能只从随机 IK 结果中选择一个可达解。连续求解至少记录每步方向误差、位置误差、最大关节步长和限位余量；路径中任一步不可达时不得把终点写入配置。离线可达仍不够：终点必须经过 `SendJointPosition()` 低速实测，并成功通过 `SetNullSpacePosture()` 和 Cartesian mode 切换。允许共同调整左右法兰的 X/Z/Y 绝对值，以便在严格镜像和保持原始反解分支之间取得可行解；scene target 必须保存 FK 得到的完整左右 TCP pose。

当前统一 initq 是在已验证安全姿态附近联合优化得到的镜像解：左臂为 `[-1.64741995, 1.54859907, 0.76728963, 1.87202577, 2.14803061, 1.47715997, 0.63020754]`，右臂为 `[-1.41133392, -1.47221452, -0.84874996, -1.69582841, -0.32155028, 1.58311903, 0.66627366]`。两侧法兰位于 `(0.61510761, ±0.30584618, 0.80234595)`，夹爪进给轴以约 `44.8°` 俯角共同指向约 `(1.2305,0,0.1195)` 的虚拟球心；J4 镜像误差约 `5.77 mm`，最小 USD hard-limit 余量约 `14.84°`。当前不使用冷启动中间点，home 直接通过一段受限 NRT 关节轨迹进入 initq。修改该姿态后必须在 Isaac Sim 环境运行 `python scripts/validate_initial_q_symmetry.py --scene <scene.yaml>`：默认要求 TCP 位置误差不超过 `0.1 mm`、姿态误差不超过 `0.01°`、J4 镜像误差不超过 `20 mm`，且所有关节距离 USD hard limit 至少 `5°`。

初始化不由 Isaac 瞬时改关节，也不使用 Cartesian IK 猜测 initq。NRT joint trajectory 由 Flexiv runtime 内部运动生成器规划；SimPlugin 力矩闭环始终为 2000 Hz。进入任务后才使用 30 Hz Cartesian target 层。

场景中的机器人 base pose 描述物理安装，不应用于补偿手柄映射，也不应为了修正目标方向而移动。

## 11. 每侧低频控制伪代码

```text
poll_rdk_status()
poll_latest_quest_packet()

if startup_hold:
    target = startup_tcp
    reset_relative_mapper()
    reset_limiter(target)

elif fresh_packet and squeeze_pressed:
    if squeeze_rising_edge:
        hand_zero = packet.hand_pose
        tcp_zero = rdk.current_tcp
        reset_limiter(tcp_zero)

    raw_target.position = tcp_zero.position + mapped_hand_delta(packet, hand_zero)
    hand_rotation_delta = packet.hand_rotation * inverse(hand_zero.rotation)
    raw_target.orientation = hand_rotation_delta * tcp_zero.orientation
    target = limiter.limit(raw_target, target_dt)
    last_valid_target = target

elif previous_source == quest and last_valid_target exists:
    target = last_valid_target

else:
    target = manual_target_frame_converted_to_rdk()

control_active = all_control_gates_ready() and user_requested_control
publish_to_this_arm_streamer(target, control_active)
```

## 12. 2000 Hz 双臂物理伪代码

```text
for every physics step:
    for arm in [left, right]:
        sim_plugin[arm].send(q[arm], dq[arm], servo_cycle)

    for arm in [left, right]:
        if sim_plugin[arm].connected and control_ready[arm]:
            torque[arm] = sim_plugin[arm].receive_target_drives()
            articulation[arm].apply_torques(torque[arm])
        else:
            articulation[arm].hold_position()
```

两侧循环可以共享同一个 Isaac physics callback 和 `servo_cycle`，但状态、命令与 readiness 必须按 arm 分开存储。

## 13. 控制数据记录

为复现一次遥操，recorder 至少应记录每侧：

- Quest packet sequence；
- squeeze 状态；
- 手柄原始 OpenXR pose；
- 映射后的相对位移/相对旋转；
- RDK 当前 TCP；
- RDK 目标 TCP；
- limiter 输出；
- `q`、`dq`；
- Studio `target_drives`；
- SimPlugin/RDK readiness；
- 控制源：`idle`、`target-frame` 或 `quest`。

所有数据需要带单调时钟时间戳和左右侧标识，避免只依赖不同进程的日志时间进行对齐。

### 13.1 Quest 采集门禁与方向语义

只读状态监视器从 publisher 的原始 Quest pose 计算几何状态。默认启动流程只使用 `SPACING`；`DIRECTION` 仅供显式严格调试模式使用：

1. `SPACING`：按当前 `axis_map` 把左右 OpenXR 位置转换到 mapped frame，令 `delta=right-left`。使用完整三维欧式距离 `norm(delta)`，目标值动态取当前 scene 左右 TCP 的距离，要求误差不超过 `0.03 m`；
2. `DIRECTION`：使用原始 pose、同一 `axis_map` 和 TCP rotation offset 得到左右完整手柄 frame。分别把 `delta/-delta` 转到左右手柄局部 frame，与双臂 TCP 连线在左右 TCP 局部 frame 中的方向比较；同时比较 `inverse(R_left)*R_right` 与机器人 TCP 的相对旋转。左右参考 TCP 分别允许附加局部 `Rz(0)` 或 `Rz(π)`，四种组合中按三项最大误差、误差总和依次最小选择唯一分支；选中分支的三项误差均不得超过 `15°`。这只消除两指夹爪的 180° roll 几何歧义，不忽略其他 pitch、yaw 或 roll 误差。

两只手柄 pose/tracking 任一缺失时距离为 `WAIT`，距离超限为 `FAIL`。默认 `start.sh` 使用 `--shared-calibration-spacing-gate` 和 `--no-strict-shared-calibration`：只有 `SPACING` 会阻止双 squeeze 锁存，Web 面板不显示 `DIRECTION`；显式严格模式才同时启用姿态结果。

## 14. 控制逻辑不可变约束

后续修改必须保持：

1. Stage2 是两份 Stage1 控制链，不是新的双臂耦合控制器。
2. Docker 只改变左 Studio 的部署位置，不改变算法和数据格式。
3. Quest 必须使用按压瞬间双零点的相对运动。
4. 松开 squeeze 后保持最后目标，不能回初始化位置。
5. 初次 Quest-to-RDK 旋转标定由左右手共享；锁定后的左右手相对零点、RDK TCP 零点、目标保持和 limiter 完全独立。DRDK 只原子同步传输同周期的两侧目标。
6. RDK base 是 Cartesian 控制的规范坐标系。
7. OpenXR 轴映射只能应用一次。
8. 目标层约 30 Hz，Studio/SimPlugin 物理闭环保持 2000 Hz。
9. Studio 返回力矩不抽帧、不合并、不缩放。
10. 未完成当前 TCP 标定或所选 RDK/DRDK backend 未 ready 时不能激活用户目标。
11. RDK/DRDK 目标层默认 NRT；不得把 30 Hz 离散目标接入 RT streaming API。
12. DRDK 必须在 Isaac/SimPlugin 启动前进入发现等待；连接后必须先用 `NRT_JOINT_POSITION + SendJointPosition()` 到达当前任务左右 `initial_q`，再切换 Cartesian。
13. 每次进入 `NRT_CARTESIAN_MOTION_FORCE` 后都必须重新调用 `SetNullSpacePosture(initial_q)`，因为 runtime 会在重新进入该模式时把零空间参考重置为当前 q。
14. 非 home `initial_q` 不能在 SimPlugin 握手时直接写入 articulation，也不能仅凭 Cartesian target + null-space 推断会得到精确 initq。
15. recorder 的协调 reset 必须使用 `Stop → ClearFault → Enable → NRT SendJointPosition(initial_q)`；不得通过 Isaac world reset、关节瞬移或旧 Cartesian target pose 恢复。
