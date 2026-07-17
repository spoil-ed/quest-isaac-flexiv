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

传输层负责：

- 连接指定的一对或单个 serial；
- 读取 runtime 当前 TCP，形成各侧启动参考；
- 接收 Isaac 发布的 `pose_base_tcp_des`；
- 将目标交给 NRT Cartesian motion-force 接口；
- 向 Isaac 返回 `ready`、参考 TCP 和实时 TCP；
- 没有新用户目标时保持最后有效目标。

DRDK 初始化顺序固定为：从与 Isaac 相同的 scene config 读取左右 `robots[].initial_q`、在 Isaac 前启动初始发现循环、SimPlugin 以安全 `bootstrap_q` 使两个 runtime connected、创建 `RobotPair` 并进入 `NRT_JOINT_POSITION`、读取切换后的当前 q 并先发送为保持指令以建立无跳变控制基准、再调用 `SendJointPosition(init_q)` 让 runtime 平滑到两侧 initq、等待关节位置和速度连续稳定、切换 `NRT_CARTESIAN_MOTION_FORCE`、重新调用 `SetNullSpacePosture((init_q_left, init_q_right))`、锁存两臂当前 TCP，最后同时发布任务 ready。初始发现可以在限定超时内重试；连接成功后的断连或 fault 必须锁存为双侧 not-ready 并等待显式 reset，不得自动清 fault 或恢复用户目标。

协调 reset 是控制协议的一部分。recorder 按 `r` 后经 gateway 生成单调递增的 `reset_seq`；Isaac 立即撤销双臂用户目标、清空 Quest clutch/mapper/limiter 和 TCP 标定，并把同一序号随左右 `flexiv_target_pose.v1` 包重复发送，直到 DRDK 确认。DRDK 对每个新序号只执行一次以下状态机：

1. Isaac 先撤销用户控制并临时关闭配置中场景物体的碰撞；刚体工件同时暂时设为 kinematic，避免失去桌面支撑后掉落。两臂 articulation 的 collision shape 不能在运行中删除或重建，否则会使 PhysX articulation tensor view 失效。这样已与环境穿插或挤压的机械臂可以脱离接触，避免清 fault 后的回程立即再次触发力矩保护。
2. `reset_stopping`：调用 `RobotPair.Stop()` 对两臂做软件受控停止；这不是实体急停按钮。
3. `reset_clearing_fault`：若存在 fault，则调用一次 `ClearFault()`，且必须确认两侧都成功。
4. 重新 `Enable()` 并等待 RobotPair operational。
5. `joint_initializing`：切换 `NRT_JOINT_POSITION`，先以切换后的当前 q 调用 `SendJointPosition()` 建立连续基准，再调用 `SendJointPosition(init_q_left, init_q_right)`；reset 使用独立的低速/低加速度上限，由 runtime 生成关节轨迹，等待位置和速度连续稳定。
6. 切换 `NRT_CARTESIAN_MOTION_FORCE`，重设 `SetNullSpacePosture(initial_q)`，重新锁存两侧当前 TCP，清空旧 Cartesian packet，并发布带相同 `reset_seq` 的双侧 ready。
7. Isaac 只在两侧 status 都确认该序号、关节状态位于任务 `initial_q` 且 SimPlugin effort 闭环正常时，才把全部 `scene_objects` 恢复到 scene config 声明的初始位置、姿态、缩放和关节状态，并把刚体线速度/角速度清零。资产保持 kinematic 且无碰撞至少一个目标更新周期，再恢复原 collision/kinematic 状态并报告 `succeeded`；recorder 收到该状态后才允许继续录制。

reset 不调用 Isaac `world.reset()`，不瞬时写 articulation，也不使用旧 TCP target pose 代替任务 `initial_q`。如果回程中再次出现 fault，同一 `reset_seq` 会执行有界恢复：重新 `Stop → ClearFault → Enable`，从新的实际 q 建立连续基准并继续回最终 `initial_q`；默认最多 3 次，不能无限重试。只有两臂到位、切回 Cartesian、重装零空间并完成场景资产复位后才发布成功。耗尽次数才发布 `reset_failed` 和最后错误原因；此时保持用户控制关闭及场景物体碰撞暂时禁用，使下一次按 `r` 可以重新执行完整事务。机械臂 articulation 自身的碰撞始终保持启用，recorder 也不因一次 reset 失败退出。

SimPlugin 的 2 kHz 状态—力矩闭环只依赖对应 runtime 已连接，不依赖 RDK/DRDK ready：每个物理周期先发送两臂状态，再等待并原样应用两套 runtime 的 `target_drives`。这是官方多机器人 bridge 的控制顺序，也避免 runtime 在 DRDK 尚未完成成对发现时因力矩未被应用而误判负载。DRDK ready 只门控 Cartesian/Quest/TargetFrame 用户目标，不门控底层 Studio 力矩闭环。

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

未标定时，双手 pose 持续发布给 `print.sh`，但不发布 `rizon4_quest_target.v1`。用户先满足第 13.1 节的 `SPACING` 和 `DIRECTION`，再同时按住两只手柄的 `squeeze`。几何条件在 settle window 内持续有效后，publisher 才确认标定；单手 squeeze、距离或方向不合格都不能确认。

令 `A` 为固定 signed-permutation，`u_left=A*p_left`、`u_right=A*p_right`，并令 `f_left/f_right` 为应用 `A` 与 TCP rotation offset 后的两只手柄水平指向。标定基向量为：

```text
e_x = normalize(project_xy(f_left + f_right))
e_y = normalize((u_left-u_right)_xy - dot((u_left-u_right)_xy, e_x) * e_x)
e_z = normalize(cross(e_x, e_y))
C   = [e_x^T; e_y^T; e_z^T]
```

必要时同时反转 `e_y/e_z`，保证 `e_z` 朝上且 `C` 为右手正交基。两手中点是标定坐标系的概念原点；由于控制只使用每只手的相对增量，平移原点会在相减时抵消。`C` 在确认瞬间锁存，松开 squeeze、暂停或重新接管都不得改变；只有重启 publisher 才重新标定。

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
- 目标方向经过角速度限制后再发送给 RDK，同时用于 TargetFrame 可视化；
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
- 左右 Frame 分别直接显示对应侧的原始 Quest 目标，不等待机械臂速度限制器追上；
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
- `initial_q_waypoints`：只用于冷启动的成对 NRT 关节中间点；左右数量必须一致。当前墙装任务先回到经过实机验证的原始安全解，再低速进入任务解；reset 直接回 `initial_q`，途中保护则从新的实际 q 低速重试，不反向重走冷启动中间点；
- `initial_q`：任务 initq；首先作为 `SendJointPosition()` 的精确关节目标，到位后再作为 Cartesian 模式的零空间参考；
- `joint_initializing`：DRDK transport 已 operational，Isaac 开放 2 kHz Studio torque，但 Quest/TargetFrame 的 `control_active` 仍为 false；
- 完成判定：DRDK 实测 `q` 和 `dq` 连续满足位置、速度与稳定时间阈值；切换 Cartesian 后，Isaac 再核对自己的 articulation `q` 和 DRDK `q`；
- `T_world_base`：以 initq 到位时的 Isaac 末端世界位姿和 DRDK 锁存 TCP 配准，而不是用 bootstrap/home TCP；
- 用户接管：左右都进入 Cartesian `ready` 后，首次 Quest squeeze 或 TargetFrame 变化沿用 Stage1 相对运动逻辑。

墙装双臂任务的默认 initq 按严格优先级求解，低优先级不得破坏高优先级：

1. 腕部方向与关节限位：两侧 TCP 局部 `+Z` 指向世界 `+X`，另外两轴与地面坐标轴平行/垂直，即法兰世界旋转接近 `R_y(+90°)`；关节不仅要在 USD hard limit 内，还必须位于 Studio URDF 的 soft limit 内并保留配置规定的安全余量；
2. 双腕距离：两侧法兰使用相同世界 X/Z，并分别位于中心线 `Y=+0.20 m` 和 `Y=-0.20 m`；
3. 原始解附近：以任务明确给定的原始参考 initq 为起点，通过小步姿态插值和位置插值连续跟踪同一 IK 支路；在前两项满足的可行解中，同时最小化十四关节欧氏距离和单关节最大变化，禁止从随机 seed 直接跳到另一反解支路；
4. 前三关节对称：只在关节距离相近的合格分支之间，再最小化 `[q1_left+q1_right+π, q2_left+q2_right, q3_left+q3_right]` 的范数，不得为了关节外观完全镜像而牺牲前三项。

修改 initq 时必须使用 USD FK 依次验证方向误差、Studio soft-limit 余量、法兰位置/间距、相对原始解的关节距离和前三关节镜像残差，不能只从随机 IK 结果中选择一个可达解。连续求解至少记录每步方向误差、位置误差、最大关节步长和限位余量；路径中任一步不可达时不得把终点写入配置。离线可达仍不够：终点必须经过 `SendJointPosition()` 低速实测，并成功通过 `SetNullSpacePosture()` 和 Cartesian mode 切换。允许共同调整左右法兰的 X/Z，以便在保持 `Y≈±0.20 m` 和相同 X/Z 的同时留在原始反解分支附近。场景 target frame 使用 `euler_deg={x:0,y:90,z:0}` 表达同一个“正前方”约定。

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

只读状态监视器必须持续发布两只手柄的原始 Quest pose，使 `scripts/print.sh` 在未按 squeeze、尚未建立相对控制锚点时也能检查双手初始化。默认输出固定为两行，且不把机器人 READY、TCP command 或 squeeze/control-active 作为通过条件：

1. `SPACING`：按当前 `axis_map` 把左右 OpenXR 位置转换到机器人 base，令 `delta=right-left`。使用完整三维欧式距离 `norm(delta)`，要求 `abs(norm(delta)-0.40)<=0.01 m`；连线方向不绑定固定 X/Y/Z 轴；
2. `DIRECTION`：把动态连线 `delta` 和两只手柄的指向都投影到机器人 base 水平 `XY` 平面。手柄指向由原始 pose、同一 `axis_map` 和 Quest publisher 配置的 TCP rotation offset 共同得到，并以 TCP 局部 `+Z` 为指向轴。左右手柄的水平指向分别与连线成 `90°±15°`，同时左右水平指向的相互夹角不超过 `15°`，从而保证两者位于连线同一侧；目标方向不再是固定 `+X/-X/±Z`，而是由实时连线动态决定。这里只检查水平指向，不限制 pitch 或绕指向轴的 roll。

两只手柄 pose/tracking 任一缺失时两行均为 `WAIT`，数值超限为 `FAIL`，通过为绿色 `PASS`。几何检查本身不要求 squeeze；两行通过后同时按住双 squeeze 才把共享坐标系从 `HOLD_BOTH_SQUEEZE` 经 `CONFIRMING` 锁定为 `LOCKED`。`print.sh` 仍是只读监视器，真正的锁存由 Quest publisher 完成；完整机器人与映射诊断只在 `--verbose` 中展示。

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
