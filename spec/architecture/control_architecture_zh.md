# Flexiv 双臂控制架构

本文是本仓库的控制逻辑规范。它说明控制目标怎样产生、怎样进行坐标变换、怎样分发给左右机械臂，以及 Studio、RDK、SimPlugin 和 Isaac Sim 在闭环中的职责。

本文不记录安装方法、进程修复过程或历史故障。运行命令和部署步骤属于 `README.md` 与 `docs/`。

## 1. 控制目标

系统控制两台 Flexiv Rizon4：

- 左臂由 Docker 内的 Elements Studio/FlexivSimulation 提供控制器。
- 右臂由宿主机上的 Elements Studio/FlexivSimulation 提供控制器。
- Isaac Sim、Quest 输入、两个 RDK target streamer、gateway 和 recorder 都运行在宿主机。
- 双臂控制是两套 Stage1 单臂控制的并列组合。左臂多出的 Docker 层只改变 Studio 的部署位置，不改变控制算法。

最终控制链如下：

```text
左手柄 ─┐                     ┌─> 左 RDK target streamer ─> Docker 左 Studio
        ├─> Quest publisher ─>│
右手柄 ─┘                     └─> 右 RDK target streamer ─> 宿主机右 Studio
                                      │                         │
                                      │ Cartesian target        │ torque command
                                      v                         v
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
- 每只手的 `squeeze` 独立控制对应机械臂；
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

### 2.3 RDK target streamer

每只机械臂有一个 `scripts/rdk_target_streamer.py`：

- 连接指定 serial；
- 读取 Studio/RDK 当前 TCP，形成该侧启动参考；
- 接收 Isaac 发布的 `pose_base_tcp_des`；
- 将目标交给 RDK Cartesian 控制接口；
- 向 Isaac 返回 `ready`、参考 TCP 和实时 TCP；
- 没有新用户目标时保持最后有效目标。

streamer 不进行 OpenXR 坐标变换，不共享左右臂状态，也不生成仿真力矩。

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
| RDK target 发布 | 30 Hz | 将目标送入两套 Studio |
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

## 5. Squeeze 相对遥操状态机

每只手独立执行以下状态机：

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
```

方向不建立相对零点。首次有效 packet 的位置目标必须等于当前 TCP；方向目标则来自当前手柄绝对姿态。可使用短暂 settle window 持续更新位置零点，过滤按键按下瞬间的平移手抖。

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

### 5.4 TRACKING：绝对姿态

末端方向使用映射后的手柄绝对姿态：

```text
q_des = q_map_openxr_to_rdk(q_hand) * q_tcp_tool_offset
```

关键性质：

- squeeze 只为 XYZ 相对位移建立零点，不为姿态建立零点；
- 手柄保持某个方向时，末端目标保持对应的绝对方向；
- OpenXR 姿态必须使用与平移相同的轴变换进行基变换，不能直接复制四元数分量；
- 目标方向经过角速度限制后再发送给 RDK，同时用于 TargetFrame 可视化；
- `packet` 是默认控制策略；`reference` 仅用于明确要求锁定末端方向的任务。

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

再次按下 squeeze 时，以新的手柄位置和当前 RDK TCP 重新建立零点。这样可以通过多次“按住移动、松开重新摆手”的方式扩展操作范围。

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
- 一侧 RDK 未 ready 时，只暂停该侧目标交接；
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
- Quest 的相对 XYZ 与绝对方向组成原始目标 `T_base_tcp_goal`；
- 左右 Frame 分别直接显示对应侧的原始 Quest 目标，不等待机械臂速度限制器追上；
- `T_base_tcp_goal` 通过该侧启动时标定得到的 `T_world_base` 转换到 Isaac 世界坐标后显示；
- 发送给 streamer 的 `T_base_tcp_control` 由 limiter 逐步逼近 `T_base_tcp_goal`；
- 2000 Hz callback 只保存待显示的目标快照，绝不在其中修改 USD；
- Isaac 主渲染循环以 30 Hz 更新可视 Frame，避免场景写操作阻塞 SimPlugin 物理闭环；
- 松开 squeeze 后 Frame 保持最后的 `T_base_tcp_goal`，limiter 可继续让机械臂逼近该固定目标；
- 再次按下 squeeze 时，以当前 RDK TCP 建立新的 XYZ 零点；
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
AND RDK streamer ready
AND RDK reference TCP available
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

每只机械臂的任务配置最终应包含：

```yaml
initialization:
  joint_positions_rad: [q1, q2, q3, q4, q5, q6, q7]
  target_frame_mode: end_effector   # 或 configured_world
  zero_alignment: measured_rdk_tcp
  control_frame: rdk_base
  quest_axis_map: -z,-x,y
```

含义：

- `joint_positions_rad`：Isaac 与两套 Studio 共同使用的任务初始关节姿态；
- `target_frame_mode=end_effector`：启动时 Frame 对齐真实末端；
- `target_frame_mode=configured_world`：仅设置可视化/任务初始 Frame，不可直接替代 RDK 零点；
- `zero_alignment=measured_rdk_tcp`：Quest 接管必须以实测 RDK TCP 对零；
- `control_frame=rdk_base`：内部控制量统一使用 RDK base；
- `quest_axis_map`：任务需要的 OpenXR signed permutation。

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
    raw_target.orientation = mapped_absolute_orientation(packet)
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

## 14. 控制逻辑不可变约束

后续修改必须保持：

1. Stage2 是两份 Stage1 控制链，不是新的双臂耦合控制器。
2. Docker 只改变左 Studio 的部署位置，不改变算法和数据格式。
3. Quest 必须使用按压瞬间双零点的相对运动。
4. 松开 squeeze 后保持最后目标，不能回初始化位置。
5. 左右手、左右 RDK、左右标定和左右 limiter 完全独立。
6. RDK base 是 Cartesian 控制的规范坐标系。
7. OpenXR 轴映射只能应用一次。
8. 目标层约 30 Hz，Studio/SimPlugin 物理闭环保持 2000 Hz。
9. Studio 返回力矩不抽帧、不合并、不缩放。
10. 未完成当前 TCP 标定或 RDK 未 ready 时不能激活用户目标。
