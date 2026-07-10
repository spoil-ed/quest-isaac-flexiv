# Quest Isaac Flexiv 目标

最终目标：用 Quest 手柄位姿遥操 Isaac Sim 中的 Flexiv Rizon4，但不使用 Isaac 自带 IK 作为最终控制器。Isaac 只负责仿真、状态读取和执行力矩；反解和控制由 Elements Studio / Flexiv 控制栈完成。

目标闭环数据流：

```text
Quest pose
  -> pose adapter
  -> T_base_tcp_des
  -> Elements Studio / Flexiv 控制栈
  -> target_drives
  -> Isaac apply_torques
  -> Isaac q, dq
  -> Studio / FlexivSimulation feedback
  -> next frame
```

当前先用 `/World/TargetBall` 替代 Quest pose 验证链路：

```text
TargetBall pose
  -> T_base_tcp_des
  -> Studio / Flexiv 控制栈
  -> target_drives
  -> Isaac apply_torques
  -> Rizon4 跟随小球
```

验收目标：拖动小球或移动 Quest 手柄时，Rizon4 在 Isaac 中实时跟随；同时能记录目标位姿、`q`、`dq`、`target_drives`，后续用于轨迹采集、LeRobot 转换和 replay。
