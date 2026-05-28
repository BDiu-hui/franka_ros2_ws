# quest3_oculus_rviz

`quest3_oculus_rviz` 是当前 Quest3 -> Franka 遥操的主功能包。它负责读取 Quest3 手柄位姿、生成 TCP 目标、在 RViz 中显示机器人状态，并可以通过 `franky` 直接控制单臂或双臂 Franka。

这个包现在支持两层控制模式:

- 手柄到 TCP 目标: `teleop_motion_mode:=velocity` 或 `position`
- Franka 执行目标: `control_command_mode:=velocity` 或 `pose`

真实机械臂跟手遥操当前推荐使用 `teleop_motion_mode:=velocity` 加 `control_command_mode:=velocity`。

## 主要内容

节点:

| 节点 | 作用 |
| --- | --- |
| `oculus_tf_node` | 从 Quest/oculus_reader 读取左右手柄位姿，发布 TF、Pose 和 marker |
| `hand_teleop_sim_node` | 通用左右手遥操节点，把手柄运动映射成 TCP 目标 |
| `right_hand_teleop_sim_node` | 兼容旧单右手入口 |
| `franka_sim_ik_node` | mock/RViz 仿真用的 Panda IK 显示节点 |
| `franky_cartesian_pose_node` | 连接真实 Franka，执行 TCP 目标和夹爪命令 |

launch:

| launch 文件 | 作用 |
| --- | --- |
| `rviz.launch.py` | mock 或单臂 RViz 预览 |
| `franka_real_teleop.launch.py` | 单臂真实 Franka 遥操 |
| `franka_dual_real_teleop.launch.py` | 双臂真实 Franka 遥操 |

## 构建

```bash
cd /home/lumos/franka_ros2_ws
source setup_env.bash
colcon build --packages-select quest3_oculus_rviz --symlink-install
source setup_env.bash
```

如果要控制真实机械臂，还需要确认 `franky` 可用:

```bash
python3 -c "import franky; print('franky ok')"
```

## Mock/RViz 预览

不连接 Quest 和真实机械臂，只看链路是否能启动:

```bash
ros2 launch quest3_oculus_rviz rviz.launch.py \
  mock:=true \
  start_rviz:=true
```

## 单臂真实遥操

基础启动:

```bash
ros2 launch quest3_oculus_rviz franka_real_teleop.launch.py \
  robot_ip:=172.16.0.3 \
  start_franka:=true \
  start_rviz:=true \
  mock:=false \
  teleop_motion_mode:=velocity \
  control_command_mode:=velocity \
  translation_scale:=2.0 \
  rotation_scale:=1.2 \
  max_linear_velocity_mps:=0.18 \
  max_angular_velocity_radps:=0.80 \
  velocity_command_duration_sec:=0.05 \
  enabled_timeout_sec:=0.08 \
  stop_on_disable:=true \
  enable_gripper:=true
```

默认方向现在按当前实机标定为:

```text
translation_x_sign = 1.0
translation_y_sign = 1.0
translation_z_sign = 1.0
roll_sign = 1.0
pitch_sign = 1.0
yaw_sign = 1.0
```

如果某个方向和手感相反，只改对应的 sign 参数，不要同时反复改 frame 和 scale。

## 双臂真实遥操

基础启动:

```bash
ros2 launch quest3_oculus_rviz franka_dual_real_teleop.launch.py \
  left_robot_ip:=172.16.0.2 \
  right_robot_ip:=172.16.0.3 \
  start_left_franka:=true \
  start_right_franka:=true \
  start_rviz:=true \
  mock:=false \
  teleop_motion_mode:=velocity \
  control_command_mode:=velocity \
  translation_scale:=2.0 \
  rotation_scale:=1.2 \
  max_linear_velocity_mps:=0.18 \
  max_angular_velocity_radps:=0.80 \
  velocity_command_duration_sec:=0.05 \
  enabled_timeout_sec:=0.08 \
  stop_on_disable:=true \
  enable_gripper:=true \
  right_gripper_buttons_enabled:=true \
  left_gripper_buttons_enabled:=false
```

双臂默认方向也已经按单臂逻辑统一:

```text
left:  x/y/z/rx/ry/rz = 1.0
right: x/y/z/rx/ry/rz = 1.0
```

如果右臂现场方向又出现相反，可以临时在 launch 命令里覆盖:

```bash
right_translation_x_sign:=-1.0
right_translation_y_sign:=-1.0
right_roll_sign:=-1.0
right_pitch_sign:=-1.0
```

## 常用参数

| 参数 | 说明 |
| --- | --- |
| `teleop_motion_mode` | 手柄映射模式，`velocity` 表示按增量跟随，`position` 表示相对锚点位置映射 |
| `control_command_mode` | Franka 执行模式，`velocity` 发笛卡尔速度，`pose` 发笛卡尔位姿 |
| `translation_scale` | 平移增益 |
| `rotation_scale` | 旋转增益 |
| `translation_deadband_m` | 平移死区，小抖动会被置零 |
| `rotation_deadband_rad` | 旋转死区，小角度抖动会被置零 |
| `delta_filter_alpha` | delta 一阶低通滤波系数，越大越跟手，越小越平滑 |
| `max_tcp_delta_body_m` | 单帧 TCP 平移增量上限 |
| `max_tcp_delta_rotvec_rad` | 单帧 TCP 旋转增量上限 |
| `max_linear_velocity_mps` | 真实机械臂最大线速度 |
| `max_angular_velocity_radps` | 真实机械臂最大角速度 |
| `max_linear_acceleration_mps2` | 真实机械臂最大线加速度 |
| `max_angular_acceleration_radps2` | 真实机械臂最大角加速度 |
| `velocity_command_duration_sec` | 速度命令持续时间，越短越不容易松 trigger 后继续跑 |
| `enabled_timeout_sec` | trigger 使能超时时间 |
| `stop_on_disable` | trigger 松开时立即给 Franka stop |

## Quest 连接检查

`mock:=false` 时需要 Quest 端软件已经打开，并且电脑能通过 USB/网络读到 oculus_reader 数据。

常见现象:

- 日志显示 `Connected to oculus_reader using USB mode`: 电脑已经连上 Quest 数据源。
- 日志反复显示 `No Quest controller pose received yet`: 通常是 Quest 端软件没有打开、手柄未唤醒、USB 权限未确认或 oculus_reader 没有持续输出。

## 夹爪

右手默认按钮:

- `A`: 打开夹爪
- `B`: 关闭夹爪

双臂默认只启用右手按钮:

```text
right_gripper_buttons_enabled = true
left_gripper_buttons_enabled = false
```

## 详细文档

更完整的操作 SOP 和参数调试记录在工作空间文档中:

- `../../docs/franka_teleop/QUEST3_FRANKA_REAL_TELEOP_SOP.md`
- `../../docs/franka_teleop/QUEST3_FRANKA_DUAL_TELEOP_SOP.md`
- `../../docs/franka_teleop/TELEOP_CONTROL_MODES.md`
- `../../docs/franka_teleop/TELEOP_MOTION_MODES_METHODS.md`
- `../../docs/franka_teleop/FRANKY_CARTESIAN_POSE_NODE_METHODS.md`

## 安全注意

- 控制真实机械臂前，确认急停、工作空间、人员位置和机器人错误状态。
- 第一次调参时优先降低 `max_linear_velocity_mps`、`max_angular_velocity_radps` 和 `relative_dynamics_factor`。
- 同一台机械臂同一时间只能有一个 FCI 控制进程。
- trigger 松开后如果仍有惯性动作，优先减小 `velocity_command_duration_sec`，确认 `enabled_timeout_sec` 足够短，并保持 `stop_on_disable:=true`。
