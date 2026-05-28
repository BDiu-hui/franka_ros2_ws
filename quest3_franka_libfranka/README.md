# quest3_franka_libfranka

`quest3_franka_libfranka` 是一个直接使用 `libfranka` 的笛卡尔位姿桥。它订阅 Quest3 遥操产生的目标 TCP 位姿和使能信号，然后通过 FCI 给 Franka 发送笛卡尔位姿命令。

当前真实遥操推荐使用 `quest3_oculus_rviz` 中的 `franky_cartesian_pose_node`。本包保留为低层 libfranka 直连实验、回退方案和对比测试。

## 主要内容

- `src/libfranka_cartesian_pose_node.cpp`: 直接连接 Franka 的 C++ ROS 2 节点。
- 可执行入口: `libfranka_cartesian_pose_node`

## 构建

```bash
cd /home/lumos/franka_ros2_ws
source setup_env.bash
colcon build --packages-select quest3_franka_libfranka --symlink-install
source setup_env.bash
```

## 启动

单臂测试:

```bash
ros2 run quest3_franka_libfranka libfranka_cartesian_pose_node --ros-args \
  -p robot_ip:=172.16.0.3 \
  -p target_pose_topic:=/franka_sim/tcp_target_pose \
  -p enabled_topic:=/quest3/right_teleop/enabled \
  -p current_pose_topic:=/franka_libfranka/current_pose
```

这个节点不会启动 Quest、RViz 或手柄映射节点。通常需要配合 `quest3_oculus_rviz` 的手柄节点一起使用。

## 话题

订阅:

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/franka_sim/tcp_target_pose` | `geometry_msgs/msg/PoseStamped` | 目标 TCP 位姿 |
| `/quest3/right_teleop/enabled` | `std_msgs/msg/Bool` | trigger 使能信号 |

发布:

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/franka_libfranka/current_pose` | `geometry_msgs/msg/PoseStamped` | 机器人当前 TCP 位姿 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 机器人关节状态 |
| `/franka_libfranka/debug` | `std_msgs/msg/String` | 控制状态调试信息 |

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_ip` | `172.16.0.3` | Franka 控制柜 IP |
| `target_pose_topic` | `/franka_sim/tcp_target_pose` | 目标位姿输入 |
| `enabled_topic` | `/quest3/right_teleop/enabled` | 遥操使能输入 |
| `current_pose_topic` | `/franka_libfranka/current_pose` | 当前位姿输出 |
| `debug_topic` | `/franka_libfranka/debug` | 调试输出 |
| `base_frame` | `panda_link0` | 位姿 frame id |
| `publish_rate_hz` | `50.0` | 状态发布频率 |
| `target_timeout_sec` | `0.25` | 目标位姿超时时间 |
| `enabled_timeout_sec` | `0.25` | 使能信号超时时间 |
| `max_linear_velocity_mps` | `0.04` | 最大线速度限制 |
| `max_angular_velocity_radps` | `0.25` | 最大角速度限制 |
| `max_initial_target_distance_m` | `0.08` | 开始运动时目标允许距离 |
| `max_initial_target_angle_rad` | `0.6` | 开始运动时目标允许角度 |
| `workspace_min` | `[0.20,-0.45,0.08]` | 工作空间下限 |
| `workspace_max` | `[0.80,0.45,0.75]` | 工作空间上限 |
| `automatic_error_recovery` | `true` | 启动时自动错误恢复 |

## 控制逻辑

- trigger 未按下或使能超时: 保持当前位姿，不跟随目标。
- 目标位姿超时: 不继续执行旧目标。
- 初始目标距离过大: 进入 `initial_target_too_far_from_current` 状态，避免突然跳动。
- 目标位置会被限制在 `workspace_min` 和 `workspace_max` 内。
- 每个控制周期按最大线速度和最大角速度向目标逼近。

## 注意事项

- 这是直接 FCI 控制链路，不要和 `franky_cartesian_pose_node`、`franky_franka_control`、`serl_franka_controllers_ros2` 的 HTTP/libfranka 控制同时占用同一台机械臂。
- 本包没有夹爪控制逻辑。
- 真实机械臂测试前，先用很小的速度限制和较小工作空间确认方向、位姿和急停都正常。
