# serl_franka_controllers_ros2

`serl_franka_controllers_ros2` 是 SERL Franka 控制器的 ROS 2 移植包，包含笛卡尔阻抗控制器、笛卡尔位姿命令控制器、关节位置控制器、RViz 阻抗调参面板，以及几个 HTTP 控制工具。

当前 Quest3 真实遥操主链路不直接依赖这个包，而是使用 `quest3_oculus_rviz` + `franky_cartesian_pose_node`。本包主要用于低层控制器实验、阻抗控制调试、HTTP 控制和 libfranka 回退测试。

## 主要内容

控制器:

| 控制器 | 说明 |
| --- | --- |
| `cartesian_impedance_controller` | 笛卡尔阻抗控制器 |
| `cartesian_pose_command_controller` | 笛卡尔目标位姿控制器 |
| `joint_position_controller` | 关节位置控制器 |

工具和脚本:

| 文件 | 说明 |
| --- | --- |
| `scripts/serl_franka_http_server.py` | ROS 2 controller 链路 HTTP 服务 |
| `scripts/libfranka_http_server.py` | 直接 libfranka HTTP 服务 |
| `libfranka_http_tool` | C++ libfranka HTTP/调试工具 |
| `src/impedance_tuning_panel.cpp` | RViz 阻抗调参面板 |

launch:

| launch 文件 | 作用 |
| --- | --- |
| `impedance.launch.py` | 启动 ros2_control 阻抗控制链路 |
| `joint.launch.py` | 启动关节控制链路 |
| `spawn_controllers.launch.py` | 加载控制器 |
| `http_control.launch.py` | 阻抗控制 + HTTP 服务 |
| `http_pose_fallback.launch.py` | HTTP 位姿回退控制 |
| `libfranka_http.launch.py` | 直接 libfranka HTTP 服务 |

## 构建

```bash
cd /home/lumos/franka_ros2_ws
source setup_env.bash
colcon build --packages-select serl_franka_controllers_ros2 --symlink-install
source setup_env.bash
```

## 控制链路选择

当前工作空间里常见有三条 Franka 控制链路:

1. `serl_franka_controllers_ros2` 的 ROS 2 阻抗控制链路。
2. `serl_franka_controllers_ros2` 的直接 libfranka HTTP 链路。
3. `franky_franka_control` 或 `quest3_oculus_rviz` 的 franky/Ruckig 链路。

同一台机械臂同一时间只能运行其中一条。多个进程同时占用 FCI 会导致连接失败、控制异常或机器人报错。

## RViz 阻抗调参面板

启动 ros2_control 阻抗链路和 RViz 调参面板:

```bash
source setup_env.bash
ros2 launch serl_franka_controllers_ros2 impedance.launch.py \
  robot_ip:=172.16.0.2 \
  robot_type:=fr3 \
  load_gripper:=true \
  start_rviz:=true \
  start_impedance_controller:=true
```

`impedance.launch.py` 默认 `start_rviz:=true` 和 `start_impedance_controller:=true`，所以也可以简写:

```bash
source setup_env.bash
ros2 launch serl_franka_controllers_ros2 impedance.launch.py \
  robot_ip:=172.16.0.2 \
  robot_type:=fr3 \
  load_gripper:=true
```

RViz 会加载 `rviz/impedance_tuning.rviz`，其中包含 `cartesian_impedance_controller` 的阻抗系数调参面板。

## ROS 2 阻抗控制 HTTP

启动:

```bash
ros2 launch serl_franka_controllers_ros2 http_control.launch.py \
  robot_ip:=172.16.0.2 \
  robot_type:=fr3 \
  start_rviz:=false \
  load_gripper:=true \
  server_port:=5000
```

启动/停止阻抗控制:

```bash
curl -X POST http://127.0.0.1:5000/startimp
curl -X POST http://127.0.0.1:5000/stopimp
```

发送目标位姿:

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0]}'
```

`/pose` 的数组格式为 `[x, y, z, qx, qy, qz, qw]`。阻抗链路适合柔顺控制和 RViz 调参；目标位姿是否能执行取决于控制器是否 active。

检查控制器:

```bash
ros2 control list_controllers
```

## 直接 libfranka HTTP

启动:

```bash
ros2 launch serl_franka_controllers_ros2 libfranka_http.launch.py \
  robot_ip:=172.16.0.2 \
  server_port:=5000
```

常用接口:

```bash
curl -X POST http://127.0.0.1:5000/getstate
curl -X POST http://127.0.0.1:5000/getpos
curl -X POST http://127.0.0.1:5000/clearerr
```

发送末端位姿:

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0], "duration_sec":0.0, "controller_mode":"joint"}'
```

常用字段:

| 字段 | 说明 |
| --- | --- |
| `arr` | `[x,y,z,qx,qy,qz,qw]` 目标位姿 |
| `duration_sec` | 运动时长，越大越慢 |
| `controller_mode` | `joint` 或 `cartesian`，一般先用 `joint` |

碰撞阈值:

```bash
curl -X POST http://127.0.0.1:5000/set_collision \
  -H "Content-Type: application/json" \
  -d '{
    "lower_torque_thresholds_nominal":[40,40,35,35,30,25,20],
    "upper_torque_thresholds_nominal":[55,55,50,50,45,40,35],
    "lower_torque_thresholds_acceleration":[40,40,35,35,30,25,20],
    "upper_torque_thresholds_acceleration":[55,55,50,50,45,40,35],
    "lower_force_thresholds_nominal":[45,45,45,35,35,35],
    "upper_force_thresholds_nominal":[60,60,60,50,50,50],
    "lower_force_thresholds_acceleration":[45,45,45,35,35,35],
    "upper_force_thresholds_acceleration":[60,60,60,50,50,50]
  }'
curl -X POST http://127.0.0.1:5000/get_collision
```

## 夹爪 HTTP

如果启动链路加载了夹爪，可以使用:

```bash
curl -X POST http://127.0.0.1:5000/reset_gripper
curl -X POST http://127.0.0.1:5000/open_gripper
curl -X POST http://127.0.0.1:5000/close_gripper
```

指定宽度:

```bash
curl -X POST http://127.0.0.1:5000/move_gripper \
  -H "Content-Type: application/json" \
  -d '{"width":0.04, "speed":0.05}'
```

## 双臂 HTTP 调试

双臂同时测试时，两个控制柜 IP、ROS namespace 和 HTTP 端口都要分开:

```bash
ros2 launch serl_franka_controllers_ros2 http_control.launch.py \
  robot_ip:=172.16.0.2 \
  robot_type:=fr3 \
  namespace:=fr3_02 \
  server_port:=5000 \
  start_rviz:=false \
  load_gripper:=false

ros2 launch serl_franka_controllers_ros2 http_control.launch.py \
  robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  namespace:=fr3_03 \
  server_port:=5001 \
  start_rviz:=false \
  load_gripper:=false
```

检查:

```bash
curl -X POST http://127.0.0.1:5000/getstate
curl -X POST http://127.0.0.1:5001/getstate
```

## 和 Quest3 遥操的关系

- 单臂/双臂 Quest3 跟手遥操请优先看 `../quest3_oculus_rviz/README.md`。
- 需要独立 HTTP 精准位姿控制时，可以看 `../franky_franka_control/README.md`。
- 本包更适合控制器开发、阻抗参数调试和 libfranka 低层接口验证。

## 安全注意

- 不要同时启动多条 FCI 控制链路。
- 使用 `/pose` 前确认目标位姿在机器人可达范围内。
- 第一次测试速度和运动距离都要小，手边保持急停。
- 如果机器人进入 error，先 `clearerr` 或在 Franka Desk/示教器确认状态，再重新启动控制链路。
