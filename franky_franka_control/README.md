# franky_franka_control

`franky_franka_control` 是一个基于 `franky` 的 Franka HTTP 控制桥。它直接连接机器人 FCI，通过 `franky`/Ruckig 生成平滑的末端轨迹，适合做精准位姿控制、简单 HTTP 调试和夹爪控制。

当前 Quest3 遥操主链路优先使用 `quest3_oculus_rviz` 里的 `franky_cartesian_pose_node`。本包主要用于独立 HTTP 调试，或者给其他程序提供一个简单的 Franka 控制接口。

## 主要内容

- `franky_franka_control/franky_http_server.py`: HTTP 服务主体，连接 Franka 和夹爪。
- `launch/franky_http.launch.py`: 启动 HTTP 服务的 ROS 2 launch 文件。
- 可执行入口: `franky_http_server.py`

## 环境要求

需要当前 Python 环境能导入 `franky`:

```bash
cd /home/lumos/franka_ros2_ws
source setup_env.bash
python3 -c "import franky; print('franky ok')"
```

如果提示找不到 `franky`，先恢复依赖:

```bash
source setup_env.bash
python3 -m pip install --user franky-control
```

## 构建

```bash
cd /home/lumos/franka_ros2_ws
source setup_env.bash
colcon build --packages-select franky_franka_control --symlink-install
source setup_env.bash
```

## 启动

```bash
ros2 launch franky_franka_control franky_http.launch.py \
  robot_ip:=172.16.0.2 \
  server_port:=5000 \
  relative_dynamics_factor:=0.2
```

常用 launch 参数:

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_ip` | 必填 | Franka 控制柜 IP |
| `server_host` | `0.0.0.0` | HTTP 服务监听地址 |
| `server_port` | `5000` | HTTP 服务端口 |
| `relative_dynamics_factor` | `0.2` | franky 速度、加速度、jerk 比例 |

## HTTP 接口

基础检查:

```bash
curl -X POST http://127.0.0.1:5000/health
curl -X POST http://127.0.0.1:5000/getstate
curl -X POST http://127.0.0.1:5000/getpos
curl -X POST http://127.0.0.1:5000/clearerr
```

发送末端位姿:

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0]}'
```

`arr` 支持两种格式:

- `[x, y, z]`: 只移动位置，保持当前姿态。
- `[x, y, z, qx, qy, qz, qw]`: 移动位置和姿态，四元数顺序是 `qx qy qz qw`。

单次请求也可以覆盖速度比例:

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0], "relative_dynamics_factor":0.1}'
```

夹爪接口:

```bash
curl -X POST http://127.0.0.1:5000/get_gripper
curl -X POST http://127.0.0.1:5000/activate_gripper
curl -X POST http://127.0.0.1:5000/reset_gripper
curl -X POST http://127.0.0.1:5000/open_gripper
curl -X POST http://127.0.0.1:5000/close_gripper
```

指定夹爪宽度:

```bash
curl -X POST http://127.0.0.1:5000/move_gripper \
  -H "Content-Type: application/json" \
  -d '{"width":0.04, "speed":0.05}'
```

碰撞阈值接口:

```bash
curl -X POST http://127.0.0.1:5000/get_collision
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
```

## 注意事项

- 同一台机械臂同一时间只能被一条 FCI 控制链路占用。不要同时启动本包、`quest3_oculus_rviz` 真实遥操、`serl_franka_controllers_ros2` 的 libfranka HTTP。
- `relative_dynamics_factor` 第一次建议用 `0.05` 到 `0.2`，确认工作空间安全后再加大。
- 如果机器人报错，先 `clearerr`，再重新发送目标。
- 这个 HTTP 服务会直接控制真实机械臂，启动前确认急停、示教器状态、工作空间和人员安全。
