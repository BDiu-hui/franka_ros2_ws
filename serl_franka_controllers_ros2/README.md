# `serl_franka_controllers_ros2` 指令

## 最新操作总结

```text
当前仓库里现在有三条可用控制链路：

1. ROS2 / 阻抗控制链路
   launch: http_control.launch.py 或 http_pose_fallback.launch.py
   接口:
   - /startimp
   - /stopimp
   - /pose

2. 纯 libfranka 非阻抗精准控制链路
   launch: libfranka_http.launch.py
   接口:
   - /pose
   - /getstate
   - /getpos
   - /clearerr

3. franky / Ruckig 精准控制链路
   launch: franky_http.launch.py
   包名: franky_franka_control
   接口:
   - /pose
   - /getstate
   - /getpos
   - /clearerr
   - /get_gripper
   - /activate_gripper
   - /reset_gripper
   - /open_gripper
   - /close_gripper
   - /move_gripper
   - /set_collision
   - /get_collision

重要:
这三条链路不要同时运行。
同一时刻只能有一条链路占用机器人 FCI。
```

### 最新推荐用法

#### 1. 阻抗控制模式

```bash
cd /home/lumos/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serl_franka_controllers_ros2 http_control.launch.py robot_ip:=172.16.0.2 robot_type:=fr3 start_rviz:=false load_gripper:=true
```

```bash
curl -X POST http://127.0.0.1:5000/startimp

curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0]}'
```

```text
这条链路适合阻抗/柔顺控制。
/pose 只有在阻抗控制 active 时才可用。
```

#### 2. 非阻抗精准控制模式

```bash
cd /home/lumos/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch serl_franka_controllers_ros2 libfranka_http.launch.py robot_ip:=172.16.0.2 server_port:=5000
```

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0], "duration_sec": 0.0, "controller_mode": "joint"}'
```

```text
这条链路直接走 libfranka，不经过 ros2_control，也不经过 MoveIt。
适合你要的“不带阻抗控制，末端准确移动到目标位姿”。

controller_mode:
- joint: 更稳，默认推荐
- cartesian: 也可用，但先建议从 joint 开始测
- duration_sec越大，运动越慢， 越小，运动越快
```

#### 2.1 libfranka 常用辅助接口

```bash
curl -X POST http://127.0.0.1:5000/getstate
curl -X POST http://127.0.0.1:5000/getpos
curl -X POST http://127.0.0.1:5000/clearerr
```

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

```text
如果纯 libfranka 控制报错，先 clearerr，再重新发送 /pose。
/set_collision 会在线下发到 libfranka，并且后续 /pose 会继续使用最新阈值。
```

#### 3. franky / Ruckig 精准控制模式

```bash
cd /home/lumos/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch franky_franka_control franky_http.launch.py robot_ip:=172.16.0.2 server_port:=5000
```

```text
这条链路直接使用 franky 控制机械臂。
franky 内部使用 Ruckig 做实时轨迹规划，适合更平滑的精准末端位姿控制。
不要同时启动 http_control.launch.py 或 libfranka_http.launch.py。
```

常用检查:

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

```text
arr 格式固定为:
[x, y, z, qx, qy, qz, qw]

注意:
- qw 在最后，不是在最前面。
- [0,0,0,1] 是 identity 姿态，不一定是 TCP 朝下。
- Franka 常见 TCP 朝下姿态通常接近 [1,0,0,0]。
- 四元数 q 和 -q 表示同一个姿态。
```

只移动位置、保持当前姿态:

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45]}'
```

控制速度比例:

```bash
ros2 launch franky_franka_control franky_http.launch.py \
  robot_ip:=172.16.0.2 \
  server_port:=5000 \
  relative_dynamics_factor:=0.2
```

```bash
curl -X POST http://127.0.0.1:5000/pose \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.43,0.0,0.45,1.0,0.0,0.0,0.0], "relative_dynamics_factor":0.3}'
```

```text
relative_dynamics_factor:
- 0.05 很慢，适合第一次试
- 0.2 正常偏稳，当前默认值
- 0.3 比较明显
- 0.5 偏快，先确认安全空间
```

在线设置碰撞阈值:

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

```text
getstate 返回字段:
- pose: [x, y, z, qx, qy, qz, qw]
- vel: 末端速度
- force: 外力估计
- torque: 外力矩估计
- q: 7 个关节角
- dq: 7 个关节速度

如果 franky 当前拿不到 jacobian，就不返回 jacobian 字段。
如果没有接入夹爪，就不返回 gripper_pos / have_gripper 字段。
```

夹爪控制:

```bash
curl -X POST http://127.0.0.1:5000/get_gripper
curl -X POST http://127.0.0.1:5000/activate_gripper
curl -X POST http://127.0.0.1:5000/reset_gripper
```

```bash
curl -X POST http://127.0.0.1:5000/open_gripper \
  -H "Content-Type: application/json" \
  -d '{"width": 0.08, "speed": 0.05}'
```

```bash
curl -X POST http://127.0.0.1:5000/close_gripper \
  -H "Content-Type: application/json" \
  -d '{"width": 0.0, "speed": 0.03, "force": 40.0, "epsilon_inner": 0.005, "epsilon_outer": 0.005}'
```

```bash
curl -X POST http://127.0.0.1:5000/move_gripper \
  -H "Content-Type: application/json" \
  -d '{"width": 0.04, "speed": 0.05}'
```

```text
franky 夹爪接口不需要额外启动 franka_gripper ROS2 action server。
它直接通过 franky.Gripper(robot_ip) 连接夹爪。
move_gripper 也兼容旧格式 {"arr":[0.04], "speed":0.05}。
```

### 4. 启动/停止阻抗控制

```bash
curl -X POST http://127.0.0.1:5000/startimp
curl -X POST http://127.0.0.1:5000/stopimp
```

```text
/startimp 会激活 cartesian_impedance_controller。
阻抗控制用于 RViz 调参、发送末端目标位姿、笛卡尔柔顺控制。
```

### 5. 读取状态

```bash
curl -X POST http://127.0.0.1:5000/getstate
curl -X POST http://127.0.0.1:5000/getpos
curl -X POST http://127.0.0.1:5000/getq
curl -X POST http://127.0.0.1:5000/getjacobian
```



### 7. 执行关节 reset / 关节 PTP

```bash
curl -X POST http://127.0.0.1:5000/jointreset \
  -H "Content-Type: application/json" \
  -d '{"arr":[0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398], "maximum_joint_velocities":[0.15,0.15,0.15,0.15,0.2,0.2,0.2], "goal_tolerance":0.01}'
```

```text
需要先curl -X POST http://127.0.0.1:5000/stopimp 在发送/jointreset

```

### 8. 双臂 HTTP 端口

```bash
ros2 launch serl_franka_controllers_ros2 http_control.launch.py robot_ip:=172.16.0.2 robot_type:=fr3 namespace:=fr3_02 server_port:=5000 start_rviz:=false load_gripper:=false
ros2 launch serl_franka_controllers_ros2 http_control.launch.py robot_ip:=172.16.0.3 robot_type:=fr3 namespace:=fr3_03 server_port:=5001 start_rviz:=false load_gripper:=false
```

```bash
curl -X POST http://127.0.0.1:5000/getstate
curl -X POST http://127.0.0.1:5001/getstate
```

```text
同一台电脑同时启动两台机械臂时，robot_ip 和 namespace 必须不同，HTTP server_port 也必须不同。
```

## 检查控制器状态

```bash
cd /home/lumos/franka_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control list_controllers
```

## 通过 HTTP 重置 gripper

```bash
curl -X POST http://127.0.0.1:5000/reset_gripper
```

## 通过 HTTP 打开 gripper

```bash
curl -X POST http://127.0.0.1:5000/open_gripper \
  -H "Content-Type: application/json" \
  -d '{"width": 0.08, "speed": 0.05}'
```

## 通过 HTTP 关闭 gripper

```bash
curl -X POST http://127.0.0.1:5000/close_gripper \
  -H "Content-Type: application/json" \
  -d '{"width": 0.0, "speed": 0.03, "force": 40.0, "epsilon_inner": 0.005, "epsilon_outer": 0.005}'
```

## 通过 HTTP 移动 gripper 到指定开口宽度

```bash
curl -X POST http://127.0.0.1:5000/move_gripper \
  -H "Content-Type: application/json" \
  -d '{"width": 0.04, "speed": 0.05}'
```
