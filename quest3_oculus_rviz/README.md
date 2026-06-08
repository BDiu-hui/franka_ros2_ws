# quest3_oculus_rviz

`quest3_oculus_rviz` 是当前 Quest3 手柄遥操 Franka/FR3 的功能包。现在推荐使用
`simple_quest_impedance_teleop_node`：从 Quest 手柄读取左右控制器位姿，计算手柄相邻两帧
delta，把 delta 映射到机器人 TCP 目标位姿，然后发布到
`cartesian_impedance_controller/equilibrium_pose`，由阻抗控制器完成跟踪。

旧的 `franky_cartesian_pose_node`、`franka_real_teleop.launch.py` 和
`franka_dual_real_teleop.launch.py` 还在包里，但当前实机摇操优先使用本文的阻抗控制链路。

## 当前链路

核心节点:

| 节点 | 作用 |
| --- | --- |
| `oculus_tf_node` | 从 Quest/`oculus_reader` 读取左右手柄原始位姿，发布 TF、`PoseStamped` 和按键 JSON |
| `simple_quest_impedance_teleop_node` | 计算手柄 delta，映射成 TCP 目标位姿，发布给阻抗控制器 |

核心 launch:

| launch 文件 | 作用 |
| --- | --- |
| `simple_impedance_teleop.launch.py` | 单臂阻抗摇操，一起启动 Franka HTTP/阻抗控制栈和 Quest teleop |
| `simple_dual_impedance_teleop.launch.py` | 双臂阻抗摇操，启动左右两套阻抗控制栈、一个 Quest reader、两个 teleop 节点 |
| `rviz.launch.py` | Quest pose/RViz 预览和旧链路调试 |

## 构建

```bash
cd /home/lumos/franka_ros2_ws
source setup_env.bash
colcon build --packages-select quest3_oculus_rviz
source setup_env.bash
```

如果改了 Python、launch 或 config 文件，需要重新 build，当前 workspace 不是 symlink install。

## 单臂阻抗摇操

默认使用右手柄，按住 `rightGrip` 使能遥操:

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py \
  robot_ip:=172.16.0.2 \
  robot_type:=fr3 \
  load_gripper:=true \
  start_rviz:=false \
  base_frame:=base
```

默认 `auto_start_impedance:=true`，launch 会自动执行等价于下面的动作:

```bash
curl -X POST http://127.0.0.1:5000/startimp
```

如果想手动启动阻抗，把 launch 参数设为 `auto_start_impedance:=false`，再自己 curl。

## 双臂阻抗摇操

左手柄控制左臂，右手柄控制右臂:

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_impedance_teleop.launch.py \
  left_robot_ip:=172.16.0.2 \
  right_robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  load_gripper:=true \
  start_rviz:=false \
  base_frame:=base
```

默认端口:

| 手臂 | HTTP 端口 | 使能 |
| --- | --- | --- |
| left | `5000` | `leftGrip` |
| right | `5001` | `rightGrip` |

默认话题:

| 手臂 | 当前 TCP pose | 阻抗目标 pose |
| --- | --- | --- |
| left | `/left/franka_robot_state_broadcaster/current_pose` | `/left/cartesian_impedance_controller/equilibrium_pose` |
| right | `/right/franka_robot_state_broadcaster/current_pose` | `/right/cartesian_impedance_controller/equilibrium_pose` |

## 手柄 Frame 和 TCP Frame

`oculus_tf_node` 直接发布 `oculus_reader` 给出的手柄齐次变换:

```text
T_quest_hand =
[ R_quest_hand  p_quest_hand ]
[      0              1      ]
```

其中 `R_quest_hand` 是手柄局部 frame 到 `quest_raw` frame 的旋转矩阵，
`p_quest_hand` 是手柄在 `quest_raw` 下的位置。双臂 launch 中默认发布:

| 手柄 | Pose topic | TF child frame |
| --- | --- | --- |
| left | `/quest3/left_controller/pose` | `quest3_left_controller_raw` |
| right | `/quest3/right_controller/pose` | `quest3_right_controller_raw` |

机器人当前 TCP 位姿来自 Franka state broadcaster:

```text
T_base_tcp =
[ R_base_tcp  p_base_tcp ]
[     0            1    ]
```

teleop 节点不会把 `R_quest_hand` 绝对对齐到 `R_base_tcp`。它只使用前后两帧的相对变化:

```text
delta_p_quest = p_quest_hand_now - p_quest_hand_prev
delta_R_quest = R_quest_hand_prev^T * R_quest_hand_now
```

按下 grip 的第一帧只做锚定:

```text
target_position = current_tcp_position
target_rotation = current_tcp_rotation
previous_hand_pose = current_hand_pose
```

之后每个周期才把手柄 delta 累加到目标 TCP pose。

## 当前坐标映射矩阵

配置里的基础轴映射矩阵是:

```text
R_quest_to_robot =
[  0   0   1 ]
[ -1   0   0 ]
[  0   1   0 ]
```

这个矩阵行列式为 `-1`，是一个带左右手系转换的轴映射，不是普通 SO(3) 旋转矩阵。
代码允许这种情况: 平移直接使用该矩阵；旋转向量会额外乘一次 handedness 修正。

基础轴关系是:

```text
robot_x <-  quest_z
robot_y <- -quest_x
robot_z <-  quest_y
```

当前配置又加了:

```yaml
translation_sign: [-1.0, 1.0, 1.0]
rotation_sign: [1.0, -1.0, 1.0]
translation_scale: 0.6
rotation_scale: 0.6
```

所以当前最终平移映射为:

```text
[delta_x_tcp]   [  0   0  -1 ] [delta_x_quest]
[delta_y_tcp] = [ -1   0   0 ] [delta_y_quest] * 0.6
[delta_z_tcp]   [  0   1   0 ] [delta_z_quest]
```

也就是:

```text
delta_x_tcp = -delta_z_quest * 0.6
delta_y_tcp = -delta_x_quest * 0.6
delta_z_tcp =  delta_y_quest * 0.6
```

旋转先把 `delta_R_quest` 转成旋转向量 `delta_rotvec_quest`，再映射到 TCP 目标姿态:

```text
[delta_rx_tcp]   [  0   0  -1 ] [delta_rx_quest]
[delta_ry_tcp] = [ -1   0   0 ] [delta_ry_quest] * 0.6
[delta_rz_tcp]   [  0  -1   0 ] [delta_rz_quest]
```

也就是:

```text
delta_rx_tcp = -delta_rz_quest * 0.6
delta_ry_tcp = -delta_rx_quest * 0.6
delta_rz_tcp = -delta_ry_quest * 0.6
```

最后目标 TCP 姿态更新为:

```text
target_position = clamp(target_position + delta_p_tcp)
target_rotation = Exp(delta_rotvec_tcp) * target_rotation
```

然后发布:

```text
PoseStamped(header.frame_id = base_frame)
  -> /cartesian_impedance_controller/equilibrium_pose
```

双臂时分别发布到 `/left/.../equilibrium_pose` 和 `/right/.../equilibrium_pose`。

## 速度和限幅

当前配置位置和旋转都比较保守:

```yaml
translation_scale: 0.6
rotation_scale: 0.6
translation_deadband_m: 0.0005
rotation_deadband_rad: 0.003
max_translation_step_m: 0.01
max_rotation_step_rad: 0.08
workspace_min: [0.20, -0.45, 0.08]
workspace_max: [0.80, 0.45, 0.75]
```

调参建议:

| 现象 | 优先改的参数 |
| --- | --- |
| 整体太快 | 降低 `translation_scale` / `rotation_scale` |
| 单帧跳动太大 | 降低 `max_translation_step_m` / `max_rotation_step_rad` |
| 小抖动明显 | 增大 `translation_deadband_m` / `rotation_deadband_rad` |
| 某个轴方向相反 | 改 `translation_sign` 或 `rotation_sign` 对应轴 |
| 活动范围不合适 | 改 `workspace_min` / `workspace_max` |

阻抗刚度/阻尼在:

```text
src/serl_franka_controllers_ros2/config/serl_franka_controllers.yaml
```

当前关键值:

```yaml
translational_stiffness: 2600.0
translational_damping: 170.0
rotational_stiffness: 550.0
rotational_damping: 4.5
```

## 调试话题

单臂默认:

| topic | 内容 |
| --- | --- |
| `/quest3/right_controller/raw_pose` | 原始右手柄 pose |
| `/quest3/simple_teleop/enabled` | teleop 是否使能 |
| `/quest3/simple_teleop/delta` | 本周期映射后的 TCP delta |
| `/quest3/simple_teleop/debug` | JSON 调试信息，包含按钮、scale、sign、delta、reader 状态 |

双臂默认:

| topic | 内容 |
| --- | --- |
| `/quest3/left_impedance_teleop/debug` | 左臂 teleop 调试 JSON |
| `/quest3/right_impedance_teleop/debug` | 右臂 teleop 调试 JSON |
| `/quest3/left_impedance_teleop/delta` | 左臂 TCP delta |
| `/quest3/right_impedance_teleop/delta` | 右臂 TCP delta |

常用检查:

```bash
ros2 topic echo /quest3/buttons
ros2 topic echo /quest3/right_controller/pose
ros2 topic echo /quest3/right_impedance_teleop/debug
```

## Quest 连接检查

真实 Quest 使用 `mock:=false`。需要 Quest 端软件已打开、手柄唤醒，并且电脑能通过 USB
或网络读取 `oculus_reader` 数据。

常见日志:

| 日志 | 含义 |
| --- | --- |
| `Connected to oculus_reader using USB mode` | 已连接 Quest 数据源 |
| `Received r controller pose from Quest` | 已收到右手柄 pose |
| `Received l controller pose from Quest` | 已收到左手柄 pose |
| `No r/l controller pose received from Quest yet` | Quest 端未输出、手柄睡眠、USB 权限未确认或数据中断 |
| `No Franka current pose received yet` | 机器人 state broadcaster 还没发布当前 TCP pose |

不连接真实 Quest 和机器人时，可以做 mock 启动检查:

```bash
ros2 launch quest3_oculus_rviz simple_dual_impedance_teleop.launch.py \
  start_left_arm:=false \
  start_right_arm:=false \
  mock:=true
```

## 数据记录 (HDF5)

`data_recorder_node` 在双臂阻抗摇操运行时把数据落盘为 `episode_<N>.hdf5`。
触发：左手柄 `leftTrig` 开始一个 episode，右手柄 `rightTrig` 停止并保存。
仅支持双臂场景。

依赖：

```bash
sudo apt install python3-opencv v4l-utils
pip install h5py
```

(`h5py`/`scipy`/`python3-opencv` 在 `package.xml` 里走 rosdep。相机改用 V4L2/UVC
鱼眼相机，通过 OpenCV (`cv2.VideoCapture` + `CAP_V4L2`) 抓帧，不再依赖
`pyrealsense2`。)

运行：

```bash
# 终端 1：双臂阻抗摇操
ros2 launch quest3_oculus_rviz simple_dual_impedance_teleop.launch.py

# 终端 2：起记录器
ros2 launch quest3_oculus_rviz data_recorder.launch.py \
  out_data_dir:=/data/quest3_recordings
```

按 leftTrig 开始 → 摇操几秒 → 按 rightTrig 停止 → 在 `out_data_dir` 下生成
`episode_0.hdf5`，下次自动从 `episode_1.hdf5` 续号。

HDF5 结构：

```
episode_N.hdf5
├── cmds
│   ├── left   (T, 6) float32  # x,y,z (m) + rz,ry,rx (deg)
│   └── right  (T, 6) float32
└── observations
    ├── cartesian_poses
    │   ├── left   (T, 6) float32
    │   └── right  (T, 6) float32
    ├── gripper_width
    │   ├── left   (T,)   float32   # 两指 position 之和 (m)
    │   └── right  (T,)   float32
    └── images
        └── front  (T, H, W, 3) uint8   # BGR8
```

`cmds` 来自摇操节点发出的 `equilibrium_pose`，`cartesian_poses` 来自
`franka_robot_state_broadcaster/current_pose`，`gripper_width` 来自
`franka_gripper/joint_states`，`images/<cam>` 通过 OpenCV V4L2 直接抓取，
不经 ROS。相机为 USB 鱼眼相机，按 `usb_path`（或 `serial`）+ `stream_index`
匹配 V4L2 设备，默认分辨率 1280x720 @30 FPS、`MJPG`，可在
`config/data_recorder.yaml` 修改。`camera_names` 列出要录的相机，每个相机用
同名块给出 `usb_path`/`serial`/`stream_index`：

```yaml
camera_names: ["front"]
front:
  usb_path: "1-8.3"   # v4l2-ctl --list-devices 查到的 usb 拓扑路径
  serial: ""          # 或改用 serial 匹配，二选一
  stream_index: 0
```

用 `v4l2-ctl --list-devices` 或查看 `/sys/class/video4linux/video*/device`
找到对应的 `usb_path`。

mock 干跑（无相机、无机器人）：

```bash
ros2 launch quest3_oculus_rviz simple_dual_impedance_teleop.launch.py \
  mock:=true start_left_arm:=false start_right_arm:=false
ros2 launch quest3_oculus_rviz data_recorder.launch.py require_cameras:=false
```

防止内存爆掉：`max_episode_sec` 默认 60 s，超时会自动 stop 并保存。

## 安全注意

- 控制真实机械臂前，确认急停、工作空间、人员位置和机器人错误状态。
- 第一次测试双臂时建议先只启动一侧: `start_left_arm:=true start_right_arm:=false`。
- 同一台机械臂同一时间只能有一个 FCI 控制进程。
- 松开 grip 后 teleop 会停止累加目标，并把目标同步到当前 TCP pose，避免下次按下跳变。
- 如果出现方向不对，优先改 sign；如果整体坐标系不对，再改 `quest_to_robot_rotation`。
