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
| `simple_dual_impedance_teleop.launch.py` | 双臂阻抗摇操，可一条命令启动，也可把 Franka 控制栈和 Quest teleop 分进程启动 |
| `simple_dual_policy.launch.py` | 双臂策略推理，启动 Franka、Wuji driver 和 HTTP 策略桥，不启动 Quest/摇操/采集 |
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
  robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  load_gripper:=false \
  start_rviz:=false
```

默认 `auto_start_impedance:=true`，launch 会自动执行等价于下面的动作:

```bash
curl -X POST http://127.0.0.1:5000/startimp
```

默认 `auto_recover_after_reflex:=true`，检测到 Franka reflex / `communication_constraints_violation`
后会自动调用 error recovery，并重新启动阻抗控制器。

如果想手动启动阻抗，把 launch 参数设为 `auto_start_impedance:=false`，再自己 curl。

### 单臂配合舞肌灵巧手

单臂 launch 也可以启动 `wuji_trigger_hand_node`。当前单臂配置默认使用右手柄，
因此默认启用右侧灵巧手:

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py \
  robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  load_gripper:=false \
  start_rviz:=false \
  start_wuji_trigger_hand:=true \
  left_wuji_enabled:=false \
  right_wuji_enabled:=true
```

控制逻辑与双臂完全相同:

```text
第一次按下 rightTrig  -> right_close_type3
第二次按下 rightTrig  -> right_released
```

机械臂仍由 `rightGrip` 使能，灵巧手由 `rightTrig` 控制。两者可以同时按下。

不连接真实灵巧手时，可以验证完整的 Quest 按键链路:

```bash
ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py \
  start_impedance_stack:=false \
  start_wuji_trigger_hand:=true \
  wuji_dry_run:=true
```

如果单臂改用左手柄，需要同时使用左手柄 teleop 配置，并传入:

```bash
left_wuji_enabled:=true right_wuji_enabled:=false
```

左手 `released` / `close_type3` 姿态已经写入
`config/wuji_trigger_hand.yaml`，但默认不启用左手硬件。

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

双臂 launch 同样默认 `auto_recover_after_reflex:=true`，左右臂各自监听 reflex 并自动清错。

默认话题:

| 手臂 | 当前 TCP pose | 阻抗目标 pose |
| --- | --- | --- |
| left | `/left/franka_robot_state_broadcaster/current_pose` | `/left/cartesian_impedance_controller/equilibrium_pose` |
| right | `/right/franka_robot_state_broadcaster/current_pose` | `/right/cartesian_impedance_controller/equilibrium_pose` |

### 双臂分进程绑核启动

所有参数都放在 `config/simple_dual_split_launch.yaml` 里:

| profile | 作用 | 包装 launch |
| --- | --- | --- |
| `all_in_one` | 一条命令启动左右 Franka、Quest reader、左右摇操节点和 Wuji | `simple_dual_all.launch.py` |
| `franka_stack` | 只启动左右 Franka 控制栈 | `simple_dual_franka_stack.launch.py` |
| `quest_teleop` | 只启动 Quest reader、左右摇操节点和 Wuji | `simple_dual_quest_teleop.launch.py` |
| `policy_inference` | 启动左右 Franka、Wuji driver 和 service 模式策略桥，不启动遥操作/采集 | `simple_dual_policy.launch.py` |

扩散策略推理使用专用入口，避免 Quest 或摇操节点覆盖策略目标:

```bash
cd ~/franka_ros2_ws
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_policy.launch.py
```

该 profile 的 Franka HTTP 端口固定为左臂 `5000`、右臂 `5001`，Wuji HTTP
策略桥监听 `127.0.0.1:8765`。其他 profile 仍默认使用 Wuji `trigger` 模式。

一条命令启动全部节点:

```bash
cd ~/franka_ros2_ws
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_all.launch.py
```

如果要把 Franka 1 kHz 控制栈和 Quest 摇操/灵巧手分成不同 launch 进程，使用两个终端启动。

终端 1 只启动左右 Franka 控制栈，不启动 Quest reader 和 teleop:

```bash
cd ~/franka_ros2_ws
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_franka_stack.launch.py
```

终端 2 只启动 Quest reader、左右摇操节点和 Wuji，不重复启动 Franka stack:

```bash
cd ~/franka_ros2_ws
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_quest_teleop.launch.py
```

左手 Wuji 不接或暂时不用时，把 `simple_dual_split_launch.yaml` 的
`quest_teleop.left_wuji_enabled` 改成 `false`。这些 CPU 编号只是推荐起点，可以根据
`htop` 和实际 CPU 拓扑调整。

如果想临时选择其他 profile，也可以直接使用通用入口:

```bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py profile:=all_in_one
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py profile:=franka_stack
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py profile:=quest_teleop
```

## Trigger 控制舞肌灵巧手

`wuji_trigger_hand_node` 是独立的灵巧手硬件节点，订阅 `/quest3/buttons`:

| 手柄输入 | 灵巧手动作 |
| --- | --- |
| `leftTrig` 每按下一次 | 左手在 `close_type3` / `released` 之间切换 |
| `rightTrig` 每按下一次 | 右手在 `right_close_type3` / `right_released` 之间切换 |

节点只在 trigger 从松开变为按下的上升沿启动一次切换；切换内部会在
`released` 和 `closed` 两个 20 维姿态之间做线性插值并分段下发。默认
`trajectory_duration_sec: 0.5`、`trajectory_rate_hz: 50.0`，也就是每次开合约
0.5 秒、25 个中间点。松开 trigger 不会再自动释放；再次按下才会释放。按键消息超过
`buttons_timeout_sec` 未更新时，已经闭合的手会自动回到 `released`。
不接硬件调试时，可设置 ROS 参数 `dry_run:=true`，或在双臂 launch 中传入
`wuji_dry_run:=true`。

当前机械臂摇操仍使用 `leftGrip/rightGrip`，与灵巧手 trigger 输入相互独立。

左右手姿态已经写入 `config/wuji_trigger_hand.yaml`。为了避免误连左手硬件，
当前仍默认只启用右手:

```yaml
left_enabled: false
left_pose_calibrated: true
right_enabled: true
right_pose_calibrated: true
```

左手投入使用时，在 launch 命令后增加:

```bash
left_wuji_enabled:=true
```

只启动 Quest 和右侧灵巧手时，可随双臂 launch 一起启动:

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_impedance_teleop.launch.py \
  start_left_arm:=false \
  start_right_arm:=false \
  start_left_teleop:=false \
  start_right_teleop:=false \
  start_wuji_trigger_hand:=true \
  left_wuji_enabled:=false \
  right_wuji_enabled:=true
```

与双臂摇操一起运行时，在原 launch 命令后增加:

```bash
start_wuji_trigger_hand:=true
```

也可以单独启动节点:

```bash
source setup_env.bash
ros2 run quest3_oculus_rviz wuji_trigger_hand_node \
  --ros-args \
  --params-file /home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/wuji_trigger_hand.yaml
```

扩散策略推理使用 `service` 模式。该模式不订阅 Quest trigger，也不运行按键超时释放逻辑，
避免遥操作命令覆盖策略输出；USB 连接、关节使能和退出释放仍由本节点统一管理。后加载的
`wuji_policy_bridge.yaml` 只覆盖控制模式:

```bash
source setup_env.bash
ros2 run quest3_oculus_rviz wuji_trigger_hand_node \
  --ros-args \
  --params-file /home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/wuji_trigger_hand.yaml \
  --params-file /home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/wuji_policy_bridge.yaml \
  -p left_enabled:=true \
  -p right_enabled:=true
```

服务只允许绑定 loopback，默认监听 `http://127.0.0.1:8765`。启动 EasyDP 前可检查:

```bash
curl http://127.0.0.1:8765/health
```

健康响应必须同时包含 `left` 和 `right`。关节目标接口是同步写入：SDK 写入失败会直接返回
HTTP 错误，调用方不会把“已排队”误认为硬件已经接受。

退出节点时默认先下发 `released`，随后关闭灵巧手关节使能。

当只启用一只灵巧手且 `left_serial/right_serial` 为 `auto` 时，节点会在启动时扫描
`/sys/bus/usb/devices`，读取 `0483:2000` 设备的 `iSerial`，然后使用检测到的序列号
显式连接。启动日志示例:

```text
[right] Auto-detected Wuji USB serial=3671354F3333
[right] Connecting Wuji hand serial=3671354F3333
```

当前 `config/wuji_trigger_hand.yaml` 已固定左右手序列号:

```yaml
left_serial: 3566377E3533
right_serial: 3671354F3333
```

如果临时更换硬件，也可以在 launch 命令里覆盖:

```bash
left_wuji_serial:=3566377E3533 \
right_wuji_serial:=3671354F3333
```

USB 序列号必须使用设备描述符里的 `iSerial`，不能使用之前记录的编号。查询命令:

```bash
lsusb -v -d 0483:2000 | grep iSerial
```

当前连接过的右手实际序列号为 `3671354F3333`。如果日志出现
`Consider relaxing some filters`，通常表示手动传入的序列号与已连接设备不一致。

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

然后封装为阻抗控制器要求的自定义消息并发布:

```text
serl_franka_controllers_ros2/CartesianImpedanceCommand
  header.frame_id = base_frame
  pose = target TCP pose
  has_master_q = false
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
触发：右手柄 `A` 键开始一个 episode，`B` 键停止并保存；左手柄 `X`
键（左手柄上与 `A` 对应的位置）删除本次运行中最近保存的 episode。
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
  out_data_dir:=$HOME/quest3_recordings
```

按 A 开始 → 摇操几秒 → 按 B 停止 → 在 `out_data_dir` 下生成
`episode_0.hdf5`。如果本条数据不要，按左手柄 X 删除；否则再次按 A
开始下一条。文件编号自动从当前目录中的最大编号继续。

### 录制结束后自动移动到随机位姿

单臂采集可以由同一个 launch 同时启动记录器，并在每次 HDF5 成功保存后让机械臂
移动到下一个随机初始位姿：

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py \
  robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  load_gripper:=false \
  start_rviz:=false \
  start_wuji_trigger_hand:=false \
  left_wuji_enabled:=false \
  right_wuji_enabled:=true \
  start_data_recorder:=true \
  out_data_dir:=$HOME/quest3_recordings \
  random_pose_after_recording:=true
```

行为与 `task_insertion_stage1/data_collection/collector.py` 的 long trajectory
采样一致：

- 第一次按 B 并成功保存时，当前 TCP 位姿会锁定为固定
  `insertion_pose`；以后所有随机位姿都围绕这个目标生成，不会逐次随机游走。
- 每次保存后，先保持末端姿态不变，沿机器人基坐标系 `+Z` 抬高 5 cm。
- 抬升完成后，再沿 `insertion_pose` 的局部 `z` 轴退回配置的距离得到
  `approach_pose`，并前往随机目标。
- 位置和 ZYX 欧拉角使用有界截断高斯；插入轴使用正半高斯。
- low/high 两组方差逐条交替，默认参数与自主采集脚本一致。
- 抬升默认用 1.5 秒，之后随机目标用 3 秒平滑插值。任一阶段按下
  `rightGrip` 都会立即取消自动移动并重新锚定遥操作。
- 只有 HDF5 写入成功后才会触发移动；空 episode 或写盘失败不会触发。

相关参数在 `config/simple_impedance_teleop.yaml` 的
`random_pose_*` 区域。首次实机验证建议先把
`random_pose_position_std_high_m` 和 `random_pose_position_bound_m`
调小，再逐步放开。若目标位姿已知，可设置
`random_pose_use_configured_insertion_pose: true`，并填写
`random_pose_configured_insertion_pose: [x, y, z, rz, ry, rx]`。

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
    │   ├── left   (T,)    float32  # Franka gripper 两指 position 之和；未启用时为 NaN
    │   └── right  (T,)    float32
    ├── hand_joint_positions
    │   ├── left   (T,20) float32  # Wuji 左手 20 维目标关节位置
    │   └── right  (T,20) float32  # Wuji 右手 20 维目标关节位置
    ├── hand_actual_joint_positions
    │   ├── left   (T,20) float32  # Wuji 左手 20 维当前/实测关节位置
    │   └── right  (T,20) float32  # Wuji 右手 20 维当前/实测关节位置
    └── images
        └── front  (T, H, W, 3) uint8   # BGR8
```

`cmds` 来自摇操节点发出的 `equilibrium_pose`，`cartesian_poses` 来自
`franka_robot_state_broadcaster/current_pose`，`gripper_width` 来自
`franka_gripper/joint_states`，`hand_joint_positions` 来自
`/hand_left/joint_commands` 和 `/hand_right/joint_commands`，记录的是下发目标；
`hand_actual_joint_positions` 来自 `/hand_left/joint_states` 和
`/hand_right/joint_states`，记录的是官方 wujihandros2 driver 读回的当前状态；
`images/<cam>` 通过 OpenCV V4L2 直接抓取，
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

## 数据记录 (HDF5)

`data_recorder_node` 在双臂阻抗摇操运行时把数据落盘为 `episode_<N>.hdf5`。
触发：右手柄 `A` 键开始一个 episode，`B` 键停止并保存；左手柄 `X`
键（左手柄上与 `A` 对应的位置）删除本次运行中最近保存的 episode。
仅支持双臂场景。

依赖：

```bash
pip install pyrealsense2 h5py
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

按 A 开始 → 摇操几秒 → 按 B 停止 → 在 `out_data_dir` 下生成
`episode_0.hdf5`。如果本条数据不要，按左手柄 X 删除；否则再次按 A
开始下一条。文件编号自动从当前目录中的最大编号继续。

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
    │   ├── left   (T,)    float32  # Franka gripper 两指 position 之和；未启用时为 NaN
    │   └── right  (T,)    float32
    ├── hand_joint_positions
    │   ├── left   (T,20) float32  # Wuji 左手 20 维目标关节位置
    │   └── right  (T,20) float32  # Wuji 右手 20 维目标关节位置
    ├── hand_actual_joint_positions
    │   ├── left   (T,20) float32  # Wuji 左手 20 维当前/实测关节位置
    │   └── right  (T,20) float32  # Wuji 右手 20 维当前/实测关节位置
    └── images
        └── front  (T, H, W, 3) uint8   # BGR8
```

`cmds` 来自摇操节点发出的 `equilibrium_pose`，`cartesian_poses` 来自
`franka_robot_state_broadcaster/current_pose`，`gripper_width` 来自
`franka_gripper/joint_states`，`hand_joint_positions` 来自
`/hand_left/joint_commands` 和 `/hand_right/joint_commands`，记录的是下发目标；
`hand_actual_joint_positions` 来自 `/hand_left/joint_states` 和
`/hand_right/joint_states`，记录的是官方 wujihandros2 driver 读回的当前状态；
`images/<cam>` 通过 OpenCV V4L2 直接抓取，
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
- 启动、恢复或松开 grip 进入 idle 时，teleop 只会把目标同步到当前 TCP pose 一次，之后保持该目标，避免负载下坠时连续跟随。
- 如果出现方向不对，优先改 sign；如果整体坐标系不对，再改 `quest_to_robot_rotation`。

## 分进程绑核启动

终端 1：启动 Franka + impedance hold，并把实时控制与辅助节点放到不同 CPU：

```bash
source setup_env.bash
ros2 launch serl_franka_controllers_ros2 http_control.launch.py \
  robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  start_rviz:=false \
  load_gripper:=false \
  auto_start_impedance:=true \
  auto_recover_after_reflex:=false \
  ros2_control_cpu:=2 \
  franka_aux_cpu:=4 \
  http_server_cpu:=6 \
  watchdog_cpu:=6
```

终端 2：只启动 Quest teleop + recorder，不重复启动 Franka stack：

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_impedance_teleop.launch.py \
  start_impedance_stack:=false \
  robot_ip:=172.16.0.3 \
  robot_type:=fr3 \
  load_gripper:=false \
  start_rviz:=false \
  start_wuji_trigger_hand:=false \
  left_wuji_enabled:=false \
  right_wuji_enabled:=true \
  start_data_recorder:=true \
  out_data_dir:=$HOME/quest3_recordings \
  random_pose_after_recording:=true \
  quest_teleop_cpu:=8 \
  data_recorder_cpu:=10,12
```
