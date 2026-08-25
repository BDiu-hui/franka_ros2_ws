# FKViewer

FKViewer 是给当前 `franka_ros2_ws` 做的本地前端控制页，风格参考
`/home/lumos/TJViewer/tj_viewer`，但默认不执行真机动作。

## 启动

```bash
cd /home/lumos/franka_ros2_ws
python3 FKViewer/server.py --host 127.0.0.1 --port 8787
```

浏览器打开 `http://127.0.0.1:8787`。主页是可隐藏侧边栏布局，四个功能会在右侧内容区内嵌切换，不再新开窗口：

- 阻抗控制：读取 `src/serl_franka_controllers_ros2/config` 和 `launch`。
- 灵巧手控制：读取 `src/wujihandros2`、`src/wujihandpy`，并列出当前 Wuji bridge YAML。
- 推理控制：读取 `/home/lumos/luolei/easydp/config`，checkpoint 使用 root / group / run / file 多级下拉。
- 摇操控制：读取 `src/quest3_oculus_rviz/config`、`launch`、`simple_dual_split_launch.yaml`
  和 `data_recorder.yaml`，双臂/单臂/采集路径都用下拉配置。

界面会保存侧栏的显示状态和拖动后的宽度。桌面窗口可通过侧栏分隔线调整工作区，
日志框也可以从右下角调整高度；窄窗口会自动切换为纵向布局。顶部操作状态条会持续显示
当前操作的执行中、完成、失败或等待退出状态，右下角通知用于补充即时结果。真机模式下的
启动、停止和离散设备动作需要二次确认；灵巧手关节滑条变化会按设置的防抖间隔直接下发，
无需再点击发送按钮。FKViewer 管理的进程退出后会显示退出码并写入操作记录。
运行日志会过滤周期性的 HTTP GET 健康探测记录；设备连接断开和恢复改用独立通知提示。
日志默认跟随最新输出，手动向上滚动后会暂停跟随，点击“回到底部”才会继续自动滚动。

所有“选择文件/文件夹”类操作都用 dropdown，不使用系统文件选择器。

如果提示端口已占用，说明 FKViewer 或其他服务已经在使用 `8787`。可以直接打开
`http://127.0.0.1:8787`，或者换一个端口启动：

```bash
python3 FKViewer/server.py --host 127.0.0.1 --port 8788
```

## 当前实现

- 阻抗控制：页面结构对齐摇操页，提供 Franka Stack、Policy 阻抗栈和 SERL
  所选 launch 的启动/停止按钮；右上角“设置”用 dropdown 选择 SERL launch、
  目标手臂、刚度/阻尼参数，并直接编辑 `simple_dual_split_launch.yaml`
  中的 `franka_stack` 和 `policy_inference` 参数。左右臂 HTTP `/health`、
  `/getstate` 状态读取，`/startimp`、`/stopimp`、`/clearerr`、
  `/update_param` 动作占位。
- 灵巧手控制：使用官方 `wujihandros2` ROS2 driver，不走 `/home/lumos/wj_ws`
  里的 SDK 直连控制。页面读取 `wuji_trigger_hand.yaml` 里的左右手
  `released/closed` 20 维关节数据，提供左右手“打开/关闭”、20 维实时滑条、
  复制当前数组、导入数组并运行。设置页可调整 launch、姿态 YAML、左右
  `joint_commands` topic、service 名称和滑条范围。
- 推理控制：固定使用 EasyDP `config/task_insertion_stage2/dual_arm_predict.yaml`，
  先启动 `policy_inference` 推理阻抗并等待左右臂 HTTP 与 Wuji 就绪，再运行
  `projects/task_insertion_stage2/client/client_dual.py`。页面支持分步操作和一键顺序启停。
- 摇操控制：窗口里只保留“启动机械臂”和“启动手柄”两个主按钮；右上角“设置”
  打开 `simple_dual_split_launch.yaml` 的所有 profile 参数，保存后下次启动立即生效。
  每个主按钮下方有对应停止按钮，只停止 FKViewer 自己启动的进程；机械臂和手柄日志
  分两个窗口实时滚动显示。“启动手柄”前可勾选 `keep status`；默认不勾选并保持原有
  启停时释放双手的逻辑，勾选后本次进程启停不会下发 `released` 姿态。该选项不影响
  Quest 按钮超时释放，也不会改变 Wuji Driver 退出时的关节失能行为。
- 统一控制：使用独立的 `unified_impedance_control` 包启动一套共享双臂阻抗与
  Wuji Driver。默认由推理控制，Quest `Y` 键在推理/摇操之间切换；摇操接管时
  推理 `/pose` 与 Wuji HTTP 命令会被入口拒绝。Quest 层继续使用原录制节点，
  A/B/X 和 HDF5 数据格式不变。

双臂分终端按钮会使用页面选择的
`src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml`：

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py \
  profile_file:=/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml \
  profile:=franka_stack

source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py \
  profile_file:=/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml \
  profile:=quest_teleop
```

命令白名单仍保留 README 里的默认 wrapper 备用，但摇操窗口不再显示这些入口：

```bash
ros2 launch quest3_oculus_rviz simple_dual_franka_stack.launch.py
ros2 launch quest3_oculus_rviz simple_dual_quest_teleop.launch.py
```

阻抗控制页额外加入了 `quest3_oculus_rviz` README 中的双臂启动入口。
为了让设置页保存的源目录 YAML 立即生效，FKViewer 实际启动时使用
`simple_dual_profile.launch.py` 显式传入 `profile_file`：

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py \
  profile_file:=/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml \
  profile:=franka_stack

source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py \
  profile_file:=/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml \
  profile:=policy_inference
```

这两个入口在页面上分别显示为“启动 Franka Stack”和“启动 Policy Stack”，
各自有独立停止按钮和实时日志窗口。README 里的
`simple_dual_franka_stack.launch.py` / `simple_dual_policy.launch.py`
仍是等价 wrapper，但直接用 wrapper 会读取 install/share 中的默认 profile；
FKViewer 采用显式 `profile_file:=src/...` 的方式避免这个问题。SERL 包内
launch 仍通过设置弹窗里的 dropdown 选择后再启动。

### 推理控制流程

推理页使用以下固定顺序：

```bash
source setup_env.bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py \
  profile_file:=/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml \
  profile:=policy_inference

cd /home/lumos/luolei/easydp
/home/lumos/miniconda3/envs/easydp/bin/python \
  projects/task_insertion_stage2/client/client_dual.py
```

第二步会等待左臂 `5000`、右臂 `5001` 和 Wuji `8765` 健康检查全部通过。
推理配置按钮编辑完整的
`/home/lumos/luolei/easydp/config/task_insertion_stage2/dual_arm_predict.yaml`；保存前使用
EasyDP 环境中的 OmegaConf 校验 YAML，并生成 `.fkviewer.bak` 备份。Checkpoint 下拉默认
服从 YAML 中的 `resume_ckpt_path`，只有手动选择文件时才作为 Hydra override 传给客户端。

### 摇操配置生效方案

ROS2 包安装后，launch 默认会从 `install/` 里的 package share 读取文件；直接改
`src/` 下的 YAML 通常不会影响已经安装的默认配置。FKViewer 摇操页采用另一种方式：
启动脚本时显式传入源文件路径：

```bash
ros2 launch quest3_oculus_rviz simple_dual_profile.launch.py \
  profile_file:=/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_split_launch.yaml \
  profile:=franka_stack
```

因此在设置弹窗里保存的是 `src/.../simple_dual_split_launch.yaml`，不需要改
`install/`，也不需要每次 `colcon build`；保存后重新点击“启动机械臂”或“启动手柄”
就会读取新参数。已经运行中的 ROS launch 不会热更新 YAML，必须先停止对应程序再重新启动。
只有改 Python、launch 文件或需要让 ROS 默认 package share 使用新
文件时，才需要重新 build。

`simple_dual_split_launch.yaml` 里也显式传入了源目录下的低层配置：

- `config/simple_dual_impedance_teleop.yaml`
- `config/wuji_trigger_hand.yaml`
- `config/data_recorder.yaml`

这样这些 YAML 改完后，同样会在下一次从 FKViewer 启动时生效，不依赖
`install/quest3_oculus_rviz/share/...` 里的旧副本。

## 真机执行开关

默认 `live_control=false`。此模式下页面会返回即将执行的命令或 HTTP payload，
但不会启动 ROS launch，也不会下发机械臂/灵巧手动作。

设备空闲后需要真机联调时再启动：

```bash
FKVIEWER_LIVE_CONTROL=1 python3 FKViewer/server.py --host 127.0.0.1 --port 8787
```

FKViewer 只会执行白名单里的动作名，不接受任意 shell 命令。停止按钮只会停止
FKViewer 自己启动的进程，不会杀掉你当前外部终端里的采数据进程。

## 可选配置

如需覆盖端口或路径，可复制新建 `FKViewer/config.json`：

```json
{
  "live_control": false,
  "arms": {
    "left": {"url": "http://127.0.0.1:5000"},
    "right": {"url": "http://127.0.0.1:5001"}
  },
  "wuji": {"url": "http://127.0.0.1:8765"},
  "easydp": {
    "url": "http://127.0.0.1:8090",
    "config": "task_insertion_stage2/dual_arm_predict"
  }
}
```
