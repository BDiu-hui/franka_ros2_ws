"""HDF5 data recorder for dual-arm Quest3 impedance teleop.

Subscribes to per-arm equilibrium (`cmds`) and current poses, gripper joint
states, and Quest3 buttons. Pulls camera frames directly from V4L2/UVC
fish-eye cameras via the local FishEyeManager. Press the configured start
button (default ``A``) to start a new episode and the stop button (default
``B``) to stop and save it as ``episode_<N>.hdf5``.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

import h5py
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from serl_franka_controllers_ros2.msg import CartesianImpedanceCommand
from std_msgs.msg import String

from quest3_oculus_rviz.fish_eye_manager import FishEyeManager


ARM_NAMES_DEFAULT = ["left", "right"]
CAMERA_NAMES_DEFAULT = ["front"]


class Quest3DataRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("quest3_data_recorder")

        self.declare_parameter("out_data_dir", "/tmp/quest3_recordings")
        self.declare_parameter("recording_rate_hz", 30.0)
        self.declare_parameter("max_episode_sec", 60.0)

        self.declare_parameter("buttons_topic", "/quest3/buttons")
        self.declare_parameter("start_button_analog", "A")
        self.declare_parameter("stop_button_analog", "B")
        self.declare_parameter("trigger_threshold", 0.5)

        self.declare_parameter("require_cameras", True)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("image_fps", 30)
        self.declare_parameter("image_fourcc", "MJPG")
        self.declare_parameter("warmup_frames", 10)
        # Fish-eye cameras are matched on V4L2 usb_path/serial/stream_index,
        # declared per-camera (same flattening trick as the arm topics below).
        self.declare_parameter("camera_names", CAMERA_NAMES_DEFAULT)
        for name in CAMERA_NAMES_DEFAULT:
            self.declare_parameter(f"{name}.usb_path", "")
            self.declare_parameter(f"{name}.serial", "")
            self.declare_parameter(f"{name}.stream_index", 0)

        self.declare_parameter("arm_names", ARM_NAMES_DEFAULT)
        for name in ARM_NAMES_DEFAULT:
            self.declare_parameter(
                f"{name}.cmd_topic",
                f"/{name}/cartesian_impedance_controller/equilibrium_pose",
            )
            self.declare_parameter(
                f"{name}.current_pose_topic",
                f"/{name}/franka_robot_state_broadcaster/current_pose",
            )
            self.declare_parameter(
                f"{name}.gripper_joint_states_topic",
                f"/{name}/franka_gripper/joint_states",
            )

        self.out_data_dir = Path(str(self.get_parameter("out_data_dir").value))
        self.recording_rate_hz = max(float(self.get_parameter("recording_rate_hz").value), 1.0)
        self.max_episode_sec = max(float(self.get_parameter("max_episode_sec").value), 1.0)
        self.buttons_topic = str(self.get_parameter("buttons_topic").value)
        self.start_button_analog = str(self.get_parameter("start_button_analog").value)
        self.stop_button_analog = str(self.get_parameter("stop_button_analog").value)
        self.trigger_threshold = float(self.get_parameter("trigger_threshold").value)
        self.require_cameras = bool(self.get_parameter("require_cameras").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.image_fps = int(self.get_parameter("image_fps").value)
        self.image_fourcc = str(self.get_parameter("image_fourcc").value)
        self.warmup_frames = int(self.get_parameter("warmup_frames").value)

        cam_names = [str(x) for x in self.get_parameter("camera_names").value]
        if not cam_names:
            raise ValueError("camera_names must not be empty")
        # name -> {usb_path?, serial?, stream_index} device selector for FishEyeManager.
        self.cameras_cfg: dict[str, dict[str, Any]] = {}
        for name in cam_names:
            for key, default in (("usb_path", ""), ("serial", ""), ("stream_index", 0)):
                if not self.has_parameter(f"{name}.{key}"):
                    self.declare_parameter(f"{name}.{key}", default)
            usb_path = str(self.get_parameter(f"{name}.usb_path").value)
            serial = str(self.get_parameter(f"{name}.serial").value)
            stream_index = int(self.get_parameter(f"{name}.stream_index").value)
            device_cfg: dict[str, Any] = {"stream_index": stream_index}
            if usb_path:
                device_cfg["usb_path"] = usb_path
            if serial:
                device_cfg["serial"] = serial
            if self.require_cameras and "usb_path" not in device_cfg and "serial" not in device_cfg:
                raise ValueError(
                    f"camera '{name}' requires usb_path or serial when require_cameras=true"
                )
            self.cameras_cfg[name] = device_cfg

        self.arm_names = [str(x) for x in self.get_parameter("arm_names").value]
        if not self.arm_names:
            raise ValueError("arm_names must not be empty")
        self.arm_topics: dict[str, dict[str, str]] = {}
        for name in self.arm_names:
            # Ensure params exist for any non-default arms supplied via YAML.
            for key in ("cmd_topic", "current_pose_topic", "gripper_joint_states_topic"):
                if not self.has_parameter(f"{name}.{key}"):
                    self.declare_parameter(f"{name}.{key}", "")
            self.arm_topics[name] = {
                "cmd": str(self.get_parameter(f"{name}.cmd_topic").value),
                "current": str(self.get_parameter(f"{name}.current_pose_topic").value),
                "gripper": str(self.get_parameter(f"{name}.gripper_joint_states_topic").value),
            }
            for key, value in self.arm_topics[name].items():
                if not value:
                    raise ValueError(f"missing topic for arm '{name}' field '{key}'")

        self.out_data_dir.mkdir(parents=True, exist_ok=True)

        # Latest snapshots, guarded by _state_lock. Each is np.ndarray | float | None.
        self._state_lock = threading.Lock()
        self._latest_cmd: dict[str, np.ndarray | None] = {n: None for n in self.arm_names}
        self._latest_current: dict[str, np.ndarray | None] = {n: None for n in self.arm_names}
        self._latest_gripper: dict[str, float | None] = {n: None for n in self.arm_names}

        # Buffers are owned by the worker thread; no lock needed.
        self._cmd_buf: dict[str, list[np.ndarray]] = {n: [] for n in self.arm_names}
        self._current_buf: dict[str, list[np.ndarray]] = {n: [] for n in self.arm_names}
        self._gripper_buf: dict[str, list[float]] = {n: [] for n in self.arm_names}
        self._image_buf: dict[str, list[np.ndarray]] = {n: [] for n in self.cameras_cfg}

        # Recording control signals.
        self._recording = False
        self._start_event = threading.Event()
        self._stop_event = threading.Event()
        self._shutdown_event = threading.Event()
        self._prev_start_pressed = False
        self._prev_stop_pressed = False

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self._subs: list[Any] = []
        for name, topics in self.arm_topics.items():
            self._subs.append(
                self.create_subscription(
                    CartesianImpedanceCommand,
                    topics["cmd"],
                    self._make_cmd_cb(name),
                    pose_qos,
                )
            )
            self._subs.append(
                self.create_subscription(
                    PoseStamped, topics["current"], self._make_current_cb(name), pose_qos
                )
            )
            self._subs.append(
                self.create_subscription(
                    JointState, topics["gripper"], self._make_gripper_cb(name), 10
                )
            )
        self.buttons_sub = self.create_subscription(
            String, self.buttons_topic, self._buttons_callback, 10
        )

        self.camera_manager: FishEyeManager | None = None
        if self.require_cameras:
            self.camera_manager = FishEyeManager(
                width=self.image_width,
                height=self.image_height,
                fps=self.image_fps,
                fourcc=self.image_fourcc,
                warmup_frames=self.warmup_frames,
            )

        self._worker = threading.Thread(target=self._recording_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(
            "Quest3 data recorder ready. "
            f"out_data_dir={self.out_data_dir}, "
            f"arms={self.arm_names}, cameras={list(self.cameras_cfg.keys())}, "
            f"trigger: {self.start_button_analog}=start / {self.stop_button_analog}=stop"
        )

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=5.0)
        if self.camera_manager is not None:
            self.camera_manager.stop_all()
        return super().destroy_node()

    # ---- callbacks -----------------------------------------------------

    @staticmethod
    def _pose_to_xyz_euler(
        msg: PoseStamped | CartesianImpedanceCommand,
    ) -> np.ndarray:
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        euler = Rotation.from_quat(quat).as_euler("zyx", degrees=True)
        return np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
             euler[0], euler[1], euler[2]],
            dtype=np.float32,
        )

    def _make_cmd_cb(self, name: str):
        def cb(msg: CartesianImpedanceCommand) -> None:
            value = self._pose_to_xyz_euler(msg)
            with self._state_lock:
                self._latest_cmd[name] = value
        return cb

    def _make_current_cb(self, name: str):
        def cb(msg: PoseStamped) -> None:
            value = self._pose_to_xyz_euler(msg)
            with self._state_lock:
                self._latest_current[name] = value
        return cb

    def _make_gripper_cb(self, name: str):
        def cb(msg: JointState) -> None:
            if len(msg.position) < 2:
                return
            width = float(msg.position[0] + msg.position[1])
            with self._state_lock:
                self._latest_gripper[name] = width
        return cb

    def _buttons_callback(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(parsed, dict):
            return
        start_val = self._button_scalar(parsed.get(self.start_button_analog, 0.0))
        stop_val = self._button_scalar(parsed.get(self.stop_button_analog, 0.0))
        start_pressed = start_val >= self.trigger_threshold
        stop_pressed = stop_val >= self.trigger_threshold

        # Rising-edge detection avoids re-triggering while the button is held.
        if start_pressed and not self._prev_start_pressed:
            if not self._recording:
                self._start_event.set()
                self.get_logger().info("start trigger detected")
            else:
                self.get_logger().warn("start trigger ignored: already recording")
        if stop_pressed and not self._prev_stop_pressed:
            if self._recording:
                self._stop_event.set()
                self.get_logger().info("stop trigger detected")
            else:
                self.get_logger().warn("stop trigger ignored: not recording")

        self._prev_start_pressed = start_pressed
        self._prev_stop_pressed = stop_pressed

    @staticmethod
    def _button_scalar(value: Any) -> float:
        if isinstance(value, (list, tuple)):
            if not value:
                return 0.0
            value = value[0]
        if isinstance(value, bool):
            return 1.0 if value else 0.0
        if isinstance(value, (int, float)):
            return float(value)
        return 0.0

    # ---- recording worker ---------------------------------------------

    def _recording_worker(self) -> None:
        if self.camera_manager is not None:
            try:
                ok = self.camera_manager.initialize_all(self.cameras_cfg)
            except Exception as e:
                self.get_logger().error(
                    f"FishEyeManager.initialize_all failed; recording disabled: {e}"
                )
                return
            if not ok:
                self.get_logger().error(
                    "FishEyeManager.initialize_all returned no cameras; recording disabled"
                )
                return
            self.get_logger().info(
                f"fish_eye cameras ready: {list(self.camera_manager.cameras.keys())}"
            )

        period = 1.0 / self.recording_rate_hz
        while not self._shutdown_event.is_set():
            loop_start = time.monotonic()

            if self._start_event.is_set():
                self._start_event.clear()
                self._reset_buffers()
                self._recording = True
                self._episode_start_time = loop_start
                self.get_logger().info("recording started")

            frames: dict[str, np.ndarray] = {}
            if self.camera_manager is not None:
                frames = self.camera_manager.get_all_frames()

            if self._recording:
                snapshot_ok = self._append_sample(frames)
                if not snapshot_ok:
                    pass  # already logged inside
                if loop_start - self._episode_start_time > self.max_episode_sec:
                    self.get_logger().warn(
                        f"max_episode_sec={self.max_episode_sec} exceeded; auto-stopping"
                    )
                    self._stop_event.set()

            if self._stop_event.is_set():
                self._stop_event.clear()
                if self._recording:
                    self._recording = False
                    self._save_current_episode()

            elapsed = time.monotonic() - loop_start
            sleep_for = period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _reset_buffers(self) -> None:
        for buf in (self._cmd_buf, self._current_buf, self._gripper_buf):
            for name in buf:
                buf[name].clear()
        for cam in self._image_buf:
            self._image_buf[cam].clear()

    def _append_sample(self, frames: dict[str, np.ndarray]) -> bool:
        with self._state_lock:
            cmds = {n: self._latest_cmd[n] for n in self.arm_names}
            currents = {n: self._latest_current[n] for n in self.arm_names}
            grippers = {n: self._latest_gripper[n] for n in self.arm_names}

        # Require every per-arm channel before we commit a row, so the
        # final dataset shape stays consistent across arms.
        for name in self.arm_names:
            if cmds[name] is None or currents[name] is None:
                self.get_logger().warn(
                    f"skipping frame: missing cmd/current for arm '{name}'",
                    throttle_duration_sec=2.0,
                )
                return False

        if self.camera_manager is not None:
            for cam in self.cameras_cfg:
                if cam not in frames:
                    self.get_logger().warn(
                        f"skipping frame: missing image from camera '{cam}'",
                        throttle_duration_sec=2.0,
                    )
                    return False

        for name in self.arm_names:
            self._cmd_buf[name].append(cmds[name])
            self._current_buf[name].append(currents[name])
            self._gripper_buf[name].append(
                float(grippers[name]) if grippers[name] is not None else float("nan")
            )
        for cam in self.cameras_cfg:
            if cam in frames:
                self._image_buf[cam].append(frames[cam])
        return True

    # ---- HDF5 writing --------------------------------------------------

    def _next_episode_path(self) -> Path:
        indices: list[int] = []
        for p in self.out_data_dir.glob("episode_*.hdf5"):
            try:
                indices.append(int(p.stem.split("_", 1)[1]))
            except (IndexError, ValueError):
                continue
        next_idx = (max(indices) + 1) if indices else 0
        return self.out_data_dir / f"episode_{next_idx}.hdf5"

    def _save_current_episode(self) -> None:
        first_arm = self.arm_names[0]
        n = len(self._cmd_buf[first_arm])
        if n == 0:
            self.get_logger().warn("stop trigger received but no frames recorded; skipping save")
            return

        # Drop the most recent samples on any short stream so all arms agree.
        for name in self.arm_names:
            n = min(n, len(self._cmd_buf[name]), len(self._current_buf[name]),
                    len(self._gripper_buf[name]))
        if self.camera_manager is not None:
            for cam in self.cameras_cfg:
                n = min(n, len(self._image_buf[cam]))
        if n == 0:
            self.get_logger().warn("nothing to save after alignment")
            return

        target = self._next_episode_path()
        try:
            with h5py.File(target, "w", rdcc_nbytes=1024 ** 2 * 2) as root:
                cmds_grp = root.create_group("cmds")
                for name in self.arm_names:
                    cmds_grp.create_dataset(
                        name, data=np.stack(self._cmd_buf[name][:n], axis=0)
                    )

                obs = root.create_group("observations")
                poses_grp = obs.create_group("cartesian_poses")
                for name in self.arm_names:
                    poses_grp.create_dataset(
                        name, data=np.stack(self._current_buf[name][:n], axis=0)
                    )

                grip_grp = obs.create_group("gripper_width")
                for name in self.arm_names:
                    grip_grp.create_dataset(
                        name,
                        data=np.asarray(self._gripper_buf[name][:n], dtype=np.float32),
                    )

                images_grp = obs.create_group("images")
                for cam in self.cameras_cfg:
                    if not self._image_buf[cam]:
                        continue
                    arr = np.stack(self._image_buf[cam][:n], axis=0)
                    images_grp.create_dataset(
                        cam,
                        data=arr,
                        dtype=arr.dtype,
                        chunks=(1, *arr.shape[1:]),
                    )
        except Exception as e:
            self.get_logger().error(f"failed to write {target}: {e}")
            return

        self.get_logger().info(f"saved {target} with {n} frames")


def main() -> None:
    rclpy.init()
    node = Quest3DataRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
