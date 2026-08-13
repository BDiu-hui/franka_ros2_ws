"""Original Quest recorder with cameras owned only during teleop authority."""

from __future__ import annotations

import threading
import time

import rclpy
from quest3_oculus_rviz.data_recorder_node import Quest3DataRecorderNode
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class AuthorityGatedDataRecorderNode(Quest3DataRecorderNode):
    """Keep the original recorder/HDF5 logic while gating camera ownership."""

    def __init__(self) -> None:
        self._camera_authority_lock = threading.Lock()
        self._camera_authority_enabled = False
        super().__init__()
        self.declare_parameter(
            "camera_authority_topic", "/unified_impedance/teleop_active"
        )
        authority_qos = QoSProfile(depth=1)
        authority_qos.reliability = ReliabilityPolicy.RELIABLE
        authority_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.camera_authority_sub = self.create_subscription(
            Bool,
            str(self.get_parameter("camera_authority_topic").value),
            self._camera_authority_callback,
            authority_qos,
        )
        self.get_logger().info("Recorder cameras wait for Quest authority")

    def _camera_authority_callback(self, msg: Bool) -> None:
        with self._camera_authority_lock:
            self._camera_authority_enabled = bool(msg.data)

    def _camera_authority_requested(self) -> bool:
        with self._camera_authority_lock:
            return self._camera_authority_enabled

    def _recording_worker(self) -> None:
        cameras_ready = False
        next_camera_retry = 0.0
        period = 1.0 / self.recording_rate_hz

        while not self._shutdown_event.is_set():
            loop_start = time.monotonic()
            camera_requested = self._camera_authority_requested()

            if camera_requested and not cameras_ready:
                if self.camera_manager is None:
                    cameras_ready = True
                elif loop_start >= next_camera_retry:
                    try:
                        cameras_ready = bool(
                            self.camera_manager.initialize_all(self.cameras_cfg)
                        )
                    except Exception as exc:  # noqa: BLE001
                        next_camera_retry = loop_start + 1.0
                        self.get_logger().error(
                            f"Camera takeover failed; retrying in 1s: {exc}"
                        )
                    else:
                        self.get_logger().info(
                            "Quest authority acquired cameras: "
                            f"{list(self.camera_manager.cameras.keys())}"
                        )
                        continue

            if not camera_requested and cameras_ready:
                if self._recording:
                    self.get_logger().warn(
                        "Camera release deferred: press B to stop recording first",
                        throttle_duration_sec=2.0,
                    )
                else:
                    if self.camera_manager is not None:
                        self.camera_manager.stop_all()
                    cameras_ready = False
                    self.get_logger().info("Inference authority released recorder cameras")
                    continue

            if self._start_event.is_set():
                self._start_event.clear()
                if not camera_requested or not cameras_ready:
                    self.get_logger().warn(
                        "start trigger ignored: Quest authority/cameras are not ready"
                    )
                elif not self._recording:
                    self._reset_buffers()
                    self._recording = True
                    self._episode_start_time = loop_start
                    self.get_logger().info("recording started")
                    self._announce(self.voice_start_text)

            frames = {}
            if cameras_ready and self.camera_manager is not None:
                frames = self.camera_manager.get_all_frames()

            if self._recording:
                self._append_sample(frames)
                if loop_start - self._episode_start_time > self.max_episode_sec:
                    self.get_logger().warn(
                        f"max_episode_sec={self.max_episode_sec} exceeded; auto-stopping"
                    )
                    self._stop_event.set()

            if self._stop_event.is_set():
                self._stop_event.clear()
                if self._recording:
                    self._recording = False
                    self._announce(self.voice_stop_text)
                    self._save_current_episode()

            if self._delete_event.is_set():
                self._delete_event.clear()
                self._delete_last_saved_episode()

            sleep_for = period - (time.monotonic() - loop_start)
            if sleep_for > 0:
                time.sleep(sleep_for)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AuthorityGatedDataRecorderNode()
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
