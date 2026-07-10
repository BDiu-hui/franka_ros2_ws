import json
import math
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from quest3_oculus_rviz.wuji_command_server import WujiCommandServer


RIGHT_RELEASED = [
    1.001, -0.083, -0.059, -0.188,
    0.226, -0.114, -0.039, -0.028,
    0.439, -0.179, -0.043, -0.138,
    0.402, -0.143, -0.030, -0.098,
    0.223, -0.204, 0.060, 0.338,
]

RIGHT_CLOSE_TYPE3 = [
    1.6, 0.165, 0.150, -0.121,
    1.3, -0.27, 1.486, 1.274,
    0.638, -0.089, 0.968, 0.879,
    0.671, -0.179, 0.974, 0.931,
    0.626, 0.054, 1.286, 1.187,
]

WUJI_USB_VENDOR_ID = "0483"
WUJI_USB_PRODUCT_ID = "2000"
AUTO_SERIAL_VALUES = {"", "auto"}
CONTROL_MODE_TRIGGER = "trigger"
CONTROL_MODE_SERVICE = "service"
CONTROL_MODES = {CONTROL_MODE_TRIGGER, CONTROL_MODE_SERVICE}
WUJI_SDK_LOCK = threading.Lock()


def as_5x4(values: list[float], parameter_name: str) -> list[list[float]]:
    if len(values) != 20:
        raise ValueError(f"{parameter_name} must contain exactly 20 values")
    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{parameter_name} contains a non-finite value")
    return [converted[index:index + 4] for index in range(0, 20, 4)]


def flatten_joint_positions(values: Any, value_name: str) -> list[float]:
    flat = np.asarray(values, dtype=np.float64).reshape(-1)
    if flat.size != 20:
        raise ValueError(f"{value_name} must contain exactly 20 values")
    if not np.all(np.isfinite(flat)):
        raise ValueError(f"{value_name} contains a non-finite value")
    return [float(value) for value in flat]


def positions_as_5x4(values: list[float], value_name: str) -> list[list[float]]:
    if len(values) != 20:
        raise ValueError(f"{value_name} must contain exactly 20 values")
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{value_name} contains a non-finite value")
    return [values[index:index + 4] for index in range(0, 20, 4)]


@dataclass(frozen=True)
class HandConfig:
    side: str
    serial: str
    command_topic: str
    trigger_value_name: str
    trigger_button_name: str
    released_pose: list[list[float]]
    closed_pose: list[list[float]]


class HandCommandWorker:
    """Serialize SDK writes for one hand without blocking ROS callbacks."""

    def __init__(
        self,
        node: Node,
        config: HandConfig,
        hand: Any,
        trajectory_duration_sec: float,
        trajectory_rate_hz: float,
    ) -> None:
        self._node = node
        self.config = config
        self.hand = hand
        self.trajectory_duration_sec = max(float(trajectory_duration_sec), 0.0)
        self.trajectory_rate_hz = max(float(trajectory_rate_hz), 1.0)
        self._hand_lock = threading.Lock()
        self._last_target_lock = threading.Lock()
        self._last_target_positions: list[float] | None = flatten_joint_positions(
            config.released_pose,
            f"{config.side}_released_pose",
        )
        self._condition = threading.Condition()
        self._pending: tuple[str, list[list[float]]] | None = None
        self._stop_requested = False
        self._active_lock = threading.Lock()
        self._command_active = False
        self._thread = threading.Thread(
            target=self._run,
            name=f"wuji_{config.side}_writer",
            daemon=True,
        )
        self._thread.start()

    def request_pose(self, pose_name: str, pose: list[list[float]]) -> None:
        with self._condition:
            self._pending = (pose_name, pose)
            self._condition.notify()

    def write_pose_sync(self, pose: list[list[float]]) -> None:
        positions = flatten_joint_positions(
            pose,
            f"{self.config.side}_target_positions",
        )
        self._write_flat_positions_sync(positions)

    def _write_flat_positions_sync(self, positions: list[float]) -> None:
        pose = positions_as_5x4(
            [float(value) for value in positions],
            f"{self.config.side}_target_positions",
        )
        with self._hand_lock:
            with WUJI_SDK_LOCK:
                self.hand.write_joint_target_position_unchecked(pose)
        self._set_last_target_positions_flat(positions)

    def write_enabled(self, enabled: bool) -> None:
        with self._hand_lock:
            with WUJI_SDK_LOCK:
                self.hand.write_joint_enabled(enabled)

    def read_actual_positions(self, timeout_sec: float) -> list[float]:
        with self._hand_lock:
            with WUJI_SDK_LOCK:
                positions = self.hand.read_joint_actual_position(timeout_sec)
        return flatten_joint_positions(
            positions,
            f"{self.config.side}_actual_positions",
        )

    def try_read_actual_positions(self, timeout_sec: float) -> list[float] | None:
        if self.has_pending_or_active_command():
            return None
        if not self._hand_lock.acquire(blocking=False):
            return None
        try:
            if not WUJI_SDK_LOCK.acquire(blocking=False):
                return None
            try:
                positions = self.hand.read_joint_actual_position(timeout_sec)
            finally:
                WUJI_SDK_LOCK.release()
        finally:
            self._hand_lock.release()
        return flatten_joint_positions(
            positions,
            f"{self.config.side}_actual_positions",
        )

    def last_target_positions(self) -> list[float] | None:
        with self._last_target_lock:
            if self._last_target_positions is None:
                return None
            return list(self._last_target_positions)

    def has_pending_or_active_command(self) -> bool:
        with self._condition:
            pending = self._pending is not None
        with self._active_lock:
            active = self._command_active
        return pending or active

    def stop(self) -> bool:
        with self._condition:
            self._stop_requested = True
            self._pending = None
            self._condition.notify()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            self._node.get_logger().warn(
                f"[{self.config.side}] Wuji writer thread did not stop "
                "within 2 seconds."
            )
            return False
        return True

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._pending is not None or self._stop_requested
                )
                if self._stop_requested:
                    return
                pose_name, pose = self._pending
                self._pending = None

            try:
                self._set_command_active(True)
                completed = self._write_trajectory(pose_name, pose)
                if completed:
                    self._node.get_logger().info(
                        f"[{self.config.side}] Wuji pose command completed: "
                        f"{pose_name}"
                    )
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().error(
                    f"[{self.config.side}] Failed to write Wuji pose "
                    f"'{pose_name}': {exc!r}"
                )
            finally:
                self._set_command_active(False)

    def _write_trajectory(self, pose_name: str, pose: list[list[float]]) -> bool:
        target_positions = flatten_joint_positions(
            pose,
            f"{self.config.side}_{pose_name}",
        )
        start_positions = self.last_target_positions() or target_positions
        steps = max(
            1,
            int(math.ceil(self.trajectory_duration_sec * self.trajectory_rate_hz)),
        )
        if self.trajectory_duration_sec <= 0.0 or steps <= 1:
            self._write_flat_positions_sync(target_positions)
            return True

        step_period = self.trajectory_duration_sec / float(steps)
        self._node.get_logger().info(
            f"[{self.config.side}] Wuji trajectory start: {pose_name}, "
            f"duration={self.trajectory_duration_sec:.3f}s, steps={steps}"
        )

        for step in range(1, steps + 1):
            alpha = float(step) / float(steps)
            command = [
                start + (target - start) * alpha
                for start, target in zip(start_positions, target_positions)
            ]
            self._write_flat_positions_sync(command)

            with self._condition:
                if self._stop_requested:
                    return False
                if self._pending is not None:
                    self._node.get_logger().info(
                        f"[{self.config.side}] Wuji trajectory interrupted: "
                        f"{pose_name}"
                    )
                    return False
                if step < steps:
                    self._condition.wait(timeout=step_period)
                    if self._stop_requested:
                        return False
                    if self._pending is not None:
                        self._node.get_logger().info(
                            f"[{self.config.side}] Wuji trajectory interrupted: "
                            f"{pose_name}"
                        )
                        return False
        return True

    def _set_last_target_positions(self, pose: list[list[float]]) -> None:
        positions = flatten_joint_positions(
            pose,
            f"{self.config.side}_target_positions",
        )
        self._set_last_target_positions_flat(positions)

    def _set_last_target_positions_flat(self, positions: list[float]) -> None:
        with self._last_target_lock:
            self._last_target_positions = [float(value) for value in positions]

    def _set_command_active(self, active: bool) -> None:
        with self._active_lock:
            self._command_active = active


class DryRunHand:
    def disable_thread_safe_check(self) -> None:
        pass

    def write_joint_enabled(self, enabled: bool) -> None:
        del enabled

    def write_joint_target_position_unchecked(
        self,
        positions: list[list[float]],
    ) -> None:
        del positions

    def read_joint_actual_position(self, timeout_sec: float) -> list[list[float]]:
        del timeout_sec
        return positions_as_5x4([0.0] * 20, "dry_run_actual_positions")


class Ros2CommandHand:
    """Adapter that sends Wuji commands through the official ROS2 driver."""

    def __init__(self, node: Node, side: str, command_topic: str) -> None:
        self._node = node
        self._side = side
        self._publisher = node.create_publisher(
            JointState,
            command_topic,
            qos_profile_sensor_data,
        )

    def disable_thread_safe_check(self) -> None:
        pass

    def write_joint_enabled(self, enabled: bool) -> None:
        del enabled

    def write_joint_target_position_unchecked(
        self,
        positions: list[list[float]],
    ) -> None:
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        # The official driver accepts a position-only 20D array in its native
        # joint order. Do not send local debug names here; non-empty names make
        # the driver switch to name matching.
        msg.position = flatten_joint_positions(
            positions,
            f"{self._side}_ros2_command_positions",
        )
        self._publisher.publish(msg)

    def read_joint_actual_position(self, timeout_sec: float) -> list[list[float]]:
        del timeout_sec
        raise RuntimeError(
            "ROS2 command backend does not read actual positions directly; "
            "subscribe to the official driver joint_states topic instead."
        )


class WujiTriggerHandNode(Node):
    """Toggle Wuji hand released/close_type3 poses from Quest index triggers."""

    def __init__(self) -> None:
        super().__init__("wuji_trigger_hand")

        self.declare_parameter("buttons_topic", "/quest3/buttons")
        self.declare_parameter("press_threshold", 0.65)
        self.declare_parameter("release_threshold", 0.35)
        self.declare_parameter("buttons_timeout_sec", 0.5)
        self.declare_parameter("watchdog_rate_hz", 10.0)
        self.declare_parameter("release_on_startup", True)
        self.declare_parameter("release_on_timeout", True)
        self.declare_parameter("release_on_shutdown", True)
        self.declare_parameter("shutdown_release_hold_sec", 0.5)
        self.declare_parameter("disable_on_shutdown", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("control_backend", "ros2")
        self.declare_parameter("trajectory_duration_sec", 0.5)
        self.declare_parameter("trajectory_rate_hz", 50.0)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_source", "target")
        self.declare_parameter("joint_state_topic_prefix", "/wuji")
        self.declare_parameter("joint_state_rate_hz", 30.0)
        self.declare_parameter("joint_state_read_timeout_sec", 0.03)
        self.declare_parameter("control_mode", CONTROL_MODE_TRIGGER)
        self.declare_parameter("command_server_host", "127.0.0.1")
        self.declare_parameter("command_server_port", 8765)
        self.declare_parameter("publish_actual_joint_states", True)
        self.declare_parameter("actual_joint_state_rate_hz", 5.0)
        self.declare_parameter("actual_joint_state_read_timeout_sec", 0.005)
        self.declare_parameter("skip_actual_read_while_commanding", True)

        self.declare_parameter("left_enabled", False)
        self.declare_parameter("left_pose_calibrated", False)
        self.declare_parameter("left_serial", "")
        self.declare_parameter("left_command_topic", "/hand_left/joint_commands")
        self.declare_parameter("left_trigger_value_name", "leftTrig")
        self.declare_parameter("left_trigger_button_name", "LTr")
        self.declare_parameter("left_released_pose", [0.0] * 20)
        self.declare_parameter("left_closed_pose", [0.0] * 20)

        self.declare_parameter("right_enabled", True)
        self.declare_parameter("right_pose_calibrated", True)
        self.declare_parameter("right_serial", "")
        self.declare_parameter("right_command_topic", "/hand_right/joint_commands")
        self.declare_parameter("right_trigger_value_name", "rightTrig")
        self.declare_parameter("right_trigger_button_name", "RTr")
        self.declare_parameter("right_released_pose", RIGHT_RELEASED)
        self.declare_parameter("right_closed_pose", RIGHT_CLOSE_TYPE3)

        self.buttons_topic = str(self.get_parameter("buttons_topic").value)
        self.press_threshold = float(
            self.get_parameter("press_threshold").value
        )
        self.release_threshold = float(
            self.get_parameter("release_threshold").value
        )
        self.buttons_timeout = max(
            float(self.get_parameter("buttons_timeout_sec").value),
            0.0,
        )
        watchdog_rate = max(
            float(self.get_parameter("watchdog_rate_hz").value),
            1.0,
        )
        self.release_on_startup = bool(
            self.get_parameter("release_on_startup").value
        )
        self.release_on_timeout = bool(
            self.get_parameter("release_on_timeout").value
        )
        self.release_on_shutdown = bool(
            self.get_parameter("release_on_shutdown").value
        )
        self.shutdown_release_hold = max(
            float(self.get_parameter("shutdown_release_hold_sec").value),
            0.0,
        )
        self.disable_on_shutdown = bool(
            self.get_parameter("disable_on_shutdown").value
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.control_backend = str(
            self.get_parameter("control_backend").value
        ).strip().lower()
        if self.control_backend not in ("ros2", "sdk"):
            raise ValueError("control_backend must be 'ros2' or 'sdk'")
        self.trajectory_duration_sec = max(
            float(self.get_parameter("trajectory_duration_sec").value),
            0.0,
        )
        self.trajectory_rate_hz = max(
            float(self.get_parameter("trajectory_rate_hz").value),
            1.0,
        )
        self.publish_joint_states = bool(
            self.get_parameter("publish_joint_states").value
        )
        self.joint_state_source = str(
            self.get_parameter("joint_state_source").value
        ).strip().lower()
        if self.joint_state_source not in ("target", "actual"):
            raise ValueError("joint_state_source must be 'target' or 'actual'")
        self.joint_state_topic_prefix = str(
            self.get_parameter("joint_state_topic_prefix").value
        ).rstrip("/")
        if not self.joint_state_topic_prefix:
            self.joint_state_topic_prefix = "/wuji"
        self.joint_state_rate_hz = max(
            float(self.get_parameter("joint_state_rate_hz").value),
            1.0,
        )
        self.joint_state_read_timeout_sec = max(
            float(self.get_parameter("joint_state_read_timeout_sec").value),
            0.0,
        )
        self.control_mode = str(
            self.get_parameter("control_mode").value
        ).strip().lower()
        if self.control_mode not in CONTROL_MODES:
            raise ValueError(
                f"control_mode must be one of {sorted(CONTROL_MODES)}, "
                f"got {self.control_mode!r}"
            )
        self.command_server_host = str(
            self.get_parameter("command_server_host").value
        ).strip()
        self.command_server_port = int(
            self.get_parameter("command_server_port").value
        )
        self.publish_actual_joint_states = bool(
            self.get_parameter("publish_actual_joint_states").value
        )
        if self.control_backend == "ros2" and self.publish_actual_joint_states:
            self.get_logger().warn(
                "publish_actual_joint_states is ignored with control_backend=ros2; "
                "record actual positions from the official driver joint_states topic."
            )
            self.publish_actual_joint_states = False
        self.actual_joint_state_rate_hz = max(
            float(self.get_parameter("actual_joint_state_rate_hz").value),
            1.0,
        )
        self.actual_joint_state_read_timeout_sec = max(
            float(self.get_parameter("actual_joint_state_read_timeout_sec").value),
            0.0,
        )
        self.skip_actual_read_while_commanding = bool(
            self.get_parameter("skip_actual_read_while_commanding").value
        )
        self._validate_thresholds()

        configs = [
            config
            for side in ("left", "right")
            if (config := self._load_hand_config(side)) is not None
        ]
        if not configs:
            raise ValueError(
                "At least one of left_enabled/right_enabled must be true"
            )
        configs = self._resolve_auto_serials(configs)
        self._validate_serials(configs)

        self.workers: dict[str, HandCommandWorker] = {}
        self.trigger_pressed: dict[str, bool] = {}
        self.hand_closed: dict[str, bool] = {}
        self.joint_state_publishers: dict[str, Any] = {}
        self.actual_joint_state_publishers: dict[str, Any] = {}
        self.last_buttons_time: float | None = None
        self.timeout_release_active = False
        self._destroying = False
        self.command_server: WujiCommandServer | None = None
        self._actual_joint_state_stop_event = threading.Event()
        self._actual_joint_state_thread: threading.Thread | None = None

        try:
            for config in configs:
                hand = self._connect_hand(config)
                hand.disable_thread_safe_check()
                worker = HandCommandWorker(
                    self,
                    config,
                    hand,
                    self.trajectory_duration_sec,
                    self.trajectory_rate_hz,
                )
                self.workers[config.side] = worker
                self.trigger_pressed[config.side] = False
                self.hand_closed[config.side] = False
                worker.write_enabled(True)
                if self.release_on_startup:
                    if self.control_mode == CONTROL_MODE_SERVICE:
                        worker.write_pose_sync(config.released_pose)
                    else:
                        worker.request_pose(
                            "released_startup",
                            config.released_pose,
                        )
                if self.publish_joint_states:
                    topic = (
                        f"{self.joint_state_topic_prefix}/"
                        f"{config.side}/joint_states"
                    )
                    self.joint_state_publishers[config.side] = (
                        self.create_publisher(JointState, topic, 10)
                    )
                if self.publish_actual_joint_states:
                    actual_topic = (
                        f"{self.joint_state_topic_prefix}/"
                        f"{config.side}/actual_joint_states"
                    )
                    self.actual_joint_state_publishers[config.side] = (
                        self.create_publisher(JointState, actual_topic, 10)
                    )
        except Exception:
            self._cleanup_hands(release=True, disable=True)
            raise

        self.buttons_sub = None
        self.watchdog_timer = None
        if self.control_mode == CONTROL_MODE_TRIGGER:
            self.buttons_sub = self.create_subscription(
                String,
                self.buttons_topic,
                self.buttons_callback,
                10,
            )
            self.watchdog_timer = self.create_timer(
                1.0 / watchdog_rate,
                self.watchdog_callback,
            )
        else:
            try:
                self.command_server = WujiCommandServer(
                    self.command_server_host,
                    self.command_server_port,
                    self._write_service_joint_targets,
                    lambda: tuple(self.workers),
                )
                self.command_server.start()
            except Exception:
                self._cleanup_hands(release=True, disable=True)
                raise
        self.joint_state_timer = None
        if self.publish_joint_states:
            self.joint_state_timer = self.create_timer(
                1.0 / self.joint_state_rate_hz,
                self.publish_joint_states_callback,
            )
        if self.publish_actual_joint_states:
            self._actual_joint_state_thread = threading.Thread(
                target=self._actual_joint_state_loop,
                name="wuji_actual_joint_state_reader",
                daemon=True,
            )
            self._actual_joint_state_thread.start()
        enabled_sides = ", ".join(sorted(self.workers))
        state_topics = {
            side: f"{self.joint_state_topic_prefix}/{side}/joint_states"
            for side in sorted(self.joint_state_publishers)
        }
        actual_state_topics = {
            side: f"{self.joint_state_topic_prefix}/{side}/actual_joint_states"
            for side in sorted(self.actual_joint_state_publishers)
        }
        self.get_logger().info(
            "Wuji trigger hand node ready. "
            f"hands={enabled_sides}, buttons={self.buttons_topic}, "
            f"control_backend={self.control_backend}, "
            f"press_threshold={self.press_threshold:.2f}, "
            f"release_threshold={self.release_threshold:.2f}, "
            f"timeout={self.buttons_timeout:.2f}s, dry_run={self.dry_run}, "
            f"trajectory_duration={self.trajectory_duration_sec:.3f}s, "
            f"trajectory_rate={self.trajectory_rate_hz:.1f}Hz, "
            f"joint_state_topics={state_topics}, "
            f"actual_joint_state_topics={actual_state_topics}, "
            f"actual_joint_state_rate={self.actual_joint_state_rate_hz:.1f}Hz, "
            f"skip_actual_read_while_commanding="
            f"{self.skip_actual_read_while_commanding}, "
            f"control_mode={self.control_mode}, "
            f"command_server="
            f"{None if self.command_server is None else self.command_server.url}"
        )

    def _write_service_joint_targets(
        self,
        side: str,
        positions: list[float],
    ) -> None:
        worker = self.workers.get(side)
        if worker is None:
            raise KeyError(f"Wuji hand {side!r} is not enabled")
        pose = positions_as_5x4(
            flatten_joint_positions(positions, f"{side}_service_joint_targets"),
            f"{side}_service_joint_targets",
        )
        worker.write_pose_sync(pose)

    def _load_hand_config(self, side: str) -> HandConfig | None:
        if not bool(self.get_parameter(f"{side}_enabled").value):
            return None
        if not bool(self.get_parameter(f"{side}_pose_calibrated").value):
            raise ValueError(
                f"{side}_enabled is true but {side}_pose_calibrated is false. "
                "Configure the measured released and closed poses before "
                "enabling this hand."
            )

        return HandConfig(
            side=side,
            serial=str(self.get_parameter(f"{side}_serial").value).strip(),
            command_topic=str(
                self.get_parameter(f"{side}_command_topic").value
            ).strip(),
            trigger_value_name=str(
                self.get_parameter(f"{side}_trigger_value_name").value
            ).strip(),
            trigger_button_name=str(
                self.get_parameter(f"{side}_trigger_button_name").value
            ).strip(),
            released_pose=as_5x4(
                list(self.get_parameter(f"{side}_released_pose").value),
                f"{side}_released_pose",
            ),
            closed_pose=as_5x4(
                list(self.get_parameter(f"{side}_closed_pose").value),
                f"{side}_closed_pose",
            ),
        )

    def _validate_thresholds(self) -> None:
        if not 0.0 <= self.release_threshold < self.press_threshold <= 1.0:
            raise ValueError(
                "Trigger thresholds must satisfy "
                "0.0 <= release_threshold < press_threshold <= 1.0"
            )

    @staticmethod
    def _validate_serials(configs: list[HandConfig]) -> None:
        if len(configs) <= 1:
            return
        missing = [
            config.side
            for config in configs
            if config.serial.lower() in AUTO_SERIAL_VALUES
        ]
        if missing:
            raise ValueError(
                "Explicit USB serial numbers are required when controlling "
                "two Wuji hands because automatic discovery cannot determine "
                "which device is left or right. "
                f"Missing: {', '.join(missing)}"
            )
        if len({config.serial for config in configs}) != len(configs):
            raise ValueError("left_serial and right_serial must be different")

    def _resolve_auto_serials(
        self,
        configs: list[HandConfig],
    ) -> list[HandConfig]:
        auto_configs = [
            config
            for config in configs
            if config.serial.lower() in AUTO_SERIAL_VALUES
        ]
        if not auto_configs:
            return configs

        if len(configs) > 1:
            return configs
        if self.dry_run:
            self.get_logger().info(
                f"[{configs[0].side}] Dry-run mode: skipping USB serial "
                "auto-discovery"
            )
            return configs

        serials = self._discover_wuji_usb_serials()
        if not serials:
            raise RuntimeError(
                "No Wuji USB device was found for 0483:2000. Check power, "
                "the USB cable, and `lsusb -d 0483:2000`."
            )
        if len(serials) > 1:
            raise RuntimeError(
                "Multiple Wuji USB devices were found, so the enabled hand "
                "cannot be selected safely. Set the hand serial explicitly. "
                f"Detected serials: {', '.join(serials)}"
            )

        detected_serial = serials[0]
        config = configs[0]
        self.get_logger().info(
            f"[{config.side}] Auto-detected Wuji USB "
            f"serial={detected_serial}"
        )
        return [replace(config, serial=detected_serial)]

    @staticmethod
    def _discover_wuji_usb_serials() -> list[str]:
        serials: set[str] = set()
        sysfs_root = Path("/sys/bus/usb/devices")
        try:
            device_paths = list(sysfs_root.iterdir())
        except OSError as exc:
            raise RuntimeError(
                f"Cannot inspect USB devices under {sysfs_root}: {exc}"
            ) from exc

        for device_path in device_paths:
            try:
                vendor = (device_path / "idVendor").read_text().strip().lower()
                product = (
                    device_path / "idProduct"
                ).read_text().strip().lower()
            except OSError:
                continue
            if (
                vendor != WUJI_USB_VENDOR_ID
                or product != WUJI_USB_PRODUCT_ID
            ):
                continue

            try:
                serial = (device_path / "serial").read_text().strip()
            except OSError:
                serial = ""
            if serial:
                serials.add(serial)

        return sorted(serials)

    def _connect_hand(self, config: HandConfig) -> Any:
        if self.dry_run:
            self.get_logger().info(
                f"[{config.side}] Dry-run mode: skipping Wuji USB connection"
            )
            return DryRunHand()

        if self.control_backend == "ros2":
            self.get_logger().info(
                f"[{config.side}] Using official Wuji ROS2 driver command topic: "
                f"{config.command_topic}"
            )
            return Ros2CommandHand(self, config.side, config.command_topic)

        try:
            import wujihandpy
        except ImportError as exc:
            raise RuntimeError(
                "Cannot import wujihandpy in the current ROS Python "
                "environment"
            ) from exc

        self.get_logger().info(
            f"[{config.side}] Connecting Wuji hand serial={config.serial}"
        )
        return wujihandpy.Hand(serial_number=config.serial)

    def buttons_callback(self, msg: String) -> None:
        try:
            buttons = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                f"Failed to parse Quest buttons JSON: {exc}"
            )
            return
        if not isinstance(buttons, dict):
            self.get_logger().warn(
                "Ignoring Quest buttons message that is not a JSON object"
            )
            return

        self.last_buttons_time = time.monotonic()
        self.timeout_release_active = False
        for side, worker in self.workers.items():
            config = worker.config
            trigger_value = max(
                self._button_scalar(
                    buttons.get(config.trigger_value_name, 0.0)
                ),
                self._button_scalar(
                    buttons.get(config.trigger_button_name, False)
                ),
            )
            was_pressed = self.trigger_pressed[side]
            if was_pressed:
                is_pressed = trigger_value > self.release_threshold
            else:
                is_pressed = trigger_value >= self.press_threshold

            if is_pressed == was_pressed:
                continue
            self.trigger_pressed[side] = is_pressed

            if not is_pressed:
                continue

            self.hand_closed[side] = not self.hand_closed[side]
            if self.hand_closed[side]:
                self.get_logger().info(f"[{side}] trigger toggle -> close_type3")
                worker.request_pose("close_type3", config.closed_pose)
            else:
                self.get_logger().info(f"[{side}] trigger toggle -> released")
                worker.request_pose("released_toggle", config.released_pose)

    def watchdog_callback(self) -> None:
        if not self.release_on_timeout or self.buttons_timeout <= 0.0:
            return
        if self.last_buttons_time is None:
            return
        if time.monotonic() - self.last_buttons_time <= self.buttons_timeout:
            return

        released_any = False
        for side, worker in self.workers.items():
            self.trigger_pressed[side] = False
            if not self.hand_closed[side]:
                continue
            self.hand_closed[side] = False
            worker.request_pose(
                "released_timeout",
                worker.config.released_pose,
            )
            released_any = True

        if released_any and not self.timeout_release_active:
            self.get_logger().warn(
                "Quest buttons timed out; requested released pose for "
                "closed Wuji hands."
            )
        self.timeout_release_active = True

    def publish_joint_states_callback(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for side, worker in self.workers.items():
            publisher = self.joint_state_publishers.get(side)
            if publisher is None:
                continue

            positions = worker.last_target_positions()
            if self.joint_state_source == "actual":
                try:
                    positions = worker.read_actual_positions(
                        self.joint_state_read_timeout_sec
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"[{side}] Failed to read Wuji actual joint positions; "
                        f"publishing last target if available: {exc!r}",
                        throttle_duration_sec=2.0,
                    )
                    positions = worker.last_target_positions()
            if positions is None:
                continue

            msg = JointState()
            msg.header.stamp = stamp
            msg.name = self._joint_names(side)
            msg.position = positions
            publisher.publish(msg)

    def _actual_joint_state_loop(self) -> None:
        period = 1.0 / self.actual_joint_state_rate_hz
        while not self._actual_joint_state_stop_event.is_set():
            loop_start = time.monotonic()
            stamp = self.get_clock().now().to_msg()
            if (
                self.skip_actual_read_while_commanding
                and any(
                    worker.has_pending_or_active_command()
                    for worker in self.workers.values()
                )
            ):
                self._actual_joint_state_stop_event.wait(period)
                continue
            for side, worker in list(self.workers.items()):
                publisher = self.actual_joint_state_publishers.get(side)
                if publisher is None:
                    continue
                try:
                    positions = worker.try_read_actual_positions(
                        self.actual_joint_state_read_timeout_sec
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"[{side}] Failed to read Wuji actual joint positions: "
                        f"{exc!r}",
                        throttle_duration_sec=2.0,
                    )
                    continue
                if positions is None:
                    continue

                msg = JointState()
                msg.header.stamp = stamp
                msg.name = self._joint_names(side)
                msg.position = positions
                publisher.publish(msg)

            elapsed = time.monotonic() - loop_start
            self._actual_joint_state_stop_event.wait(max(0.0, period - elapsed))

    def destroy_node(self) -> bool:
        if self._destroying:
            return True
        self._destroying = True
        if self.command_server is not None:
            self.command_server.stop()
            self.command_server = None
        self._actual_joint_state_stop_event.set()
        if self._actual_joint_state_thread is not None:
            self._actual_joint_state_thread.join(timeout=2.0)
            if self._actual_joint_state_thread.is_alive():
                self.get_logger().warn(
                    "Wuji actual joint state reader thread did not stop "
                    "within 2 seconds."
                )
        self._cleanup_hands(
            release=self.release_on_shutdown,
            disable=self.disable_on_shutdown,
            release_hold_sec=self.shutdown_release_hold,
        )
        return super().destroy_node()

    def _cleanup_hands(
        self,
        *,
        release: bool,
        disable: bool,
        release_hold_sec: float = 0.0,
    ) -> None:
        stopped_workers: list[HandCommandWorker] = []
        for worker in self.workers.values():
            if worker.stop():
                stopped_workers.append(worker)

        if release:
            for worker in stopped_workers:
                try:
                    worker.write_pose_sync(worker.config.released_pose)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(
                        f"[{worker.config.side}] Wuji release cleanup "
                        f"failed: {exc!r}"
                    )
            if stopped_workers and release_hold_sec > 0.0:
                time.sleep(release_hold_sec)

        if not disable:
            return
        for worker in stopped_workers:
            try:
                worker.write_enabled(False)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"[{worker.config.side}] Wuji disable cleanup "
                    f"failed: {exc!r}"
                )

    @staticmethod
    def _button_scalar(value: Any) -> float:
        if isinstance(value, (list, tuple)):
            value = value[0] if value else 0.0
        if isinstance(value, bool):
            return 1.0 if value else 0.0
        if isinstance(value, (int, float)):
            return float(value)
        return 0.0

    @staticmethod
    def _joint_names(side: str) -> list[str]:
        return joint_names(side)


def joint_names(side: str) -> list[str]:
    return [f"{side}_wuji_joint_{index:02d}" for index in range(20)]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: WujiTriggerHandNode | None = None
    try:
        node = WujiTriggerHandNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
