import json
import math
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


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


def as_5x4(values: list[float], parameter_name: str) -> list[list[float]]:
    if len(values) != 20:
        raise ValueError(f"{parameter_name} must contain exactly 20 values")
    converted = [float(value) for value in values]
    if not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{parameter_name} contains a non-finite value")
    return [converted[index:index + 4] for index in range(0, 20, 4)]


@dataclass(frozen=True)
class HandConfig:
    side: str
    serial: str
    trigger_value_name: str
    trigger_button_name: str
    released_pose: list[list[float]]
    closed_pose: list[list[float]]


class HandCommandWorker:
    """Serialize SDK writes for one hand without blocking ROS callbacks."""

    def __init__(self, node: Node, config: HandConfig, hand: Any) -> None:
        self._node = node
        self.config = config
        self.hand = hand
        self._hand_lock = threading.Lock()
        self._condition = threading.Condition()
        self._pending: tuple[str, list[list[float]]] | None = None
        self._stop_requested = False
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
        with self._hand_lock:
            self.hand.write_joint_target_position_unchecked(pose)

    def write_enabled(self, enabled: bool) -> None:
        with self._hand_lock:
            self.hand.write_joint_enabled(enabled)

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
                self.write_pose_sync(pose)
                self._node.get_logger().info(
                    f"[{self.config.side}] Wuji pose command: {pose_name}"
                )
            except Exception as exc:  # noqa: BLE001
                self._node.get_logger().error(
                    f"[{self.config.side}] Failed to write Wuji pose "
                    f"'{pose_name}': {exc!r}"
                )


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

        self.declare_parameter("left_enabled", False)
        self.declare_parameter("left_pose_calibrated", False)
        self.declare_parameter("left_serial", "")
        self.declare_parameter("left_trigger_value_name", "leftTrig")
        self.declare_parameter("left_trigger_button_name", "LTr")
        self.declare_parameter("left_released_pose", [0.0] * 20)
        self.declare_parameter("left_closed_pose", [0.0] * 20)

        self.declare_parameter("right_enabled", True)
        self.declare_parameter("right_pose_calibrated", True)
        self.declare_parameter("right_serial", "")
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
        self.last_buttons_time: float | None = None
        self.timeout_release_active = False
        self._destroying = False

        try:
            for config in configs:
                hand = self._connect_hand(config)
                hand.disable_thread_safe_check()
                worker = HandCommandWorker(self, config, hand)
                self.workers[config.side] = worker
                self.trigger_pressed[config.side] = False
                self.hand_closed[config.side] = False
                worker.write_enabled(True)
                if self.release_on_startup:
                    worker.request_pose(
                        "released_startup",
                        config.released_pose,
                    )
        except Exception:
            self._cleanup_hands(release=True, disable=True)
            raise

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
        enabled_sides = ", ".join(sorted(self.workers))
        self.get_logger().info(
            "Wuji trigger hand node ready. "
            f"hands={enabled_sides}, buttons={self.buttons_topic}, "
            f"press_threshold={self.press_threshold:.2f}, "
            f"release_threshold={self.release_threshold:.2f}, "
            f"timeout={self.buttons_timeout:.2f}s, dry_run={self.dry_run}"
        )

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
                worker.request_pose("close_type3", config.closed_pose)
            else:
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

    def destroy_node(self) -> bool:
        if self._destroying:
            return True
        self._destroying = True
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
