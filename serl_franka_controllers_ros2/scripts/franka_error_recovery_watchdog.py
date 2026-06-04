#!/usr/bin/python3

import os
import threading
import time
from pathlib import Path
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from lifecycle_msgs.msg import State as LifecycleState
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool


class FrankaErrorRecoveryWatchdog(Node):
    """Clear Franka reflex errors and restart impedance after hardware-interface faults."""

    def __init__(self) -> None:
        super().__init__("franka_error_recovery_watchdog")

        self.declare_parameter("enabled", True)
        self.declare_parameter("controller_manager", "controller_manager")
        self.declare_parameter("error_recovery_action", "action_server/error_recovery")
        self.declare_parameter("franka_state_topic", "franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("recovering_topic", "franka_error_recovery_watchdog/recovering")
        self.declare_parameter("impedance_controller", "cartesian_impedance_controller")
        self.declare_parameter(
            "deactivate_controllers_on_restart",
            ["cartesian_pose_command_controller", "joint_position_controller"],
        )
        self.declare_parameter("deactivate_before_recovery", True)
        self.declare_parameter("restart_impedance_after_recovery", True)
        self.declare_parameter("hardware_component_name", "FrankaHardwareInterface")
        self.declare_parameter("reactivate_hardware_after_recovery", True)
        self.declare_parameter("hardware_state_poll_interval_sec", 0.1)
        self.declare_parameter("trigger_patterns", [
            "motion aborted by reflex",
            "communication_constraints_violation",
            "joint_velocity_violation",
            "cartesian_motion_generator_joint_velocity_violation",
        ])
        self.declare_parameter("cooldown_sec", 4.0)
        self.declare_parameter("pre_recovery_delay_sec", 0.2)
        self.declare_parameter("post_recovery_delay_sec", 0.8)
        self.declare_parameter("request_timeout_sec", 8.0)
        self.declare_parameter("recovery_retry_count", 3)
        self.declare_parameter("recovery_retry_interval_sec", 1.0)
        self.declare_parameter("controller_switch_timeout_sec", 5.0)
        self.declare_parameter("log_file_polling_enabled", True)
        self.declare_parameter("log_file_poll_interval_sec", 0.2)
        self.declare_parameter("log_file_max_age_sec", 120.0)
        self.declare_parameter("log_file_glob", "ros2_control_node_*.log")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.controller_manager = str(self.get_parameter("controller_manager").value).strip()
        self.error_recovery_action = str(self.get_parameter("error_recovery_action").value)
        self.franka_state_topic = str(self.get_parameter("franka_state_topic").value)
        self.recovering_topic = str(self.get_parameter("recovering_topic").value)
        self.impedance_controller = str(self.get_parameter("impedance_controller").value)
        self.deactivate_before_recovery = bool(self.get_parameter("deactivate_before_recovery").value)
        self.restart_impedance_after_recovery = bool(
            self.get_parameter("restart_impedance_after_recovery").value
        )
        self.hardware_component_name = str(
            self.get_parameter("hardware_component_name").value
        ).strip()
        self.reactivate_hardware_after_recovery = bool(
            self.get_parameter("reactivate_hardware_after_recovery").value
        )
        self.hardware_state_poll_interval_sec = max(
            0.02,
            float(self.get_parameter("hardware_state_poll_interval_sec").value),
        )
        self.trigger_patterns = [
            str(pattern) for pattern in self.get_parameter("trigger_patterns").value if str(pattern)
        ]
        self.deactivate_controllers_on_restart = [
            str(name) for name in self.get_parameter("deactivate_controllers_on_restart").value if str(name)
        ]
        self.cooldown_sec = max(0.0, float(self.get_parameter("cooldown_sec").value))
        self.pre_recovery_delay_sec = max(0.0, float(self.get_parameter("pre_recovery_delay_sec").value))
        self.post_recovery_delay_sec = max(0.0, float(self.get_parameter("post_recovery_delay_sec").value))
        self.request_timeout_sec = max(0.1, float(self.get_parameter("request_timeout_sec").value))
        self.recovery_retry_count = max(1, int(self.get_parameter("recovery_retry_count").value))
        self.recovery_retry_interval_sec = max(
            0.1,
            float(self.get_parameter("recovery_retry_interval_sec").value),
        )
        self.controller_switch_timeout_sec = max(
            0.1,
            float(self.get_parameter("controller_switch_timeout_sec").value),
        )
        self.log_file_polling_enabled = bool(self.get_parameter("log_file_polling_enabled").value)
        self.log_file_poll_interval_sec = max(
            0.05,
            float(self.get_parameter("log_file_poll_interval_sec").value),
        )
        self.log_file_max_age_sec = max(
            1.0,
            float(self.get_parameter("log_file_max_age_sec").value),
        )
        self.log_file_glob = str(self.get_parameter("log_file_glob").value)

        controller_manager_prefix = (
            self.controller_manager.rstrip("/") if self.controller_manager else "controller_manager"
        )
        switch_service = (
            f"{self.controller_manager.rstrip('/')}/switch_controller"
            if self.controller_manager
            else "controller_manager/switch_controller"
        )
        list_service = f"{controller_manager_prefix}/list_controllers"
        list_hardware_service = f"{controller_manager_prefix}/list_hardware_components"
        set_hardware_state_service = f"{controller_manager_prefix}/set_hardware_component_state"
        self.error_recovery_client = ActionClient(self, ErrorRecovery, self.error_recovery_action)
        self.switch_controller_client = self.create_client(SwitchController, switch_service)
        self.list_controllers_client = self.create_client(ListControllers, list_service)
        self.list_hardware_client = self.create_client(
            ListHardwareComponents, list_hardware_service
        )
        self.set_hardware_state_client = self.create_client(
            SetHardwareComponentState, set_hardware_state_service
        )
        self.recovering_pub = self.create_publisher(Bool, self.recovering_topic, 10)

        qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Log, "/rosout", self._rosout_callback, qos)
        self.create_subscription(
            FrankaRobotState,
            self.franka_state_topic,
            self._franka_state_callback,
            self._best_effort_qos(),
        )

        self._lock = threading.Lock()
        self._recovery_running = False
        self._last_recovery_start = 0.0
        self._last_state_error_signature = ""
        self._start_wall_time = time.time()
        self._log_file_offsets: dict[Path, int] = {}
        self._log_file_dir = self._resolve_log_file_dir()
        if self.log_file_polling_enabled:
            self.create_timer(self.log_file_poll_interval_sec, self._poll_ros2_control_logs)
        self._publish_recovering(False)

        self.get_logger().info(
            "Franka error recovery watchdog ready. "
            f"enabled={self.enabled}, action={self.error_recovery_action}, "
            f"state_topic={self.franka_state_topic}, switch_service={switch_service}, "
            f"restart_impedance={self.restart_impedance_after_recovery}, "
            f"log_polling={self.log_file_polling_enabled}, log_dir={self._log_file_dir}"
        )

    def _rosout_callback(self, msg: Log) -> None:
        if not self.enabled:
            return
        if msg.name == self.get_logger().name:
            return
        stamp_sec = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        if stamp_sec > 0.0 and stamp_sec < self._start_wall_time - 5.0:
            return

        text = f"{msg.name}: {msg.msg}"
        if not any(pattern in text for pattern in self.trigger_patterns):
            return

        self._request_recovery("rosout", text)

    def _poll_ros2_control_logs(self) -> None:
        if not self.enabled:
            return
        now = time.time()
        for path in self._recent_log_files(now):
            try:
                self._poll_one_log_file(path)
            except OSError as exc:
                self.get_logger().warn(f"Could not read ROS control log file {path}: {exc}")

    def _poll_one_log_file(self, path: Path) -> None:
        size = path.stat().st_size
        offset = self._log_file_offsets.get(path)
        if offset is None:
            offset = 0 if path.stat().st_mtime >= self._start_wall_time - 2.0 else size
        if size < offset:
            offset = 0
        if size == offset:
            self._log_file_offsets[path] = offset
            return

        with path.open("r", encoding="utf-8", errors="replace") as log_file:
            log_file.seek(offset)
            while True:
                line = log_file.readline()
                if not line:
                    break
                text = line.strip()
                if any(pattern in text for pattern in self.trigger_patterns):
                    self._request_recovery("log_file", text)
                    break
            self._log_file_offsets[path] = log_file.tell()

    def _franka_state_callback(self, msg: FrankaRobotState) -> None:
        if not self.enabled:
            return

        current_errors = self._active_error_names(msg.current_errors)
        last_motion_errors = self._active_error_names(msg.last_motion_errors)
        in_reflex = msg.robot_mode == FrankaRobotState.ROBOT_MODE_REFLEX
        if not in_reflex and not current_errors:
            return

        signature = (
            f"robot_mode={self._robot_mode_name(msg.robot_mode)}, "
            f"current_errors={current_errors}, last_motion_errors={last_motion_errors}"
        )
        if signature != self._last_state_error_signature:
            self.get_logger().warn(f"Detected Franka error from robot_state: {signature}")
            self._last_state_error_signature = signature
        self._request_recovery("robot_state", signature)

    def _request_recovery(self, source: str, detail: str) -> None:
        now = time.monotonic()
        with self._lock:
            if self._recovery_running:
                return
            if now - self._last_recovery_start < self.cooldown_sec:
                return
            self._recovery_running = True
            self._last_recovery_start = now

        thread = threading.Thread(
            target=self._recover_from_reflex,
            args=(source, detail),
            name="franka_reflex_recovery",
            daemon=True,
        )
        thread.start()

    def _recover_from_reflex(self, source: str, trigger_text: str) -> None:
        try:
            self.get_logger().warn(
                "Detected Franka reflex/error; starting automatic recovery. "
                f"source={source}, trigger='{trigger_text[:240]}'"
            )
            self._publish_recovering(True)
            if self.pre_recovery_delay_sec > 0.0:
                time.sleep(self.pre_recovery_delay_sec)

            if self.deactivate_before_recovery:
                self._deactivate_controllers_for_recovery()

            self._clear_error_with_retries()

            if self.post_recovery_delay_sec > 0.0:
                time.sleep(self.post_recovery_delay_sec)

            if self.reactivate_hardware_after_recovery:
                self._ensure_hardware_component_active()

            if self.restart_impedance_after_recovery:
                self._restart_impedance_controller()

            self.get_logger().info("Automatic Franka recovery finished; impedance teleop can continue.")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Automatic Franka recovery failed: {exc}")
        finally:
            self._publish_recovering(False)
            with self._lock:
                self._recovery_running = False

    def _clear_error_with_retries(self) -> None:
        last_error = None
        for attempt in range(1, self.recovery_retry_count + 1):
            try:
                self._clear_error_once()
                self.get_logger().info(f"Franka error recovery action finished on attempt {attempt}.")
                return
            except Exception as exc:  # pylint: disable=broad-except
                last_error = exc
                if attempt >= self.recovery_retry_count:
                    break
                self.get_logger().warn(
                    f"Franka error recovery attempt {attempt}/{self.recovery_retry_count} failed: {exc}"
                )
                time.sleep(self.recovery_retry_interval_sec)
        raise RuntimeError(f"Franka error recovery failed after retries: {last_error}")

    def _deactivate_controllers_for_recovery(self) -> None:
        try:
            self._switch_controllers([], self._controllers_to_deactivate_for_recovery())
            self.get_logger().info("Stopped active command controllers before error recovery.")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"Could not stop controllers before recovery, continuing anyway: {exc}")

    def _clear_error_once(self) -> None:
        if not self.error_recovery_client.wait_for_server(timeout_sec=self.request_timeout_sec):
            raise RuntimeError(f"Error recovery action server is unavailable: {self.error_recovery_action}")

        goal_future = self.error_recovery_client.send_goal_async(ErrorRecovery.Goal())
        goal_handle = self._await_future(goal_future, self.request_timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Error recovery goal was rejected")

        result_future = goal_handle.get_result_async()
        self._await_future(result_future, self.request_timeout_sec)

    def _ensure_hardware_component_active(self) -> None:
        # After a reflex, FrankaHardwareInterface.read() returns ERROR; the
        # controller_manager then transitions the hardware component out of
        # ACTIVE and the resource_manager marks every command interface as
        # "Not available". libfranka's automaticErrorRecovery() never touches
        # this lifecycle state, so we must walk it back to ACTIVE explicitly
        # before any controller switch can succeed.
        if not self.hardware_component_name:
            return
        try:
            state_id = self._get_hardware_component_state_id()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"Could not query hardware component state for "
                f"'{self.hardware_component_name}': {exc}"
            )
            return

        if state_id == LifecycleState.PRIMARY_STATE_ACTIVE:
            return

        if state_id == LifecycleState.PRIMARY_STATE_UNCONFIGURED:
            self._set_hardware_component_state_with_retry(
                LifecycleState.PRIMARY_STATE_INACTIVE, "inactive"
            )
        # From INACTIVE (or just-configured) → ACTIVE.
        self._set_hardware_component_state_with_retry(
            LifecycleState.PRIMARY_STATE_ACTIVE, "active"
        )
        self._wait_for_hardware_state(LifecycleState.PRIMARY_STATE_ACTIVE)
        self.get_logger().info(
            f"Re-activated hardware component '{self.hardware_component_name}' "
            "to restore command interfaces."
        )

    def _set_hardware_component_state_with_retry(
        self, target_id: int, target_label: str
    ) -> None:
        last_error: Exception | None = None
        for attempt in range(1, self.recovery_retry_count + 1):
            try:
                self._set_hardware_component_state(target_id, target_label)
                return
            except Exception as exc:  # pylint: disable=broad-except
                last_error = exc
                if attempt >= self.recovery_retry_count:
                    break
                self.get_logger().warn(
                    f"Hardware state transition to '{target_label}' "
                    f"attempt {attempt}/{self.recovery_retry_count} failed: {exc}"
                )
                time.sleep(self.recovery_retry_interval_sec)
        raise RuntimeError(
            f"Hardware state transition to '{target_label}' failed after retries: {last_error}"
        )

    def _set_hardware_component_state(self, target_id: int, target_label: str) -> None:
        if not self.set_hardware_state_client.wait_for_service(
            timeout_sec=self.request_timeout_sec
        ):
            raise RuntimeError("set_hardware_component_state service is unavailable")
        request = SetHardwareComponentState.Request()
        request.name = self.hardware_component_name
        request.target_state.id = target_id
        request.target_state.label = target_label
        response = self._await_future(
            self.set_hardware_state_client.call_async(request),
            self.request_timeout_sec,
        )
        if not response.ok:
            raise RuntimeError(
                f"set_hardware_component_state returned ok=False for "
                f"target='{target_label}', actual state id={response.state.id}, "
                f"label='{response.state.label}'"
            )

    def _get_hardware_component_state_id(self) -> int:
        components = self._list_hardware_components()
        if self.hardware_component_name not in components:
            raise RuntimeError(
                f"Hardware component '{self.hardware_component_name}' not found. "
                f"Known: {list(components.keys())}"
            )
        return components[self.hardware_component_name]

    def _list_hardware_components(self) -> dict[str, int]:
        if not self.list_hardware_client.wait_for_service(
            timeout_sec=self.request_timeout_sec
        ):
            raise RuntimeError("list_hardware_components service is unavailable")
        response = self._await_future(
            self.list_hardware_client.call_async(ListHardwareComponents.Request()),
            self.request_timeout_sec,
        )
        return {component.name: component.state.id for component in response.component}

    def _wait_for_hardware_state(self, target_id: int) -> None:
        deadline = time.time() + self.controller_switch_timeout_sec
        last_id = None
        while time.time() < deadline:
            last_id = self._get_hardware_component_state_id()
            if last_id == target_id:
                return
            time.sleep(self.hardware_state_poll_interval_sec)
        raise RuntimeError(
            f"Hardware component '{self.hardware_component_name}' did not reach "
            f"state id={target_id} (last observed id={last_id})"
        )

    def _restart_impedance_controller(self) -> None:
        # If the pre-recovery deactivation was rejected (typical when effort
        # interfaces are "Not available" during the reflex), the impedance
        # controller is still reported as "active" but has lost its live
        # command-interface binding. An activate call would then be a no-op
        # and _wait_for_controller_state would pass immediately on the stale
        # state. Force a real deactivate -> activate cycle in that case.
        states = self._list_controller_states()
        if states.get(self.impedance_controller) == "active":
            deactivate_list = self._unique_controller_names(
                [self.impedance_controller] + self._controllers_to_deactivate()
            )
            self._switch_controllers_with_retry([], deactivate_list)
            self._wait_for_controller_state(self.impedance_controller, "inactive")
            self.get_logger().info(
                f"Deactivated stale impedance controller before re-activation: {deactivate_list}"
            )

        self._switch_controllers_with_retry(
            [self.impedance_controller], self._controllers_to_deactivate()
        )
        self._wait_for_controller_state(self.impedance_controller, "active")
        self.get_logger().info(
            "Impedance controller restart succeeded "
            f"activate={[self.impedance_controller]}, deactivate={self._controllers_to_deactivate()}"
        )

    def _switch_controllers_with_retry(
        self, activate: List[str], deactivate: List[str]
    ) -> None:
        last_error: Exception | None = None
        for attempt in range(1, self.recovery_retry_count + 1):
            try:
                self._switch_controllers(activate, deactivate)
                return
            except Exception as exc:  # pylint: disable=broad-except
                last_error = exc
                if attempt >= self.recovery_retry_count:
                    break
                self.get_logger().warn(
                    f"Switch controllers attempt {attempt}/{self.recovery_retry_count} failed: {exc}"
                )
                time.sleep(self.recovery_retry_interval_sec)
        raise RuntimeError(f"Failed to switch controllers after retries: {last_error}")

    def _switch_controllers(self, activate: List[str], deactivate: List[str]) -> None:
        if not self.switch_controller_client.wait_for_service(timeout_sec=self.request_timeout_sec):
            raise RuntimeError("Controller manager switch_controller service is unavailable")

        request = SwitchController.Request()
        request.activate_controllers = [name for name in activate if name]
        request.deactivate_controllers = [name for name in deactivate if name]
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout.sec = int(self.controller_switch_timeout_sec)
        request.timeout.nanosec = int(
            (self.controller_switch_timeout_sec - request.timeout.sec) * 1_000_000_000
        )
        response = self._await_future(
            self.switch_controller_client.call_async(request),
            self.request_timeout_sec,
        )
        if not response.ok:
            raise RuntimeError(
                "Failed to switch controllers "
                f"activate={request.activate_controllers}, deactivate={request.deactivate_controllers}"
            )

    def _controllers_to_deactivate(self) -> List[str]:
        return [
            name
            for name in self.deactivate_controllers_on_restart
            if name and name != self.impedance_controller
        ]

    def _controllers_to_deactivate_for_recovery(self) -> List[str]:
        controllers = [self.impedance_controller]
        controllers.extend(self.deactivate_controllers_on_restart)
        return self._unique_controller_names(controllers)

    @staticmethod
    def _unique_controller_names(names: List[str]) -> List[str]:
        unique = []
        seen = set()
        for name in names:
            if not name or name in seen:
                continue
            seen.add(name)
            unique.append(name)
        return unique

    def _wait_for_controller_state(self, controller_name: str, desired_state: str) -> None:
        deadline = time.time() + self.controller_switch_timeout_sec
        last_states = {}
        while time.time() < deadline:
            last_states = self._list_controller_states()
            if last_states.get(controller_name) == desired_state:
                return
            time.sleep(0.1)
        raise RuntimeError(
            f"Controller '{controller_name}' did not reach state '{desired_state}'. "
            f"Current controllers={last_states}"
        )

    def _list_controller_states(self) -> dict[str, str]:
        if not self.list_controllers_client.wait_for_service(timeout_sec=self.request_timeout_sec):
            raise RuntimeError("Controller manager list_controllers service is unavailable")
        response = self._await_future(
            self.list_controllers_client.call_async(ListControllers.Request()),
            self.request_timeout_sec,
        )
        return {controller.name: controller.state for controller in response.controller}

    def _publish_recovering(self, recovering: bool) -> None:
        msg = Bool()
        msg.data = recovering
        self.recovering_pub.publish(msg)

    def _recent_log_files(self, now: float) -> list[Path]:
        if not self._log_file_dir.exists():
            return []
        paths = []
        for path in self._log_file_dir.glob(self.log_file_glob):
            try:
                if now - path.stat().st_mtime <= self.log_file_max_age_sec:
                    paths.append(path)
            except OSError:
                continue
        return sorted(paths, key=lambda p: p.stat().st_mtime, reverse=True)[:4]

    @staticmethod
    def _resolve_log_file_dir() -> Path:
        ros_log_dir = os.environ.get("ROS_LOG_DIR")
        if ros_log_dir:
            return Path(ros_log_dir).expanduser()

        cwd_log_dir = Path.cwd() / "log" / "ros"
        if cwd_log_dir.exists():
            return cwd_log_dir

        ros_home = Path(os.environ.get("ROS_HOME", "~/.ros")).expanduser()
        return ros_home / "log"

    @staticmethod
    def _active_error_names(errors) -> List[str]:
        return [
            name
            for name in errors.get_fields_and_field_types().keys()
            if bool(getattr(errors, name))
        ]

    @staticmethod
    def _robot_mode_name(robot_mode: int) -> str:
        names = {
            FrankaRobotState.ROBOT_MODE_OTHER: "OTHER",
            FrankaRobotState.ROBOT_MODE_IDLE: "IDLE",
            FrankaRobotState.ROBOT_MODE_MOVE: "MOVE",
            FrankaRobotState.ROBOT_MODE_GUIDING: "GUIDING",
            FrankaRobotState.ROBOT_MODE_REFLEX: "REFLEX",
            FrankaRobotState.ROBOT_MODE_USER_STOPPED: "USER_STOPPED",
            FrankaRobotState.ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY: "AUTOMATIC_ERROR_RECOVERY",
        }
        return names.get(robot_mode, f"UNKNOWN({robot_mode})")

    @staticmethod
    def _best_effort_qos() -> QoSProfile:
        return QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    @staticmethod
    def _await_future(future, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.01)
        raise RuntimeError("Timed out waiting for ROS response")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrankaErrorRecoveryWatchdog()
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
