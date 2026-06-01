#!/usr/bin/python3

import threading
import time
from typing import List

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from controller_manager_msgs.srv import SwitchController
from franka_msgs.action import ErrorRecovery
from franka_msgs.msg import FrankaRobotState
from rcl_interfaces.msg import Log


class FrankaErrorRecoveryWatchdog(Node):
    """Clear Franka reflex errors and restart impedance after hardware-interface faults."""

    def __init__(self) -> None:
        super().__init__("franka_error_recovery_watchdog")

        self.declare_parameter("enabled", True)
        self.declare_parameter("controller_manager", "controller_manager")
        self.declare_parameter("error_recovery_action", "action_server/error_recovery")
        self.declare_parameter("franka_state_topic", "franka_robot_state_broadcaster/robot_state")
        self.declare_parameter("impedance_controller", "cartesian_impedance_controller")
        self.declare_parameter(
            "deactivate_controllers_on_restart",
            ["cartesian_pose_command_controller", "joint_position_controller"],
        )
        self.declare_parameter("deactivate_before_recovery", True)
        self.declare_parameter("restart_impedance_after_recovery", True)
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

        self.enabled = bool(self.get_parameter("enabled").value)
        self.controller_manager = str(self.get_parameter("controller_manager").value).strip()
        self.error_recovery_action = str(self.get_parameter("error_recovery_action").value)
        self.franka_state_topic = str(self.get_parameter("franka_state_topic").value)
        self.impedance_controller = str(self.get_parameter("impedance_controller").value)
        self.deactivate_before_recovery = bool(self.get_parameter("deactivate_before_recovery").value)
        self.restart_impedance_after_recovery = bool(
            self.get_parameter("restart_impedance_after_recovery").value
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

        switch_service = (
            f"{self.controller_manager.rstrip('/')}/switch_controller"
            if self.controller_manager
            else "controller_manager/switch_controller"
        )
        self.error_recovery_client = ActionClient(self, ErrorRecovery, self.error_recovery_action)
        self.switch_controller_client = self.create_client(SwitchController, switch_service)

        qos = QoSProfile(
            depth=100,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
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

        self.get_logger().info(
            "Franka error recovery watchdog ready. "
            f"enabled={self.enabled}, action={self.error_recovery_action}, "
            f"state_topic={self.franka_state_topic}, switch_service={switch_service}, "
            f"restart_impedance={self.restart_impedance_after_recovery}"
        )

    def _rosout_callback(self, msg: Log) -> None:
        if not self.enabled:
            return
        if msg.name == self.get_logger().name:
            return

        text = f"{msg.name}: {msg.msg}"
        if not any(pattern in text for pattern in self.trigger_patterns):
            return

        self._request_recovery("rosout", text)

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
            if self.pre_recovery_delay_sec > 0.0:
                time.sleep(self.pre_recovery_delay_sec)

            if self.deactivate_before_recovery:
                self._deactivate_controllers_for_recovery()

            self._clear_error_with_retries()

            if self.restart_impedance_after_recovery:
                if self.post_recovery_delay_sec > 0.0:
                    time.sleep(self.post_recovery_delay_sec)
                self._restart_impedance_controller()

            self.get_logger().info("Automatic Franka recovery finished; impedance teleop can continue.")
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Automatic Franka recovery failed: {exc}")
        finally:
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
            self._switch_controllers([], self._controllers_to_deactivate())
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

    def _restart_impedance_controller(self) -> None:
        self._switch_controllers([self.impedance_controller], self._controllers_to_deactivate())
        self.get_logger().info(
            "Impedance controller restart requested "
            f"activate={[self.impedance_controller]}, deactivate={self._controllers_to_deactivate()}"
        )

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
