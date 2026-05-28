import json
import math
import types
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from quest3_oculus_rviz.franky_control_modes import make_franky_control_mode


class ControlStatus(str, Enum):
    CONNECTING = "connecting"
    DISABLED = "disabled"
    WAITING_FOR_TARGET = "waiting_for_target"
    INITIAL_TARGET_TOO_FAR = "initial_target_too_far_from_current"
    RUNNING = "running"
    EXCEPTION = "exception"
    STOPPED = "stopped"


@dataclass
class Target:
    valid: bool = False
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=float))
    stamp: float = 0.0


@dataclass
class CurrentState:
    valid: bool = False
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    orientation: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=float))
    q: list[float] = field(default_factory=list)
    dq: list[float] = field(default_factory=list)
    tau_j: list[float] = field(default_factory=list)
    stamp: float = 0.0


def _values(value: Any) -> list[float]:
    if value is None:
        return []
    if callable(value):
        value = value()
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (int, float)):
        return [float(value)]
    if isinstance(value, (str, bytes, dict)):
        return []
    try:
        result: list[float] = []
        for item in value:
            if hasattr(item, "__iter__") and not isinstance(item, (str, bytes, dict)):
                result.extend(_values(item))
            else:
                result.append(float(item))
        return result
    except TypeError:
        return []


def _attr(obj: Any, *names: str) -> Any:
    for name in names:
        if hasattr(obj, name):
            value = getattr(obj, name)
            return value() if callable(value) else value
    return None


def _quat_from_matrix(matrix: list[float]) -> list[float]:
    m00, m01, m02 = matrix[0], matrix[1], matrix[2]
    m10, m11, m12 = matrix[4], matrix[5], matrix[6]
    m20, m21, m22 = matrix[8], matrix[9], matrix[10]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return [(m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale]
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return [0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale]
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return [(m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale]
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return [(m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale]


def _affine_to_pose(affine: Any) -> tuple[np.ndarray, np.ndarray]:
    translation = _values(_attr(affine, "translation", "position", "translation_vector"))
    quaternion = _values(_attr(affine, "quaternion", "orientation", "rotation_quaternion"))
    if len(translation) >= 3 and len(quaternion) >= 4:
        return np.array(translation[:3], dtype=float), _normalize_quaternion(quaternion[:4])

    raw = _values(affine)
    if len(raw) == 7:
        return np.array(raw[:3], dtype=float), _normalize_quaternion(raw[3:7])
    if len(raw) == 16:
        return np.array([raw[12], raw[13], raw[14]], dtype=float), _normalize_quaternion(_quat_from_matrix(raw))
    raise RuntimeError(f"Could not convert franky Affine to pose; available attributes: {dir(affine)}")


def _normalize_quaternion(quaternion: Any) -> np.ndarray:
    result = np.array(quaternion, dtype=float)
    if result.shape[0] != 4:
        raise ValueError("Quaternion must have exactly 4 values")
    norm = float(np.linalg.norm(result))
    if norm < 1e-9:
        raise ValueError("Quaternion norm is zero")
    return result / norm


def _quaternion_angle(a: np.ndarray, b: np.ndarray) -> float:
    dot = abs(float(np.dot(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def _slerp(a: np.ndarray, b: np.ndarray, ratio: float) -> np.ndarray:
    b = b.copy()
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b *= -1.0
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return _normalize_quaternion(a + ratio * (b - a))
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    scale_a = math.cos(theta) - dot * sin_theta / sin_theta_0
    scale_b = sin_theta / sin_theta_0
    return _normalize_quaternion(scale_a * a + scale_b * b)


def _quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _normalize_quaternion(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def _quaternion_inverse(quaternion: np.ndarray) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quaternion_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis = rotvec / angle
    half_angle = 0.5 * angle
    return _normalize_quaternion(
        np.concatenate([axis * math.sin(half_angle), [math.cos(half_angle)]])
    )


def _rotvec_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    if q[3] < 0.0:
        q = -q
    vector_norm = float(np.linalg.norm(q[:3]))
    if vector_norm < 1e-12:
        return np.zeros(3, dtype=float)
    angle = 2.0 * math.atan2(vector_norm, q[3])
    return q[:3] * (angle / vector_norm)


def _quaternion_error_rotvec(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    current_q = _normalize_quaternion(current)
    target_q = _normalize_quaternion(target)
    if float(np.dot(current_q, target_q)) < 0.0:
        target_q = -target_q
    delta_q = _quaternion_multiply(target_q, _quaternion_inverse(current_q))
    return _rotvec_from_quaternion(delta_q)


class FrankyCartesianPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("franky_cartesian_pose")

        self.declare_parameter("robot_ip", "172.16.0.3")
        self.declare_parameter("target_pose_topic", "/franka_sim/tcp_target_pose")
        self.declare_parameter("enabled_topic", "/quest3/right_teleop/enabled")
        self.declare_parameter("current_pose_topic", "/franka_franky/current_pose")
        self.declare_parameter("debug_topic", "/franka_franky/debug")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("base_frame", "panda_link0")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_rate_hz", 30.0)
        self.declare_parameter("control_command_mode", "pose")
        self.declare_parameter("velocity_command_duration_sec", 0.15)
        self.declare_parameter("command_target_lookahead_sec", 0.0)
        self.declare_parameter("target_timeout_sec", 0.25)
        self.declare_parameter("enabled_timeout_sec", 0.25)
        self.declare_parameter("max_linear_velocity_mps", 0.04)
        self.declare_parameter("max_angular_velocity_radps", 0.25)
        self.declare_parameter("max_linear_acceleration_mps2", 0.25)
        self.declare_parameter("max_angular_acceleration_radps2", 1.0)
        self.declare_parameter("max_initial_target_distance_m", 0.08)
        self.declare_parameter("max_initial_target_angle_rad", 0.6)
        self.declare_parameter("workspace_min", [0.20, -0.45, 0.08])
        self.declare_parameter("workspace_max", [0.80, 0.45, 0.75])
        self.declare_parameter(
            "joint_names",
            [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ],
        )
        self.declare_parameter("finger_joint_names", ["panda_finger_joint1", "panda_finger_joint2"])
        self.declare_parameter("finger_width", 0.02)
        self.declare_parameter("enable_gripper", True)
        self.declare_parameter("gripper_command_topic", "/quest3/right_teleop/gripper_command")
        self.declare_parameter("gripper_close_width_m", 0.0)
        self.declare_parameter("gripper_speed_mps", 0.04)
        self.declare_parameter("gripper_close_use_grasp", False)
        self.declare_parameter("gripper_force_n", 30.0)
        self.declare_parameter("gripper_epsilon_inner_m", 0.005)
        self.declare_parameter("gripper_epsilon_outer_m", 0.005)
        self.declare_parameter("relative_dynamics_factor", 0.05)
        self.declare_parameter("stop_relative_dynamics_factor", -1.0)
        self.declare_parameter("automatic_error_recovery", True)
        self.declare_parameter("error_recovery_cooldown_sec", 1.0)
        self.declare_parameter("post_error_recovery_hold_sec", 0.6)
        self.declare_parameter("stop_on_disable", True)
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter(
            "fallback_joint_positions",
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        )

        self.robot_ip = self.get_parameter("robot_ip").value
        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.enabled_topic = self.get_parameter("enabled_topic").value
        self.current_pose_topic = self.get_parameter("current_pose_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self.control_command_mode = str(self.get_parameter("control_command_mode").value).strip().lower()
        if self.control_command_mode not in ("pose", "velocity"):
            self.get_logger().warn(
                f"Unknown control_command_mode='{self.control_command_mode}', using 'pose'."
            )
            self.control_command_mode = "pose"
        self.control_command_strategy = make_franky_control_mode(self.control_command_mode)
        self.velocity_command_duration = max(
            float(self.get_parameter("velocity_command_duration_sec").value),
            0.02,
        )
        self.command_target_lookahead = float(self.get_parameter("command_target_lookahead_sec").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.enabled_timeout_sec = float(self.get_parameter("enabled_timeout_sec").value)
        self.max_linear_velocity = float(self.get_parameter("max_linear_velocity_mps").value)
        self.max_angular_velocity = float(self.get_parameter("max_angular_velocity_radps").value)
        self.max_linear_acceleration = max(
            float(self.get_parameter("max_linear_acceleration_mps2").value),
            0.0,
        )
        self.max_angular_acceleration = max(
            float(self.get_parameter("max_angular_acceleration_radps2").value),
            0.0,
        )
        self.max_initial_target_distance = float(self.get_parameter("max_initial_target_distance_m").value)
        self.max_initial_target_angle = float(self.get_parameter("max_initial_target_angle_rad").value)
        self.workspace_min = self._load_vector3_parameter("workspace_min")
        self.workspace_max = self._load_vector3_parameter("workspace_max")
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.finger_joint_names = list(self.get_parameter("finger_joint_names").value)
        self.finger_width = float(self.get_parameter("finger_width").value)
        self.enable_gripper = bool(self.get_parameter("enable_gripper").value)
        self.gripper_command_topic = str(self.get_parameter("gripper_command_topic").value)
        self.gripper_close_width = float(self.get_parameter("gripper_close_width_m").value)
        self.gripper_speed = float(self.get_parameter("gripper_speed_mps").value)
        self.gripper_close_use_grasp = bool(self.get_parameter("gripper_close_use_grasp").value)
        self.gripper_force = float(self.get_parameter("gripper_force_n").value)
        self.gripper_epsilon_inner = float(self.get_parameter("gripper_epsilon_inner_m").value)
        self.gripper_epsilon_outer = float(self.get_parameter("gripper_epsilon_outer_m").value)
        self.relative_dynamics_factor = float(self.get_parameter("relative_dynamics_factor").value)
        configured_stop_dynamics = float(self.get_parameter("stop_relative_dynamics_factor").value)
        self.stop_relative_dynamics_factor = (
            self.relative_dynamics_factor
            if configured_stop_dynamics < 0.0
            else configured_stop_dynamics
        )
        self.automatic_error_recovery = bool(self.get_parameter("automatic_error_recovery").value)
        self.error_recovery_cooldown_sec = float(self.get_parameter("error_recovery_cooldown_sec").value)
        self.post_error_recovery_hold_sec = max(
            float(self.get_parameter("post_error_recovery_hold_sec").value),
            0.0,
        )
        self.stop_on_disable = bool(self.get_parameter("stop_on_disable").value)
        self.reconnect_interval_sec = float(self.get_parameter("reconnect_interval_sec").value)
        self.fallback_joint_positions = list(self.get_parameter("fallback_joint_positions").value)

        if len(self.joint_names) != 7:
            raise RuntimeError("joint_names must contain exactly 7 names")
        if len(self.finger_joint_names) != 2:
            raise RuntimeError("finger_joint_names must contain exactly 2 names")
        if len(self.fallback_joint_positions) != 7:
            raise RuntimeError("fallback_joint_positions must contain exactly 7 values")

        try:
            from franky._franky import (  # pylint: disable=import-outside-toplevel
                Affine,
                CartesianMotion,
                CartesianStopMotion,
                CartesianVelocityMotion,
                CartesianVelocityStopMotion,
                Gripper,
                ReferenceType,
                Twist,
                Duration,
            )
            from franky.robot import Robot  # pylint: disable=import-outside-toplevel
        except ImportError as exc:
            raise RuntimeError(
                "Python package 'franky' is not available in this environment. "
                "From /home/lumos/franka_ros2_ws run: source setup_env.bash. "
                "If .rosdeps was deleted, restore franky from the backup workspace or reinstall franky-control."
            ) from exc

        self.franky = types.SimpleNamespace(
            Affine=Affine,
            CartesianMotion=CartesianMotion,
            CartesianStopMotion=CartesianStopMotion,
            CartesianVelocityMotion=CartesianVelocityMotion,
            CartesianVelocityStopMotion=CartesianVelocityStopMotion,
            Gripper=Gripper,
            ReferenceType=ReferenceType,
            Twist=Twist,
            Duration=Duration,
            Robot=Robot,
        )
        self.robot_lock = threading.RLock()
        self.target_lock = threading.RLock()
        self.state_lock = threading.RLock()
        self.status_lock = threading.RLock()
        self.gripper_lock = threading.RLock()
        self.command_send_lock = threading.RLock()

        self.target = Target()
        self.teleop_enabled = False
        self.enabled_stamp = 0.0
        self.current_state = CurrentState()
        self.status = ControlStatus.CONNECTING
        self.last_exception = ""
        self.command_position: np.ndarray | None = None
        self.command_orientation: np.ndarray | None = None
        self.command_linear_velocity = np.zeros(3, dtype=float)
        self.command_angular_velocity = np.zeros(3, dtype=float)
        self.motion_active = False
        self.running = True
        self.robot = None
        self.gripper = None
        self.gripper_thread: threading.Thread | None = None
        self.last_gripper_command = ""
        self.last_gripper_result = "disabled" if not self.enable_gripper else "idle"
        self.last_gripper_exception = ""
        self.last_connect_attempt = 0.0
        self.last_error_recovery_attempt = 0.0
        self.last_error_recovery_result = "not_attempted"
        self.recovery_hold_until = 0.0

        self.target_sub = self.create_subscription(PoseStamped, self.target_pose_topic, self.target_callback, 10)
        self.enabled_sub = self.create_subscription(Bool, self.enabled_topic, self.enabled_callback, 10)
        self.gripper_sub = None
        if self.enable_gripper:
            self.gripper_sub = self.create_subscription(
                String,
                self.gripper_command_topic,
                self.gripper_command_callback,
                10,
            )
        self.current_pose_pub = self.create_publisher(PoseStamped, self.current_pose_topic, 10)
        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self._try_connect_robot()
        publish_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.publish_timer = self.create_timer(publish_period, self.publish_state_timer)
        self.control_thread = threading.Thread(target=self.control_loop, name="franky_cartesian_control", daemon=True)
        self.control_thread.start()

        self.get_logger().info(
            "Franky Cartesian pose node started. "
            f"robot_ip={self.robot_ip} target={self.target_pose_topic} enabled={self.enabled_topic}"
        )
        if self.enable_gripper:
            self.get_logger().info(
                f"Franky gripper commands enabled on {self.gripper_command_topic}: open/close"
            )

    def destroy_node(self) -> bool:
        self.running = False
        if hasattr(self, "control_thread") and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        if hasattr(self, "gripper_thread") and self.gripper_thread and self.gripper_thread.is_alive():
            self.gripper_thread.join(timeout=0.5)
        if self.motion_active:
            self._send_stop_motion()
        return super().destroy_node()

    def _connect_robot(self) -> Any:
        robot = self.franky.Robot(self.robot_ip)
        robot.relative_dynamics_factor = self.relative_dynamics_factor
        if self.automatic_error_recovery:
            robot.recover_from_errors()
        return robot

    def _try_connect_robot(self) -> bool:
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval_sec:
            return False
        self.last_connect_attempt = now
        self._set_status(ControlStatus.CONNECTING)
        try:
            robot = self._connect_robot()
        except Exception as exc:  # pylint: disable=broad-except
            with self.robot_lock:
                self.robot = None
            self._set_exception(exc)
            return False
        with self.robot_lock:
            self.robot = robot
        self.last_exception = ""
        self._set_status(ControlStatus.DISABLED)
        self.get_logger().info(f"Connected to Franky robot at {self.robot_ip}")
        return True

    def _load_vector3_parameter(self, name: str) -> np.ndarray:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise RuntimeError(f"{name} must contain exactly 3 values")
        return np.array(values, dtype=float)

    def target_callback(self, msg: PoseStamped) -> None:
        try:
            orientation = _normalize_quaternion(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
            )
        except ValueError as exc:
            self.get_logger().warn(f"Ignoring invalid target pose: {exc}")
            return

        target = Target(
            valid=True,
            position=np.array(
                [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ],
                dtype=float,
            ),
            orientation=orientation,
            stamp=time.monotonic(),
        )
        with self.target_lock:
            self.target = target

    def enabled_callback(self, msg: Bool) -> None:
        enabled = bool(msg.data)
        should_stop_now = False
        with self.target_lock:
            was_enabled = self.teleop_enabled
            self.teleop_enabled = enabled
            self.enabled_stamp = time.monotonic()
            if was_enabled and not enabled:
                self.target = Target()
                should_stop_now = True

        if should_stop_now:
            self._handle_teleop_disabled()

    def _handle_teleop_disabled(self) -> None:
        with self.command_send_lock:
            with self.state_lock:
                current = self.current_state
            if current.valid:
                self.command_position = current.position.copy()
                self.command_orientation = current.orientation.copy()
            else:
                self.command_position = None
                self.command_orientation = None
            self._reset_command_dynamics()
            self._stop_if_active(force=True)
            self._set_status(ControlStatus.DISABLED)

    def gripper_command_callback(self, msg: String) -> None:
        command = self._parse_gripper_command(msg.data)
        if command not in ("open", "close"):
            self.get_logger().warn(f"Ignoring unknown gripper command: {msg.data}")
            return
        self._start_gripper_command(command)

    @staticmethod
    def _parse_gripper_command(data: str) -> str:
        text = str(data).strip().lower()
        if not text:
            return ""
        if text.startswith("{"):
            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                return text
            text = str(payload.get("command", payload.get("action", ""))).strip().lower()
        if text in ("open", "release"):
            return "open"
        if text in ("close", "grasp"):
            return "close"
        return text

    def _start_gripper_command(self, command: str) -> None:
        if not self.enable_gripper:
            return
        with self.gripper_lock:
            if self.gripper_thread is not None and self.gripper_thread.is_alive():
                self.last_gripper_result = f"busy_ignored_{command}"
                self.get_logger().warn(
                    f"Ignoring gripper command '{command}' because a previous command is still running."
                )
                return
            self.last_gripper_command = command
            self.last_gripper_result = "running"
            self.last_gripper_exception = ""
            self.gripper_thread = threading.Thread(
                target=self._run_gripper_command,
                args=(command,),
                name=f"franky_gripper_{command}",
                daemon=True,
            )
            self.gripper_thread.start()

    def _get_or_connect_gripper(self) -> Any:
        with self.gripper_lock:
            if self.gripper is None:
                self.gripper = self.franky.Gripper(self.robot_ip)
                self.get_logger().info(f"Connected to Franky gripper at {self.robot_ip}")
            return self.gripper

    def _run_gripper_command(self, command: str) -> None:
        try:
            gripper = self._get_or_connect_gripper()
            if command == "open":
                result = gripper.open(self.gripper_speed)
            else:
                close_width = max(self.gripper_close_width, 0.0)
                if self.gripper_close_use_grasp:
                    result = gripper.grasp(
                        close_width,
                        self.gripper_speed,
                        self.gripper_force,
                        self.gripper_epsilon_inner,
                        self.gripper_epsilon_outer,
                    )
                else:
                    result = gripper.move(close_width, self.gripper_speed)
            self._update_finger_width_from_gripper(gripper)
            with self.gripper_lock:
                self.last_gripper_result = "ok" if result else "failed"
            self.get_logger().info(f"Franky gripper command '{command}' result={result}")
        except Exception as exc:  # pylint: disable=broad-except
            with self.gripper_lock:
                self.last_gripper_result = "exception"
                self.last_gripper_exception = str(exc)
                self.gripper = None
            self.get_logger().error(f"Franky gripper command '{command}' failed: {exc}")

    def _update_finger_width_from_gripper(self, gripper: Any | None = None) -> float:
        if gripper is None:
            with self.gripper_lock:
                gripper = self.gripper
        if gripper is None:
            return self.finger_width
        try:
            total_width = float(gripper.width)
        except Exception:  # pylint: disable=broad-except
            return self.finger_width
        self.finger_width = max(total_width * 0.5, 0.0)
        return self.finger_width

    def control_loop(self) -> None:
        period = 1.0 / max(self.command_rate_hz, 1.0)
        last_time = time.monotonic()
        while self.running:
            now = time.monotonic()
            dt = max(now - last_time, 1e-3)
            last_time = now
            try:
                self._control_once(now, dt)
            except Exception as exc:  # pylint: disable=broad-except
                self._set_exception(exc)
            time.sleep(period)
        self._set_status(ControlStatus.STOPPED)

    def _control_once(self, now: float, dt: float) -> None:
        with self.target_lock:
            target = self.target
            enabled = self.teleop_enabled and self._seconds_since(self.enabled_stamp, now) <= self.enabled_timeout_sec

        with self.state_lock:
            current = self.current_state

        if self.robot is None:
            self._set_status(ControlStatus.CONNECTING)
            return

        if not current.valid:
            self._set_status(ControlStatus.WAITING_FOR_TARGET)
            return

        if now < self.recovery_hold_until:
            self.command_position = current.position.copy()
            self.command_orientation = current.orientation.copy()
            self._reset_command_dynamics()
            self.motion_active = False
            self._set_status(ControlStatus.WAITING_FOR_TARGET)
            return

        target_fresh = target.valid and self._seconds_since(target.stamp, now) <= self.target_timeout_sec
        if not enabled:
            self.command_position = current.position.copy()
            self.command_orientation = current.orientation.copy()
            self._reset_command_dynamics()
            self._stop_if_active()
            self._set_status(ControlStatus.DISABLED)
            return

        if not target_fresh:
            self.command_position = current.position.copy()
            self.command_orientation = current.orientation.copy()
            self._reset_command_dynamics()
            self._stop_if_active()
            self._set_status(ControlStatus.WAITING_FOR_TARGET)
            return

        target_position = np.clip(target.position, self.workspace_min, self.workspace_max)
        target_orientation = target.orientation

        if self.command_position is None or self.command_orientation is None or not self.motion_active:
            self.command_position = current.position.copy()
            self.command_orientation = current.orientation.copy()
            self._reset_command_dynamics()
            if not self._initial_target_is_close_enough(current, target_position, target_orientation):
                self._stop_if_active()
                self._set_status(ControlStatus.INITIAL_TARGET_TOO_FAR)
                return

        self.command_position, self.command_orientation = self._step_toward(
            current.position,
            current.orientation,
            target_position,
            target_orientation,
            dt,
        )

        with self.command_send_lock:
            with self.target_lock:
                still_enabled = (
                    self.teleop_enabled
                    and self._seconds_since(self.enabled_stamp, time.monotonic()) <= self.enabled_timeout_sec
                )
            if not still_enabled:
                self.command_position = current.position.copy()
                self.command_orientation = current.orientation.copy()
                self._reset_command_dynamics()
                self._stop_if_active(force=True)
                self._set_status(ControlStatus.DISABLED)
                return

            self.control_command_strategy.send(
                self,
                self.command_position,
                self.command_orientation,
            )
        self._set_status(ControlStatus.RUNNING)

    def publish_state_timer(self) -> None:
        have_state = self._read_current_state()
        with self.state_lock:
            state = self.current_state
        if have_state and state.valid:
            self._publish_current_pose(state)
            self._publish_joint_state(state)
        else:
            self._publish_fallback_joint_state()
        self._publish_debug(have_state)

    def _read_current_state(self) -> bool:
        if self.robot is None:
            self._try_connect_robot()
            return False
        try:
            with self.robot_lock:
                if self.robot is None:
                    return False
                cartesian_state = self.robot.current_cartesian_state
                joint_state = self.robot.current_joint_state
                robot_state = self.robot.state
            position, orientation = _affine_to_pose(cartesian_state.pose.end_effector_pose)
            q = _values(joint_state.position)[:7]
            dq = _values(joint_state.velocity)[:7]
            tau_j = _values(_attr(robot_state, "tau_J"))[:7]
            with self.state_lock:
                self.current_state = CurrentState(
                    valid=True,
                    position=position,
                    orientation=orientation,
                    q=q,
                    dq=dq,
                    tau_j=tau_j,
                    stamp=time.monotonic(),
                )
            return True
        except Exception as exc:  # pylint: disable=broad-except
            self._set_exception(exc)
            return False

    def _send_cartesian_motion(self, position: np.ndarray, orientation: np.ndarray) -> None:
        with self.robot_lock:
            if self.robot is None:
                raise RuntimeError("Franky robot is not connected")
            self.robot.relative_dynamics_factor = self.relative_dynamics_factor
            affine = self.franky.Affine(position.astype(float), orientation.astype(float))
            motion = self.franky.CartesianMotion(
                affine,
                self.franky.ReferenceType.Absolute,
                self.relative_dynamics_factor,
                True,
            )
            self.robot.move(motion, asynchronous=True)
            self.motion_active = True

    def _send_cartesian_velocity(self, linear_velocity: np.ndarray, angular_velocity: np.ndarray) -> None:
        with self.robot_lock:
            if self.robot is None:
                raise RuntimeError("Franky robot is not connected")
            self.robot.relative_dynamics_factor = self.relative_dynamics_factor
            twist = self.franky.Twist(
                np.array(linear_velocity, dtype=float),
                np.array(angular_velocity, dtype=float),
            )
            duration_ms = max(int(self.velocity_command_duration * 1000.0), 20)
            motion = self.franky.CartesianVelocityMotion(
                twist,
                self.franky.Duration(duration_ms),
                self.relative_dynamics_factor,
            )
            self.robot.move(motion, asynchronous=True)
            self.motion_active = True

    def _send_cartesian_pose_stop(self) -> None:
        with self.robot_lock:
            if self.robot is None:
                self.motion_active = False
                return
            motion = self.franky.CartesianStopMotion(self.stop_relative_dynamics_factor)
            self.robot.move(motion, asynchronous=True)

    def _send_cartesian_velocity_stop(self) -> None:
        with self.robot_lock:
            if self.robot is None:
                self.motion_active = False
                return
            motion = self.franky.CartesianVelocityStopMotion(self.stop_relative_dynamics_factor)
            self.robot.move(motion, asynchronous=True)

    def _send_stop_motion(self) -> None:
        try:
            self.control_command_strategy.stop(self)
        except Exception:  # pylint: disable=broad-except
            try:
                with self.robot_lock:
                    self.robot.stop()
            except Exception:  # pylint: disable=broad-except
                pass
        self.motion_active = False

    def _stop_if_active(self, force: bool = False) -> None:
        should_stop = force or (
            self.stop_on_disable
            and (self.motion_active or self.control_command_mode == "velocity")
        )
        if should_stop:
            self._send_stop_motion()
        else:
            self.motion_active = False
        self._reset_command_dynamics()

    def _reset_command_dynamics(self) -> None:
        self.command_linear_velocity = np.zeros(3, dtype=float)
        self.command_angular_velocity = np.zeros(3, dtype=float)

    def _initial_target_is_close_enough(
        self,
        current: CurrentState,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
    ) -> bool:
        distance = float(np.linalg.norm(target_position - current.position))
        if self.max_initial_target_distance > 0.0 and distance > self.max_initial_target_distance:
            return False
        angle = _quaternion_angle(current.orientation, target_orientation)
        return not (self.max_initial_target_angle > 0.0 and angle > self.max_initial_target_angle)

    def _step_toward(
        self,
        current_position: np.ndarray,
        current_orientation: np.ndarray,
        target_position: np.ndarray,
        target_orientation: np.ndarray,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        delta = target_position - current_position
        distance = float(np.linalg.norm(delta))
        linear_dt = max(dt, self.command_target_lookahead)
        if distance <= 1e-9 or linear_dt <= 1e-9:
            self.command_linear_velocity = np.zeros(3, dtype=float)
            next_position = current_position.copy()
        else:
            desired_linear_velocity = self._limit_vector_norm(
                delta / linear_dt,
                self.max_linear_velocity,
            )
            self.command_linear_velocity = self._limit_vector_change(
                desired_linear_velocity,
                self.command_linear_velocity,
                self.max_linear_acceleration * max(dt, 1e-3),
            )
            command_delta = self.command_linear_velocity * linear_dt
            command_delta_norm = float(np.linalg.norm(command_delta))
            alignment = float(np.dot(command_delta, delta)) / max(command_delta_norm * distance, 1e-12)
            if alignment <= 0.0:
                command_delta = np.zeros(3, dtype=float)
                self.command_linear_velocity = np.zeros(3, dtype=float)
            elif command_delta_norm > distance and alignment > 0.98:
                command_delta = delta
                self.command_linear_velocity = delta / linear_dt
            next_position = current_position + command_delta

        angular_delta = _quaternion_error_rotvec(current_orientation, target_orientation)
        angle = float(np.linalg.norm(angular_delta))
        angular_dt = max(dt, self.command_target_lookahead)
        if angle <= 1e-9 or angular_dt <= 1e-9:
            self.command_angular_velocity = np.zeros(3, dtype=float)
            next_orientation = current_orientation.copy()
        else:
            desired_angular_velocity = self._limit_vector_norm(
                angular_delta / angular_dt,
                self.max_angular_velocity,
            )
            self.command_angular_velocity = self._limit_vector_change(
                desired_angular_velocity,
                self.command_angular_velocity,
                self.max_angular_acceleration * max(dt, 1e-3),
            )
            command_rotvec = self.command_angular_velocity * angular_dt
            command_angle = float(np.linalg.norm(command_rotvec))
            alignment = float(np.dot(command_rotvec, angular_delta)) / max(command_angle * angle, 1e-12)
            if alignment <= 0.0:
                command_rotvec = np.zeros(3, dtype=float)
                self.command_angular_velocity = np.zeros(3, dtype=float)
            elif command_angle > angle and alignment > 0.98:
                command_rotvec = angular_delta
                self.command_angular_velocity = angular_delta / angular_dt
            next_orientation = _quaternion_multiply(
                _quaternion_from_rotvec(command_rotvec),
                current_orientation,
            )
        return next_position, next_orientation

    @staticmethod
    def _limit_vector_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
        if maximum_norm <= 0.0:
            return vector
        norm = float(np.linalg.norm(vector))
        if norm <= maximum_norm or norm < 1e-12:
            return vector
        return vector * (maximum_norm / norm)

    def _limit_vector_change(
        self,
        desired: np.ndarray,
        current: np.ndarray,
        maximum_change: float,
    ) -> np.ndarray:
        if maximum_change <= 0.0:
            return desired
        return current + self._limit_vector_norm(desired - current, maximum_change)

    def _publish_current_pose(self, state: CurrentState) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(state.position[0])
        msg.pose.position.y = float(state.position[1])
        msg.pose.position.z = float(state.position[2])
        msg.pose.orientation.x = float(state.orientation[0])
        msg.pose.orientation.y = float(state.orientation[1])
        msg.pose.orientation.z = float(state.orientation[2])
        msg.pose.orientation.w = float(state.orientation[3])
        self.current_pose_pub.publish(msg)

    def _publish_joint_state(self, state: CurrentState) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.name = self.joint_names + self.finger_joint_names
        arm_q = self._pad(state.q, 7)
        arm_dq = self._pad(state.dq, 7)
        arm_tau = self._pad(state.tau_j, 7)
        finger_width = self._update_finger_width_from_gripper()
        msg.position = arm_q + [finger_width for _ in self.finger_joint_names]
        msg.velocity = arm_dq + [0.0 for _ in self.finger_joint_names]
        msg.effort = arm_tau + [0.0 for _ in self.finger_joint_names]
        self.joint_state_pub.publish(msg)

    def _publish_fallback_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.name = self.joint_names + self.finger_joint_names
        finger_width = self._update_finger_width_from_gripper()
        msg.position = [float(value) for value in self.fallback_joint_positions] + [
            finger_width for _ in self.finger_joint_names
        ]
        msg.velocity = [0.0 for _ in msg.name]
        msg.effort = [0.0 for _ in msg.name]
        self.joint_state_pub.publish(msg)

    def _publish_debug(self, have_current_state: bool) -> None:
        with self.gripper_lock:
            gripper_payload = {
                "enabled": self.enable_gripper,
                "command_topic": self.gripper_command_topic,
                "last_command": self.last_gripper_command,
                "last_result": self.last_gripper_result,
                "busy": self.gripper_thread is not None and self.gripper_thread.is_alive(),
                "finger_joint_width_m": self.finger_width,
                "close_width_m": self.gripper_close_width,
                "speed_mps": self.gripper_speed,
                "close_use_grasp": self.gripper_close_use_grasp,
            }
            if self.last_gripper_exception:
                gripper_payload["exception"] = self.last_gripper_exception
        with self.status_lock:
            payload = {
                "status": self.status.value,
                "backend": "franky",
                "robot_ip": self.robot_ip,
                "have_current_state": have_current_state,
                "target_topic": self.target_pose_topic,
                "enabled_topic": self.enabled_topic,
                "joint_state_topic": self.joint_state_topic,
                "base_frame": self.base_frame,
                "command_rate_hz": self.command_rate_hz,
                "control_command_mode": self.control_command_mode,
                "velocity_command_duration_sec": self.velocity_command_duration,
                "command_target_lookahead_sec": self.command_target_lookahead,
                "max_linear_velocity_mps": self.max_linear_velocity,
                "max_angular_velocity_radps": self.max_angular_velocity,
                "max_linear_acceleration_mps2": self.max_linear_acceleration,
                "max_angular_acceleration_radps2": self.max_angular_acceleration,
                "command_linear_velocity_mps": self.command_linear_velocity.tolist(),
                "command_angular_velocity_radps": self.command_angular_velocity.tolist(),
                "relative_dynamics_factor": self.relative_dynamics_factor,
                "stop_relative_dynamics_factor": self.stop_relative_dynamics_factor,
                "automatic_error_recovery": self.automatic_error_recovery,
                "error_recovery_cooldown_sec": self.error_recovery_cooldown_sec,
                "post_error_recovery_hold_sec": self.post_error_recovery_hold_sec,
                "last_error_recovery_result": self.last_error_recovery_result,
                "recovery_hold_remaining_sec": max(self.recovery_hold_until - time.monotonic(), 0.0),
                "gripper": gripper_payload,
            }
            if self.last_exception:
                payload["exception"] = self.last_exception
        msg = String()
        msg.data = json.dumps(payload)
        self.debug_pub.publish(msg)

    @staticmethod
    def _seconds_since(stamp: float, now: float) -> float:
        if stamp <= 0.0:
            return math.inf
        return now - stamp

    @staticmethod
    def _pad(values: list[float], size: int) -> list[float]:
        result = [float(value) for value in values[:size]]
        return result + [0.0] * max(size - len(result), 0)

    def _set_status(self, status: ControlStatus) -> None:
        with self.status_lock:
            self.status = status

    def _set_exception(self, exc: Exception) -> None:
        with self.status_lock:
            self.status = ControlStatus.EXCEPTION
            self.last_exception = str(exc)
        self.get_logger().error(f"Franky control failed: {exc}")
        self._start_recovery_hold()
        if self.automatic_error_recovery:
            self._attempt_error_recovery(exc)

    def _start_recovery_hold(self) -> None:
        if self.post_error_recovery_hold_sec <= 0.0:
            return
        self.recovery_hold_until = max(
            self.recovery_hold_until,
            time.monotonic() + self.post_error_recovery_hold_sec,
        )

    def _attempt_error_recovery(self, exc: Exception) -> None:
        now = time.monotonic()
        if now - self.last_error_recovery_attempt < self.error_recovery_cooldown_sec:
            return
        self.last_error_recovery_attempt = now

        with self.robot_lock:
            robot = self.robot
            if robot is None:
                self.last_error_recovery_result = "skipped_no_robot"
                return

            self.last_error_recovery_result = "running"
            try:
                robot.recover_from_errors()
                robot.relative_dynamics_factor = self.relative_dynamics_factor
            except Exception as recovery_exc:  # pylint: disable=broad-except
                self.robot = None
                self.motion_active = False
                self.command_position = None
                self.command_orientation = None
                self._reset_command_dynamics()
                self._start_recovery_hold()
                self.last_error_recovery_result = f"failed: {recovery_exc}"
                self.get_logger().error(
                    "Franky automatic error recovery failed; will reconnect: "
                    f"{recovery_exc}"
                )
                return

        with self.state_lock:
            current = self.current_state
        if current.valid:
            self.command_position = current.position.copy()
            self.command_orientation = current.orientation.copy()
        else:
            self.command_position = None
            self.command_orientation = None
        with self.target_lock:
            self.target = Target()
        self.motion_active = False
        self._reset_command_dynamics()
        self.last_exception = ""
        self.last_error_recovery_result = "ok"
        self._start_recovery_hold()
        self._set_status(ControlStatus.WAITING_FOR_TARGET)
        self.get_logger().warn(
            "Franky automatic error recovery completed after control exception: "
            f"{exc}"
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = FrankyCartesianPoseNode()
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
