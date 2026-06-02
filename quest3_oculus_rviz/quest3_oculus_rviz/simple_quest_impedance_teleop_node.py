import json
import math
import time
from typing import Any

import numpy as np
import rclpy
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf_transformations import quaternion_from_matrix, quaternion_matrix


class SimpleQuestImpedanceTeleopNode(Node):
    """Minimal Meta Quest -> Franka impedance teleop bridge."""

    def __init__(self) -> None:
        super().__init__("simple_quest_impedance_teleop")

        self.declare_parameter("ip_address", "")
        self.declare_parameter("port", 5555)
        self.declare_parameter("input_mode", "reader")
        self.declare_parameter("controller_key", "r")
        self.declare_parameter("controller_pose_topic", "/quest3/right_controller/pose")
        self.declare_parameter("buttons_topic", "/quest3/buttons")
        self.declare_parameter("topic_pose_timeout_sec", 0.25)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter(
            "current_pose_topic",
            "/franka_robot_state_broadcaster/current_pose",
        )
        self.declare_parameter(
            "target_pose_topic",
            "/cartesian_impedance_controller/equilibrium_pose",
        )
        self.declare_parameter(
            "franka_state_topic",
            "/franka_robot_state_broadcaster/robot_state",
        )
        self.declare_parameter(
            "recovery_status_topic",
            "/franka_error_recovery_watchdog/recovering",
        )
        self.declare_parameter(
            "raw_pose_topic",
            "/quest3/right_controller/raw_pose",
        )
        self.declare_parameter(
            "enabled_topic",
            "/quest3/simple_teleop/enabled",
        )
        self.declare_parameter(
            "delta_topic",
            "/quest3/simple_teleop/delta",
        )
        self.declare_parameter(
            "debug_topic",
            "/quest3/simple_teleop/debug",
        )
        self.declare_parameter("enable_button_name", "")
        self.declare_parameter("enable_analog_name", "rightGrip")
        self.declare_parameter("enable_threshold", 0.5)
        self.declare_parameter("translation_scale", 1.0)
        self.declare_parameter("rotation_scale", 1.0)
        self.declare_parameter("translation_deadband_m", 0.0005)
        self.declare_parameter("rotation_deadband_rad", 0.003)
        self.declare_parameter("max_translation_step_m", 0.02)
        self.declare_parameter("max_rotation_step_rad", 0.12)
        self.declare_parameter("quest_pose_grace_period_sec", 0.25)
        self.declare_parameter("reader_restart_timeout_sec", 2.0)
        self.declare_parameter("post_recovery_hold_sec", 1.0)
        self.declare_parameter("sync_target_with_current_pose_when_idle", True)
        self.declare_parameter("workspace_min", [0.20, -0.45, 0.08])
        self.declare_parameter("workspace_max", [0.80, 0.45, 0.75])
        self.declare_parameter("translation_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("rotation_sign", [1.0, 1.0, 1.0])
        self.declare_parameter("rotation_calibration_logging", False)
        self.declare_parameter("rotation_calibration_log_interval_sec", 0.5)
        self.declare_parameter("rotation_calibration_min_norm_rad", 0.01)
        self.declare_parameter(
            "translation_quest_to_robot_rotation",
            [
                0.0, 0.0, 1.0,
                -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
            ],
        )
        self.declare_parameter(
            "quest_to_robot_rotation",
            [
                0.0, 0.0, 1.0,
                -1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
            ],
        )

        self.ip_address = str(self.get_parameter("ip_address").value).strip()
        self.port = int(self.get_parameter("port").value)
        self.input_mode = str(self.get_parameter("input_mode").value).strip().lower()
        self.controller_key = str(self.get_parameter("controller_key").value).strip().lower()
        self.controller_pose_topic = str(self.get_parameter("controller_pose_topic").value)
        self.buttons_topic = str(self.get_parameter("buttons_topic").value)
        self.topic_pose_timeout = max(
            float(self.get_parameter("topic_pose_timeout_sec").value),
            0.0,
        )
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.current_pose_topic = str(self.get_parameter("current_pose_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.franka_state_topic = str(self.get_parameter("franka_state_topic").value)
        self.recovery_status_topic = str(self.get_parameter("recovery_status_topic").value)
        self.raw_pose_topic = str(self.get_parameter("raw_pose_topic").value)
        self.enabled_topic = str(self.get_parameter("enabled_topic").value)
        self.delta_topic = str(self.get_parameter("delta_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.enable_button_name = str(self.get_parameter("enable_button_name").value).strip()
        self.enable_analog_name = str(self.get_parameter("enable_analog_name").value).strip()
        self.enable_threshold = float(self.get_parameter("enable_threshold").value)
        self.translation_scale = float(self.get_parameter("translation_scale").value)
        self.rotation_scale = float(self.get_parameter("rotation_scale").value)
        self.translation_deadband = max(float(self.get_parameter("translation_deadband_m").value), 0.0)
        self.rotation_deadband = max(float(self.get_parameter("rotation_deadband_rad").value), 0.0)
        self.max_translation_step = max(float(self.get_parameter("max_translation_step_m").value), 0.0)
        self.max_rotation_step = max(float(self.get_parameter("max_rotation_step_rad").value), 0.0)
        self.quest_pose_grace_period = max(
            float(self.get_parameter("quest_pose_grace_period_sec").value),
            0.0,
        )
        self.reader_restart_timeout = max(
            float(self.get_parameter("reader_restart_timeout_sec").value),
            0.0,
        )
        self.post_recovery_hold_sec = max(
            float(self.get_parameter("post_recovery_hold_sec").value),
            0.0,
        )
        self.sync_target_when_idle = bool(
            self.get_parameter("sync_target_with_current_pose_when_idle").value
        )
        self.workspace_min = self._load_vector3_parameter("workspace_min")
        self.workspace_max = self._load_vector3_parameter("workspace_max")
        self.translation_sign = self._load_vector3_parameter("translation_sign")
        self.rotation_sign = self._load_vector3_parameter("rotation_sign")
        (
            self.translation_quest_to_robot_rotation,
            _,
        ) = self._load_axis_map_parameter(
            "translation_quest_to_robot_rotation",
            warn_left_handed=False,
        )
        self.rotation_calibration_logging = bool(
            self.get_parameter("rotation_calibration_logging").value
        )
        self.rotation_calibration_log_interval = max(
            float(self.get_parameter("rotation_calibration_log_interval_sec").value),
            0.05,
        )
        self.rotation_calibration_min_norm = max(
            float(self.get_parameter("rotation_calibration_min_norm_rad").value),
            0.0,
        )
        (
            self.quest_to_robot_rotation,
            self.quest_to_robot_handedness,
        ) = self._load_axis_map_parameter("quest_to_robot_rotation")
        if self.input_mode not in ("reader", "topics"):
            raise ValueError("input_mode must be 'reader' or 'topics'")
        if self.controller_key not in ("r", "l"):
            raise ValueError("controller_key must be 'r' or 'l'")

        self.current_position = np.zeros(3, dtype=float)
        self.current_rotation = np.eye(3, dtype=float)
        self.target_position = np.zeros(3, dtype=float)
        self.target_rotation = np.eye(3, dtype=float)
        self.have_current_pose = False
        self.franka_in_error = False
        self.recovery_active = False
        self.recovery_hold_until = 0.0
        self.idle_target_synced = False
        self.last_franka_error_signature = ""
        self.teleop_enabled = False
        self.previous_hand_position: np.ndarray | None = None
        self.previous_hand_rotation: np.ndarray | None = None

        self.last_valid_transforms: dict[str, np.ndarray] = {}
        self.last_valid_buttons: dict[str, Any] = {}
        self.last_quest_pose_time: float | None = None
        self.last_raw_transform_keys: list[str] = []
        self.last_raw_button_keys: list[str] = []
        self.using_cached_quest_pose = False
        self.reader_restart_count = 0
        self.last_reader_restart_time = 0.0
        self.reader_start_time = time.monotonic()
        self.retired_readers: list[Any] = []
        self.topic_hand_position: np.ndarray | None = None
        self.topic_hand_rotation: np.ndarray | None = None
        self.topic_buttons: dict[str, Any] = {}
        self.last_topic_pose_time: float | None = None

        self.last_missing_quest_log = 0.0
        self.last_missing_robot_pose_log = 0.0
        self.logged_first_quest_pose = False
        self.warned_frame_mismatch = False
        self.rotation_calibration_window_start = time.monotonic()
        self.rotation_calibration_sample_count = 0
        self.rotation_calibration_sum_quest = np.zeros(3, dtype=float)
        self.rotation_calibration_sum_mapped = np.zeros(3, dtype=float)
        self.rotation_calibration_sum_signed = np.zeros(3, dtype=float)
        self.rotation_calibration_sum_command = np.zeros(3, dtype=float)

        current_pose_qos = QoSProfile(depth=10)
        current_pose_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            self.current_pose_topic,
            self.current_pose_callback,
            current_pose_qos,
        )
        self.franka_state_sub = self.create_subscription(
            FrankaRobotState,
            self.franka_state_topic,
            self.franka_state_callback,
            current_pose_qos,
        )
        self.recovery_status_sub = self.create_subscription(
            Bool,
            self.recovery_status_topic,
            self.recovery_status_callback,
            10,
        )
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.raw_pose_pub = self.create_publisher(PoseStamped, self.raw_pose_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, self.enabled_topic, 10)
        self.delta_pub = self.create_publisher(TwistStamped, self.delta_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        self.reader = None
        if self.input_mode == "reader":
            from oculus_reader.reader import OculusReader

            self.OculusReader = OculusReader
            self.reader_ip = self.ip_address if self.ip_address else None
            self.reader = self._create_reader()
        else:
            self.pose_sub = self.create_subscription(
                PoseStamped,
                self.controller_pose_topic,
                self.controller_pose_callback,
                10,
            )
            self.buttons_sub = self.create_subscription(
                String,
                self.buttons_topic,
                self.buttons_callback,
                10,
            )
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        if self.input_mode == "reader":
            mode = f"network {self.reader_ip}:{self.port}" if self.reader_ip else "USB"
        else:
            mode = f"topics pose={self.controller_pose_topic}, buttons={self.buttons_topic}"
        self.get_logger().info(
            "Simple Quest impedance teleop ready. "
            f"Quest={mode}, controller={self.controller_key}, "
            f"current_pose={self.current_pose_topic}, target={self.target_pose_topic}, "
            f"recovery_status={self.recovery_status_topic}, "
            f"rotation_calibration_logging={self.rotation_calibration_logging}"
        )

    def destroy_node(self) -> bool:
        if getattr(self, "reader", None) is not None:
            self._stop_reader(self.reader)
        for reader in getattr(self, "retired_readers", []):
            self._stop_reader(reader)
        return super().destroy_node()

    def current_pose_callback(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self.base_frame and not self.warned_frame_mismatch:
            self.get_logger().warn(
                "Current pose frame does not match base_frame: "
                f"pose='{msg.header.frame_id}', base_frame='{self.base_frame}'. "
                "Using the numeric pose values directly."
            )
            self.warned_frame_mismatch = True

        self.current_position = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=float,
        )
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self.current_rotation = quaternion_matrix(quat)[:3, :3]
        self.have_current_pose = True

    def franka_state_callback(self, msg: FrankaRobotState) -> None:
        current_errors = self._active_error_names(msg.current_errors)
        in_error = bool(current_errors) or msg.robot_mode == FrankaRobotState.ROBOT_MODE_REFLEX
        if in_error:
            signature = (
                f"robot_mode={self._robot_mode_name(msg.robot_mode)}, "
                f"current_errors={current_errors}, "
                f"last_motion_errors={self._active_error_names(msg.last_motion_errors)}"
            )
            if signature != self.last_franka_error_signature:
                self.get_logger().warn(f"Franka is in error; pausing teleop target updates: {signature}")
                self.last_franka_error_signature = signature
            self._set_idle_target(force_sync=not self.franka_in_error)
        elif self.franka_in_error:
            self.get_logger().info("Franka error state cleared; teleop will re-anchor on next grip input.")
            self.last_franka_error_signature = ""
            self._set_idle_target(force_sync=True)
        self.franka_in_error = in_error

    def recovery_status_callback(self, msg: Bool) -> None:
        if msg.data:
            if not self.recovery_active:
                self.get_logger().warn(
                    "Franka recovery watchdog is active; pausing teleop target updates."
                )
                self._set_idle_target(force_sync=True)
            else:
                self._set_idle_target()
            self.recovery_active = True
            return

        if self.recovery_active:
            self.get_logger().info(
                "Franka recovery watchdog finished; teleop will re-anchor on next grip input."
            )
            self._start_recovery_hold()
            self._set_idle_target(force_sync=True)
        self.recovery_active = False

    def controller_pose_callback(self, msg: PoseStamped) -> None:
        self.topic_hand_position = np.array(
            [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ],
            dtype=float,
        )
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self.topic_hand_rotation = self._orthonormalize(quaternion_matrix(quat)[:3, :3])
        self.last_topic_pose_time = time.monotonic()

    def buttons_callback(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Failed to parse Quest buttons JSON: {exc}")
            return
        if isinstance(parsed, dict):
            self.topic_buttons = parsed

    def timer_callback(self) -> None:
        stamp = self.get_clock().now().to_msg()
        now = time.monotonic()

        raw_transforms, raw_buttons = self._read_raw_quest_data(now)
        raw_transforms = raw_transforms or {}
        raw_buttons = raw_buttons or {}
        self.last_raw_transform_keys = sorted(raw_transforms.keys())
        self.last_raw_button_keys = sorted(raw_buttons.keys())
        self.using_cached_quest_pose = False

        if self.controller_key in raw_transforms:
            self.last_valid_transforms = dict(raw_transforms)
            self.last_valid_buttons = dict(raw_buttons)
            self.last_quest_pose_time = now
            transforms = raw_transforms
            buttons = raw_buttons
        else:
            pose_age = self._quest_pose_age(now)
            use_cached_pose = (
                self.controller_key in self.last_valid_transforms
                and pose_age is not None
                and pose_age <= self.quest_pose_grace_period
            )
            if use_cached_pose:
                transforms = self.last_valid_transforms
                buttons = self.last_valid_buttons
                self.using_cached_quest_pose = True
            else:
                transforms = raw_transforms
                buttons = raw_buttons

        if self.controller_key not in transforms:
            self._publish_enabled(False)
            self._publish_zero_delta(stamp)
            self._restart_reader_if_stale(now)
            self._log_missing_quest(now)
            idle_synced = self._set_idle_target()
            if self.have_current_pose:
                self._publish_target_pose(stamp)
            self._publish_debug(
                enabled=False,
                buttons=buttons,
                delta_world=np.zeros(3, dtype=float),
                delta_robot=np.zeros(3, dtype=float),
                delta_rotvec_quest=np.zeros(3, dtype=float),
                delta_rotvec_robot=np.zeros(3, dtype=float),
                note=(
                    f"no_{self.controller_key}_controller_pose_anchor_captured"
                    if idle_synced
                    else f"no_{self.controller_key}_controller_pose_idle_hold"
                ),
            )
            return

        if not self.logged_first_quest_pose:
            self.get_logger().info(
                f"Received {self.controller_key} controller pose from Quest. "
                f"transform_keys={sorted(transforms.keys())}, "
                f"button_keys={sorted(buttons.keys())}"
            )
            self.logged_first_quest_pose = True

        controller_transform = np.asarray(transforms[self.controller_key], dtype=float)
        raw_position = controller_transform[:3, 3].copy()
        raw_rotation = self._orthonormalize(controller_transform[:3, :3])
        self._publish_raw_pose(stamp, raw_position, raw_rotation)

        enabled = self._is_enabled(buttons)
        recovery_hold_active = now < self.recovery_hold_until
        if self.franka_in_error or self.recovery_active or recovery_hold_active:
            self._publish_enabled(False)
            self._set_idle_target()
            self._publish_zero_delta(stamp)
            self._publish_debug(
                enabled=False,
                buttons=buttons,
                delta_world=np.zeros(3, dtype=float),
                delta_robot=np.zeros(3, dtype=float),
                delta_rotvec_quest=np.zeros(3, dtype=float),
                delta_rotvec_robot=np.zeros(3, dtype=float),
                note=self._recovery_hold_note(now),
            )
            return

        self._publish_enabled(enabled)

        if not self.have_current_pose:
            self.teleop_enabled = False
            self.previous_hand_position = None
            self.previous_hand_rotation = None
            self._publish_zero_delta(stamp)
            self._log_missing_robot_pose(now)
            self._publish_debug(
                enabled=False,
                buttons=buttons,
                delta_world=np.zeros(3, dtype=float),
                delta_robot=np.zeros(3, dtype=float),
                delta_rotvec_quest=np.zeros(3, dtype=float),
                delta_rotvec_robot=np.zeros(3, dtype=float),
                note="waiting_robot_pose",
            )
            return

        if not enabled:
            idle_synced = self._set_idle_target()
            self._publish_target_pose(stamp)
            self._publish_zero_delta(stamp)
            self._publish_debug(
                enabled=False,
                buttons=buttons,
                delta_world=np.zeros(3, dtype=float),
                delta_robot=np.zeros(3, dtype=float),
                delta_rotvec_quest=np.zeros(3, dtype=float),
                delta_rotvec_robot=np.zeros(3, dtype=float),
                note="idle_anchor_captured" if idle_synced else "idle_hold",
            )
            return

        if (
            not self.teleop_enabled
            or self.previous_hand_position is None
            or self.previous_hand_rotation is None
        ):
            self.teleop_enabled = True
            self.idle_target_synced = False
            self.previous_hand_position = raw_position.copy()
            self.previous_hand_rotation = raw_rotation.copy()
            self.target_position = self.current_position.copy()
            self.target_rotation = self.current_rotation.copy()
            self._reset_rotation_calibration_accumulators(now)
            self._log_rotation_calibration_anchor()
            self._publish_target_pose(stamp)
            self._publish_zero_delta(stamp)
            self._publish_debug(
                enabled=True,
                buttons=buttons,
                delta_world=np.zeros(3, dtype=float),
                delta_robot=np.zeros(3, dtype=float),
                delta_rotvec_quest=np.zeros(3, dtype=float),
                delta_rotvec_robot=np.zeros(3, dtype=float),
                note="anchor_captured",
            )
            return

        delta_world = raw_position - self.previous_hand_position
        delta_robot = self.translation_quest_to_robot_rotation @ delta_world
        delta_robot = delta_robot * self.translation_sign
        delta_robot = self._apply_deadband(delta_robot, self.translation_deadband)
        delta_robot = self._limit_norm(delta_robot * self.translation_scale, self.max_translation_step)

        delta_rotation = self.previous_hand_rotation.T @ raw_rotation
        delta_rotvec_quest = self._rotation_vector_from_matrix(delta_rotation)
        # Rotation vectors are axial vectors. If the configured axis map is
        # left-handed (det=-1), we need one extra sign flip after the map.
        delta_rotvec_robot_mapped = (
            self.quest_to_robot_handedness
            * (self.quest_to_robot_rotation @ delta_rotvec_quest)
        )
        delta_rotvec_robot_signed = delta_rotvec_robot_mapped * self.rotation_sign
        delta_rotvec_robot = self._apply_deadband(delta_rotvec_robot_signed, self.rotation_deadband)
        delta_rotvec_robot = self._limit_norm(
            delta_rotvec_robot * self.rotation_scale,
            self.max_rotation_step,
        )

        self.target_position = self._clamp_position(self.target_position + delta_robot)
        self.target_rotation = self._orthonormalize(
            self._rotation_matrix_from_rotvec(delta_rotvec_robot) @ self.target_rotation
        )
        self.previous_hand_position = raw_position.copy()
        self.previous_hand_rotation = raw_rotation.copy()

        self._update_rotation_calibration_log(
            now,
            delta_rotvec_quest,
            delta_rotvec_robot_mapped,
            delta_rotvec_robot_signed,
            delta_rotvec_robot,
        )

        self._publish_target_pose(stamp)
        self._publish_delta(stamp, delta_robot, delta_rotvec_robot)
        self._publish_debug(
            enabled=True,
            buttons=buttons,
            delta_world=delta_world,
            delta_robot=delta_robot,
            delta_rotvec_quest=delta_rotvec_quest,
            delta_rotvec_robot=delta_rotvec_robot,
            note="running",
        )

    def _set_idle_target(self, *, force_sync: bool = False) -> bool:
        was_teleop_enabled = self.teleop_enabled
        self.teleop_enabled = False
        self.previous_hand_position = None
        self.previous_hand_rotation = None
        self._reset_rotation_calibration_accumulators(time.monotonic())
        should_sync = (
            self.sync_target_when_idle
            and self.have_current_pose
            and (force_sync or was_teleop_enabled or not self.idle_target_synced)
        )
        if should_sync:
            self.target_position = self.current_position.copy()
            self.target_rotation = self.current_rotation.copy()
            self.idle_target_synced = True
            return True
        if not self.have_current_pose:
            self.idle_target_synced = False
        return False

    def _start_recovery_hold(self) -> None:
        if self.post_recovery_hold_sec <= 0.0:
            return
        self.recovery_hold_until = max(
            self.recovery_hold_until,
            time.monotonic() + self.post_recovery_hold_sec,
        )

    def _recovery_hold_note(self, now: float) -> str:
        if self.recovery_active:
            return "franka_recovery_hold"
        if now < self.recovery_hold_until:
            return "post_recovery_hold"
        return "franka_error_hold"

    def _is_enabled(self, buttons: dict[str, Any]) -> bool:
        if self.enable_analog_name:
            if self._button_scalar(buttons.get(self.enable_analog_name, 0.0)) >= self.enable_threshold:
                return True
        if self.enable_button_name:
            return self._button_bool(buttons.get(self.enable_button_name, False))
        return False

    def _publish_target_pose(self, stamp: Any) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(self.target_position[0])
        pose.pose.position.y = float(self.target_position[1])
        pose.pose.position.z = float(self.target_position[2])

        transform = np.eye(4)
        transform[:3, :3] = self.target_rotation
        quat = quaternion_from_matrix(transform)
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        self.target_pose_pub.publish(pose)

    def _publish_raw_pose(self, stamp: Any, position: np.ndarray, rotation: np.ndarray) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "quest_raw"
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])

        transform = np.eye(4)
        transform[:3, :3] = rotation
        quat = quaternion_from_matrix(transform)
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        self.raw_pose_pub.publish(pose)

    def _publish_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self.enabled_pub.publish(msg)

    def _publish_zero_delta(self, stamp: Any) -> None:
        self._publish_delta(stamp, np.zeros(3, dtype=float), np.zeros(3, dtype=float))

    def _publish_delta(
        self,
        stamp: Any,
        delta_position: np.ndarray,
        delta_rotvec: np.ndarray,
    ) -> None:
        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = float(delta_position[0])
        twist.twist.linear.y = float(delta_position[1])
        twist.twist.linear.z = float(delta_position[2])
        twist.twist.angular.x = float(delta_rotvec[0])
        twist.twist.angular.y = float(delta_rotvec[1])
        twist.twist.angular.z = float(delta_rotvec[2])
        self.delta_pub.publish(twist)

    def _publish_debug(
        self,
        *,
        enabled: bool,
        buttons: dict[str, Any],
        delta_world: np.ndarray,
        delta_robot: np.ndarray,
        delta_rotvec_quest: np.ndarray,
        delta_rotvec_robot: np.ndarray,
        note: str,
    ) -> None:
        debug = {
            "enabled": enabled,
            "note": note,
            "enable_button_name": self.enable_button_name,
            "enable_analog_name": self.enable_analog_name,
            "enable_threshold": self.enable_threshold,
            "translation_scale": self.translation_scale,
            "rotation_scale": self.rotation_scale,
            "translation_sign": self.translation_sign.tolist(),
            "rotation_sign": self.rotation_sign.tolist(),
            "translation_quest_to_robot_rotation": self.translation_quest_to_robot_rotation.reshape(-1).tolist(),
            "quest_to_robot_rotation": self.quest_to_robot_rotation.reshape(-1).tolist(),
            "quest_reader": self._reader_diagnostics(time.monotonic()),
            "franka_in_error": self.franka_in_error,
            "recovery_active": self.recovery_active,
            "idle_target_synced": self.idle_target_synced,
            "delta_world_m": delta_world.tolist(),
            "delta_robot_m": delta_robot.tolist(),
            "delta_rotvec_quest_rad": delta_rotvec_quest.tolist(),
            "delta_rotvec_robot_rad": delta_rotvec_robot.tolist(),
            "current_pose": {
                "position": self.current_position.tolist(),
            },
            "target_pose": {
                "position": self.target_position.tolist(),
            },
            "buttons": self._to_jsonable(buttons),
        }
        msg = String()
        msg.data = json.dumps(debug, sort_keys=True)
        self.debug_pub.publish(msg)

    def _reset_rotation_calibration_accumulators(self, now: float) -> None:
        self.rotation_calibration_window_start = now
        self.rotation_calibration_sample_count = 0
        self.rotation_calibration_sum_quest = np.zeros(3, dtype=float)
        self.rotation_calibration_sum_mapped = np.zeros(3, dtype=float)
        self.rotation_calibration_sum_signed = np.zeros(3, dtype=float)
        self.rotation_calibration_sum_command = np.zeros(3, dtype=float)

    def _log_rotation_calibration_anchor(self) -> None:
        if not self.rotation_calibration_logging:
            return
        self.get_logger().info(
            "ROT_CAL anchor "
            f"controller={self.controller_key} "
            f"rotation_scale={self.rotation_scale:.4f} "
            f"rotation_sign={self._fmt_vector3(self.rotation_sign)} "
            f"handedness={self.quest_to_robot_handedness:+.0f} "
            f"quest_to_robot_rotation={self.quest_to_robot_rotation.reshape(-1).tolist()}"
        )

    def _update_rotation_calibration_log(
        self,
        now: float,
        delta_rotvec_quest: np.ndarray,
        delta_rotvec_robot_mapped: np.ndarray,
        delta_rotvec_robot_signed: np.ndarray,
        delta_rotvec_robot_command: np.ndarray,
    ) -> None:
        if not self.rotation_calibration_logging:
            return

        self.rotation_calibration_sum_quest += delta_rotvec_quest
        self.rotation_calibration_sum_mapped += delta_rotvec_robot_mapped
        self.rotation_calibration_sum_signed += delta_rotvec_robot_signed
        self.rotation_calibration_sum_command += delta_rotvec_robot_command
        self.rotation_calibration_sample_count += 1

        elapsed = max(now - self.rotation_calibration_window_start, 0.0)
        if elapsed < self.rotation_calibration_log_interval:
            return

        quest_norm = float(np.linalg.norm(self.rotation_calibration_sum_quest))
        command_norm = float(np.linalg.norm(self.rotation_calibration_sum_command))
        if quest_norm < self.rotation_calibration_min_norm and command_norm < self.rotation_calibration_min_norm:
            self._reset_rotation_calibration_accumulators(now)
            return

        current_rpy = self._matrix_to_rpy_xyz(self.current_rotation)
        target_rpy = self._matrix_to_rpy_xyz(self.target_rotation)
        self.get_logger().info(
            "ROT_CAL window "
            f"controller={self.controller_key} "
            f"duration_sec={elapsed:.3f} "
            f"samples={self.rotation_calibration_sample_count} "
            f"quest_sum_rad={self._fmt_vector3(self.rotation_calibration_sum_quest)} "
            f"mapped_no_sign_rad={self._fmt_vector3(self.rotation_calibration_sum_mapped)} "
            f"signed_no_scale_rad={self._fmt_vector3(self.rotation_calibration_sum_signed)} "
            f"tcp_command_sum_rad={self._fmt_vector3(self.rotation_calibration_sum_command)} "
            f"dominant_quest={self._dominant_axis(self.rotation_calibration_sum_quest)} "
            f"dominant_tcp_command={self._dominant_axis(self.rotation_calibration_sum_command)} "
            f"current_tcp_rpy_rad={self._fmt_vector3(current_rpy)} "
            f"target_tcp_rpy_rad={self._fmt_vector3(target_rpy)}"
        )
        self._reset_rotation_calibration_accumulators(now)

    def _log_missing_quest(self, now: float) -> None:
        if now - self.last_missing_quest_log > 3.0:
            self.get_logger().warn(
                f"No {self.controller_key} controller pose received from Quest yet. "
                f"diagnostics={self._reader_diagnostics(now)}"
            )
            self.last_missing_quest_log = now

    def _log_missing_robot_pose(self, now: float) -> None:
        if now - self.last_missing_robot_pose_log > 3.0:
            self.get_logger().warn(
                "No Franka current pose received yet; waiting before sending impedance targets."
            )
            self.last_missing_robot_pose_log = now

    def _read_raw_quest_data(self, now: float) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        if self.input_mode == "reader":
            return self.reader.get_transformations_and_buttons()

        transforms: dict[str, np.ndarray] = {}
        pose_is_fresh = (
            self.topic_hand_position is not None
            and self.topic_hand_rotation is not None
            and self.last_topic_pose_time is not None
            and now - self.last_topic_pose_time <= self.topic_pose_timeout
        )
        if pose_is_fresh:
            transform = np.eye(4, dtype=float)
            transform[:3, :3] = self.topic_hand_rotation
            transform[:3, 3] = self.topic_hand_position
            transforms[self.controller_key] = transform

        return transforms, dict(self.topic_buttons)

    def _create_reader(self) -> Any:
        self.reader_start_time = time.monotonic()
        return self.OculusReader(
            ip_address=self.reader_ip,
            port=self.port,
            print_FPS=False,
        )

    def _stop_reader(self, reader: Any) -> None:
        if reader is None:
            return
        try:
            reader.running = False
            thread = getattr(reader, "thread", None)
            if thread is not None:
                thread.join(timeout=1.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed while stopping OculusReader: {exc!r}")

    def _restart_reader_if_stale(self, now: float) -> None:
        if self.input_mode != "reader":
            return
        if self.reader_restart_timeout <= 0.0:
            return

        pose_age = self._quest_pose_age(now)
        if pose_age is None:
            stale_for = now - self.reader_start_time
        else:
            stale_for = pose_age

        if stale_for < self.reader_restart_timeout:
            return
        if now - self.last_reader_restart_time < self.reader_restart_timeout:
            return

        self.last_reader_restart_time = now
        self.reader_restart_count += 1
        self.get_logger().warn(
            "Restarting OculusReader after stale Quest pose data. "
            f"diagnostics={self._reader_diagnostics(now)}"
        )

        old_reader = self.reader
        try:
            new_reader = self._create_reader()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to restart OculusReader: {exc!r}")
            return

        self._stop_reader(old_reader)
        self.retired_readers.append(old_reader)
        self.reader = new_reader

    def _quest_pose_age(self, now: float) -> float | None:
        if self.last_quest_pose_time is None:
            return None
        return max(now - self.last_quest_pose_time, 0.0)

    def _reader_thread_alive(self) -> bool | None:
        if self.reader is None:
            return None
        thread = getattr(self.reader, "thread", None)
        if thread is None:
            return None
        return bool(thread.is_alive())

    def _reader_diagnostics(self, now: float) -> dict[str, Any]:
        return {
            "input_mode": self.input_mode,
            "controller_key": self.controller_key,
            "raw_transform_keys": self.last_raw_transform_keys,
            "raw_button_keys": self.last_raw_button_keys,
            "cached_transform_keys": sorted(self.last_valid_transforms.keys()),
            "cached_button_keys": sorted(self.last_valid_buttons.keys()),
            "using_cached_pose": self.using_cached_quest_pose,
            "last_valid_pose_age_sec": self._quest_pose_age(now),
            "reader_running": bool(getattr(self.reader, "running", False)) if self.reader is not None else None,
            "reader_thread_alive": self._reader_thread_alive(),
            "reader_restart_count": self.reader_restart_count,
            "topic_pose_age_sec": None
            if self.last_topic_pose_time is None
            else max(now - self.last_topic_pose_time, 0.0),
            "topic_pose_timeout_sec": self.topic_pose_timeout,
            "quest_pose_grace_period_sec": self.quest_pose_grace_period,
            "reader_restart_timeout_sec": self.reader_restart_timeout,
        }

    def _load_vector3_parameter(self, name: str) -> np.ndarray:
        values = list(self.get_parameter(name).value)
        if len(values) != 3:
            raise ValueError(f"{name} must contain exactly 3 values")
        return np.array(values, dtype=float)

    def _load_axis_map_parameter(
        self,
        name: str,
        *,
        warn_left_handed: bool = True,
    ) -> tuple[np.ndarray, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 9:
            raise ValueError(f"{name} must contain exactly 9 values")
        matrix = np.array(values, dtype=float).reshape((3, 3))
        if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6):
            raise ValueError(f"{name} must be an orthonormal 3x3 matrix")

        determinant = float(np.linalg.det(matrix))
        if not math.isclose(abs(determinant), 1.0, rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError(f"{name} must have determinant +/-1")

        handedness = 1.0 if determinant >= 0.0 else -1.0
        if warn_left_handed and handedness < 0.0:
            self.get_logger().warn(
                f"{name} is left-handed (det=-1). "
                "This is allowed; rotation deltas get an extra sign correction."
            )
        return matrix, handedness

    def _clamp_position(self, position: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(position, self.workspace_min), self.workspace_max)

    @staticmethod
    def _button_bool(value: Any) -> bool:
        if isinstance(value, (list, tuple)):
            if not value:
                return False
            value = value[0]
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return float(value) > 0.5
        return bool(value)

    @staticmethod
    def _button_scalar(value: Any) -> float:
        if isinstance(value, (list, tuple)):
            if not value:
                return 0.0
            value = value[0]
        if isinstance(value, (int, float)):
            return float(value)
        return 0.0

    @staticmethod
    def _to_jsonable(value: Any) -> Any:
        if isinstance(value, dict):
            return {
                key: SimpleQuestImpedanceTeleopNode._to_jsonable(item)
                for key, item in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [SimpleQuestImpedanceTeleopNode._to_jsonable(item) for item in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        return value

    @staticmethod
    def _active_error_names(errors: Any) -> list[str]:
        return [
            name
            for name in errors.get_fields_and_field_types().keys()
            if bool(getattr(errors, name))
        ]

    @staticmethod
    def _dominant_axis(vector: np.ndarray) -> str:
        axes = ("rx", "ry", "rz")
        if vector.size != 3:
            return "none"
        index = int(np.argmax(np.abs(vector)))
        value = float(vector[index])
        if abs(value) < 1e-9:
            return "none"
        return f"{'+' if value >= 0.0 else '-'}{axes[index]}({value:+.5f})"

    @staticmethod
    def _fmt_vector3(vector: np.ndarray | list[float] | tuple[float, float, float]) -> str:
        values = [float(value) for value in vector]
        return "[" + ", ".join(f"{value:+.5f}" for value in values[:3]) + "]"

    @staticmethod
    def _matrix_to_rpy_xyz(rotation: np.ndarray) -> np.ndarray:
        sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
        singular = sy < 1e-9
        if not singular:
            roll = math.atan2(rotation[2, 1], rotation[2, 2])
            pitch = math.atan2(-rotation[2, 0], sy)
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        else:
            roll = math.atan2(-rotation[1, 2], rotation[1, 1])
            pitch = math.atan2(-rotation[2, 0], sy)
            yaw = 0.0
        return np.array([roll, pitch, yaw], dtype=float)

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
    def _apply_deadband(vector: np.ndarray, threshold: float) -> np.ndarray:
        if threshold <= 0.0:
            return vector
        norm = float(np.linalg.norm(vector))
        if norm <= threshold:
            return np.zeros_like(vector)
        return vector * ((norm - threshold) / norm)

    @staticmethod
    def _limit_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
        if maximum_norm <= 0.0:
            return vector
        norm = float(np.linalg.norm(vector))
        if norm < 1e-12 or norm <= maximum_norm:
            return vector
        return vector * (maximum_norm / norm)

    @staticmethod
    def _rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
        cos_angle = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
        angle = math.acos(cos_angle)
        skew_vector = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=float,
        )
        if angle < 1e-6:
            return 0.5 * skew_vector
        sin_angle = math.sin(angle)
        if abs(sin_angle) < 1e-6:
            return np.zeros(3, dtype=float)
        axis = skew_vector / (2.0 * sin_angle)
        return axis * angle

    @staticmethod
    def _rotation_matrix_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
        angle = float(np.linalg.norm(rotvec))
        if angle < 1e-9:
            return np.eye(3, dtype=float)
        axis = rotvec / angle
        skew = np.array(
            [
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0],
            ],
            dtype=float,
        )
        return np.eye(3, dtype=float) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)

    @staticmethod
    def _orthonormalize(rotation: np.ndarray) -> np.ndarray:
        u, _, vt = np.linalg.svd(rotation)
        result = u @ vt
        if np.linalg.det(result) < 0.0:
            u[:, -1] *= -1.0
            result = u @ vt
        return result


def main() -> None:
    rclpy.init()
    node = SimpleQuestImpedanceTeleopNode()
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
