import json
import math
import os
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf_transformations import (
    euler_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
)
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from quest3_oculus_rviz.teleop_motion_modes import make_teleop_motion_mode


class RightHandTeleopSimNode(Node):
    """Convert Quest right-controller input into a simulated Franka TCP command."""

    def __init__(self) -> None:
        super().__init__("right_hand_teleop_sim")

        self.declare_parameter("hand_label", "right")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("tcp_frame", "franka_sim_tcp")
        self.declare_parameter("controller_control_frame", "quest3_right_controller")
        self.declare_parameter("attitude_frame", "quest3_right_attitude")
        self.declare_parameter("attitude_frame_position", [0.25, -0.55, 0.35])
        self.declare_parameter(
            "controller_axis_map",
            [
                0.0, 1.0, 0.0,
                -0.7071067812, 0.0, -0.7071067812,
                -0.7071067812, 0.0, 0.7071067812,
            ],
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trigger_threshold", 0.5)
        self.declare_parameter("align_controller_frame_to_tcp", False)
        self.declare_parameter("teleop_motion_mode", "velocity")
        self.declare_parameter("translation_scale", 1.0)
        self.declare_parameter("translation_x_sign", 1.0)
        self.declare_parameter("translation_y_sign", 1.0)
        self.declare_parameter("translation_z_sign", 1.0)
        self.declare_parameter("translation_deadband_m", 0.0015)
        self.declare_parameter("rotation_deadband_rad", 0.004)
        self.declare_parameter("delta_filter_alpha", 0.45)
        self.declare_parameter("translation_delta_filter_alpha", -1.0)
        self.declare_parameter("rotation_delta_filter_alpha", -1.0)
        self.declare_parameter("max_tcp_delta_body_m", 0.025)
        self.declare_parameter("max_tcp_delta_rotvec_rad", 0.035)
        self.declare_parameter("position_max_tcp_offset_m", 0.35)
        self.declare_parameter("position_max_tcp_rotvec_rad", 0.50)
        self.declare_parameter("position_reanchor_when_stationary", False)
        self.declare_parameter("position_stationary_translation_m", 0.0010)
        self.declare_parameter("position_stationary_rotation_rad", 0.0030)
        self.declare_parameter("position_stationary_hold_sec", 0.12)
        self.declare_parameter("position_resume_translation_m", 0.0030)
        self.declare_parameter("position_resume_rotation_rad", 0.0200)
        self.declare_parameter("rotation_scale", 1.0)
        self.declare_parameter("target_lead_time_sec", 0.0)
        self.declare_parameter("max_controller_angle_rad", 0.9)
        self.declare_parameter("start_xyz", [0.45, 0.0, 0.35])
        self.declare_parameter("start_rpy", [0.0, math.pi, 0.0])
        self.declare_parameter(
            "tcp_axis_map",
            [
                -1.0, 0.0, 0.0,
                0.0, -1.0, 0.0,
                0.0, 0.0, 1.0,
            ],
        )
        self.declare_parameter("workspace_min", [0.20, -0.45, 0.08])
        self.declare_parameter("workspace_max", [0.80, 0.45, 0.75])
        self.declare_parameter("roll_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pose_log_enabled", True)
        self.declare_parameter("pose_log_rate_hz", 10.0)
        self.declare_parameter("pose_log_dir", "")
        self.declare_parameter("controller_pose_topic", "quest3/right_controller/pose")
        self.declare_parameter("buttons_topic", "quest3/buttons")
        self.declare_parameter("twist_topic", "quest3/right_teleop/twist")
        self.declare_parameter("enabled_topic", "quest3/right_teleop/enabled")
        self.declare_parameter("debug_topic", "quest3/right_teleop/debug")
        self.declare_parameter("target_pose_topic", "franka_sim/tcp_target_pose")
        self.declare_parameter("marker_topic", "franka_sim/tcp_markers")
        self.declare_parameter("trigger_button_name", "RTr")
        self.declare_parameter("trigger_value_name", "rightTrig")
        self.declare_parameter("external_tcp_pose_topic", "")
        self.declare_parameter("sync_external_tcp_when_idle", False)
        self.declare_parameter("external_tcp_pose_timeout_sec", 0.25)
        self.declare_parameter("gripper_buttons_enabled", True)
        self.declare_parameter("gripper_command_topic", "quest3/right_teleop/gripper_command")
        self.declare_parameter("gripper_open_button_name", "A")
        self.declare_parameter("gripper_close_button_name", "B")

        self.hand_label = str(self.get_parameter("hand_label").value)
        self.world_frame = self.get_parameter("world_frame").value
        self.tcp_frame = self.get_parameter("tcp_frame").value
        self.controller_control_frame = self.get_parameter("controller_control_frame").value
        self.attitude_frame = self.get_parameter("attitude_frame").value
        self.attitude_frame_position = np.array(
            self.get_parameter("attitude_frame_position").value,
            dtype=float,
        )
        self.controller_axis_map = self.load_axis_map()
        self.tcp_axis_map = self.load_rotation_map("tcp_axis_map")
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.trigger_threshold = float(self.get_parameter("trigger_threshold").value)
        self.align_controller_frame_to_tcp = bool(self.get_parameter("align_controller_frame_to_tcp").value)
        self.teleop_motion_mode = str(self.get_parameter("teleop_motion_mode").value).strip().lower()
        if self.teleop_motion_mode not in ("velocity", "position"):
            self.get_logger().warn(
                f"Unknown teleop_motion_mode='{self.teleop_motion_mode}', using 'velocity'."
            )
            self.teleop_motion_mode = "velocity"
        self.teleop_motion_strategy = make_teleop_motion_mode(self.teleop_motion_mode)
        self.translation_scale = float(self.get_parameter("translation_scale").value)
        self.translation_axis_sign = np.array(
            [
                float(self.get_parameter("translation_x_sign").value),
                float(self.get_parameter("translation_y_sign").value),
                float(self.get_parameter("translation_z_sign").value),
            ],
            dtype=float,
        )
        self.translation_deadband = max(float(self.get_parameter("translation_deadband_m").value), 0.0)
        self.rotation_deadband = max(float(self.get_parameter("rotation_deadband_rad").value), 0.0)
        self.delta_filter_alpha = self.clamp(float(self.get_parameter("delta_filter_alpha").value), 0.0, 1.0)
        self.translation_delta_filter_alpha = self.load_optional_filter_alpha(
            "translation_delta_filter_alpha",
            self.delta_filter_alpha,
        )
        self.rotation_delta_filter_alpha = self.load_optional_filter_alpha(
            "rotation_delta_filter_alpha",
            self.delta_filter_alpha,
        )
        self.max_tcp_delta_body = max(float(self.get_parameter("max_tcp_delta_body_m").value), 0.0)
        self.max_tcp_delta_rotvec = max(float(self.get_parameter("max_tcp_delta_rotvec_rad").value), 0.0)
        self.position_max_tcp_offset = max(
            float(self.get_parameter("position_max_tcp_offset_m").value),
            0.0,
        )
        self.position_max_tcp_rotvec = max(
            float(self.get_parameter("position_max_tcp_rotvec_rad").value),
            0.0,
        )
        self.position_reanchor_when_stationary = bool(
            self.get_parameter("position_reanchor_when_stationary").value
        )
        self.position_stationary_translation = max(
            float(self.get_parameter("position_stationary_translation_m").value),
            0.0,
        )
        self.position_stationary_rotation = max(
            float(self.get_parameter("position_stationary_rotation_rad").value),
            0.0,
        )
        self.position_stationary_hold = max(
            float(self.get_parameter("position_stationary_hold_sec").value),
            0.0,
        )
        self.position_resume_translation = max(
            float(self.get_parameter("position_resume_translation_m").value),
            self.position_stationary_translation,
        )
        self.position_resume_rotation = max(
            float(self.get_parameter("position_resume_rotation_rad").value),
            self.position_stationary_rotation,
        )
        self.rotation_scale = float(self.get_parameter("rotation_scale").value)
        self.max_controller_angle = float(self.get_parameter("max_controller_angle_rad").value)
        self.workspace_min = np.array(self.get_parameter("workspace_min").value, dtype=float)
        self.workspace_max = np.array(self.get_parameter("workspace_max").value, dtype=float)
        self.roll_sign = float(self.get_parameter("roll_sign").value)
        self.pitch_sign = float(self.get_parameter("pitch_sign").value)
        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.pose_log_enabled = bool(self.get_parameter("pose_log_enabled").value)
        self.pose_log_rate_hz = float(self.get_parameter("pose_log_rate_hz").value)
        self.controller_pose_topic = str(self.get_parameter("controller_pose_topic").value)
        self.buttons_topic = str(self.get_parameter("buttons_topic").value)
        self.twist_topic = str(self.get_parameter("twist_topic").value)
        self.enabled_topic = str(self.get_parameter("enabled_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.trigger_button_name = str(self.get_parameter("trigger_button_name").value)
        self.trigger_value_name = str(self.get_parameter("trigger_value_name").value)
        self.external_tcp_pose_topic = str(self.get_parameter("external_tcp_pose_topic").value).strip()
        self.sync_external_tcp_when_idle = bool(self.get_parameter("sync_external_tcp_when_idle").value)
        self.external_tcp_pose_timeout = float(self.get_parameter("external_tcp_pose_timeout_sec").value)
        self.gripper_buttons_enabled = bool(self.get_parameter("gripper_buttons_enabled").value)
        self.gripper_command_topic = str(self.get_parameter("gripper_command_topic").value)
        self.gripper_open_button_name = str(self.get_parameter("gripper_open_button_name").value)
        self.gripper_close_button_name = str(self.get_parameter("gripper_close_button_name").value)
        self.pose_log_file = None
        self.last_pose_log_time = 0.0
        self.latest_external_tcp_pose: PoseStamped | None = None
        self.latest_external_tcp_pose_time = 0.0
        self.warned_external_frame_mismatch = False
        self.warned_external_base_unavailable = False
        self.last_tcp_base_source = "internal"
        self.last_target_lead_scale = 1.0
        self.last_update_dt_sec = 0.0

        start_xyz = np.array(self.get_parameter("start_xyz").value, dtype=float)
        start_rpy = self.get_parameter("start_rpy").value
        self.tcp_position = self.clamp_position(start_xyz)
        start_rotation = euler_matrix(float(start_rpy[0]), float(start_rpy[1]), float(start_rpy[2]))[:3, :3]
        self.tcp_rotation = start_rotation @ self.tcp_axis_map

        self.latest_pose: PoseStamped | None = None
        self.latest_buttons: dict[str, Any] = {}
        self.previous_gripper_open_pressed = False
        self.previous_gripper_close_pressed = False
        self.last_gripper_button_command = ""
        self.prev_enabled = False
        self.anchor_controller_position: np.ndarray | None = None
        self.anchor_controller_rotation: np.ndarray | None = None
        self.controller_frame_alignment_active = False
        self.anchor_tcp_position: np.ndarray | None = None
        self.anchor_tcp_rotation: np.ndarray | None = None
        self.anchor_tcp_base_source = "internal_anchor"
        self.previous_controller_position: np.ndarray | None = None
        self.previous_controller_rotation: np.ndarray | None = None
        self.last_controller_frame_delta_position_raw = np.zeros(3, dtype=float)
        self.last_controller_frame_delta_rotvec = np.zeros(3, dtype=float)
        self.last_controller_delta_position_raw = np.zeros(3, dtype=float)
        self.last_controller_delta_position_control = np.zeros(3, dtype=float)
        self.last_controller_delta_rotvec = np.zeros(3, dtype=float)
        self.last_tcp_delta_body = np.zeros(3, dtype=float)
        self.last_tcp_delta_body_unfiltered = np.zeros(3, dtype=float)
        self.last_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.last_tcp_delta_rotvec_unfiltered = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_body = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.position_stationary_since: float | None = None
        self.position_stationary_active = False
        self.position_paused_for_stationary = False
        self.position_stationary_reanchor_count = 0
        self.last_update_time = time.monotonic()

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.controller_pose_topic,
            self.pose_callback,
            10,
        )
        self.buttons_sub = self.create_subscription(String, self.buttons_topic, self.buttons_callback, 10)
        self.external_tcp_pose_sub = None
        if self.external_tcp_pose_topic:
            self.external_tcp_pose_sub = self.create_subscription(
                PoseStamped,
                self.external_tcp_pose_topic,
                self.external_tcp_pose_callback,
                10,
            )

        self.twist_pub = self.create_publisher(TwistStamped, self.twist_topic, 10)
        self.enabled_pub = self.create_publisher(Bool, self.enabled_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.gripper_command_pub = self.create_publisher(String, self.gripper_command_topic, 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        if self.pose_log_enabled:
            self.pose_log_file = self.open_pose_log_file()

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f"{self.hand_label} hand 6D teleop ready. Hold trigger to move TCP. "
            f"mode={self.teleop_motion_mode} pose={self.controller_pose_topic} "
            f"target={self.target_pose_topic}"
        )
        if self.gripper_buttons_enabled:
            self.get_logger().info(
                f"{self.hand_label} gripper buttons ready: "
                f"{self.gripper_open_button_name}=open, {self.gripper_close_button_name}=close, "
                f"topic={self.gripper_command_topic}"
            )
        if self.external_tcp_pose_topic and self.sync_external_tcp_when_idle:
            self.get_logger().info(
                f"Idle TCP target will sync from external pose topic: {self.external_tcp_pose_topic}"
            )

    def destroy_node(self) -> bool:
        if self.pose_log_file is not None:
            self.pose_log_file.close()
            self.pose_log_file = None
        return super().destroy_node()

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def buttons_callback(self, msg: String) -> None:
        try:
            self.latest_buttons = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_buttons = {}
        self.process_gripper_buttons()

    def process_gripper_buttons(self) -> None:
        if not self.gripper_buttons_enabled:
            self.previous_gripper_open_pressed = False
            self.previous_gripper_close_pressed = False
            return

        open_pressed = self.button_bool(self.gripper_open_button_name)
        close_pressed = self.button_bool(self.gripper_close_button_name)
        open_edge = open_pressed and not self.previous_gripper_open_pressed
        close_edge = close_pressed and not self.previous_gripper_close_pressed

        command = ""
        if open_edge and not close_pressed:
            command = "open"
        elif close_edge and not open_pressed:
            command = "close"
        elif open_edge or close_edge:
            self.get_logger().warn(
                "Ignoring simultaneous gripper open/close button press."
            )

        self.previous_gripper_open_pressed = open_pressed
        self.previous_gripper_close_pressed = close_pressed
        if command:
            self.publish_gripper_command(command)

    def publish_gripper_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.gripper_command_pub.publish(msg)
        self.last_gripper_button_command = command
        self.get_logger().info(
            f"{self.hand_label} gripper command: {command} -> {self.gripper_command_topic}"
        )

    def external_tcp_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_external_tcp_pose = msg
        self.latest_external_tcp_pose_time = time.monotonic()
        if self.should_sync_external_tcp():
            self.sync_tcp_from_pose(msg)

    def timer_callback(self) -> None:
        now = time.monotonic()
        dt = min(max(now - self.last_update_time, 0.0), 0.1)
        self.last_update_time = now
        self.last_update_dt_sec = dt

        stamp = self.get_clock().now().to_msg()
        trigger_enabled = self.is_trigger_pressed() and self.latest_pose is not None
        previous_tcp_position = self.tcp_position.copy()
        previous_tcp_rotation = self.tcp_rotation.copy()

        if trigger_enabled and not self.prev_enabled:
            self.capture_motion_anchor()
            self.get_logger().info(
                f"{self.hand_label} trigger pressed: {self.teleop_motion_mode} tracking started."
            )

        if not trigger_enabled:
            self.clear_motion_anchor()
            if self.should_sync_external_tcp():
                self.sync_tcp_from_pose(self.latest_external_tcp_pose)

        if trigger_enabled:
            self.update_tcp_from_controller_pose(now, dt)

        motion_enabled = trigger_enabled and not self.position_paused_for_stationary
        twist = self.compute_twist_from_tcp_delta(
            previous_tcp_position,
            previous_tcp_rotation,
            dt,
            motion_enabled,
        )

        self.prev_enabled = trigger_enabled
        self.publish_outputs(twist, motion_enabled, stamp)

    def capture_motion_anchor(self) -> None:
        mapped_controller_rotation = self.current_mapped_controller_rotation()
        anchor_tcp_base_source = "internal_anchor"
        now = time.monotonic()
        if self.latest_external_tcp_pose is not None and self.external_tcp_pose_is_fresh(now):
            self.tcp_position, self.tcp_rotation = self.tcp_state_from_pose(self.latest_external_tcp_pose)
            anchor_tcp_base_source = "external_anchor"

        self.anchor_controller_position = self.current_controller_position()
        # Control deltas are now frame-to-frame increments so a still controller
        # produces a zero TCP delta while the trigger remains pressed.
        self.anchor_controller_rotation = mapped_controller_rotation
        self.anchor_tcp_position = self.tcp_position.copy()
        self.anchor_tcp_rotation = self.tcp_rotation.copy()
        self.anchor_tcp_base_source = anchor_tcp_base_source
        self.previous_controller_position = self.anchor_controller_position.copy()
        self.previous_controller_rotation = self.anchor_controller_rotation.copy()
        self.reset_frame_deltas()
        self.controller_frame_alignment_active = self.align_controller_frame_to_tcp

    def clear_motion_anchor(self) -> None:
        self.anchor_controller_position = None
        self.anchor_controller_rotation = None
        self.controller_frame_alignment_active = False
        self.anchor_tcp_position = None
        self.anchor_tcp_rotation = None
        self.anchor_tcp_base_source = "internal_anchor"
        self.previous_controller_position = None
        self.previous_controller_rotation = None
        self.reset_frame_deltas()

    def reset_frame_deltas(self) -> None:
        self.last_controller_frame_delta_position_raw = np.zeros(3, dtype=float)
        self.last_controller_frame_delta_rotvec = np.zeros(3, dtype=float)
        self.last_controller_delta_position_raw = np.zeros(3, dtype=float)
        self.last_controller_delta_position_control = np.zeros(3, dtype=float)
        self.last_controller_delta_rotvec = np.zeros(3, dtype=float)
        self.last_tcp_delta_body = np.zeros(3, dtype=float)
        self.last_tcp_delta_body_unfiltered = np.zeros(3, dtype=float)
        self.last_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.last_tcp_delta_rotvec_unfiltered = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_body = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.position_stationary_since = None
        self.position_stationary_active = False
        self.position_paused_for_stationary = False
        self.last_target_lead_scale = 1.0
        self.last_update_dt_sec = 0.0

    def position_controller_is_stationary(
        self,
        frame_delta_position: np.ndarray,
        frame_delta_rotvec: np.ndarray,
        now: float,
    ) -> bool:
        if not self.position_reanchor_when_stationary or self.teleop_motion_mode != "position":
            return False

        translation_stationary = (
            float(np.linalg.norm(frame_delta_position)) <= self.position_stationary_translation
        )
        rotation_stationary = (
            float(np.linalg.norm(frame_delta_rotvec)) <= self.position_stationary_rotation
        )
        if not (translation_stationary and rotation_stationary):
            self.position_stationary_since = None
            self.position_stationary_active = False
            return False

        if self.position_stationary_since is None:
            self.position_stationary_since = now
            self.position_stationary_active = False
            return False

        self.position_stationary_active = (
            now - self.position_stationary_since >= self.position_stationary_hold
        )
        return self.position_stationary_active

    def position_controller_resume_requested(
        self,
        frame_delta_position: np.ndarray,
        frame_delta_rotvec: np.ndarray,
    ) -> bool:
        translation_moved = (
            float(np.linalg.norm(frame_delta_position)) >= self.position_resume_translation
        )
        rotation_moved = (
            float(np.linalg.norm(frame_delta_rotvec)) >= self.position_resume_rotation
        )
        return translation_moved or rotation_moved

    def reanchor_position_target(
        self,
        controller_position: np.ndarray,
        controller_rotation: np.ndarray,
        now: float,
    ) -> None:
        if self.latest_external_tcp_pose is not None and self.external_tcp_pose_is_fresh(now):
            self.tcp_position, self.tcp_rotation = self.tcp_state_from_pose(self.latest_external_tcp_pose)
            self.anchor_tcp_base_source = "external_stationary_reanchor"
        else:
            self.anchor_tcp_base_source = "internal_stationary_reanchor"

        self.anchor_controller_position = controller_position.copy()
        self.anchor_controller_rotation = controller_rotation.copy()
        self.anchor_tcp_position = self.tcp_position.copy()
        self.anchor_tcp_rotation = self.tcp_rotation.copy()
        self.previous_controller_position = controller_position.copy()
        self.previous_controller_rotation = controller_rotation.copy()
        self.filtered_tcp_delta_body = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.position_stationary_reanchor_count += 1

    def update_position_stationary_hold_state(
        self,
        controller_position: np.ndarray,
        controller_rotation: np.ndarray,
        now: float,
    ) -> None:
        if self.latest_external_tcp_pose is not None and self.external_tcp_pose_is_fresh(now):
            self.tcp_position, self.tcp_rotation = self.tcp_state_from_pose(self.latest_external_tcp_pose)
            self.anchor_tcp_position = self.tcp_position.copy()
            self.anchor_tcp_rotation = self.tcp_rotation.copy()
            self.anchor_tcp_base_source = "external_stationary_hold"
            self.last_tcp_base_source = "external_stationary_hold"
        else:
            self.anchor_tcp_position = self.tcp_position.copy()
            self.anchor_tcp_rotation = self.tcp_rotation.copy()
            self.anchor_tcp_base_source = "internal_stationary_hold"
            self.last_tcp_base_source = "internal_stationary_hold"

        self.anchor_controller_position = controller_position.copy()
        self.anchor_controller_rotation = controller_rotation.copy()
        self.previous_controller_position = controller_position.copy()
        self.previous_controller_rotation = controller_rotation.copy()
        self.last_controller_delta_position_raw = np.zeros(3, dtype=float)
        self.last_controller_delta_position_control = np.zeros(3, dtype=float)
        self.last_controller_delta_rotvec = np.zeros(3, dtype=float)
        self.last_tcp_delta_body = np.zeros(3, dtype=float)
        self.last_tcp_delta_body_unfiltered = np.zeros(3, dtype=float)
        self.last_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.last_tcp_delta_rotvec_unfiltered = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_body = np.zeros(3, dtype=float)
        self.filtered_tcp_delta_rotvec = np.zeros(3, dtype=float)
        self.last_target_lead_scale = 1.0

    def should_sync_external_tcp(self) -> bool:
        return (
            self.sync_external_tcp_when_idle
            and self.latest_external_tcp_pose is not None
            and not self.prev_enabled
            and self.anchor_tcp_position is None
        )

    def tcp_state_from_pose(self, msg: PoseStamped) -> tuple[np.ndarray, np.ndarray]:
        if (
            msg.header.frame_id
            and msg.header.frame_id != self.world_frame
            and not self.warned_external_frame_mismatch
        ):
            self.get_logger().warn(
                "External TCP pose frame differs from teleop world_frame: "
                f"external='{msg.header.frame_id}', world_frame='{self.world_frame}'. "
                "Using numeric pose values directly."
            )
            self.warned_external_frame_mismatch = True

        position = self.clamp_position(
            np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        )
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        rotation = quaternion_matrix(quat)[:3, :3]
        return position, rotation

    def sync_tcp_from_pose(self, msg: PoseStamped) -> None:
        self.tcp_position, self.tcp_rotation = self.tcp_state_from_pose(msg)
        self.last_tcp_base_source = "external_idle"

    def external_tcp_pose_is_fresh(self, now: float) -> bool:
        if self.latest_external_tcp_pose is None:
            return False
        if self.external_tcp_pose_timeout <= 0.0:
            return True
        return now - self.latest_external_tcp_pose_time <= self.external_tcp_pose_timeout

    def current_tcp_base_pose(self, now: float) -> tuple[np.ndarray, np.ndarray, str]:
        if self.latest_external_tcp_pose is not None and self.external_tcp_pose_is_fresh(now):
            position, rotation = self.tcp_state_from_pose(self.latest_external_tcp_pose)
            return position, rotation, "external_live"

        if self.external_tcp_pose_topic and not self.warned_external_base_unavailable:
            self.get_logger().warn(
                "External TCP pose is not available/fresh while teleop is enabled; "
                "falling back to the internally integrated target pose."
            )
            self.warned_external_base_unavailable = True

        return self.tcp_position.copy(), self.tcp_rotation.copy(), "internal_fallback"

    def update_tcp_from_controller_pose(self, now: float, dt: float) -> None:
        self.teleop_motion_strategy.update(self, now, dt)

    def target_lead_scale(self, dt: float, tcp_base_source: str) -> float:
        del dt, tcp_base_source
        return 1.0

    def compute_twist_from_tcp_delta(
        self,
        previous_tcp_position: np.ndarray,
        previous_tcp_rotation: np.ndarray,
        dt: float,
        enabled: bool,
    ) -> TwistStamped:
        twist = TwistStamped()
        twist.header.frame_id = self.world_frame
        twist.header.stamp = self.get_clock().now().to_msg()

        if not enabled or dt <= 1e-6:
            return twist

        linear = (self.tcp_position - previous_tcp_position) / dt
        angular = self.rotation_vector_from_matrix(previous_tcp_rotation.T @ self.tcp_rotation) / dt

        twist.twist.linear.x = float(linear[0])
        twist.twist.linear.y = float(linear[1])
        twist.twist.linear.z = float(linear[2])
        twist.twist.angular.x = float(angular[0])
        twist.twist.angular.y = float(angular[1])
        twist.twist.angular.z = float(angular[2])
        return twist

    def publish_outputs(self, twist: TwistStamped, enabled: bool, stamp: Any) -> None:
        twist.header.stamp = stamp
        self.twist_pub.publish(twist)

        enabled_msg = Bool()
        enabled_msg.data = enabled
        self.enabled_pub.publish(enabled_msg)

        tcp_pose = self.make_tcp_pose(stamp)
        self.tcp_pose_pub.publish(tcp_pose)
        self.publish_tcp_tf(tcp_pose)
        self.publish_controller_control_tf(stamp)
        self.publish_controller_attitude_tf(stamp)
        self.marker_pub.publish(self.make_markers(stamp))

        debug = {
            "control_mode": f"{self.hand_label}_controller_6d_pose",
            "teleop_motion_mode": self.teleop_motion_mode,
            "hand": self.hand_label,
            "enabled": enabled,
            "trigger_pressed": self.is_trigger_pressed(),
            "input_ready": self.latest_pose is not None,
            "trigger": self.trigger_value(),
            "anchor_active": self.anchor_controller_position is not None,
            "external_tcp_sync_enabled": self.sync_external_tcp_when_idle,
            "external_tcp_pose_ready": self.latest_external_tcp_pose is not None,
            "tcp_base_source": self.last_tcp_base_source,
            "publish_rate_hz": self.publish_rate_hz,
            "update_dt_sec": self.last_update_dt_sec,
            "target_lead_scale": self.last_target_lead_scale,
            "translation_axis_sign": self.translation_axis_sign.tolist(),
            "translation_deadband_m": self.translation_deadband,
            "rotation_deadband_rad": self.rotation_deadband,
            "delta_filter_alpha": self.delta_filter_alpha,
            "translation_delta_filter_alpha": self.translation_delta_filter_alpha,
            "rotation_delta_filter_alpha": self.rotation_delta_filter_alpha,
            "max_tcp_delta_body_m": self.max_tcp_delta_body,
            "max_tcp_delta_rotvec_rad": self.max_tcp_delta_rotvec,
            "position_max_tcp_offset_m": self.position_max_tcp_offset,
            "position_max_tcp_rotvec_rad": self.position_max_tcp_rotvec,
            "position_reanchor_when_stationary": self.position_reanchor_when_stationary,
            "position_stationary_translation_m": self.position_stationary_translation,
            "position_stationary_rotation_rad": self.position_stationary_rotation,
            "position_stationary_hold_sec": self.position_stationary_hold,
            "position_resume_translation_m": self.position_resume_translation,
            "position_resume_rotation_rad": self.position_resume_rotation,
            "position_stationary_active": self.position_stationary_active,
            "position_paused_for_stationary": self.position_paused_for_stationary,
            "position_resume_requested": self.position_controller_resume_requested(
                self.last_controller_frame_delta_position_raw,
                self.last_controller_frame_delta_rotvec,
            ),
            "position_stationary_reanchor_count": self.position_stationary_reanchor_count,
            "gripper_buttons_enabled": self.gripper_buttons_enabled,
            "gripper_command_topic": self.gripper_command_topic,
            "gripper_open_button": self.gripper_open_button_name,
            "gripper_close_button": self.gripper_close_button_name,
            "last_gripper_button_command": self.last_gripper_button_command,
            "controller_frame_aligned_to_tcp": self.controller_frame_alignment_active,
            "controller_frame_delta_position_raw": self.last_controller_frame_delta_position_raw.tolist(),
            "controller_frame_delta_position_raw_norm_m": float(
                np.linalg.norm(self.last_controller_frame_delta_position_raw)
            ),
            "controller_frame_delta_rotvec": self.last_controller_frame_delta_rotvec.tolist(),
            "controller_frame_delta_rotvec_norm_rad": float(
                np.linalg.norm(self.last_controller_frame_delta_rotvec)
            ),
            "controller_delta_position_raw": list(self.controller_delta_position_raw()),
            "controller_delta_position_control": list(self.controller_delta_position_control()),
            "controller_delta_rotvec": list(self.controller_delta_rotvec()),
            "tcp_delta_body": list(self.tcp_delta_body()),
            "tcp_delta_body_unfiltered": self.last_tcp_delta_body_unfiltered.tolist(),
            "tcp_delta_rotvec": self.last_tcp_delta_rotvec.tolist(),
            "tcp_delta_rotvec_unfiltered": self.last_tcp_delta_rotvec_unfiltered.tolist(),
            "controller_delta_position_raw_norm_m": float(
                np.linalg.norm(self.last_controller_delta_position_raw)
            ),
            "controller_delta_position_control_norm_m": float(
                np.linalg.norm(self.last_controller_delta_position_control)
            ),
            "controller_delta_rotvec_norm_rad": float(
                np.linalg.norm(self.last_controller_delta_rotvec)
            ),
            "tcp_delta_body_norm_m": float(np.linalg.norm(self.last_tcp_delta_body)),
            "tcp_delta_body_unfiltered_norm_m": float(
                np.linalg.norm(self.last_tcp_delta_body_unfiltered)
            ),
            "tcp_delta_rotvec_norm_rad": float(np.linalg.norm(self.last_tcp_delta_rotvec)),
            "tcp_delta_rotvec_unfiltered_norm_rad": float(
                np.linalg.norm(self.last_tcp_delta_rotvec_unfiltered)
            ),
            "twist": {
                "linear": [twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z],
                "angular": [twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z],
            },
            "tcp_xyz": self.tcp_position.tolist(),
        }
        debug_msg = String()
        debug_msg.data = json.dumps(debug, sort_keys=True)
        self.debug_pub.publish(debug_msg)
        self.log_poses(tcp_pose, twist, enabled, debug)

    def make_tcp_pose(self, stamp: Any) -> PoseStamped:
        transform = np.eye(4)
        transform[:3, :3] = self.tcp_rotation
        quat = quaternion_from_matrix(transform)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = float(self.tcp_position[0])
        pose.pose.position.y = float(self.tcp_position[1])
        pose.pose.position.z = float(self.tcp_position[2])
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])
        return pose

    def open_pose_log_file(self):
        configured_dir = str(self.get_parameter("pose_log_dir").value).strip()
        if configured_dir:
            log_dir = Path(configured_dir).expanduser()
        else:
            log_dir = Path(os.environ.get("TELEOP_ROOT", Path.cwd())) / "log"
        log_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = log_dir / f"quest3_tcp_pose_{timestamp}.jsonl"
        log_file = log_path.open("a", encoding="utf-8", buffering=1)
        self.get_logger().info(f"Pose log file: {log_path}")
        return log_file

    def log_poses(self, tcp_pose: PoseStamped, twist: TwistStamped, enabled: bool, debug: dict[str, Any]) -> None:
        if self.pose_log_file is None or self.latest_pose is None:
            return

        now = time.monotonic()
        if now - self.last_pose_log_time < 1.0 / max(self.pose_log_rate_hz, 1e-6):
            return
        self.last_pose_log_time = now

        mapped_controller_quat = self.mapped_controller_quaternion()
        aligned_controller_quat = self.aligned_controller_quaternion()
        robot_tcp_pose = self.latest_external_tcp_pose
        robot_tcp_pose_age_sec = None
        robot_tcp_pose_fresh = False
        target_minus_robot = None
        if robot_tcp_pose is not None:
            robot_tcp_pose_age_sec = now - self.latest_external_tcp_pose_time
            robot_tcp_pose_fresh = self.external_tcp_pose_is_fresh(now)
            target_minus_robot = self.pose_delta_to_dict(robot_tcp_pose, tcp_pose)

        record = {
            "time_sec": self.get_clock().now().nanoseconds * 1e-9,
            "hand": self.hand_label,
            "enabled": enabled,
            "trigger": self.trigger_value(),
            "controller_pose_raw": self.pose_to_dict(self.latest_pose),
            "controller_orientation_mapped_xyzw": mapped_controller_quat,
            "controller_orientation_aligned_xyzw": aligned_controller_quat,
            "franka_sim_tcp_pose": self.pose_to_dict(tcp_pose),
            "robot_tcp_pose": self.pose_to_dict(robot_tcp_pose) if robot_tcp_pose is not None else None,
            "robot_tcp_pose_topic": self.external_tcp_pose_topic,
            "robot_tcp_pose_age_sec": robot_tcp_pose_age_sec,
            "robot_tcp_pose_fresh": robot_tcp_pose_fresh,
            "target_minus_robot": target_minus_robot,
            "twist": {
                "frame_id": twist.header.frame_id,
                "linear": {
                    "x": twist.twist.linear.x,
                    "y": twist.twist.linear.y,
                    "z": twist.twist.linear.z,
                },
                "angular": {
                    "x": twist.twist.angular.x,
                    "y": twist.twist.angular.y,
                    "z": twist.twist.angular.z,
                },
            },
            "debug": debug,
        }
        line = json.dumps(record, sort_keys=True)
        self.pose_log_file.write(line + "\n")
        # self.get_logger().info(
        #     "pose_log "
        #     f"enabled={enabled} "
        #     f"ctrl_raw_xyz={self.xyz_string(self.latest_pose)} "
        #     f"tcp_xyz={self.xyz_string(tcp_pose)} "
        #     f"tcp_qxyzw={self.quat_string(tcp_pose)}"
        # )

    def mapped_controller_quaternion(self) -> list[float]:
        transform = np.eye(4)
        transform[:3, :3] = self.current_mapped_controller_rotation()
        quat = quaternion_from_matrix(transform)
        return [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]

    def aligned_controller_quaternion(self) -> list[float]:
        transform = np.eye(4)
        transform[:3, :3] = self.current_aligned_controller_rotation()
        quat = quaternion_from_matrix(transform)
        return [float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])]

    @staticmethod
    def pose_to_dict(msg: PoseStamped) -> dict[str, Any]:
        return {
            "frame_id": msg.header.frame_id,
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
                "z": msg.pose.position.z,
            },
            "orientation_xyzw": {
                "x": msg.pose.orientation.x,
                "y": msg.pose.orientation.y,
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w,
            },
        }

    def pose_delta_to_dict(self, source_pose: PoseStamped, target_pose: PoseStamped) -> dict[str, Any]:
        source_position = np.array(
            [
                source_pose.pose.position.x,
                source_pose.pose.position.y,
                source_pose.pose.position.z,
            ],
            dtype=float,
        )
        target_position = np.array(
            [
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
            ],
            dtype=float,
        )
        position_delta = target_position - source_position

        source_rotation = quaternion_matrix(
            [
                source_pose.pose.orientation.x,
                source_pose.pose.orientation.y,
                source_pose.pose.orientation.z,
                source_pose.pose.orientation.w,
            ]
        )[:3, :3]
        target_rotation = quaternion_matrix(
            [
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w,
            ]
        )[:3, :3]
        rotation_delta = self.rotation_vector_from_matrix(source_rotation.T @ target_rotation)

        return {
            "position_m": {
                "x": float(position_delta[0]),
                "y": float(position_delta[1]),
                "z": float(position_delta[2]),
            },
            "position_norm_m": float(np.linalg.norm(position_delta)),
            "rotation_rotvec_rad": {
                "x": float(rotation_delta[0]),
                "y": float(rotation_delta[1]),
                "z": float(rotation_delta[2]),
            },
            "rotation_norm_rad": float(np.linalg.norm(rotation_delta)),
        }

    @staticmethod
    def xyz_string(msg: PoseStamped) -> str:
        return f"({msg.pose.position.x:.4f}, {msg.pose.position.y:.4f}, {msg.pose.position.z:.4f})"

    @staticmethod
    def quat_string(msg: PoseStamped) -> str:
        return (
            f"({msg.pose.orientation.x:.4f}, {msg.pose.orientation.y:.4f}, "
            f"{msg.pose.orientation.z:.4f}, {msg.pose.orientation.w:.4f})"
        )

    def publish_tcp_tf(self, pose: PoseStamped) -> None:
        tf_msg = TransformStamped()
        tf_msg.header = pose.header
        tf_msg.child_frame_id = self.tcp_frame
        tf_msg.transform.translation.x = pose.pose.position.x
        tf_msg.transform.translation.y = pose.pose.position.y
        tf_msg.transform.translation.z = pose.pose.position.z
        tf_msg.transform.rotation = pose.pose.orientation
        self.br.sendTransform(tf_msg)

    def publish_controller_control_tf(self, stamp: Any) -> None:
        if self.latest_pose is None:
            return

        quat = self.aligned_controller_quaternion()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.controller_control_frame
        tf_msg.transform.translation.x = self.latest_pose.pose.position.x
        tf_msg.transform.translation.y = self.latest_pose.pose.position.y
        tf_msg.transform.translation.z = self.latest_pose.pose.position.z
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.br.sendTransform(tf_msg)

    def publish_controller_attitude_tf(self, stamp: Any) -> None:
        if self.latest_pose is None:
            return

        quat = self.aligned_controller_quaternion()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.attitude_frame
        tf_msg.transform.translation.x = float(self.attitude_frame_position[0])
        tf_msg.transform.translation.y = float(self.attitude_frame_position[1])
        tf_msg.transform.translation.z = float(self.attitude_frame_position[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.br.sendTransform(tf_msg)

    def make_markers(self, stamp: Any) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(self.make_workspace_marker(stamp))
        markers.markers.append(self.make_sphere_marker(stamp))
        axis_colors = [(1.0, 0.05, 0.05), (0.05, 0.9, 0.1), (0.1, 0.3, 1.0)]
        for axis_index, color in enumerate(axis_colors):
            markers.markers.append(self.make_axis_marker(stamp, axis_index, color))
        return markers

    def make_workspace_marker(self, stamp: Any) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = "franka_sim_workspace"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        center = 0.5 * (self.workspace_min + self.workspace_max)
        size = self.workspace_max - self.workspace_min
        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(size[0])
        marker.scale.y = float(size[1])
        marker.scale.z = float(size[2])
        marker.color.r = 0.4
        marker.color.g = 0.4
        marker.color.b = 0.4
        marker.color.a = 0.08
        return marker

    def make_sphere_marker(self, stamp: Any) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = "franka_sim_tcp"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.tcp_position[0])
        marker.pose.position.y = float(self.tcp_position[1])
        marker.pose.position.z = float(self.tcp_position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.scale.z = 0.045
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 0.95
        return marker

    def make_axis_marker(self, stamp: Any, axis_index: int, rgb: tuple[float, float, float]) -> Marker:
        origin = self.tcp_position
        endpoint = origin + self.tcp_rotation[:, axis_index] * 0.16

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = "franka_sim_tcp_axes"
        marker.id = 10 + axis_index
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.points = [self.to_point(origin), self.to_point(endpoint)]
        marker.scale.x = 0.012
        marker.scale.y = 0.024
        marker.scale.z = 0.035
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.95
        return marker

    def is_trigger_pressed(self) -> bool:
        return self.button_bool(self.trigger_button_name) or self.trigger_value() >= self.trigger_threshold

    def trigger_value(self) -> float:
        value = self.latest_buttons.get(self.trigger_value_name, 0.0)
        if isinstance(value, list) and value:
            return float(value[0])
        if isinstance(value, tuple) and value:
            return float(value[0])
        if isinstance(value, (float, int)):
            return float(value)
        return 1.0 if self.button_bool(self.trigger_button_name) else 0.0

    def button_bool(self, name: str) -> bool:
        if not name:
            return False
        value = self.latest_buttons.get(name, False)
        if isinstance(value, (list, tuple)):
            if not value:
                return False
            value = value[0]
        if isinstance(value, bool):
            return value
        if isinstance(value, (float, int)):
            return float(value) >= self.trigger_threshold
        return bool(value)

    def current_controller_position(self) -> np.ndarray:
        if self.latest_pose is None:
            return np.zeros(3)
        return np.array(
            [
                self.latest_pose.pose.position.x,
                self.latest_pose.pose.position.y,
                self.latest_pose.pose.position.z,
            ],
            dtype=float,
        )

    def controller_delta_position_raw(self) -> tuple[float, float, float]:
        if self.anchor_controller_position is None or self.latest_pose is None:
            return 0.0, 0.0, 0.0

        delta = self.last_controller_delta_position_raw
        return float(delta[0]), float(delta[1]), float(delta[2])

    def controller_delta_position_control(self) -> tuple[float, float, float]:
        if self.anchor_controller_rotation is None:
            return 0.0, 0.0, 0.0

        delta_control = self.last_controller_delta_position_control
        return float(delta_control[0]), float(delta_control[1]), float(delta_control[2])

    def tcp_delta_body(self) -> tuple[float, float, float]:
        delta = self.last_tcp_delta_body
        return float(delta[0]), float(delta[1]), float(delta[2])

    def controller_delta_rotvec(self) -> tuple[float, float, float]:
        if self.anchor_controller_rotation is None or self.latest_pose is None:
            return 0.0, 0.0, 0.0

        rotvec = self.last_controller_delta_rotvec
        return float(rotvec[0]), float(rotvec[1]), float(rotvec[2])

    def clamped_rotation_vector(self, delta_rotation: np.ndarray) -> np.ndarray:
        rotvec = self.rotation_vector_from_matrix(delta_rotation)
        return np.array(
            [
                self.clamp(float(rotvec[0]), -self.max_controller_angle, self.max_controller_angle),
                self.clamp(float(rotvec[1]), -self.max_controller_angle, self.max_controller_angle),
                self.clamp(float(rotvec[2]), -self.max_controller_angle, self.max_controller_angle),
            ],
            dtype=float,
        )

    @staticmethod
    def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
        cos_angle = (np.trace(rotation) - 1.0) * 0.5
        cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
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
            return np.zeros(3)

        axis = skew_vector / (2.0 * sin_angle)
        return axis * angle

    @staticmethod
    def rotation_matrix_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
        angle = float(np.linalg.norm(rotvec))
        if angle < 1e-9:
            return np.eye(3)

        axis = rotvec / angle
        skew = np.array(
            [
                [0.0, -axis[2], axis[1]],
                [axis[2], 0.0, -axis[0]],
                [-axis[1], axis[0], 0.0],
            ],
            dtype=float,
        )
        return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)

    def current_controller_rotation(self) -> np.ndarray:
        return self.current_mapped_controller_rotation()

    def current_aligned_controller_rotation(self) -> np.ndarray:
        if (
            not self.controller_frame_alignment_active
            or self.previous_controller_rotation is None
        ):
            return self.current_mapped_controller_rotation()

        return self.tcp_rotation

    def current_mapped_controller_rotation(self) -> np.ndarray:
        if self.latest_pose is None:
            return np.eye(3)
        quat = [
            self.latest_pose.pose.orientation.x,
            self.latest_pose.pose.orientation.y,
            self.latest_pose.pose.orientation.z,
            self.latest_pose.pose.orientation.w,
        ]
        raw_rotation = quaternion_matrix(quat)[:3, :3]
        return raw_rotation @ self.controller_axis_map

    def load_axis_map(self) -> np.ndarray:
        return self.load_rotation_map("controller_axis_map")

    def load_rotation_map(self, parameter_name: str) -> np.ndarray:
        values = list(self.get_parameter(parameter_name).value)
        if len(values) != 9:
            raise ValueError(f"{parameter_name} must contain 9 row-major values")
        axis_map = np.array(values, dtype=float).reshape((3, 3))

        should_be_identity = axis_map.T @ axis_map
        if not np.allclose(should_be_identity, np.eye(3), atol=1e-6):
            raise ValueError(f"{parameter_name} must be an orthonormal rotation matrix")
        if np.linalg.det(axis_map) < 0.0:
            raise ValueError(f"{parameter_name} must be right-handed, determinant must be +1")
        return axis_map

    @staticmethod
    def apply_vector_deadband(vector: np.ndarray, threshold: float) -> np.ndarray:
        if threshold <= 0.0:
            return vector
        norm = float(np.linalg.norm(vector))
        if norm <= threshold:
            return np.zeros_like(vector)
        return vector * ((norm - threshold) / norm)

    @staticmethod
    def clamp_vector_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
        if maximum_norm <= 0.0:
            return vector
        norm = float(np.linalg.norm(vector))
        if norm <= maximum_norm or norm < 1e-12:
            return vector
        return vector * (maximum_norm / norm)

    def filtered_delta(
        self,
        current_delta: np.ndarray,
        previous_delta: np.ndarray,
        alpha: float | None = None,
    ) -> np.ndarray:
        if float(np.linalg.norm(current_delta)) < 1e-12:
            return np.zeros_like(current_delta)
        if alpha is None:
            alpha = self.delta_filter_alpha
        return alpha * current_delta + (1.0 - alpha) * previous_delta

    def load_optional_filter_alpha(self, parameter_name: str, fallback: float) -> float:
        configured = float(self.get_parameter(parameter_name).value)
        if configured < 0.0:
            return fallback
        return self.clamp(configured, 0.0, 1.0)

    def clamp_position(self, position: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(position, self.workspace_min), self.workspace_max)

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return min(max(value, minimum), maximum)

    @staticmethod
    def to_point(values: np.ndarray) -> Point:
        point = Point()
        point.x = float(values[0])
        point.y = float(values[1])
        point.z = float(values[2])
        return point


def main() -> None:
    rclpy.init()
    node = RightHandTeleopSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
