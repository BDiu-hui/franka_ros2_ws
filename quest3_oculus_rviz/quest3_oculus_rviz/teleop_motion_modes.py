"""Hand-side TCP target generators for Quest teleoperation."""

from __future__ import annotations

import numpy as np


class VelocityTeleopMotionMode:
    """Generate TCP targets from frame-to-frame controller increments."""

    name = "velocity"

    def update(self, node, now: float, dt: float) -> None:
        if (
            node.previous_controller_position is None
            or node.previous_controller_rotation is None
            or node.latest_pose is None
        ):
            return

        current_controller_position = node.current_controller_position()
        current_controller_rotation = node.current_controller_rotation()

        delta_raw = current_controller_position - node.previous_controller_position
        controller_delta_rotation = (
            node.previous_controller_rotation.T @ current_controller_rotation
        )
        controller_delta_rotvec = node.clamped_rotation_vector(controller_delta_rotation)
        node.last_controller_frame_delta_position_raw = delta_raw
        node.last_controller_frame_delta_rotvec = controller_delta_rotvec
        delta_control = node.previous_controller_rotation.T @ delta_raw
        signed_delta_control = delta_control * node.translation_axis_sign
        deadbanded_delta_control = node.apply_vector_deadband(
            signed_delta_control,
            node.translation_deadband,
        )
        tcp_base_position, tcp_base_rotation, tcp_base_source = node.current_tcp_base_pose(now)
        target_lead_scale = node.target_lead_scale(dt, tcp_base_source)
        tcp_delta_body_unfiltered = (
            deadbanded_delta_control * node.translation_scale * target_lead_scale
        )
        tcp_delta_body_clamped = node.clamp_vector_norm(
            tcp_delta_body_unfiltered,
            node.max_tcp_delta_body,
        )
        node.filtered_tcp_delta_body = node.filtered_delta(
            tcp_delta_body_clamped,
            node.filtered_tcp_delta_body,
        )
        tcp_delta_body = node.filtered_tcp_delta_body
        node.tcp_position = node.clamp_position(
            tcp_base_position + tcp_base_rotation @ tcp_delta_body
        )

        rx, ry, rz = controller_delta_rotvec
        tcp_delta_rotvec = np.array(
            [
                node.roll_sign * rx,
                node.pitch_sign * ry,
                node.yaw_sign * rz,
            ],
            dtype=float,
        )
        tcp_delta_rotvec = node.apply_vector_deadband(
            tcp_delta_rotvec,
            node.rotation_deadband,
        )
        tcp_delta_rotvec_unfiltered = (
            tcp_delta_rotvec * node.rotation_scale * target_lead_scale
        )
        tcp_delta_rotvec_clamped = node.clamp_vector_norm(
            tcp_delta_rotvec_unfiltered,
            node.max_tcp_delta_rotvec,
        )
        node.filtered_tcp_delta_rotvec = node.filtered_delta(
            tcp_delta_rotvec_clamped,
            node.filtered_tcp_delta_rotvec,
        )
        tcp_delta_rotation = node.rotation_matrix_from_rotvec(node.filtered_tcp_delta_rotvec)
        node.tcp_rotation = tcp_base_rotation @ tcp_delta_rotation

        node.last_controller_delta_position_raw = delta_raw
        node.last_controller_delta_position_control = delta_control
        node.last_controller_delta_rotvec = controller_delta_rotvec
        node.last_tcp_delta_body = tcp_delta_body
        node.last_tcp_delta_body_unfiltered = tcp_delta_body_unfiltered
        node.last_tcp_delta_rotvec = node.filtered_tcp_delta_rotvec
        node.last_tcp_delta_rotvec_unfiltered = tcp_delta_rotvec_unfiltered
        node.last_tcp_base_source = tcp_base_source
        node.last_target_lead_scale = target_lead_scale
        node.previous_controller_position = current_controller_position
        node.previous_controller_rotation = current_controller_rotation


class PositionTeleopMotionMode:
    """Generate TCP targets from controller displacement relative to the trigger anchor."""

    name = "position"

    def update(self, node, now: float, dt: float) -> None:
        del dt
        if (
            node.anchor_controller_position is None
            or node.anchor_controller_rotation is None
            or node.anchor_tcp_position is None
            or node.anchor_tcp_rotation is None
            or node.latest_pose is None
        ):
            return

        current_controller_position = node.current_controller_position()
        current_controller_rotation = node.current_controller_rotation()

        if node.previous_controller_position is None:
            frame_delta_position = np.zeros(3, dtype=float)
        else:
            frame_delta_position = current_controller_position - node.previous_controller_position
        if node.previous_controller_rotation is None:
            frame_delta_rotvec = np.zeros(3, dtype=float)
        else:
            frame_delta_rotation = node.previous_controller_rotation.T @ current_controller_rotation
            frame_delta_rotvec = node.rotation_vector_from_matrix(frame_delta_rotation)
        node.last_controller_frame_delta_position_raw = frame_delta_position
        node.last_controller_frame_delta_rotvec = frame_delta_rotvec

        if node.position_controller_is_stationary(
            frame_delta_position,
            frame_delta_rotvec,
            now,
        ):
            if not node.position_paused_for_stationary:
                node.reanchor_position_target(
                    current_controller_position,
                    current_controller_rotation,
                    now,
                )
                node.position_paused_for_stationary = True
            node.update_position_stationary_hold_state(
                current_controller_position,
                current_controller_rotation,
                now,
            )
            return

        if node.position_paused_for_stationary:
            if not node.position_controller_resume_requested(
                frame_delta_position,
                frame_delta_rotvec,
            ):
                node.update_position_stationary_hold_state(
                    current_controller_position,
                    current_controller_rotation,
                    now,
                )
                return

            node.position_paused_for_stationary = False
            node.position_stationary_since = None
            node.position_stationary_active = False
            node.reanchor_position_target(
                current_controller_position,
                current_controller_rotation,
                now,
            )
            node.update_position_stationary_hold_state(
                current_controller_position,
                current_controller_rotation,
                now,
            )
            return

        delta_raw = current_controller_position - node.anchor_controller_position
        delta_control = node.anchor_controller_rotation.T @ delta_raw
        signed_delta_control = delta_control * node.translation_axis_sign
        deadbanded_delta_control = node.apply_vector_deadband(
            signed_delta_control,
            node.translation_deadband,
        )
        tcp_delta_body_unfiltered = deadbanded_delta_control * node.translation_scale
        tcp_delta_body_clamped = node.clamp_vector_norm(
            tcp_delta_body_unfiltered,
            node.position_max_tcp_offset,
        )
        node.filtered_tcp_delta_body = node.filtered_delta(
            tcp_delta_body_clamped,
            node.filtered_tcp_delta_body,
        )
        tcp_delta_body = node.filtered_tcp_delta_body
        node.tcp_position = node.clamp_position(
            node.anchor_tcp_position + node.anchor_tcp_rotation @ tcp_delta_body
        )

        controller_delta_rotation = (
            node.anchor_controller_rotation.T @ current_controller_rotation
        )
        controller_delta_rotvec = node.clamped_rotation_vector(controller_delta_rotation)
        rx, ry, rz = controller_delta_rotvec
        tcp_delta_rotvec = np.array(
            [
                node.roll_sign * rx,
                node.pitch_sign * ry,
                node.yaw_sign * rz,
            ],
            dtype=float,
        )
        tcp_delta_rotvec = node.apply_vector_deadband(
            tcp_delta_rotvec,
            node.rotation_deadband,
        )
        tcp_delta_rotvec_unfiltered = tcp_delta_rotvec * node.rotation_scale
        tcp_delta_rotvec_clamped = node.clamp_vector_norm(
            tcp_delta_rotvec_unfiltered,
            node.position_max_tcp_rotvec,
        )
        node.filtered_tcp_delta_rotvec = node.filtered_delta(
            tcp_delta_rotvec_clamped,
            node.filtered_tcp_delta_rotvec,
        )
        tcp_delta_rotation = node.rotation_matrix_from_rotvec(node.filtered_tcp_delta_rotvec)
        node.tcp_rotation = node.anchor_tcp_rotation @ tcp_delta_rotation

        node.last_controller_delta_position_raw = delta_raw
        node.last_controller_delta_position_control = delta_control
        node.last_controller_delta_rotvec = controller_delta_rotvec
        node.last_tcp_delta_body = tcp_delta_body
        node.last_tcp_delta_body_unfiltered = tcp_delta_body_unfiltered
        node.last_tcp_delta_rotvec = node.filtered_tcp_delta_rotvec
        node.last_tcp_delta_rotvec_unfiltered = tcp_delta_rotvec_unfiltered
        node.last_tcp_base_source = node.anchor_tcp_base_source
        node.last_target_lead_scale = 1.0
        node.previous_controller_position = current_controller_position
        node.previous_controller_rotation = current_controller_rotation


def make_teleop_motion_mode(name: str):
    normalized_name = str(name).strip().lower()
    modes = {
        VelocityTeleopMotionMode.name: VelocityTeleopMotionMode(),
        PositionTeleopMotionMode.name: PositionTeleopMotionMode(),
    }
    try:
        return modes[normalized_name]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported teleop_motion_mode='{name}'. "
            f"Expected one of: {', '.join(sorted(modes))}"
        ) from exc
