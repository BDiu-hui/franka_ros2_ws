"""Franky-side command senders for Cartesian teleoperation."""

from __future__ import annotations


class PoseFrankyControlMode:
    """Send short absolute Cartesian pose motions toward the current target."""

    name = "pose"

    def send(self, node, position, orientation) -> None:
        node._send_cartesian_motion(position, orientation)

    def stop(self, node) -> None:
        node._send_cartesian_pose_stop()


class VelocityFrankyControlMode:
    """Send short Cartesian velocity motions toward the current target."""

    name = "velocity"

    def send(self, node, position, orientation) -> None:
        del position, orientation
        node._send_cartesian_velocity(
            node.command_linear_velocity,
            node.command_angular_velocity,
        )

    def stop(self, node) -> None:
        node._send_cartesian_velocity_stop()


def make_franky_control_mode(name: str):
    normalized_name = str(name).strip().lower()
    modes = {
        PoseFrankyControlMode.name: PoseFrankyControlMode(),
        VelocityFrankyControlMode.name: VelocityFrankyControlMode(),
    }
    try:
        return modes[normalized_name]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported control_command_mode='{name}'. "
            f"Expected one of: {', '.join(sorted(modes))}"
        ) from exc
