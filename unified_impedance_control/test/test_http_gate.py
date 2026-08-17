import json
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from serl_franka_controllers_ros2.msg import CartesianImpedanceCommand
from std_msgs.msg import Bool

from unified_impedance_control.control_authority_node import ControlAuthorityNode


class _Messages:
    def __init__(self) -> None:
        self.values = []

    def publish(self, msg) -> None:
        self.values.append(msg)


def _post_expect_conflict(url: str, payload: dict) -> dict:
    request = Request(
        url,
        data=json.dumps(payload).encode(),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    try:
        urlopen(request, timeout=1.0)
    except HTTPError as exc:
        assert exc.code == 409
        return json.loads(exc.read())
    raise AssertionError("request was not rejected")


def _get_json(url: str) -> dict:
    with urlopen(url, timeout=1.0) as response:
        return json.loads(response.read())


def test_teleop_authority_rejects_policy_arm_and_wuji_commands() -> None:
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "left_gateway_port:=0",
            "-p", "right_gateway_port:=0",
            "-p", "wuji_gateway_port:=0",
        ]
    )
    node = ControlAuthorityNode()
    try:
        node.state.teleop_active = True
        left_port = node._servers[0].server_address[1]  # pylint: disable=protected-access
        wuji_port = node._servers[2].server_address[1]  # pylint: disable=protected-access
        arm = _post_expect_conflict(
            f"http://127.0.0.1:{left_port}/pose",
            {"arr": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
        )
        hand = _post_expect_conflict(
            f"http://127.0.0.1:{wuji_port}/hands/left/joint_targets",
            {"positions": [0.0] * 20},
        )
        actual_msg = JointState()
        actual_msg.position = [float(index) / 10.0 for index in range(20)]
        node._cache_actual_hand_state("left", actual_msg)  # pylint: disable=protected-access
        actual = _get_json(
            f"http://127.0.0.1:{wuji_port}/hands/left/actual_joint_positions"
        )
        assert arm["ok"] is False
        assert hand["ok"] is False
        assert actual["positions"] == list(actual_msg.position)
        assert actual["age_ms"] >= 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_teleop_handoff_holds_current_pose_and_drops_stale_targets() -> None:
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "left_gateway_port:=0",
            "-p", "right_gateway_port:=0",
            "-p", "wuji_gateway_port:=0",
        ]
    )
    node = ControlAuthorityNode()
    try:
        poses = _Messages()
        enabled = _Messages()
        node.pose_publishers["left"] = poses
        node.teleop_enabled_publishers["left"] = enabled

        current = PoseStamped()
        current.pose.position.x = 0.42
        current.pose.orientation.w = 1.0
        node._cache_current_arm_pose("left", current)  # pylint: disable=protected-access
        node.state.teleop_active = True
        node._begin_teleop_handoff()  # pylint: disable=protected-access
        assert poses.values[-1].pose.position.x == 0.42

        stale = CartesianImpedanceCommand()
        stale.header.stamp = node.get_clock().now().to_msg()
        stale.pose.position.x = 9.0
        node._relay_teleop_pose("left", stale)  # pylint: disable=protected-access
        assert len(poses.values) == 1

        node._relay_teleop_enabled("left", Bool(data=True))  # pylint: disable=protected-access
        assert node._handoff_pending["left"]  # pylint: disable=protected-access
        node._relay_teleop_enabled("left", Bool(data=False))  # pylint: disable=protected-access
        node._relay_teleop_enabled("left", Bool(data=True))  # pylint: disable=protected-access
        fresh = CartesianImpedanceCommand()
        fresh.header.stamp.sec = stale.header.stamp.sec + 1
        fresh.pose.position.x = 0.43
        node._relay_teleop_pose("left", fresh)  # pylint: disable=protected-access
        assert [msg.pose.position.x for msg in poses.values] == [0.42, 0.43]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_authority_voice_prompts_use_english_phrases() -> None:
    rclpy.init(
        args=[
            "--ros-args",
            "-p", "left_gateway_port:=0",
            "-p", "right_gateway_port:=0",
            "-p", "wuji_gateway_port:=0",
        ]
    )
    node = ControlAuthorityNode()
    try:
        with patch(
            "unified_impedance_control.control_authority_node.subprocess.Popen"
        ) as popen:
            node._speak_authority("teleop")  # pylint: disable=protected-access
            node._speak_authority("inference")  # pylint: disable=protected-access
        assert [item.args[0] for item in popen.call_args_list] == [
            ["/usr/bin/spd-say", "Teleoperation control"],
            ["/usr/bin/spd-say", "Inference control"],
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()
