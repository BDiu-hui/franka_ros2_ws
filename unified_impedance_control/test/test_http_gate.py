import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import rclpy

from unified_impedance_control.control_authority_node import ControlAuthorityNode


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
        assert arm["ok"] is False
        assert hand["ok"] is False
    finally:
        node.destroy_node()
        rclpy.shutdown()

