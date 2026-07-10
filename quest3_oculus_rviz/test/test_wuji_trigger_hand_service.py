from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from urllib.request import Request, urlopen

import pytest
import rclpy

from quest3_oculus_rviz.wuji_trigger_hand_node import (
    Ros2CommandHand,
    WujiTriggerHandNode,
)


def _get_json(url: str) -> dict:
    with urlopen(url, timeout=1.0) as response:
        return json.loads(response.read())


def _post_json(url: str, payload: dict) -> dict:
    request = Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with urlopen(request, timeout=1.0) as response:
        return json.loads(response.read())


def test_service_mode_controls_two_dry_run_hands() -> None:
    config_path = Path(__file__).resolve().parents[1] / "config" / "wuji_trigger_hand.yaml"
    rclpy.init(
        args=[
            "--ros-args",
            "--params-file",
            str(config_path),
            "-p",
            "dry_run:=true",
            "-p",
            "control_mode:=service",
            "-p",
            "left_enabled:=true",
            "-p",
            "right_enabled:=true",
            "-p",
            "release_on_startup:=false",
            "-p",
            "publish_joint_states:=false",
            "-p",
            "command_server_port:=0",
        ]
    )
    node = WujiTriggerHandNode()
    try:
        assert node.buttons_sub is None
        assert node.watchdog_timer is None
        assert node.command_server is not None
        assert _get_json(f"{node.command_server.url}/health") == {
            "ok": True,
            "hands": ["left", "right"],
        }

        positions = [float(index) / 10.0 for index in range(20)]
        result = _post_json(
            f"{node.command_server.url}/hands/left/joint_targets",
            {"positions": positions},
        )
        assert result == {"ok": True, "side": "left"}
        assert node.workers["left"].last_target_positions() == positions
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_trigger_mode_keeps_quest_subscription_and_watchdog() -> None:
    config_path = Path(__file__).resolve().parents[1] / "config" / "wuji_trigger_hand.yaml"
    rclpy.init(
        args=[
            "--ros-args",
            "--params-file",
            str(config_path),
            "-p",
            "dry_run:=true",
            "-p",
            "release_on_startup:=false",
            "-p",
            "publish_joint_states:=false",
        ]
    )
    node = WujiTriggerHandNode()
    try:
        assert node.buttons_sub is not None
        assert node.watchdog_timer is not None
        assert node.command_server is None
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_ros2_hand_readiness_requires_driver_subscription() -> None:
    hand = object.__new__(Ros2CommandHand)
    hand._publisher = type(
        "Publisher",
        (),
        {"get_subscription_count": lambda self: 0},
    )()
    assert hand.has_command_subscriber() is False

    hand._publisher = type(
        "Publisher",
        (),
        {"get_subscription_count": lambda self: 1},
    )()
    assert hand.has_command_subscriber() is True


def test_service_command_rejects_unsubscribed_ros2_driver() -> None:
    hand = object.__new__(Ros2CommandHand)
    hand._publisher = type(
        "Publisher",
        (),
        {"get_subscription_count": lambda self: 0},
    )()
    node = object.__new__(WujiTriggerHandNode)
    node.workers = {"left": SimpleNamespace(hand=hand)}

    with pytest.raises(RuntimeError, match="driver is not subscribed"):
        node._write_service_joint_targets("left", [0.0] * 20)
