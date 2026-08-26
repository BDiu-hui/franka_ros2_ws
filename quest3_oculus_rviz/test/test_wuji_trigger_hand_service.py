from __future__ import annotations

import json
import time
from pathlib import Path
from types import SimpleNamespace
from urllib.request import Request, urlopen

import pytest
import rclpy

from quest3_oculus_rviz.wuji_trigger_hand_node import (
    Ros2CommandHand,
    WujiTriggerHandNode,
)


class _RecordingHand:
    def __init__(self) -> None:
        self.enabled: list[bool] = []
        self.targets: list[list[list[float]]] = []

    def disable_thread_safe_check(self) -> None:
        pass

    def write_joint_enabled(self, enabled: bool) -> None:
        self.enabled.append(enabled)

    def write_joint_target_position_unchecked(
        self,
        positions: list[list[float]],
    ) -> None:
        self.targets.append([list(finger) for finger in positions])

    def read_joint_actual_position(self, timeout_sec: float) -> list[list[float]]:
        del timeout_sec
        return [[0.0] * 4 for _ in range(5)]


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
            "control_backend:=sdk",
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

        actual = _get_json(
            f"{node.command_server.url}/hands/left/actual_joint_positions"
        )
        assert actual["positions"] == [0.0] * 20
        assert actual["timestamp_monotonic_ns"] > 0
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

        requests: list[str] = []
        worker = node.workers["right"]
        worker.request_pose = lambda name, _pose: requests.append(name)
        high = SimpleNamespace(data=json.dumps({"rightTrig": 1.0}))
        low = SimpleNamespace(data=json.dumps({"rightTrig": 0.0}))

        node.buttons_callback(high)
        node.buttons_callback(low)
        node.buttons_callback(high)
        node.buttons_callback(low)
        node.buttons_callback(high)
        node.last_buttons_time = time.monotonic() - 1.0
        node.watchdog_callback()

        assert requests == [
            "close_type3",
            "released_toggle",
            "close_type3",
            "released_timeout",
        ]
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_default_mode_releases_on_startup_and_shutdown(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config_path = (
        Path(__file__).resolve().parents[1] / "config" / "wuji_trigger_hand.yaml"
    )
    hand = _RecordingHand()
    monkeypatch.setattr(
        WujiTriggerHandNode,
        "_connect_hand",
        lambda _node, _config: hand,
    )
    rclpy.init(
        args=[
            "--ros-args",
            "--params-file",
            str(config_path),
            "-p",
            "control_mode:=service",
            "-p",
            "publish_joint_states:=false",
            "-p",
            "command_server_port:=0",
            "-p",
            "shutdown_release_hold_sec:=0.0",
        ]
    )
    node = WujiTriggerHandNode()
    try:
        assert len(hand.targets) == 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    assert len(hand.targets) == 2
    assert hand.enabled == [True, False]


def test_keep_close_locks_closed_pose_and_suppresses_teleop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config_path = (
        Path(__file__).resolve().parents[1] / "config" / "wuji_trigger_hand.yaml"
    )
    hand = _RecordingHand()
    monkeypatch.setattr(
        WujiTriggerHandNode,
        "_connect_hand",
        lambda _node, _config: hand,
    )
    rclpy.init(
        args=[
            "--ros-args",
            "--params-file",
            str(config_path),
            "-p",
            "keep_close:=true",
            "-p",
            "publish_joint_states:=false",
            "-p",
            "trajectory_duration_sec:=0.05",
            "-p",
            "trajectory_rate_hz:=100.0",
            "-p",
            "shutdown_release_hold_sec:=0.0",
        ]
    )
    node = WujiTriggerHandNode()
    try:
        worker = node.workers["right"]
        deadline = time.monotonic() + 1.0
        while worker.has_pending_or_active_command() and time.monotonic() < deadline:
            time.sleep(0.01)

        assert not worker.has_pending_or_active_command()
        closed_pose = worker.config.closed_pose
        assert hand.targets
        assert all(target == closed_pose for target in hand.targets)
        assert node.hand_closed["right"] is True
        assert node.buttons_sub is None
        assert node.watchdog_timer is None
        assert node.command_server is None

        target_count = len(hand.targets)
        node.buttons_callback(SimpleNamespace(data=json.dumps({"rightTrig": 1.0})))
        node.last_buttons_time = time.monotonic() - 1.0
        node.watchdog_callback()

        assert len(hand.targets) == target_count
        assert node.hand_closed["right"] is True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    assert all(target == closed_pose for target in hand.targets)
    assert hand.enabled == [True, False]


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
