import time
from pathlib import Path

import rclpy
from std_msgs.msg import Bool

import quest3_oculus_rviz.data_recorder_node as recorder_module
from unified_impedance_control.authority_data_recorder_node import (
    AuthorityGatedDataRecorderNode,
)


class FakeCameraManager:
    def __init__(self, **_kwargs) -> None:
        self.cameras = {}
        self.initialize_count = 0
        self.stop_count = 0

    def initialize_all(self, config) -> bool:
        self.initialize_count += 1
        self.cameras = {name: {} for name in config}
        return True

    def get_all_frames(self) -> dict:
        return {}

    def stop_all(self) -> None:
        self.stop_count += 1
        self.cameras = {}


def wait_for(predicate, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition did not become true")


def test_recorder_opens_and_closes_cameras_with_authority(monkeypatch) -> None:
    monkeypatch.setattr(recorder_module, "FishEyeManager", FakeCameraManager)
    config = (
        Path(__file__).resolve().parents[2]
        / "quest3_oculus_rviz/config/data_recorder.yaml"
    )
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    node = AuthorityGatedDataRecorderNode()
    try:
        manager = node.camera_manager
        assert manager.initialize_count == 0

        node._camera_authority_callback(Bool(data=True))
        wait_for(lambda: manager.initialize_count == 1)

        node._camera_authority_callback(Bool(data=False))
        wait_for(lambda: manager.stop_count == 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
