"""RealSense camera helper used by the data recorder node.

Adapted from easy_dp's data_collection/robot/camera/realsense.py so this
package has no runtime dependency on the easy_dp repo. Imports pyrealsense2
lazily so the module can be loaded for type/structure checks even when the
library is missing (pip install pyrealsense2).
"""

from __future__ import annotations

import numpy as np


REALSENSE_CONFIG = {
    "back": "419122270393",
    "front": "419122270479",
}


class RealSenseCamera:
    """Wrapper class for a single RealSense camera."""

    def __init__(self, serial_number: str, width: int = 640, height: int = 480, fps: int = 30) -> None:
        self.serial_number = serial_number
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.connected = False

    def initialize(self) -> bool:
        import pyrealsense2 as rs

        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            self.config.enable_device(self.serial_number)
            self.config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                self.fps,
            )
            self.pipeline.start(self.config)
            self.connected = True
            return True
        except Exception as e:
            print(f"Failed to initialize camera {self.serial_number}: {e}")
            self.connected = False
            return False

    def get_frame(self) -> np.ndarray | None:
        if not self.connected or not self.pipeline:
            return None
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None
            return np.asanyarray(color_frame.get_data())
        except Exception as e:
            print(f"Failed to get frame from camera {self.serial_number}: {e}")
            return None

    def stop(self) -> None:
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.connected = False


class RealSenseManager:
    """Manager class for multiple RealSense cameras."""

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.cameras: dict[str, dict] = {}

    def initialize_all(self, cfg: dict[str, str] | None = None) -> bool:
        import pyrealsense2 as rs

        cfg = cfg if cfg is not None else REALSENSE_CONFIG
        retry = True
        force_reset = False
        success_count = 0

        while retry:
            retry = False
            self.stop_all()
            self.cameras.clear()

            ctx = rs.context()
            devices = list(ctx.query_devices())

            if force_reset:
                print("realsense force reset True")
                for device in devices:
                    try:
                        device.hardware_reset()
                    except Exception as e:
                        print(f"hardware_reset failed: {e}")

            missing = False
            for name, serial_number in cfg.items():
                d = None
                for device in devices:
                    if device.get_info(rs.camera_info.serial_number) == serial_number:
                        d = device
                        break
                if d is None:
                    print(f"cannot find realsense {name} {serial_number}")
                    missing = True
                    continue
                self.cameras[name] = {
                    "name": d.get_info(rs.camera_info.name),
                    "serial_number": serial_number,
                    "camera": RealSenseCamera(serial_number, self.width, self.height, self.fps),
                }

            if missing:
                return False

            success_count = 0
            for name, info in self.cameras.items():
                if info["camera"].initialize():
                    success_count += 1

            print(f"Successfully initialized {success_count}/{len(self.cameras)} cameras")

            init_frame = 0
            while init_frame < 10:
                init_frame += 1
                print(f"warm up get frames {init_frame}")
                data = self.get_all_frames()
                if len(data) != len(self.cameras):
                    retry = True
                    break
            force_reset = not force_reset

        return success_count > 0

    def get_all_frames(self) -> dict[str, np.ndarray]:
        frames: dict[str, np.ndarray] = {}
        for name, info in self.cameras.items():
            if info["camera"].connected:
                frame = info["camera"].get_frame()
                if frame is not None:
                    frames[name] = frame
        return frames

    def stop_all(self) -> None:
        for name, info in self.cameras.items():
            info["camera"].stop()
        if self.cameras:
            print("All cameras stopped.")

    def list_cameras(self) -> int:
        import pyrealsense2 as rs

        ctx = rs.context()
        devices = list(ctx.query_devices())
        print(f"Detected {len(devices)} RealSense camera(s):")
        for device in devices:
            serial_number = device.get_info(rs.camera_info.serial_number)
            device_name = device.get_info(rs.camera_info.name)
            print(f"  Serial: {serial_number}, Name: {device_name}")
        return len(devices)
