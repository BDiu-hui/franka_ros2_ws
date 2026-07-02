"""Fish-eye (V4L2/OpenCV) camera helper used by the data recorder node.

Adapted from easy_dp's data_collection/robot/camera/fish_eye.py so this package
has no runtime dependency on the easy_dp repo. Cameras are USB UVC devices
matched by ``usb_path``/``serial``/``stream_index`` rather than by RealSense
serial. ``FishEyeManager`` mirrors the old ``RealSenseManager`` interface
(``initialize_all`` / ``get_all_frames`` / ``stop_all`` / ``list_cameras``).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class V4L2Device:
    device_path: Path
    name: str
    serial: str | None
    usb_path: str
    stream_index: int
    vendor_id: str | None
    product_id: str | None

    @property
    def stable_id(self) -> str:
        usb_id = f"{self.vendor_id or 'unknown'}:{self.product_id or 'unknown'}"
        serial = self.serial or "no-serial"
        return f"usb-{usb_id}-{serial}@{self.usb_path}-stream{self.stream_index}"


def _read_optional_text(path: Path) -> str | None:
    try:
        return path.read_text().strip()
    except (FileNotFoundError, OSError):
        return None


def _video_node_sort_key(path: Path) -> tuple[int, str]:
    suffix = path.name.removeprefix("video")
    return (int(suffix), path.name) if suffix.isdigit() else (2**31 - 1, path.name)


def discover_v4l2_devices(
    sys_class_root: Path = Path("/sys/class/video4linux"),
    dev_root: Path = Path("/dev"),
) -> list[V4L2Device]:
    devices = []
    for video_node in sorted(sys_class_root.glob("video*"), key=_video_node_sort_key):
        interface_dir = (video_node / "device").resolve()
        usb_device_dir = interface_dir.parent
        stream_index_text = _read_optional_text(video_node / "index")
        if stream_index_text is None:
            continue
        try:
            stream_index = int(stream_index_text)
        except ValueError:
            continue

        devices.append(
            V4L2Device(
                device_path=dev_root / video_node.name,
                name=_read_optional_text(video_node / "name") or video_node.name,
                serial=_read_optional_text(usb_device_dir / "serial"),
                usb_path=usb_device_dir.name,
                stream_index=stream_index,
                vendor_id=_read_optional_text(usb_device_dir / "idVendor"),
                product_id=_read_optional_text(usb_device_dir / "idProduct"),
            )
        )
    return devices


def _format_devices(devices: Sequence[V4L2Device]) -> str:
    if not devices:
        return "<none>"
    return ", ".join(
        f"{device.device_path} ({device.name}, {device.stable_id})"
        for device in devices
    )


def _cfg_get(cfg: Any, key: str, default: Any = None) -> Any:
    if cfg is None:
        return default
    if isinstance(cfg, Mapping):
        return cfg.get(key, default)
    if hasattr(cfg, key):
        return getattr(cfg, key)
    return default


def resolve_v4l2_device(
    camera_name: str,
    device_cfg: Any,
    detected_devices: Sequence[V4L2Device],
) -> V4L2Device:
    serial = _cfg_get(device_cfg, "serial")
    usb_path = _cfg_get(device_cfg, "usb_path")
    stream_index = int(_cfg_get(device_cfg, "stream_index", 0))
    if serial is None and usb_path is None:
        raise KeyError(
            f"fish_eye camera {camera_name!r} requires 'serial' or 'usb_path'"
        )

    matches = list(detected_devices)
    if serial is not None:
        matches = [device for device in matches if device.serial == str(serial)]
    if usb_path is not None:
        matches = [device for device in matches if device.usb_path == str(usb_path)]
    matches = [device for device in matches if device.stream_index == stream_index]

    selector = (
        f"serial={serial!r}, usb_path={usb_path!r}, stream_index={stream_index!r}"
    )
    if not matches:
        raise RuntimeError(
            f"cannot find fish_eye camera {camera_name!r} matching {selector}; "
            f"detected devices: {_format_devices(detected_devices)}"
        )
    if len(matches) > 1:
        raise RuntimeError(
            f"fish_eye camera {camera_name!r} selector is ambiguous ({selector}); "
            f"matches: {_format_devices(matches)}"
        )
    return matches[0]


def fourcc_to_str(fourcc: int | float) -> str:
    return "".join(chr((int(fourcc) >> (8 * i)) & 0xFF) for i in range(4))


def normalize_opencv_config(cfg: Any) -> dict[str, Any]:
    if not isinstance(cfg, Mapping) and not hasattr(cfg, "items"):
        raise TypeError("fish_eye camera config must be a mapping")
    normalized = {str(name): value for name, value in cfg.items()}
    if not normalized:
        raise ValueError("fish_eye camera config must contain at least one camera")
    return normalized


class OpenCVCamera:
    """Wrapper class for a single V4L2/UVC camera."""

    def __init__(
        self,
        device: V4L2Device,
        width: int,
        height: int,
        fps: int,
        fourcc: str,
    ) -> None:
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.capture: cv2.VideoCapture | None = None
        self.connected = False

    def initialize(self) -> None:
        capture = cv2.VideoCapture(str(self.device.device_path), cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                f"failed to open fish_eye camera {self.device.device_path} "
                f"({self.device.stable_id})"
            )

        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (actual_width, actual_height) != (self.width, self.height):
            capture.release()
            raise RuntimeError(
                f"camera {self.device.stable_id} rejected resolution "
                f"{self.width}x{self.height}; actual {actual_width}x{actual_height}"
            )

        self.capture = capture
        self.connected = True
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        actual_fourcc = fourcc_to_str(capture.get(cv2.CAP_PROP_FOURCC))
        print(
            f"Opened {self.device.device_path}: {self.device.stable_id}, "
            f"{actual_width}x{actual_height}, fps={actual_fps:.2f}, "
            f"fourcc={actual_fourcc}"
        )

    def get_frame(self) -> np.ndarray | None:
        if not self.connected or self.capture is None:
            return None
        ok, frame = self.capture.read()
        return frame if ok else None

    def stop(self) -> None:
        if self.capture is not None:
            self.capture.release()
        self.capture = None
        self.connected = False


class FishEyeManager:
    """Manager class for multiple V4L2/UVC fish-eye cameras."""

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        fourcc: str = "MJPG",
        warmup_frames: int = 10,
    ) -> None:
        if len(fourcc) != 4:
            raise ValueError("fish_eye camera fourcc must contain exactly 4 characters")
        if warmup_frames < 0:
            raise ValueError("fish_eye camera warmup_frames must be non-negative")
        self.width = width
        self.height = height
        self.fps = fps
        self.fourcc = fourcc
        self.warmup_frames = warmup_frames
        self.cameras: dict[str, dict[str, Any]] = {}

    def initialize_all(self, cfg: Any = None) -> bool:
        device_configs = normalize_opencv_config(cfg)
        detected_devices = discover_v4l2_devices()
        self.stop_all()
        self.cameras = {}

        try:
            for camera_name, device_cfg in device_configs.items():
                device = resolve_v4l2_device(camera_name, device_cfg, detected_devices)
                camera = OpenCVCamera(
                    device=device,
                    width=self.width,
                    height=self.height,
                    fps=self.fps,
                    fourcc=self.fourcc,
                )
                camera.initialize()
                self.cameras[camera_name] = {
                    "name": device.name,
                    "serial_number": device.stable_id,
                    "camera": camera,
                }
                print(
                    f"Mapped fish_eye camera {camera_name!r} -> "
                    f"{device.device_path}, usb_path={device.usb_path}, "
                    f"stream_index={device.stream_index}, serial={device.serial!r}"
                )

            for _ in range(self.warmup_frames):
                frames = self.get_all_frames()
                if len(frames) != len(self.cameras):
                    missing = sorted(set(self.cameras) - set(frames))
                    raise RuntimeError(
                        f"failed to warm up fish_eye cameras: missing frames from {missing}"
                    )
        except Exception:
            self.stop_all()
            raise

        print(f"Successfully initialized {len(self.cameras)} fish_eye camera(s)")
        return bool(self.cameras)

    def get_all_frames(self) -> dict[str, np.ndarray]:
        frames: dict[str, np.ndarray] = {}
        for camera_name, info in self.cameras.items():
            frame = info["camera"].get_frame()
            if frame is not None:
                frames[camera_name] = frame
        return frames

    def stop_all(self) -> None:
        for info in self.cameras.values():
            info["camera"].stop()

    def list_cameras(self) -> int:
        devices = discover_v4l2_devices()
        print(f"Detected {len(devices)} V4L2 video node(s):")
        for device in devices:
            print(
                f"  Device: {device.device_path}, Name: {device.name}, "
                f"usb_path={device.usb_path}, stream_index={device.stream_index}, "
                f"serial={device.serial!r}, stable_id={device.stable_id}"
            )
        return len(devices)
