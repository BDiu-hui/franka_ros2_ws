from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


try:
    import tkinter as tk
except ImportError as exc:  # pragma: no cover - depends on system packages
    tk = None
    _TK_IMPORT_ERROR = exc
else:
    _TK_IMPORT_ERROR = None


@dataclass
class SeriesSample:
    stamp: float
    values: tuple[float, float, float]


class TeleopRealtimePlotNode(Node):
    """Collect controller and TCP positions for a small Tk 3D trajectory view."""

    def __init__(self) -> None:
        super().__init__("teleop_realtime_plot")

        self.declare_parameter("controller_pose_topic", "quest3/right_controller/pose")
        self.declare_parameter("tcp_pose_topic", "/franka_franky/current_pose")
        self.declare_parameter("enabled_topic", "/quest3/right_teleop/enabled")
        self.declare_parameter("window_sec", 20.0)
        self.declare_parameter("max_points", 2500)
        self.declare_parameter("plot_update_hz", 20.0)
        self.declare_parameter("relative_to_first_sample", True)
        self.declare_parameter("record_controller_only_when_enabled", True)
        self.declare_parameter("reset_controller_trace_on_enable", True)
        self.declare_parameter("controller_label", "Quest controller")
        self.declare_parameter("tcp_label", "Franka TCP")
        self.declare_parameter("view_yaw_deg", -45.0)
        self.declare_parameter("view_pitch_deg", 32.0)
        self.declare_parameter("auto_scale_margin", 1.25)

        self.controller_pose_topic = str(self.get_parameter("controller_pose_topic").value)
        self.tcp_pose_topic = str(self.get_parameter("tcp_pose_topic").value)
        self.enabled_topic = str(self.get_parameter("enabled_topic").value)
        self.window_sec = max(float(self.get_parameter("window_sec").value), 1.0)
        self.max_points = max(int(self.get_parameter("max_points").value), 100)
        self.plot_update_hz = max(float(self.get_parameter("plot_update_hz").value), 1.0)
        self.relative_to_first_sample = bool(
            self.get_parameter("relative_to_first_sample").value
        )
        self.record_controller_only_when_enabled = bool(
            self.get_parameter("record_controller_only_when_enabled").value
        )
        self.reset_controller_trace_on_enable = bool(
            self.get_parameter("reset_controller_trace_on_enable").value
        )
        self.controller_label = str(self.get_parameter("controller_label").value)
        self.tcp_label = str(self.get_parameter("tcp_label").value)
        self.view_yaw_deg = float(self.get_parameter("view_yaw_deg").value)
        self.view_pitch_deg = float(self.get_parameter("view_pitch_deg").value)
        self.auto_scale_margin = max(
            float(self.get_parameter("auto_scale_margin").value), 1.0
        )

        self.controller_samples: Deque[SeriesSample] = deque(maxlen=self.max_points)
        self.tcp_samples: Deque[SeriesSample] = deque(maxlen=self.max_points)
        self.controller_origin: tuple[float, float, float] | None = None
        self.tcp_origin: tuple[float, float, float] | None = None
        self.teleop_enabled = False
        self.reset_controller_on_next_sample = False
        self.start_time = time.monotonic()

        self.create_subscription(
            PoseStamped,
            self.controller_pose_topic,
            self.controller_pose_callback,
            20,
        )
        self.create_subscription(PoseStamped, self.tcp_pose_topic, self.tcp_pose_callback, 20)
        self.create_subscription(Bool, self.enabled_topic, self.enabled_callback, 10)

        self.get_logger().info(
            "Realtime teleop plot ready. "
            f"controller={self.controller_pose_topic} tcp={self.tcp_pose_topic} "
            f"enabled={self.enabled_topic}"
        )

    def enabled_callback(self, msg: Bool) -> None:
        was_enabled = self.teleop_enabled
        self.teleop_enabled = bool(msg.data)
        if (
            self.teleop_enabled
            and not was_enabled
            and self.reset_controller_trace_on_enable
        ):
            self.reset_controller_on_next_sample = True

    def controller_pose_callback(self, msg: PoseStamped) -> None:
        if self.record_controller_only_when_enabled and not self.teleop_enabled:
            return
        position = self.pose_position(msg)
        if self.reset_controller_on_next_sample:
            self.controller_origin = None
            self.controller_samples.clear()
            self.reset_controller_on_next_sample = False
        if self.controller_origin is None:
            self.controller_origin = position
        self.controller_samples.append(
            SeriesSample(time.monotonic(), self.relative_position(position, self.controller_origin))
        )

    def tcp_pose_callback(self, msg: PoseStamped) -> None:
        position = self.pose_position(msg)
        if self.tcp_origin is None:
            self.tcp_origin = position
        self.tcp_samples.append(
            SeriesSample(time.monotonic(), self.relative_position(position, self.tcp_origin))
        )

    def reset_origins(self) -> None:
        self.controller_origin = None
        self.tcp_origin = None
        self.controller_samples.clear()
        self.tcp_samples.clear()
        self.start_time = time.monotonic()

    def trim_old_samples(self) -> None:
        cutoff = time.monotonic() - self.window_sec
        while self.controller_samples and self.controller_samples[0].stamp < cutoff:
            self.controller_samples.popleft()
        while self.tcp_samples and self.tcp_samples[0].stamp < cutoff:
            self.tcp_samples.popleft()

    @staticmethod
    def pose_position(msg: PoseStamped) -> tuple[float, float, float]:
        return (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )

    def relative_position(
        self,
        position: tuple[float, float, float],
        origin: tuple[float, float, float] | None,
    ) -> tuple[float, float, float]:
        if not self.relative_to_first_sample or origin is None:
            return position
        return (
            position[0] - origin[0],
            position[1] - origin[1],
            position[2] - origin[2],
        )


class TeleopPlotWindow:
    AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")
    AXIS_NAMES = ("X", "Y", "Z")
    CONTROLLER_PATH_COLOR = "#f59e0b"
    TCP_PATH_COLOR = "#38bdf8"

    def __init__(self, node: TeleopRealtimePlotNode) -> None:
        if tk is None:
            raise RuntimeError(
                "python3 tkinter is not available. Install python3-tk to use the realtime plot."
            ) from _TK_IMPORT_ERROR

        self.node = node
        self.root = tk.Tk()
        self.root.title("Quest3 / Franka TCP realtime 3D trajectory")
        self.root.geometry("1180x760")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(self.root, bg="#111418", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        toolbar = tk.Frame(self.root)
        toolbar.pack(fill=tk.X)
        reset_button = tk.Button(toolbar, text="Reset origin", command=self.node.reset_origins)
        reset_button.pack(side=tk.LEFT, padx=8, pady=6)
        self.status_var = tk.StringVar()
        tk.Label(toolbar, textvariable=self.status_var, anchor="w").pack(
            side=tk.LEFT, fill=tk.X, expand=True
        )

        self.running = True
        self.update_period_ms = max(int(1000.0 / self.node.plot_update_hz), 20)

    def close(self) -> None:
        self.running = False
        self.root.quit()

    def run(self) -> None:
        self.root.after(self.update_period_ms, self.tick)
        self.root.mainloop()

    def tick(self) -> None:
        if not self.running:
            return
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.trim_old_samples()
        self.draw()
        self.root.after(self.update_period_ms, self.tick)

    def draw(self) -> None:
        width = max(int(self.canvas.winfo_width()), 800)
        height = max(int(self.canvas.winfo_height()), 520)
        self.canvas.delete("all")

        samples_controller = list(self.node.controller_samples)
        samples_tcp = list(self.node.tcp_samples)
        spatial_limit = self.compute_spatial_limit(samples_controller + samples_tcp)

        margin = 32
        top = 46
        gap = 24
        bottom = 26
        if width >= 980:
            panel_width = (width - margin * 2 - gap) // 2
            panel_height = height - top - bottom
            controller_box = (margin, top, margin + panel_width, top + panel_height)
            tcp_left = margin + panel_width + gap
            tcp_box = (tcp_left, top, tcp_left + panel_width, top + panel_height)
        else:
            panel_width = width - margin * 2
            panel_height = (height - top - bottom - gap) // 2
            controller_box = (margin, top, margin + panel_width, top + panel_height)
            tcp_top = top + panel_height + gap
            tcp_box = (margin, tcp_top, margin + panel_width, tcp_top + panel_height)

        self.draw_space_panel(
            controller_box,
            self.node.controller_label,
            samples_controller,
            self.CONTROLLER_PATH_COLOR,
            spatial_limit,
        )
        self.draw_space_panel(
            tcp_box,
            self.node.tcp_label,
            samples_tcp,
            self.TCP_PATH_COLOR,
            spatial_limit,
        )
        self.draw_legend(width)

        self.status_var.set(
            f"{self.node.controller_pose_topic}: {len(self.node.controller_samples)} samples    "
            f"{self.node.tcp_pose_topic}: {len(self.node.tcp_samples)} samples    "
            f"trigger={'pressed' if self.node.teleop_enabled else 'released'}    "
            f"window={self.node.window_sec:.1f}s    range=+/-{spatial_limit:.3f}m"
        )

    def draw_space_panel(
        self,
        box: tuple[int, int, int, int],
        title: str,
        samples: list[SeriesSample],
        path_color: str,
        spatial_limit: float,
    ) -> None:
        x0, y0, x1, y1 = box
        self.canvas.create_rectangle(x0, y0, x1, y1, outline="#3a424d", width=1)
        self.canvas.create_text(
            x0,
            y0 - 18,
            text=title,
            anchor="w",
            fill="#e6edf3",
            font=("TkDefaultFont", 12, "bold"),
        )

        self.draw_space_grid(box, spatial_limit)
        if len(samples) < 2:
            self.canvas.create_text(
                (x0 + x1) / 2,
                (y0 + y1) / 2,
                text="waiting for pose samples",
                fill="#8b949e",
            )
            return

        path_points: list[float] = []
        for sample in samples:
            sx, sy = self.project_to_panel(sample.values, box, spatial_limit)
            path_points.extend((sx, sy))
        if len(path_points) >= 4:
            self.canvas.create_line(path_points, fill=path_color, width=2, smooth=False)

        start_x, start_y = self.project_to_panel(samples[0].values, box, spatial_limit)
        current_x, current_y = self.project_to_panel(samples[-1].values, box, spatial_limit)
        self.canvas.create_oval(
            start_x - 4,
            start_y - 4,
            start_x + 4,
            start_y + 4,
            fill="#8b949e",
            outline="",
        )
        self.canvas.create_oval(
            current_x - 6,
            current_y - 6,
            current_x + 6,
            current_y + 6,
            fill=path_color,
            outline="#ffffff",
        )

        latest = samples[-1].values
        self.canvas.create_text(
            x0 + 12,
            y1 - 16,
            text=f"current: x={latest[0]:+.3f}m  y={latest[1]:+.3f}m  z={latest[2]:+.3f}m",
            anchor="w",
            fill="#c9d1d9",
        )

    def draw_space_grid(self, box: tuple[int, int, int, int], spatial_limit: float) -> None:
        x0, y0, x1, y1 = box
        origin = (0.0, 0.0, 0.0)
        origin_x, origin_y = self.project_to_panel(origin, box, spatial_limit)

        grid_values = (-spatial_limit, -spatial_limit / 2, spatial_limit / 2, spatial_limit)
        for value in grid_values:
            self.draw_projected_line(
                box,
                spatial_limit,
                (-spatial_limit, value, 0.0),
                (spatial_limit, value, 0.0),
                "#20262d",
            )
            self.draw_projected_line(
                box,
                spatial_limit,
                (value, -spatial_limit, 0.0),
                (value, spatial_limit, 0.0),
                "#20262d",
            )

        self.canvas.create_oval(
            origin_x - 3,
            origin_y - 3,
            origin_x + 3,
            origin_y + 3,
            fill="#c9d1d9",
            outline="",
        )
        self.canvas.create_text(
            origin_x + 6,
            origin_y + 12,
            text="origin",
            anchor="w",
            fill="#8b949e",
        )

        axes = (
            ((spatial_limit, 0.0, 0.0), "X", self.AXIS_COLORS[0]),
            ((0.0, spatial_limit, 0.0), "Y", self.AXIS_COLORS[1]),
            ((0.0, 0.0, spatial_limit), "Z", self.AXIS_COLORS[2]),
        )
        negative_axes = (
            ((-spatial_limit, 0.0, 0.0), self.AXIS_COLORS[0]),
            ((0.0, -spatial_limit, 0.0), self.AXIS_COLORS[1]),
            ((0.0, 0.0, -spatial_limit), self.AXIS_COLORS[2]),
        )
        for end, color in negative_axes:
            self.draw_projected_line(
                box, spatial_limit, origin, end, color, width=1, dash=(3, 3)
            )
        for end, label, color in axes:
            self.draw_projected_line(box, spatial_limit, origin, end, color, width=3)
            lx, ly = self.project_to_panel(end, box, spatial_limit)
            self.canvas.create_text(
                lx,
                ly - 10,
                text=f"+{label}",
                fill=color,
                font=("TkDefaultFont", 11, "bold"),
            )

        self.canvas.create_text(
            x0 + 12,
            y0 + 14,
            text=f"scale: +/-{spatial_limit:.3f} m",
            anchor="w",
            fill="#8b949e",
        )

    def draw_projected_line(
        self,
        box: tuple[int, int, int, int],
        spatial_limit: float,
        start: tuple[float, float, float],
        end: tuple[float, float, float],
        color: str,
        width: int = 1,
        dash: tuple[int, int] | None = None,
    ) -> None:
        x0, y0 = self.project_to_panel(start, box, spatial_limit)
        x1, y1 = self.project_to_panel(end, box, spatial_limit)
        self.canvas.create_line(x0, y0, x1, y1, fill=color, width=width, dash=dash)

    def draw_legend(self, width: int) -> None:
        x = width - 360
        y = 20
        for name, color in zip(self.AXIS_NAMES, self.AXIS_COLORS):
            self.canvas.create_line(x, y, x + 22, y, fill=color, width=3)
            self.canvas.create_text(x + 30, y, text=name, anchor="w", fill="#e6edf3")
            x += 56
        self.canvas.create_line(x, y, x + 22, y, fill=self.CONTROLLER_PATH_COLOR, width=3)
        self.canvas.create_text(x + 30, y, text="controller", anchor="w", fill="#e6edf3")
        x += 118
        self.canvas.create_line(x, y, x + 22, y, fill=self.TCP_PATH_COLOR, width=3)
        self.canvas.create_text(x + 30, y, text="tcp", anchor="w", fill="#e6edf3")

    def compute_spatial_limit(self, samples: list[SeriesSample]) -> float:
        max_abs = 0.005
        for sample in samples:
            max_abs = max(max_abs, *(abs(value) for value in sample.values))
        scaled = max_abs * self.node.auto_scale_margin
        if scaled < 0.02:
            return 0.02
        if scaled < 0.10:
            return math.ceil(scaled * 100.0) / 100.0
        return math.ceil(scaled * 20.0) / 20.0

    def project_to_panel(
        self,
        point: tuple[float, float, float],
        box: tuple[int, int, int, int],
        spatial_limit: float,
    ) -> tuple[float, float]:
        x, y, z = point
        yaw = math.radians(self.node.view_yaw_deg)
        pitch = math.radians(self.node.view_pitch_deg)

        x_rot = x * math.cos(yaw) - y * math.sin(yaw)
        y_rot = x * math.sin(yaw) + y * math.cos(yaw)
        screen_x = x_rot
        screen_y = z * math.cos(pitch) - y_rot * math.sin(pitch)

        x0, y0, x1, y1 = box
        usable_width = max((x1 - x0) * 0.72, 1.0)
        usable_height = max((y1 - y0) * 0.72, 1.0)
        scale = min(usable_width, usable_height) / (2.0 * max(spatial_limit, 1e-6))
        center_x = (x0 + x1) / 2
        center_y = (y0 + y1) / 2 + (y1 - y0) * 0.04
        return center_x + screen_x * scale, center_y - screen_y * scale


def main() -> None:
    rclpy.init()
    node = TeleopRealtimePlotNode()
    window = TeleopPlotWindow(node)
    try:
        window.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
