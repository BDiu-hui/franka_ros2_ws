"""Y-button authority gate for one shared impedance/Wuji hardware stack."""

from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from serl_franka_controllers_ros2.msg import CartesianImpedanceCommand
from std_msgs.msg import Bool, String


MAX_BODY_BYTES = 1024 * 1024
HAND_JOINT_COUNT = 20


def button_pressed(value: Any, threshold: float = 0.5) -> bool:
    if isinstance(value, (list, tuple)):
        value = value[0] if value else 0.0
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value) >= threshold
    return False


class AuthorityState:
    """Small thread-safe state shared by ROS callbacks and HTTP workers."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.teleop_active = False
        self.recording = False
        self.last_episode = ""
        self.switch_count = 0
        self.last_switch_monotonic = time.monotonic()
        self._previous_buttons: dict[str, bool] = {}

    def update_buttons(
        self,
        buttons: dict[str, Any],
        *,
        takeover_button: str,
        start_button: str,
        stop_button: str,
    ) -> tuple[bool, bool]:
        values = {
            takeover_button: button_pressed(buttons.get(takeover_button, 0.0)),
            start_button: button_pressed(buttons.get(start_button, 0.0)),
            stop_button: button_pressed(buttons.get(stop_button, 0.0)),
        }
        switched = False
        blocked = False
        with self.lock:
            if values[takeover_button] and not self._previous_buttons.get(takeover_button, False):
                if self.teleop_active and self.recording:
                    blocked = True
                else:
                    self.teleop_active = not self.teleop_active
                    self.switch_count += 1
                    self.last_switch_monotonic = time.monotonic()
                    switched = True
            if (
                self.teleop_active
                and values[start_button]
                and not self._previous_buttons.get(start_button, False)
            ):
                self.recording = True
            if values[stop_button] and not self._previous_buttons.get(stop_button, False):
                self.recording = False
            self._previous_buttons.update(values)
        return switched, blocked

    def set_episode_saved(self, path: str) -> None:
        with self.lock:
            self.recording = False
            self.last_episode = path

    def is_teleop(self) -> bool:
        with self.lock:
            return self.teleop_active

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "authority": "teleop" if self.teleop_active else "inference",
                "teleop_active": self.teleop_active,
                "recording": self.recording,
                "last_episode": self.last_episode,
                "switch_count": self.switch_count,
                "seconds_since_switch": round(
                    max(0.0, time.monotonic() - self.last_switch_monotonic), 3
                ),
            }

    def run_when(self, *, teleop: bool, callback: Callable[[], None]) -> bool:
        with self.lock:
            if self.teleop_active != teleop:
                return False
            callback()
            return True


class _GatewayServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        node: "ControlAuthorityNode",
        kind: str,
        backend_url: str = "",
        arm: str = "",
    ) -> None:
        self.node = node
        self.kind = kind
        self.backend_url = backend_url.rstrip("/")
        self.arm = arm
        super().__init__(address, _GatewayHandler)


class _GatewayHandler(BaseHTTPRequestHandler):
    server: _GatewayServer

    def do_GET(self) -> None:  # noqa: N802
        self._dispatch()

    def do_POST(self) -> None:  # noqa: N802
        self._dispatch()

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _dispatch(self) -> None:
        path = urlparse(self.path).path
        if path == "/control_authority":
            self._write_json(HTTPStatus.OK, {"ok": True, **self.server.node.status_payload()})
            return
        body = self._read_body()
        if body is None:
            return
        if self.server.kind == "arm":
            self._dispatch_arm(path, body)
        else:
            self._dispatch_wuji(path, body)

    def _dispatch_arm(self, path: str, body: bytes) -> None:
        if self.command == "POST" and path == "/pose":
            try:
                payload = json.loads(body or b"{}")
                result = self.server.node.publish_policy_pose(self.server.arm, payload)
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                self._write_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
                return
            if result is None:
                self._write_json(
                    HTTPStatus.CONFLICT,
                    {"ok": False, "error": "Quest teleoperation owns control; policy /pose rejected"},
                )
                return
            self._write_json(HTTPStatus.OK, result)
            return
        if self.command == "POST" and path == "/pose_precise" and self.server.node.state.is_teleop():
            self._write_json(
                HTTPStatus.CONFLICT,
                {"ok": False, "error": "Quest teleoperation owns control; policy /pose_precise rejected"},
            )
            return
        self._proxy(body)

    def _dispatch_wuji(self, path: str, body: bytes) -> None:
        if self.command == "GET" and path == "/health":
            hands = self.server.node.ready_hands()
            self._write_json(
                HTTPStatus.OK,
                {
                    "ok": set(hands) == {"left", "right"},
                    "hands": hands,
                    **self.server.node.status_payload(),
                },
            )
            return
        parts = [part for part in path.split("/") if part]
        if (
            self.command == "GET"
            and len(parts) == 3
            and parts[0] == "hands"
            and parts[2] == "actual_joint_positions"
        ):
            side = parts[1]
            try:
                positions, timestamp_ns = self.server.node.actual_hand_state(side)
            except KeyError as exc:
                self._write_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": str(exc)})
                return
            except RuntimeError as exc:
                self._write_json(
                    HTTPStatus.SERVICE_UNAVAILABLE,
                    {"ok": False, "error": str(exc)},
                )
                return
            self._write_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "side": side,
                    "positions": positions,
                    "timestamp_monotonic_ns": timestamp_ns,
                    "age_ms": max(0.0, (time.monotonic_ns() - timestamp_ns) / 1_000_000.0),
                },
            )
            return
        if self.command != "POST" or len(parts) != 3 or parts[0] != "hands" or parts[2] != "joint_targets":
            self._write_json(HTTPStatus.NOT_FOUND, {"ok": False, "error": "not found"})
            return
        side = parts[1]
        try:
            payload = json.loads(body or b"{}")
            positions = [float(value) for value in payload.get("positions", [])]
            if len(positions) != HAND_JOINT_COUNT:
                raise ValueError("positions must contain exactly 20 values")
            if not all(math.isfinite(value) for value in positions):
                raise ValueError("positions contains a non-finite value")
            sent = self.server.node.publish_policy_hand(side, positions)
        except (AttributeError, json.JSONDecodeError, TypeError, ValueError) as exc:
            self._write_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return
        if not sent:
            self._write_json(
                HTTPStatus.CONFLICT,
                {"ok": False, "error": "Quest teleoperation owns control; policy Wuji command rejected"},
            )
            return
        self._write_json(HTTPStatus.OK, {"ok": True, "side": side})

    def _proxy(self, body: bytes) -> None:
        request = Request(
            f"{self.server.backend_url}{self.path}",
            data=body if self.command != "GET" else None,
            method=self.command,
            headers={"Content-Type": self.headers.get("Content-Type", "application/json")},
        )
        try:
            response = urlopen(request, timeout=3.0)
            with response:
                self._write_raw(response.status, response.read(), response.headers.get("Content-Type"))
        except HTTPError as exc:
            self._write_raw(exc.code, exc.read(), exc.headers.get("Content-Type"))
        except (URLError, TimeoutError, OSError) as exc:
            self._write_json(HTTPStatus.BAD_GATEWAY, {"ok": False, "error": str(exc)})

    def _read_body(self) -> bytes | None:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = -1
        if length < 0 or length > MAX_BODY_BYTES:
            self._write_json(HTTPStatus.BAD_REQUEST, {"ok": False, "error": "invalid request body size"})
            return None
        return self.rfile.read(length) if length else b""

    def _write_json(self, status: HTTPStatus, payload: dict[str, Any]) -> None:
        self._write_raw(status.value, json.dumps(payload, separators=(",", ":")).encode(), "application/json")

    def _write_raw(self, status: int, body: bytes, content_type: str | None) -> None:
        self.send_response(int(status))
        self.send_header("Content-Type", content_type or "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class ControlAuthorityNode(Node):
    def __init__(self) -> None:
        super().__init__("unified_control_authority")
        defaults = {
            "buttons_topic": "/quest3/buttons",
            "teleop_buttons_topic": "/unified_impedance/teleop/buttons",
            "authority_topic": "/unified_impedance/control_authority",
            "episode_saved_topic": "/quest3/data_recorder/episode_saved",
            "takeover_button": "Y",
            "record_start_button": "A",
            "record_stop_button": "B",
            "voice_prompts_enabled": True,
            "voice_command": "/usr/bin/spd-say",
            "left_teleop_pose_topic": "/unified_impedance/teleop/left/equilibrium_pose",
            "right_teleop_pose_topic": "/unified_impedance/teleop/right/equilibrium_pose",
            "left_output_pose_topic": "/left/cartesian_impedance_controller/equilibrium_pose",
            "right_output_pose_topic": "/right/cartesian_impedance_controller/equilibrium_pose",
            "left_teleop_enabled_topic": "/unified_impedance/teleop/left/enabled",
            "right_teleop_enabled_topic": "/unified_impedance/teleop/right/enabled",
            "left_output_enabled_topic": "/quest3/left_impedance_teleop/enabled",
            "right_output_enabled_topic": "/quest3/right_impedance_teleop/enabled",
            "left_current_pose_topic": "/left/franka_robot_state_broadcaster/current_pose",
            "right_current_pose_topic": "/right/franka_robot_state_broadcaster/current_pose",
            "handoff_current_pose_timeout_sec": 0.25,
            "left_teleop_hand_topic": "/unified_impedance/teleop/hand_left/joint_commands",
            "right_teleop_hand_topic": "/unified_impedance/teleop/hand_right/joint_commands",
            "left_output_hand_topic": "/hand_left/joint_commands",
            "right_output_hand_topic": "/hand_right/joint_commands",
            "left_actual_hand_topic": "/hand_left/joint_states",
            "right_actual_hand_topic": "/hand_right/joint_states",
            "left_gateway_host": "127.0.0.1",
            "left_gateway_port": 5000,
            "left_backend_url": "http://127.0.0.1:5100",
            "right_gateway_host": "127.0.0.1",
            "right_gateway_port": 5001,
            "right_backend_url": "http://127.0.0.1:5101",
            "wuji_gateway_host": "127.0.0.1",
            "wuji_gateway_port": 8765,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.state = AuthorityState()
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.authority_pub = self.create_publisher(
            String, str(self.get_parameter("authority_topic").value), latched_qos
        )
        self.teleop_active_pub = self.create_publisher(
            Bool, "/unified_impedance/teleop_active", latched_qos
        )
        self.teleop_buttons_pub = self.create_publisher(
            String, str(self.get_parameter("teleop_buttons_topic").value), 10
        )

        self.pose_publishers = {
            "left": self.create_publisher(
                CartesianImpedanceCommand,
                str(self.get_parameter("left_output_pose_topic").value),
                10,
            ),
            "right": self.create_publisher(
                CartesianImpedanceCommand,
                str(self.get_parameter("right_output_pose_topic").value),
                10,
            ),
        }
        self.hand_publishers = {
            "left": self.create_publisher(
                JointState,
                str(self.get_parameter("left_output_hand_topic").value),
                qos_profile_sensor_data,
            ),
            "right": self.create_publisher(
                JointState,
                str(self.get_parameter("right_output_hand_topic").value),
                qos_profile_sensor_data,
            ),
        }
        self._actual_hand_state_lock = threading.Lock()
        self._actual_hand_states: dict[str, tuple[list[float], int]] = {}
        self.teleop_enabled_publishers = {
            "left": self.create_publisher(
                Bool, str(self.get_parameter("left_output_enabled_topic").value), 10
            ),
            "right": self.create_publisher(
                Bool, str(self.get_parameter("right_output_enabled_topic").value), 10
            ),
        }
        self._current_arm_poses: dict[str, tuple[list[float], float]] = {}
        self._handoff_pending = {"left": False, "right": False}
        self._handoff_saw_grip_released = {"left": False, "right": False}
        self._teleop_pose_after_ns = {"left": 0, "right": 0}
        for side in ("left", "right"):
            self.create_subscription(
                CartesianImpedanceCommand,
                str(self.get_parameter(f"{side}_teleop_pose_topic").value),
                lambda msg, arm=side: self._relay_teleop_pose(arm, msg),
                10,
            )
            self.create_subscription(
                Bool,
                str(self.get_parameter(f"{side}_teleop_enabled_topic").value),
                lambda msg, arm=side: self._relay_teleop_enabled(arm, msg),
                10,
            )
            self.create_subscription(
                PoseStamped,
                str(self.get_parameter(f"{side}_current_pose_topic").value),
                lambda msg, arm=side: self._cache_current_arm_pose(arm, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointState,
                str(self.get_parameter(f"{side}_teleop_hand_topic").value),
                lambda msg, arm=side: self._relay_teleop_hand(arm, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointState,
                str(self.get_parameter(f"{side}_actual_hand_topic").value),
                lambda msg, arm=side: self._cache_actual_hand_state(arm, msg),
                qos_profile_sensor_data,
            )
        self.buttons_sub = self.create_subscription(
            String,
            str(self.get_parameter("buttons_topic").value),
            self._buttons_callback,
            10,
        )
        self.episode_sub = self.create_subscription(
            String,
            str(self.get_parameter("episode_saved_topic").value),
            self._episode_saved_callback,
            10,
        )

        self._servers = [
            self._start_server(
                str(self.get_parameter("left_gateway_host").value),
                int(self.get_parameter("left_gateway_port").value),
                "arm",
                str(self.get_parameter("left_backend_url").value),
                "left",
            ),
            self._start_server(
                str(self.get_parameter("right_gateway_host").value),
                int(self.get_parameter("right_gateway_port").value),
                "arm",
                str(self.get_parameter("right_backend_url").value),
                "right",
            ),
            self._start_server(
                str(self.get_parameter("wuji_gateway_host").value),
                int(self.get_parameter("wuji_gateway_port").value),
                "wuji",
            ),
        ]
        self._publish_authority()
        gateway_ports = "/".join(
            str(self.get_parameter(name).value)
            for name in ("left_gateway_port", "right_gateway_port", "wuji_gateway_port")
        )
        self.get_logger().info(
            "Unified authority ready: default=inference, Y toggles Quest takeover; "
            f"gateways={gateway_ports}"
        )

    def _start_server(
        self,
        host: str,
        port: int,
        kind: str,
        backend_url: str = "",
        arm: str = "",
    ) -> _GatewayServer:
        server = _GatewayServer((host, port), self, kind, backend_url, arm)
        threading.Thread(
            target=server.serve_forever,
            name=f"{kind}_{arm or 'wuji'}_gateway",
            daemon=True,
        ).start()
        return server

    def _buttons_callback(self, msg: String) -> None:
        try:
            buttons = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(buttons, dict):
            return
        switched, blocked = self.state.update_buttons(
            buttons,
            takeover_button=str(self.get_parameter("takeover_button").value),
            start_button=str(self.get_parameter("record_start_button").value),
            stop_button=str(self.get_parameter("record_stop_button").value),
        )
        if switched:
            if self.state.is_teleop():
                self._begin_teleop_handoff()
            else:
                self._clear_teleop_handoff()
            self._publish_authority()
            self._speak_authority(self.state.snapshot()["authority"])
            self.get_logger().warn(
                f"Control authority changed to {self.state.snapshot()['authority']} by Y button"
            )
        elif blocked:
            self.get_logger().warn(
                "Y ignored while recording; press B to stop and save, then press Y again"
            )
        if self.state.is_teleop():
            self.teleop_buttons_pub.publish(msg)

    def _speak_authority(self, authority: str) -> None:
        if not bool(self.get_parameter("voice_prompts_enabled").value):
            return
        phrase = {
            "teleop": "Teleoperation control",
            "inference": "Inference control",
        }.get(authority)
        command = str(self.get_parameter("voice_command").value).strip()
        if not phrase or not command:
            return
        try:
            subprocess.Popen(
                [command, phrase],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            self.get_logger().warn(f"Authority voice prompt failed: {exc}")

    def _episode_saved_callback(self, msg: String) -> None:
        self.state.set_episode_saved(msg.data)

    def _publish_authority(self) -> None:
        snapshot = self.state.snapshot()
        authority = String()
        authority.data = snapshot["authority"]
        self.authority_pub.publish(authority)
        active = Bool()
        active.data = snapshot["teleop_active"]
        self.teleop_active_pub.publish(active)
        if not active.data:
            disabled = Bool()
            disabled.data = False
            for publisher in self.teleop_enabled_publishers.values():
                publisher.publish(disabled)

    def _relay_teleop_pose(self, side: str, msg: CartesianImpedanceCommand) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._handoff_pending[side] or stamp_ns <= self._teleop_pose_after_ns[side]:
            return
        self.state.run_when(
            teleop=True,
            callback=lambda: self.pose_publishers[side].publish(msg),
        )

    def _relay_teleop_hand(self, side: str, msg: JointState) -> None:
        self.state.run_when(
            teleop=True,
            callback=lambda: self.hand_publishers[side].publish(msg),
        )

    def _relay_teleop_enabled(self, side: str, msg: Bool) -> None:
        if not self.state.is_teleop():
            return
        if self._handoff_pending[side]:
            if not msg.data:
                self._handoff_saw_grip_released[side] = True
            elif self._handoff_saw_grip_released[side]:
                self._handoff_pending[side] = False
                self._teleop_pose_after_ns[side] = self.get_clock().now().nanoseconds
                self.get_logger().info(
                    f"{side} teleop handoff armed from a fresh Grip anchor"
                )
            else:
                return
        self.state.run_when(
            teleop=True,
            callback=lambda: self.teleop_enabled_publishers[side].publish(msg),
        )

    def _cache_current_arm_pose(self, side: str, msg: PoseStamped) -> None:
        values = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
            float(msg.pose.orientation.x),
            float(msg.pose.orientation.y),
            float(msg.pose.orientation.z),
            float(msg.pose.orientation.w),
        ]
        if all(math.isfinite(value) for value in values):
            self._current_arm_poses[side] = (values, time.monotonic())

    def _begin_teleop_handoff(self) -> None:
        disabled = Bool()
        disabled.data = False
        timeout = float(self.get_parameter("handoff_current_pose_timeout_sec").value)
        held: list[str] = []
        for side in ("left", "right"):
            self._handoff_pending[side] = True
            self._handoff_saw_grip_released[side] = False
            self._teleop_pose_after_ns[side] = self.get_clock().now().nanoseconds
            self.teleop_enabled_publishers[side].publish(disabled)
            cached = self._current_arm_poses.get(side)
            if cached is None or time.monotonic() - cached[1] > timeout:
                self.get_logger().warn(
                    f"{side} teleop handoff has no fresh current pose; keeping old commands blocked"
                )
                continue
            values = cached[0]
            hold = CartesianImpedanceCommand()
            hold.header.stamp = self.get_clock().now().to_msg()
            hold.header.frame_id = "base"
            hold.pose.position.x, hold.pose.position.y, hold.pose.position.z = values[:3]
            (
                hold.pose.orientation.x,
                hold.pose.orientation.y,
                hold.pose.orientation.z,
                hold.pose.orientation.w,
            ) = values[3:]
            hold.has_master_q = False
            self.pose_publishers[side].publish(hold)
            held.append(side)
        self.get_logger().info(
            "Teleop handoff pending: holding current pose for "
            f"{held}; release and press each Grip to arm motion"
        )

    def _clear_teleop_handoff(self) -> None:
        for side in ("left", "right"):
            self._handoff_pending[side] = False
            self._handoff_saw_grip_released[side] = False
            self._teleop_pose_after_ns[side] = 0

    def publish_policy_pose(self, side: str, payload: dict[str, Any]) -> dict[str, Any] | None:
        if side not in self.pose_publishers:
            raise ValueError(f"unknown arm {side!r}")
        pose = payload.get("arr")
        if not isinstance(pose, list) or len(pose) != 7:
            raise ValueError("pose arr must contain [x, y, z, qx, qy, qz, qw]")
        values = [float(value) for value in pose]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose contains a non-finite value")
        master_q = payload.get("q")
        if master_q is not None:
            if not isinstance(master_q, list) or len(master_q) != 7:
                raise ValueError("q must contain 7 joint angles")
            master_q = [float(value) for value in master_q]

        def publish() -> None:
            msg = CartesianImpedanceCommand()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base"
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = values[:3]
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ) = values[3:]
            msg.has_master_q = master_q is not None
            if master_q is not None:
                msg.master_q = master_q
            self.pose_publishers[side].publish(msg)

        if not self.state.run_when(teleop=False, callback=publish):
            return None
        return {"ok": True, "message": "Moved", "pose": values, "method": "unified_authority_gate"}

    def publish_policy_hand(self, side: str, positions: list[float]) -> bool:
        if side not in self.hand_publishers:
            raise ValueError(f"unknown Wuji hand {side!r}")

        def publish() -> None:
            if self.hand_publishers[side].get_subscription_count() < 1:
                raise ValueError(f"Wuji hand {side!r} driver is not subscribed")
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = positions
            self.hand_publishers[side].publish(msg)

        return self.state.run_when(teleop=False, callback=publish)

    def ready_hands(self) -> list[str]:
        return [
            side
            for side, publisher in self.hand_publishers.items()
            if publisher.get_subscription_count() > 0
        ]

    def _cache_actual_hand_state(self, side: str, msg: JointState) -> None:
        positions = [float(value) for value in msg.position]
        if len(positions) != HAND_JOINT_COUNT or not all(
            math.isfinite(value) for value in positions
        ):
            return
        with self._actual_hand_state_lock:
            self._actual_hand_states[side] = (positions, time.monotonic_ns())

    def actual_hand_state(self, side: str) -> tuple[list[float], int]:
        if side not in self.hand_publishers:
            raise KeyError(f"unknown hand {side!r}")
        with self._actual_hand_state_lock:
            state = self._actual_hand_states.get(side)
            if state is None:
                raise RuntimeError(f"no actual joint state received for hand {side!r}")
            positions, timestamp_ns = state
            return list(positions), timestamp_ns

    def status_payload(self) -> dict[str, Any]:
        return {
            **self.state.snapshot(),
            "wuji_ready_hands": self.ready_hands(),
            "teleop_pose_pending": [
                side for side, pending in self._handoff_pending.items() if pending
            ],
        }

    def destroy_node(self) -> bool:
        for server in getattr(self, "_servers", []):
            server.shutdown()
            server.server_close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ControlAuthorityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
