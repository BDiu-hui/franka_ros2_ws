import json
import math
import time
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf_transformations import quaternion_from_matrix
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray


class Quest3OculusTfNode(Node):
    def __init__(self) -> None:
        super().__init__("quest3_oculus_tf")

        self.declare_parameter("mock", False)
        self.declare_parameter("ip_address", "")
        self.declare_parameter("port", 5555)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("right_frame", "quest3_right_controller")
        self.declare_parameter("left_frame", "quest3_left_controller")
        self.declare_parameter("log_raw", False)

        self.mock = self.get_parameter("mock").get_parameter_value().bool_value
        self.ip_address = self.get_parameter("ip_address").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.right_frame = self.get_parameter("right_frame").get_parameter_value().string_value
        self.left_frame = self.get_parameter("left_frame").get_parameter_value().string_value
        self.log_raw = self.get_parameter("log_raw").get_parameter_value().bool_value

        self.br = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "quest3/controller_markers", 10)
        self.buttons_pub = self.create_publisher(String, "quest3/buttons", 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, "quest3/right_controller/pose", 10)
        self.left_pose_pub = self.create_publisher(PoseStamped, "quest3/left_controller/pose", 10)
        self.reader = None
        self._mock_t0 = time.monotonic()
        self._last_wait_log = 0.0

        if self.mock:
            self.get_logger().info("Running in mock mode; no Quest device is required.")
        else:
            from oculus_reader.reader import OculusReader

            ip = self.ip_address if self.ip_address else None
            self.reader = OculusReader(ip_address=ip, port=self.port, print_FPS=False)
            mode = f"network {ip}:{self.port}" if ip else "USB"
            self.get_logger().info(f"Connected to oculus_reader using {mode} mode.")

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)

    def destroy_node(self) -> bool:
        if self.reader is not None:
            self.reader.stop()
        return super().destroy_node()

    def timer_callback(self) -> None:
        if self.mock:
            transformations, buttons = self.mock_data()
        else:
            transformations, buttons = self.reader.get_transformations_and_buttons()
            transformations = transformations or {}
            buttons = buttons or {}

        now = self.get_clock().now().to_msg()
        published = False

        if "r" in transformations:
            self.publish_transform(transformations["r"], self.right_frame, now)
            self.publish_pose(transformations["r"], self.right_pose_pub, now)
            published = True
        if "l" in transformations:
            self.publish_transform(transformations["l"], self.left_frame, now)
            self.publish_pose(transformations["l"], self.left_pose_pub, now)
            published = True

        if published:
            self.publish_markers(transformations, now)
            msg = String()
            msg.data = json.dumps(self.to_jsonable(buttons), sort_keys=True)
            self.buttons_pub.publish(msg)
            if self.log_raw:
                self.get_logger().info(f"buttons={msg.data}")
        else:
            self.log_waiting_for_pose()

    def publish_transform(self, transform: np.ndarray, child_frame: str, stamp: Any) -> None:
        translation = transform[:3, 3]
        quat = quaternion_from_matrix(transform)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.br.sendTransform(tf_msg)

    def publish_pose(self, transform: np.ndarray, publisher: Any, stamp: Any) -> None:
        translation = transform[:3, 3]
        quat = quaternion_from_matrix(transform)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        publisher.publish(pose_msg)

    def publish_markers(self, transformations: dict[str, np.ndarray], stamp: Any) -> None:
        markers = MarkerArray()
        if "r" in transformations:
            markers.markers.append(
                self.make_controller_marker(0, transformations["r"], self.right_frame, stamp, (0.95, 0.18, 0.12))
            )
        if "l" in transformations:
            markers.markers.append(
                self.make_controller_marker(1, transformations["l"], self.left_frame, stamp, (0.12, 0.42, 0.95))
            )
        self.marker_pub.publish(markers)

    def make_controller_marker(
        self,
        marker_id: int,
        transform: np.ndarray,
        frame_name: str,
        stamp: Any,
        rgb: tuple[float, float, float],
    ) -> Marker:
        translation = transform[:3, 3]
        quat = quaternion_from_matrix(transform)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = "quest3_controllers"
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(translation[0])
        marker.pose.position.y = float(translation[1])
        marker.pose.position.z = float(translation[2])
        marker.pose.orientation.x = float(quat[0])
        marker.pose.orientation.y = float(quat[1])
        marker.pose.orientation.z = float(quat[2])
        marker.pose.orientation.w = float(quat[3])
        marker.scale.x = 0.18
        marker.scale.y = 0.035
        marker.scale.z = 0.035
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.95
        marker.frame_locked = False
        marker.text = frame_name
        return marker

    def mock_data(self) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        t = time.monotonic() - self._mock_t0
        right = self.make_mock_transform(
            0.38 + 0.08 * math.sin(t),
            -0.22,
            1.10 + 0.06 * math.sin(0.7 * t),
            yaw=0.5 * math.sin(0.9 * t),
        )
        left = self.make_mock_transform(
            0.38 + 0.08 * math.sin(t + math.pi),
            0.22,
            1.10 + 0.06 * math.cos(0.7 * t),
            yaw=0.5 * math.cos(0.9 * t),
        )
        buttons = {
            "A": math.sin(0.55 * t) > 0.85,
            "B": math.sin(0.55 * t + math.pi) > 0.85,
            "RTr": math.sin(t) > 0.7,
            "rightTrig": (max(math.sin(t), 0.0),),
            "rightJS": (0.55 * math.sin(0.8 * t), 0.55 * math.cos(0.6 * t)),
            "RG": False,
            "LTr": math.sin(t + math.pi) > 0.7,
            "leftTrig": (max(math.sin(t + math.pi), 0.0),),
        }
        return {"r": right, "l": left}, buttons

    @staticmethod
    def make_mock_transform(x: float, y: float, z: float, yaw: float) -> np.ndarray:
        c = math.cos(yaw)
        s = math.sin(yaw)
        transform = np.eye(4)
        transform[:3, :3] = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        transform[:3, 3] = [x, y, z]
        return transform

    @staticmethod
    def to_jsonable(value: Any) -> Any:
        if isinstance(value, dict):
            return {key: Quest3OculusTfNode.to_jsonable(val) for key, val in value.items()}
        if isinstance(value, tuple):
            return [Quest3OculusTfNode.to_jsonable(val) for val in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        return value

    def log_waiting_for_pose(self) -> None:
        now = time.monotonic()
        if now - self._last_wait_log > 3.0:
            self.get_logger().warn("No Quest controller pose received yet.")
            self._last_wait_log = now


def main() -> None:
    rclpy.init()
    node = Quest3OculusTfNode()
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
