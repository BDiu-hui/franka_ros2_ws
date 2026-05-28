import math
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_matrix


class FrankaSimIkNode(Node):
    """Numerical IK for animating the Panda/Franka model in RViz."""

    JOINT_NAMES = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ]
    FINGER_NAMES = ["panda_finger_joint1", "panda_finger_joint2"]

    JOINT_LIMITS = np.array(
        [
            [-2.9671, 2.9671],
            [-1.8326, 1.8326],
            [-2.9671, 2.9671],
            [-3.1416, 0.0873],
            [-2.9671, 2.9671],
            [-0.0873, 3.8223],
            [-2.9671, 2.9671],
        ],
        dtype=float,
    )
    HOME_Q = np.array([0.0, -0.4, 0.0, -2.4, 0.0, 2.0, 0.8], dtype=float)

    # Joint origins copied from moveit_resources_panda_description/urdf/panda.urdf.
    JOINT_ORIGINS = [
        ([0.0, 0.0, 0.333], [0.0, 0.0, 0.0]),
        ([0.0, 0.0, 0.0], [-math.pi / 2.0, 0.0, 0.0]),
        ([0.0, -0.316, 0.0], [math.pi / 2.0, 0.0, 0.0]),
        ([0.0825, 0.0, 0.0], [math.pi / 2.0, 0.0, 0.0]),
        ([-0.0825, 0.384, 0.0], [-math.pi / 2.0, 0.0, 0.0]),
        ([0.0, 0.0, 0.0], [math.pi / 2.0, 0.0, 0.0]),
        ([0.088, 0.0, 0.0], [math.pi / 2.0, 0.0, 0.0]),
    ]
    LINK8_FIXED = ([0.0, 0.0, 0.107], [0.0, 0.0, 0.0])

    def __init__(self) -> None:
        super().__init__("franka_sim_ik")

        self.declare_parameter("target_pose_topic", "franka_sim/tcp_target_pose")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("iterations_per_tick", 8)
        self.declare_parameter("damping", 0.045)
        self.declare_parameter("orientation_weight", 0.65)
        self.declare_parameter("max_joint_step_rad", 0.06)
        self.declare_parameter("nullspace_gain", 0.015)
        self.declare_parameter("finger_width", 0.02)
        self.declare_parameter("joint_name_prefix", "")
        self.declare_parameter("joint_names", [])
        self.declare_parameter("finger_joint_names", [])

        target_pose_topic = self.get_parameter("target_pose_topic").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.iterations_per_tick = int(self.get_parameter("iterations_per_tick").value)
        self.damping = float(self.get_parameter("damping").value)
        self.orientation_weight = float(self.get_parameter("orientation_weight").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step_rad").value)
        self.nullspace_gain = float(self.get_parameter("nullspace_gain").value)
        self.finger_width = float(self.get_parameter("finger_width").value)
        joint_name_prefix = str(self.get_parameter("joint_name_prefix").value)
        configured_joint_names = list(self.get_parameter("joint_names").value)
        configured_finger_joint_names = list(self.get_parameter("finger_joint_names").value)
        self.joint_names = (
            configured_joint_names
            if configured_joint_names
            else [f"{joint_name_prefix}{name}" for name in self.JOINT_NAMES]
        )
        self.finger_names = (
            configured_finger_joint_names
            if configured_finger_joint_names
            else [f"{joint_name_prefix}{name}" for name in self.FINGER_NAMES]
        )
        if len(self.joint_names) != 7:
            raise RuntimeError("joint_names must contain exactly 7 names")
        if len(self.finger_names) != 2:
            raise RuntimeError("finger_joint_names must contain exactly 2 names")

        self.q = self.HOME_Q.copy()
        self.target_pose: PoseStamped | None = None
        self.last_position_error = float("nan")
        self.last_orientation_error = float("nan")

        self.target_sub = self.create_subscription(
            PoseStamped,
            target_pose_topic,
            self.target_callback,
            10,
        )
        self.joint_pub = self.create_publisher(JointState, joint_state_topic, 10)

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(f"Franka sim IK ready. Tracking {target_pose_topic}.")

    def target_callback(self, msg: PoseStamped) -> None:
        self.target_pose = msg

    def timer_callback(self) -> None:
        if self.target_pose is not None:
            target_transform = self.pose_to_matrix(self.target_pose)
            for _ in range(max(self.iterations_per_tick, 1)):
                self.solve_one_step(target_transform)
        self.publish_joint_state()

    def solve_one_step(self, target_transform: np.ndarray) -> None:
        current_transform, jacobian = self.forward_kinematics_and_jacobian(self.q)
        position_error = target_transform[:3, 3] - current_transform[:3, 3]
        orientation_error = self.rotation_vector_from_matrix(
            target_transform[:3, :3] @ current_transform[:3, :3].T
        )

        self.last_position_error = float(np.linalg.norm(position_error))
        self.last_orientation_error = float(np.linalg.norm(orientation_error))

        error = np.concatenate(
            [
                position_error,
                self.orientation_weight * orientation_error,
            ]
        )
        weighted_jacobian = jacobian.copy()
        weighted_jacobian[3:, :] *= self.orientation_weight

        jj_t = weighted_jacobian @ weighted_jacobian.T
        damping_matrix = (self.damping * self.damping) * np.eye(6)
        jacobian_pinv = weighted_jacobian.T @ np.linalg.solve(jj_t + damping_matrix, np.eye(6))
        task_step = jacobian_pinv @ error

        nullspace = np.eye(7) - jacobian_pinv @ weighted_jacobian
        joint_center_step = self.nullspace_gain * (self.HOME_Q - self.q)
        step = task_step + nullspace @ joint_center_step
        step = np.clip(step, -self.max_joint_step, self.max_joint_step)
        self.q = np.clip(self.q + step, self.JOINT_LIMITS[:, 0], self.JOINT_LIMITS[:, 1])

    def publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names + self.finger_names
        msg.position = self.q.tolist() + [self.finger_width, self.finger_width]
        self.joint_pub.publish(msg)

    def forward_kinematics_and_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        transform = np.eye(4)
        joint_positions = []
        joint_axes = []

        for (xyz, rpy), joint_value in zip(self.JOINT_ORIGINS, q):
            joint_frame = transform @ self.origin_transform(xyz, rpy)
            joint_positions.append(joint_frame[:3, 3].copy())
            joint_axes.append(joint_frame[:3, :3] @ np.array([0.0, 0.0, 1.0]))
            transform = joint_frame @ self.z_rotation_transform(float(joint_value))

        end_transform = transform @ self.origin_transform(*self.LINK8_FIXED)
        end_position = end_transform[:3, 3]

        jacobian = np.zeros((6, 7), dtype=float)
        for index, (joint_position, joint_axis) in enumerate(zip(joint_positions, joint_axes)):
            jacobian[:3, index] = np.cross(joint_axis, end_position - joint_position)
            jacobian[3:, index] = joint_axis
        return end_transform, jacobian

    @staticmethod
    def pose_to_matrix(msg: PoseStamped) -> np.ndarray:
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        transform = quaternion_matrix(quat)
        transform[:3, 3] = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]
        return transform

    @staticmethod
    def origin_transform(xyz: list[float], rpy: list[float]) -> np.ndarray:
        transform = np.eye(4)
        transform[:3, :3] = FrankaSimIkNode.rpy_matrix(rpy[0], rpy[1], rpy[2])
        transform[:3, 3] = xyz
        return transform

    @staticmethod
    def z_rotation_transform(angle: float) -> np.ndarray:
        transform = np.eye(4)
        c = math.cos(angle)
        s = math.sin(angle)
        transform[:3, :3] = np.array(
            [
                [c, -s, 0.0],
                [s, c, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        return transform

    @staticmethod
    def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
        return rz @ ry @ rx

    @staticmethod
    def rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
        cos_angle = (np.trace(rotation) - 1.0) * 0.5
        cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
        angle = math.acos(cos_angle)
        skew_vector = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=float,
        )
        if angle < 1e-6:
            return 0.5 * skew_vector
        sin_angle = math.sin(angle)
        if abs(sin_angle) < 1e-6:
            return np.zeros(3)
        axis = skew_vector / (2.0 * sin_angle)
        return axis * angle


def main() -> None:
    rclpy.init()
    node = FrankaSimIkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
