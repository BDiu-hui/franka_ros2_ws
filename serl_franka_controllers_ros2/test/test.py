import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


def clip_translation(vector):
    norm = np.linalg.norm(vector)
    if norm > 0.06:
        return (vector / norm) * 0.06
    return vector


def clip_rotation(vector):
    norm = np.linalg.norm(vector)
    if norm > 0.4:
        return (vector / norm) * 0.4
    return vector


def make_pose_msg(pose):
    msg = PoseStamped()
    msg.pose.position.x = pose[0]
    msg.pose.position.y = pose[1]
    msg.pose.position.z = pose[2]
    msg.pose.orientation.x = pose[3]
    msg.pose.orientation.y = pose[4]
    msg.pose.orientation.z = pose[5]
    msg.pose.orientation.w = pose[6]
    return msg


def make_double_parameter(name, value):
    return Parameter(
        name=name,
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)),
    )


class FrankaControlExample(Node):
    def __init__(self):
        super().__init__("franka_control_api")
        self.publisher = self.create_publisher(
            PoseStamped, "/cartesian_impedance_controller/equilibrium_pose", 10
        )
        self.parameters_client = self.create_client(
            SetParameters, "/cartesian_impedance_controller/set_parameters"
        )
        self.parameters_client.wait_for_service(timeout_sec=5.0)

    def set_controller_parameters(self, params):
        request = SetParameters.Request()
        request.parameters = [make_double_parameter(name, value) for name, value in params.items()]
        future = self.parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def publish_pose(self, pose):
        msg = make_pose_msg(pose)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrankaControlExample()

    try:
        ee_rot = np.array([0.0, 0.0, 1.0, 0.0])
        ee_pos = np.array([0.5, 0.0, 0.3])

        for _ in range(100):
            node.publish_pose(np.concatenate([ee_pos, ee_rot]))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)

        ee_pos1 = np.array([0.5, 0.0, 0.25])
        ee_rot1 = R.from_euler("xyz", [0.0, 0.3, 0.0]).as_quat()
        node.set_controller_parameters(
            {
                "translational_clip_x": 0.01,
                "translational_clip_neg_x": 0.01,
                "translational_clip_y": 0.01,
                "translational_clip_neg_y": 0.01,
                "translational_clip_z": 0.01,
                "translational_clip_neg_z": 0.01,
                "rotational_clip_x": 0.05,
                "rotational_clip_neg_x": 0.05,
                "rotational_clip_y": 0.05,
                "rotational_clip_neg_y": 0.05,
                "rotational_clip_z": 0.05,
                "rotational_clip_neg_z": 0.05,
            }
        )
        delta_ee_pos = clip_translation(ee_pos1 - ee_pos) / 500
        delta_ee_rot = clip_rotation(ee_rot1 - ee_rot) / 500
        for _ in range(500):
            ee_pos += delta_ee_pos
            ee_rot += delta_ee_rot
            node.publish_pose(np.concatenate([ee_pos, ee_rot]))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)

        ee_pos2 = np.array([0.5, 0.0, 0.25])
        ee_rot2 = R.from_euler("xyz", [0.0, 0.9, 0.0]).as_quat()
        node.set_controller_parameters(
            {
                "translational_clip_x": 0.005,
                "translational_clip_neg_x": 0.005,
                "translational_clip_y": 0.005,
                "translational_clip_neg_y": 0.005,
                "translational_clip_z": 0.005,
                "translational_clip_neg_z": 0.005,
            }
        )
        delta_ee_pos = clip_translation(ee_pos2 - ee_pos) / 500
        delta_ee_rot = clip_rotation(ee_rot2 - ee_rot) / 500
        for _ in range(500):
            ee_pos += delta_ee_pos
            ee_rot += delta_ee_rot
            node.publish_pose(np.concatenate([ee_pos, ee_rot]))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)

        ee_pos3 = np.array([0.5, 0.0, 0.35])
        ee_rot3 = R.from_euler("xyz", [0.0, 0.9, 0.0]).as_quat()
        delta_ee_pos = clip_translation(ee_pos3 - ee_pos) / 500
        delta_ee_rot = clip_rotation(ee_rot3 - ee_rot) / 500
        for _ in range(500):
            ee_pos += delta_ee_pos
            ee_rot += delta_ee_rot
            node.publish_pose(np.concatenate([ee_pos, ee_rot]))
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
