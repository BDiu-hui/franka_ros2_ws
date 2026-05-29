from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    quest_ip_address = LaunchConfiguration("quest_ip_address")
    quest_port = LaunchConfiguration("quest_port")
    base_frame = LaunchConfiguration("base_frame")
    current_pose_topic = LaunchConfiguration("current_pose_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")

    default_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "config", "simple_impedance_teleop.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("quest_port", default_value="5555"),
            DeclareLaunchArgument("base_frame", default_value="fr3_link0"),
            DeclareLaunchArgument(
                "current_pose_topic",
                default_value="/franka_robot_state_broadcaster/current_pose",
            ),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/cartesian_impedance_controller/equilibrium_pose",
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="simple_quest_impedance_teleop_node",
                name="simple_quest_impedance_teleop",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "ip_address": quest_ip_address,
                        "port": ParameterValue(quest_port, value_type=int),
                        "base_frame": base_frame,
                        "current_pose_topic": current_pose_topic,
                        "target_pose_topic": target_pose_topic,
                    },
                ],
            ),
        ]
    )
