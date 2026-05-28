from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    controller_manager = LaunchConfiguration("controller_manager")
    controllers_file = LaunchConfiguration("controllers_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace", default_value="", description="Optional ROS namespace for the controller spawners"
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Fully qualified controller_manager node name",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("serl_franka_controllers_ros2"), "config", "serl_franka_controllers.yaml"]
                ),
                description="Controller parameter YAML used by the spawner",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "joint_position_controller",
                    "--controller-manager",
                    controller_manager,
                    "--param-file",
                    controllers_file,
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "cartesian_impedance_controller",
                    "--controller-manager",
                    controller_manager,
                    "--param-file",
                    controllers_file,
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
            ),
        ]
    )
