from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    server_host = LaunchConfiguration("server_host")
    server_port = LaunchConfiguration("server_port")
    right_server_port = LaunchConfiguration("right_server_port")
    left_helper_cpu = LaunchConfiguration("left_helper_cpu")
    right_helper_cpu = LaunchConfiguration("right_helper_cpu")
    package_prefix = FindPackagePrefix("serl_franka_controllers_ros2")
    helper_path = PathJoinSubstitution(
        [package_prefix, "lib", "serl_franka_controllers_ros2", "libfranka_http_tool"]
    )
    server_path = PathJoinSubstitution(
        [package_prefix, "lib", "serl_franka_controllers_ros2", "libfranka_http_server.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3"),
            DeclareLaunchArgument("server_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("server_port", default_value="5000"),
            DeclareLaunchArgument("right_server_port", default_value="5001"),
            DeclareLaunchArgument("left_helper_cpu", default_value=""),
            DeclareLaunchArgument("right_helper_cpu", default_value=""),
            ExecuteProcess(
                cmd=[
                    server_path,
                    "--left-robot-ip",
                    left_robot_ip,
                    "--right-robot-ip",
                    right_robot_ip,
                    "--host",
                    server_host,
                    "--left-port",
                    server_port,
                    "--right-port",
                    right_server_port,
                    "--helper-path",
                    helper_path,
                    "--left-helper-cpu",
                    left_helper_cpu,
                    "--right-helper-cpu",
                    right_helper_cpu,
                ],
                output="screen",
            ),
        ]
    )
