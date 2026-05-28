from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    server_host = LaunchConfiguration("server_host")
    server_port = LaunchConfiguration("server_port")
    helper_path = PathJoinSubstitution(
        [FindPackagePrefix("serl_franka_controllers_ros2"), "lib", "serl_franka_controllers_ros2", "libfranka_http_tool"]
    )
    server_path = PathJoinSubstitution(
        [FindPackagePrefix("serl_franka_controllers_ros2"), "lib", "serl_franka_controllers_ros2", "libfranka_http_server.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="Hostname or IP address of the Franka arm"),
            DeclareLaunchArgument(
                "server_host",
                default_value="0.0.0.0",
                description="Host interface used by the standalone libfranka HTTP bridge",
            ),
            DeclareLaunchArgument(
                "server_port",
                default_value="5001",
                description="TCP port used by the standalone libfranka HTTP bridge",
            ),
            ExecuteProcess(
                cmd=[
                    server_path,
                    "--robot-ip",
                    robot_ip,
                    "--host",
                    server_host,
                    "--port",
                    server_port,
                    "--helper-path",
                    helper_path,
                ],
                output="screen",
            ),
        ]
    )
