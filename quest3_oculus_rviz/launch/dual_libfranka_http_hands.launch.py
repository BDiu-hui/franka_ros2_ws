from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_server_port = LaunchConfiguration("left_server_port")
    right_server_port = LaunchConfiguration("right_server_port")
    left_helper_cpu = LaunchConfiguration("left_helper_cpu")
    right_helper_cpu = LaunchConfiguration("right_helper_cpu")
    left_wuji_serial = LaunchConfiguration("left_wuji_serial")
    right_wuji_serial = LaunchConfiguration("right_wuji_serial")
    wujihand_state_rate = LaunchConfiguration("wujihand_state_rate")
    left_wujihand_driver_cpu = LaunchConfiguration("left_wujihand_driver_cpu")
    right_wujihand_driver_cpu = LaunchConfiguration("right_wujihand_driver_cpu")
    wuji_cpu = LaunchConfiguration("wuji_cpu")

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3"),
            DeclareLaunchArgument("left_server_port", default_value="5000"),
            DeclareLaunchArgument("right_server_port", default_value="5001"),
            DeclareLaunchArgument("left_helper_cpu", default_value="2-3"),
            DeclareLaunchArgument("right_helper_cpu", default_value="8-9"),
            DeclareLaunchArgument("left_wuji_serial", default_value="348534683533"),
            DeclareLaunchArgument("right_wuji_serial", default_value="3671354F3333"),
            DeclareLaunchArgument("wujihand_state_rate", default_value="200.0"),
            DeclareLaunchArgument("left_wujihand_driver_cpu", default_value="24"),
            DeclareLaunchArgument("right_wujihand_driver_cpu", default_value="25"),
            DeclareLaunchArgument("wuji_cpu", default_value="20"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("serl_franka_controllers_ros2"),
                            "launch",
                            "dual_libfranka_http.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "left_robot_ip": left_robot_ip,
                    "right_robot_ip": right_robot_ip,
                    "server_port": left_server_port,
                    "right_server_port": right_server_port,
                    "left_helper_cpu": left_helper_cpu,
                    "right_helper_cpu": right_helper_cpu,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("quest3_oculus_rviz"),
                            "launch",
                            "simple_dual_impedance_teleop.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "start_left_arm": "false",
                    "start_right_arm": "false",
                    "start_left_teleop": "false",
                    "start_right_teleop": "false",
                    "start_quest_reader": "false",
                    "start_rviz": "false",
                    "start_data_recorder": "false",
                    "start_wuji_trigger_hand": "true",
                    "start_wujihand_driver": "true",
                    "wuji_control_mode": "service",
                    "left_wuji_enabled": "true",
                    "right_wuji_enabled": "true",
                    "left_wuji_serial": left_wuji_serial,
                    "right_wuji_serial": right_wuji_serial,
                    "wujihand_state_rate": wujihand_state_rate,
                    "left_wujihand_driver_cpu": left_wujihand_driver_cpu,
                    "right_wujihand_driver_cpu": right_wujihand_driver_cpu,
                    "wuji_cpu": wuji_cpu,
                }.items(),
            ),
        ]
    )
