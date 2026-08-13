"""One hardware/impedance stack shared by policy and Quest control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _taskset_prefix(cpu):
    return PythonExpression(["'taskset -c ", cpu, "' if '", cpu, "' else ''"])


def _arm_stack(side: str, robot_ip, backend_port, ros2_cpu, aux_cpu, http_cpu, watchdog_cpu):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("serl_franka_controllers_ros2"), "launch", "http_control.launch.py"]
            )
        ),
        launch_arguments={
            "robot_type": "fr3",
            "arm_prefix": "",
            "namespace": side,
            "robot_ip": robot_ip,
            "load_gripper": "false",
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
            "joint_state_rate": "30",
            "ros2_control_cpu": ros2_cpu,
            "franka_aux_cpu": aux_cpu,
            "http_server_cpu": http_cpu,
            "watchdog_cpu": watchdog_cpu,
            "start_rviz": "false",
            "start_impedance_controller": "false",
            # Backend ports are loopback-only. Policy clients can only use the
            # authority-gated public ports 5000/5001.
            "server_host": "127.0.0.1",
            "server_port": backend_port,
            "base_frame": "base",
            "controller_manager": "controller_manager",
            "auto_clear_error": "false",
            "auto_start_impedance": "true",
            "auto_start_delay_sec": "8.0",
            "auto_recover_after_reflex": "true",
            "recovery_watchdog_cooldown_sec": "1.5",
            "pose_fallback_to_ik": "false",
            "pose_auto_activate_impedance": "false",
            "pose_ik_timeout_sec": "5.0",
            "pose_fallback_goal_tolerance": "0.005",
        }.items(),
    )


def generate_launch_description():
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_backend_port = LaunchConfiguration("left_backend_port")
    right_backend_port = LaunchConfiguration("right_backend_port")
    left_wuji_serial = LaunchConfiguration("left_wuji_serial")
    right_wuji_serial = LaunchConfiguration("right_wuji_serial")
    wujihand_state_rate = LaunchConfiguration("wujihand_state_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3"),
            DeclareLaunchArgument("left_backend_port", default_value="5100"),
            DeclareLaunchArgument("right_backend_port", default_value="5101"),
            DeclareLaunchArgument("left_wuji_serial", default_value="348534683533"),
            DeclareLaunchArgument("right_wuji_serial", default_value="3671354F3333"),
            DeclareLaunchArgument("wujihand_state_rate", default_value="200.0"),
            DeclareLaunchArgument("left_ros2_control_cpu", default_value="2-3"),
            DeclareLaunchArgument("left_franka_aux_cpu", default_value="4"),
            DeclareLaunchArgument("left_http_server_cpu", default_value="6"),
            DeclareLaunchArgument("left_watchdog_cpu", default_value="6"),
            DeclareLaunchArgument("right_ros2_control_cpu", default_value="8-9"),
            DeclareLaunchArgument("right_franka_aux_cpu", default_value="10"),
            DeclareLaunchArgument("right_http_server_cpu", default_value="12"),
            DeclareLaunchArgument("right_watchdog_cpu", default_value="12"),
            DeclareLaunchArgument("left_wujihand_driver_cpu", default_value="24"),
            DeclareLaunchArgument("right_wujihand_driver_cpu", default_value="25"),
            Node(
                package="unified_impedance_control",
                executable="control_authority_node",
                name="unified_control_authority",
                output="screen",
                parameters=[
                    {
                        "left_backend_url": ParameterValue(
                            PythonExpression(["'http://127.0.0.1:", left_backend_port, "'"]),
                            value_type=str,
                        ),
                        "right_backend_url": ParameterValue(
                            PythonExpression(["'http://127.0.0.1:", right_backend_port, "'"]),
                            value_type=str,
                        ),
                    }
                ],
            ),
            _arm_stack(
                "left",
                left_robot_ip,
                left_backend_port,
                LaunchConfiguration("left_ros2_control_cpu"),
                LaunchConfiguration("left_franka_aux_cpu"),
                LaunchConfiguration("left_http_server_cpu"),
                LaunchConfiguration("left_watchdog_cpu"),
            ),
            _arm_stack(
                "right",
                right_robot_ip,
                right_backend_port,
                LaunchConfiguration("right_ros2_control_cpu"),
                LaunchConfiguration("right_franka_aux_cpu"),
                LaunchConfiguration("right_http_server_cpu"),
                LaunchConfiguration("right_watchdog_cpu"),
            ),
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace="hand_left",
                output="screen",
                emulate_tty=True,
                prefix=_taskset_prefix(LaunchConfiguration("left_wujihand_driver_cpu")),
                parameters=[
                    {
                        "serial_number": ParameterValue(left_wuji_serial, value_type=str),
                        "publish_rate": ParameterValue(wujihand_state_rate, value_type=float),
                    }
                ],
            ),
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace="hand_right",
                output="screen",
                emulate_tty=True,
                prefix=_taskset_prefix(LaunchConfiguration("right_wujihand_driver_cpu")),
                parameters=[
                    {
                        "serial_number": ParameterValue(right_wuji_serial, value_type=str),
                        "publish_rate": ParameterValue(wujihand_state_rate, value_type=float),
                    }
                ],
            ),
        ]
    )
