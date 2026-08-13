"""Quest teleoperation, trigger-hand bridge, and the unchanged recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _taskset_prefix(cpu):
    return PythonExpression(["'taskset -c ", cpu, "' if '", cpu, "' else ''"])


def generate_launch_description():
    teleop_config = LaunchConfiguration("teleop_config_file")
    wuji_config = LaunchConfiguration("wuji_config_file")
    recorder_config = LaunchConfiguration("data_recorder_config_file")
    left_wuji_serial = LaunchConfiguration("left_wuji_serial")
    right_wuji_serial = LaunchConfiguration("right_wuji_serial")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "teleop_config_file",
                default_value="/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/simple_dual_impedance_teleop.yaml",
            ),
            DeclareLaunchArgument(
                "wuji_config_file",
                default_value="/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/wuji_trigger_hand.yaml",
            ),
            DeclareLaunchArgument(
                "data_recorder_config_file",
                default_value="/home/lumos/franka_ros2_ws/src/quest3_oculus_rviz/config/data_recorder.yaml",
            ),
            DeclareLaunchArgument("out_data_dir", default_value="/home/lumos/quest3_recordings"),
            DeclareLaunchArgument("left_wuji_serial", default_value="348534683533"),
            DeclareLaunchArgument("right_wuji_serial", default_value="3671354F3333"),
            DeclareLaunchArgument("require_cameras", default_value="true"),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("quest_port", default_value="5555"),
            DeclareLaunchArgument("quest_publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("quest_reader_cpu", default_value="14"),
            DeclareLaunchArgument("left_teleop_cpu", default_value="16"),
            DeclareLaunchArgument("right_teleop_cpu", default_value="18"),
            DeclareLaunchArgument("wuji_cpu", default_value="20"),
            DeclareLaunchArgument("data_recorder_cpu", default_value="22"),
            Node(
                package="quest3_oculus_rviz",
                executable="oculus_tf_node",
                name="quest3_oculus_tf",
                output="screen",
                prefix=_taskset_prefix(LaunchConfiguration("quest_reader_cpu")),
                parameters=[
                    {
                        "mock": False,
                        "ip_address": LaunchConfiguration("quest_ip_address"),
                        "port": ParameterValue(LaunchConfiguration("quest_port"), value_type=int),
                        "publish_rate_hz": ParameterValue(
                            LaunchConfiguration("quest_publish_rate_hz"), value_type=float
                        ),
                        "world_frame": "quest_raw",
                        "right_frame": "quest3_right_controller_raw",
                        "left_frame": "quest3_left_controller_raw",
                    }
                ],
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="simple_quest_impedance_teleop_node",
                name="left_simple_quest_impedance_teleop",
                output="screen",
                prefix=_taskset_prefix(LaunchConfiguration("left_teleop_cpu")),
                parameters=[
                    teleop_config,
                    {
                        "target_pose_topic": "/unified_impedance/teleop/left/equilibrium_pose",
                        "enabled_topic": "/unified_impedance/teleop/left/enabled",
                    },
                ],
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="simple_quest_impedance_teleop_node",
                name="right_simple_quest_impedance_teleop",
                output="screen",
                prefix=_taskset_prefix(LaunchConfiguration("right_teleop_cpu")),
                parameters=[
                    teleop_config,
                    {
                        "target_pose_topic": "/unified_impedance/teleop/right/equilibrium_pose",
                        "enabled_topic": "/unified_impedance/teleop/right/enabled",
                    },
                ],
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="wuji_trigger_hand_node",
                name="wuji_trigger_hand",
                output="screen",
                prefix=_taskset_prefix(LaunchConfiguration("wuji_cpu")),
                parameters=[
                    wuji_config,
                    {
                        "buttons_topic": "/unified_impedance/teleop/buttons",
                        "control_mode": "trigger",
                        "left_enabled": True,
                        "right_enabled": True,
                        "left_serial": ParameterValue(left_wuji_serial, value_type=str),
                        "right_serial": ParameterValue(right_wuji_serial, value_type=str),
                        "left_command_topic": "/unified_impedance/teleop/hand_left/joint_commands",
                        "right_command_topic": "/unified_impedance/teleop/hand_right/joint_commands",
                    },
                ],
            ),
            # This subclasses the original recorder and changes only camera
            # ownership. A/B/X, subscribed data, and HDF5 writing are reused.
            Node(
                package="unified_impedance_control",
                executable="authority_data_recorder_node",
                name="quest3_data_recorder",
                output="screen",
                prefix=_taskset_prefix(LaunchConfiguration("data_recorder_cpu")),
                parameters=[
                    recorder_config,
                    {
                        "out_data_dir": LaunchConfiguration("out_data_dir"),
                        "require_cameras": ParameterValue(
                            LaunchConfiguration("require_cameras"), value_type=bool
                        ),
                    },
                ],
            ),
        ]
    )
