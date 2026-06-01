from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def http_control_include(
    *,
    condition,
    namespace: str,
    robot_ip,
    server_port,
    robot_type,
    load_gripper,
    start_rviz,
    base_frame,
    auto_start_impedance,
    auto_start_delay_sec,
    auto_recover_after_reflex,
    recovery_watchdog_cooldown_sec,
):
    http_control_launch = PathJoinSubstitution(
        [FindPackageShare("serl_franka_controllers_ros2"), "launch", "http_control.launch.py"]
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(http_control_launch),
        launch_arguments={
            "robot_type": robot_type,
            "arm_prefix": "",
            "namespace": namespace,
            "robot_ip": robot_ip,
            "load_gripper": load_gripper,
            "use_fake_hardware": "false",
            "fake_sensor_commands": "false",
            "joint_state_rate": "30",
            "start_rviz": start_rviz,
            "start_impedance_controller": "false",
            "server_host": "0.0.0.0",
            "server_port": server_port,
            "base_frame": base_frame,
            "controller_manager": "controller_manager",
            "auto_clear_error": "false",
            "auto_start_impedance": auto_start_impedance,
            "auto_start_delay_sec": auto_start_delay_sec,
            "auto_recover_after_reflex": auto_recover_after_reflex,
            "recovery_watchdog_cooldown_sec": recovery_watchdog_cooldown_sec,
            "pose_fallback_to_ik": "false",
            "pose_auto_activate_impedance": "false",
            "pose_ik_timeout_sec": "5.0",
            "pose_fallback_goal_tolerance": "0.005",
        }.items(),
        condition=IfCondition(condition),
    )


def simple_teleop_node(*, name: str, config_file, condition):
    return Node(
        package="quest3_oculus_rviz",
        executable="simple_quest_impedance_teleop_node",
        name=name,
        output="screen",
        parameters=[config_file],
        condition=IfCondition(condition),
    )


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    robot_type = LaunchConfiguration("robot_type")
    load_gripper = LaunchConfiguration("load_gripper")
    start_left_arm = LaunchConfiguration("start_left_arm")
    start_right_arm = LaunchConfiguration("start_right_arm")
    start_left_teleop = LaunchConfiguration("start_left_teleop")
    start_right_teleop = LaunchConfiguration("start_right_teleop")
    start_rviz = LaunchConfiguration("start_rviz")
    base_frame = LaunchConfiguration("base_frame")
    auto_start_impedance = LaunchConfiguration("auto_start_impedance")
    auto_start_delay_sec = LaunchConfiguration("auto_start_delay_sec")
    auto_recover_after_reflex = LaunchConfiguration("auto_recover_after_reflex")
    recovery_watchdog_cooldown_sec = LaunchConfiguration("recovery_watchdog_cooldown_sec")
    left_server_port = LaunchConfiguration("left_server_port")
    right_server_port = LaunchConfiguration("right_server_port")
    mock = LaunchConfiguration("mock")
    quest_ip_address = LaunchConfiguration("quest_ip_address")
    quest_port = LaunchConfiguration("quest_port")
    quest_publish_rate_hz = LaunchConfiguration("quest_publish_rate_hz")

    default_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "config", "simple_dual_impedance_teleop.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3"),
            DeclareLaunchArgument("robot_type", default_value="fr3"),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("start_left_arm", default_value="true"),
            DeclareLaunchArgument("start_right_arm", default_value="true"),
            DeclareLaunchArgument("start_left_teleop", default_value="true"),
            DeclareLaunchArgument("start_right_teleop", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base"),
            DeclareLaunchArgument("auto_start_impedance", default_value="true"),
            DeclareLaunchArgument("auto_start_delay_sec", default_value="3.0"),
            DeclareLaunchArgument("auto_recover_after_reflex", default_value="true"),
            DeclareLaunchArgument("recovery_watchdog_cooldown_sec", default_value="4.0"),
            DeclareLaunchArgument("left_server_port", default_value="5000"),
            DeclareLaunchArgument("right_server_port", default_value="5001"),
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("quest_port", default_value="5555"),
            DeclareLaunchArgument("quest_publish_rate_hz", default_value="50.0"),
            http_control_include(
                condition=start_left_arm,
                namespace="left",
                robot_ip=left_robot_ip,
                server_port=left_server_port,
                robot_type=robot_type,
                load_gripper=load_gripper,
                start_rviz=start_rviz,
                base_frame=base_frame,
                auto_start_impedance=auto_start_impedance,
                auto_start_delay_sec=auto_start_delay_sec,
                auto_recover_after_reflex=auto_recover_after_reflex,
                recovery_watchdog_cooldown_sec=recovery_watchdog_cooldown_sec,
            ),
            http_control_include(
                condition=start_right_arm,
                namespace="right",
                robot_ip=right_robot_ip,
                server_port=right_server_port,
                robot_type=robot_type,
                load_gripper=load_gripper,
                start_rviz=start_rviz,
                base_frame=base_frame,
                auto_start_impedance=auto_start_impedance,
                auto_start_delay_sec=auto_start_delay_sec,
                auto_recover_after_reflex=auto_recover_after_reflex,
                recovery_watchdog_cooldown_sec=recovery_watchdog_cooldown_sec,
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="oculus_tf_node",
                name="quest3_oculus_tf",
                output="screen",
                parameters=[
                    {
                        "mock": ParameterValue(mock, value_type=bool),
                        "ip_address": quest_ip_address,
                        "port": ParameterValue(quest_port, value_type=int),
                        "publish_rate_hz": ParameterValue(quest_publish_rate_hz, value_type=float),
                        "world_frame": "quest_raw",
                        "right_frame": "quest3_right_controller_raw",
                        "left_frame": "quest3_left_controller_raw",
                    }
                ],
            ),
            simple_teleop_node(
                name="left_simple_quest_impedance_teleop",
                config_file=config_file,
                condition=start_left_teleop,
            ),
            simple_teleop_node(
                name="right_simple_quest_impedance_teleop",
                config_file=config_file,
                condition=start_right_teleop,
            ),
        ]
    )
