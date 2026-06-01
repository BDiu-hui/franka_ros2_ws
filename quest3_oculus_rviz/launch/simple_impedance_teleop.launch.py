from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    start_impedance_stack = LaunchConfiguration("start_impedance_stack")
    robot_type = LaunchConfiguration("robot_type")
    arm_prefix = LaunchConfiguration("arm_prefix")
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    start_rviz = LaunchConfiguration("start_rviz")
    start_impedance_controller = LaunchConfiguration("start_impedance_controller")
    server_host = LaunchConfiguration("server_host")
    server_port = LaunchConfiguration("server_port")
    controller_manager = LaunchConfiguration("controller_manager")
    auto_clear_error = LaunchConfiguration("auto_clear_error")
    auto_start_impedance = LaunchConfiguration("auto_start_impedance")
    auto_start_delay_sec = LaunchConfiguration("auto_start_delay_sec")
    auto_recover_after_reflex = LaunchConfiguration("auto_recover_after_reflex")
    recovery_watchdog_cooldown_sec = LaunchConfiguration("recovery_watchdog_cooldown_sec")
    pose_fallback_to_ik = LaunchConfiguration("pose_fallback_to_ik")
    pose_auto_activate_impedance = LaunchConfiguration("pose_auto_activate_impedance")
    pose_ik_timeout_sec = LaunchConfiguration("pose_ik_timeout_sec")
    pose_fallback_goal_tolerance = LaunchConfiguration("pose_fallback_goal_tolerance")
    quest_ip_address = LaunchConfiguration("quest_ip_address")
    quest_port = LaunchConfiguration("quest_port")
    base_frame = LaunchConfiguration("base_frame")
    current_pose_topic = LaunchConfiguration("current_pose_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")

    default_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "config", "simple_impedance_teleop.yaml"]
    )
    http_control_launch = PathJoinSubstitution(
        [FindPackageShare("serl_franka_controllers_ros2"), "launch", "http_control.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "start_impedance_stack",
                default_value="true",
                description="Whether to launch Franka hardware plus the SERL HTTP impedance stack",
            ),
            DeclareLaunchArgument("robot_type", default_value="fr3"),
            DeclareLaunchArgument("arm_prefix", default_value=""),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("fake_sensor_commands", default_value="false"),
            DeclareLaunchArgument("joint_state_rate", default_value="30"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("start_impedance_controller", default_value="false"),
            DeclareLaunchArgument("server_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("server_port", default_value="5000"),
            DeclareLaunchArgument("controller_manager", default_value="controller_manager"),
            DeclareLaunchArgument("auto_clear_error", default_value="false"),
            DeclareLaunchArgument(
                "auto_start_impedance",
                default_value="true",
                description="Automatically run the equivalent of POST /startimp after startup",
            ),
            DeclareLaunchArgument("auto_start_delay_sec", default_value="8.0"),
            DeclareLaunchArgument(
                "auto_recover_after_reflex",
                default_value="true",
                description="Automatically clear Franka reflex errors and restart impedance",
            ),
            DeclareLaunchArgument("recovery_watchdog_cooldown_sec", default_value="1.5"),
            DeclareLaunchArgument("pose_fallback_to_ik", default_value="false"),
            DeclareLaunchArgument("pose_auto_activate_impedance", default_value="false"),
            DeclareLaunchArgument("pose_ik_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("pose_fallback_goal_tolerance", default_value="0.005"),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("quest_port", default_value="5555"),
            DeclareLaunchArgument("base_frame", default_value="base"),
            DeclareLaunchArgument(
                "current_pose_topic",
                default_value="/franka_robot_state_broadcaster/current_pose",
            ),
            DeclareLaunchArgument(
                "target_pose_topic",
                default_value="/cartesian_impedance_controller/equilibrium_pose",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(http_control_launch),
                launch_arguments={
                    "robot_type": robot_type,
                    "arm_prefix": arm_prefix,
                    "namespace": namespace,
                    "robot_ip": robot_ip,
                    "load_gripper": load_gripper,
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "joint_state_rate": joint_state_rate,
                    "start_rviz": start_rviz,
                    "start_impedance_controller": start_impedance_controller,
                    "server_host": server_host,
                    "server_port": server_port,
                    "base_frame": base_frame,
                    "controller_manager": controller_manager,
                    "auto_clear_error": auto_clear_error,
                    "auto_start_impedance": auto_start_impedance,
                    "auto_start_delay_sec": auto_start_delay_sec,
                    "auto_recover_after_reflex": auto_recover_after_reflex,
                    "recovery_watchdog_cooldown_sec": recovery_watchdog_cooldown_sec,
                    "pose_fallback_to_ik": pose_fallback_to_ik,
                    "pose_auto_activate_impedance": pose_auto_activate_impedance,
                    "pose_ik_timeout_sec": pose_ik_timeout_sec,
                    "pose_fallback_goal_tolerance": pose_fallback_goal_tolerance,
                }.items(),
                condition=IfCondition(start_impedance_stack),
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
