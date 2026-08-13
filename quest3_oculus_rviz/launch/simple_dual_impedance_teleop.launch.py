from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _taskset_prefix(cpu):
    return PythonExpression(["'taskset -c ", cpu, "' if '", cpu, "' else ''"])


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
    ros2_control_cpu,
    franka_aux_cpu,
    http_server_cpu,
    watchdog_cpu,
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
            "ros2_control_cpu": ros2_control_cpu,
            "franka_aux_cpu": franka_aux_cpu,
            "http_server_cpu": http_server_cpu,
            "watchdog_cpu": watchdog_cpu,
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


def simple_teleop_node(*, name: str, config_file, condition, cpu):
    return Node(
        package="quest3_oculus_rviz",
        executable="simple_quest_impedance_teleop_node",
        name=name,
        output="screen",
        prefix=_taskset_prefix(cpu),
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
    start_quest_reader = LaunchConfiguration("start_quest_reader")
    start_rviz = LaunchConfiguration("start_rviz")
    base_frame = LaunchConfiguration("base_frame")
    auto_start_impedance = LaunchConfiguration("auto_start_impedance")
    auto_start_delay_sec = LaunchConfiguration("auto_start_delay_sec")
    auto_recover_after_reflex = LaunchConfiguration("auto_recover_after_reflex")
    recovery_watchdog_cooldown_sec = LaunchConfiguration("recovery_watchdog_cooldown_sec")
    left_server_port = LaunchConfiguration("left_server_port")
    right_server_port = LaunchConfiguration("right_server_port")
    left_ros2_control_cpu = LaunchConfiguration("left_ros2_control_cpu")
    left_franka_aux_cpu = LaunchConfiguration("left_franka_aux_cpu")
    left_http_server_cpu = LaunchConfiguration("left_http_server_cpu")
    left_watchdog_cpu = LaunchConfiguration("left_watchdog_cpu")
    right_ros2_control_cpu = LaunchConfiguration("right_ros2_control_cpu")
    right_franka_aux_cpu = LaunchConfiguration("right_franka_aux_cpu")
    right_http_server_cpu = LaunchConfiguration("right_http_server_cpu")
    right_watchdog_cpu = LaunchConfiguration("right_watchdog_cpu")
    mock = LaunchConfiguration("mock")
    quest_ip_address = LaunchConfiguration("quest_ip_address")
    quest_port = LaunchConfiguration("quest_port")
    quest_publish_rate_hz = LaunchConfiguration("quest_publish_rate_hz")
    quest_reader_cpu = LaunchConfiguration("quest_reader_cpu")
    left_teleop_cpu = LaunchConfiguration("left_teleop_cpu")
    right_teleop_cpu = LaunchConfiguration("right_teleop_cpu")
    start_wuji_trigger_hand = LaunchConfiguration("start_wuji_trigger_hand")
    start_wujihand_driver = LaunchConfiguration("start_wujihand_driver")
    wuji_config_file = LaunchConfiguration("wuji_config_file")
    wuji_control_mode = LaunchConfiguration("wuji_control_mode")
    left_wuji_enabled = LaunchConfiguration("left_wuji_enabled")
    right_wuji_enabled = LaunchConfiguration("right_wuji_enabled")
    left_wuji_serial = LaunchConfiguration("left_wuji_serial")
    right_wuji_serial = LaunchConfiguration("right_wuji_serial")
    wuji_dry_run = LaunchConfiguration("wuji_dry_run")
    wujihand_state_rate = LaunchConfiguration("wujihand_state_rate")
    left_wujihand_driver_cpu = LaunchConfiguration("left_wujihand_driver_cpu")
    right_wujihand_driver_cpu = LaunchConfiguration("right_wujihand_driver_cpu")
    wuji_cpu = LaunchConfiguration("wuji_cpu")
    start_data_recorder = LaunchConfiguration("start_data_recorder")
    data_recorder_config_file = LaunchConfiguration("data_recorder_config_file")
    out_data_dir = LaunchConfiguration("out_data_dir")
    require_cameras = LaunchConfiguration("require_cameras")
    data_recorder_cpu = LaunchConfiguration("data_recorder_cpu")

    default_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "config", "simple_dual_impedance_teleop.yaml"]
    )
    default_wuji_config = PathJoinSubstitution(
        [
            FindPackageShare("quest3_oculus_rviz"),
            "config",
            "wuji_trigger_hand.yaml",
        ]
    )
    default_data_recorder_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "config", "data_recorder.yaml"]
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
            DeclareLaunchArgument("start_quest_reader", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base"),
            DeclareLaunchArgument("auto_start_impedance", default_value="true"),
            DeclareLaunchArgument("auto_start_delay_sec", default_value="8.0"),
            DeclareLaunchArgument("auto_recover_after_reflex", default_value="true"),
            DeclareLaunchArgument("recovery_watchdog_cooldown_sec", default_value="1.5"),
            DeclareLaunchArgument("left_server_port", default_value="5000"),
            DeclareLaunchArgument("right_server_port", default_value="5001"),
            DeclareLaunchArgument("left_ros2_control_cpu", default_value=""),
            DeclareLaunchArgument("left_franka_aux_cpu", default_value=""),
            DeclareLaunchArgument("left_http_server_cpu", default_value=""),
            DeclareLaunchArgument("left_watchdog_cpu", default_value=""),
            DeclareLaunchArgument("right_ros2_control_cpu", default_value=""),
            DeclareLaunchArgument("right_franka_aux_cpu", default_value=""),
            DeclareLaunchArgument("right_http_server_cpu", default_value=""),
            DeclareLaunchArgument("right_watchdog_cpu", default_value=""),
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("quest_port", default_value="5555"),
            DeclareLaunchArgument("quest_publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("quest_reader_cpu", default_value=""),
            DeclareLaunchArgument("left_teleop_cpu", default_value=""),
            DeclareLaunchArgument("right_teleop_cpu", default_value=""),
            DeclareLaunchArgument("start_wuji_trigger_hand", default_value="false"),
            DeclareLaunchArgument("start_wujihand_driver", default_value="false"),
            DeclareLaunchArgument("wuji_config_file", default_value=default_wuji_config),
            DeclareLaunchArgument("wuji_control_mode", default_value="trigger"),
            DeclareLaunchArgument("left_wuji_enabled", default_value="false"),
            DeclareLaunchArgument("right_wuji_enabled", default_value="true"),
            DeclareLaunchArgument("left_wuji_serial", default_value="348534683533"),
            DeclareLaunchArgument("right_wuji_serial", default_value="3671354F3333"),
            DeclareLaunchArgument("wuji_dry_run", default_value="false"),
            DeclareLaunchArgument("wujihand_state_rate", default_value="1000.0"),
            DeclareLaunchArgument("left_wujihand_driver_cpu", default_value=""),
            DeclareLaunchArgument("right_wujihand_driver_cpu", default_value=""),
            DeclareLaunchArgument("wuji_cpu", default_value=""),
            DeclareLaunchArgument("start_data_recorder", default_value="false"),
            DeclareLaunchArgument(
                "data_recorder_config_file",
                default_value=default_data_recorder_config,
            ),
            DeclareLaunchArgument(
                "out_data_dir",
                default_value="/tmp/quest3_recordings",
            ),
            DeclareLaunchArgument("require_cameras", default_value="true"),
            DeclareLaunchArgument("data_recorder_cpu", default_value=""),
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
                ros2_control_cpu=left_ros2_control_cpu,
                franka_aux_cpu=left_franka_aux_cpu,
                http_server_cpu=left_http_server_cpu,
                watchdog_cpu=left_watchdog_cpu,
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
                ros2_control_cpu=right_ros2_control_cpu,
                franka_aux_cpu=right_franka_aux_cpu,
                http_server_cpu=right_http_server_cpu,
                watchdog_cpu=right_watchdog_cpu,
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="oculus_tf_node",
                name="quest3_oculus_tf",
                output="screen",
                prefix=_taskset_prefix(quest_reader_cpu),
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
                condition=IfCondition(start_quest_reader),
            ),
            simple_teleop_node(
                name="left_simple_quest_impedance_teleop",
                config_file=config_file,
                condition=start_left_teleop,
                cpu=left_teleop_cpu,
            ),
            simple_teleop_node(
                name="right_simple_quest_impedance_teleop",
                config_file=config_file,
                condition=start_right_teleop,
                cpu=right_teleop_cpu,
            ),
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace="hand_left",
                output="screen",
                emulate_tty=True,
                prefix=_taskset_prefix(left_wujihand_driver_cpu),
                parameters=[
                    {
                        "serial_number": ParameterValue(
                            left_wuji_serial,
                            value_type=str,
                        ),
                        "publish_rate": ParameterValue(
                            wujihand_state_rate,
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_wujihand_driver,
                            "' == 'true' and '",
                            left_wuji_enabled,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="wujihand_driver",
                executable="wujihand_driver_node",
                name="wujihand_driver",
                namespace="hand_right",
                output="screen",
                emulate_tty=True,
                prefix=_taskset_prefix(right_wujihand_driver_cpu),
                parameters=[
                    {
                        "serial_number": ParameterValue(
                            right_wuji_serial,
                            value_type=str,
                        ),
                        "publish_rate": ParameterValue(
                            wujihand_state_rate,
                            value_type=float,
                        ),
                    }
                ],
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_wujihand_driver,
                            "' == 'true' and '",
                            right_wuji_enabled,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="wuji_trigger_hand_node",
                name="wuji_trigger_hand",
                output="screen",
                parameters=[
                    wuji_config_file,
                    {
                        "left_enabled": ParameterValue(
                            left_wuji_enabled,
                            value_type=bool,
                        ),
                        "right_enabled": ParameterValue(
                            right_wuji_enabled,
                            value_type=bool,
                        ),
                        "left_serial": ParameterValue(
                            left_wuji_serial,
                            value_type=str,
                        ),
                        "right_serial": ParameterValue(
                            right_wuji_serial,
                            value_type=str,
                        ),
                        "dry_run": ParameterValue(
                            wuji_dry_run,
                            value_type=bool,
                        ),
                        "control_mode": ParameterValue(
                            wuji_control_mode,
                            value_type=str,
                        ),
                    },
                ],
                condition=IfCondition(start_wuji_trigger_hand),
                prefix=_taskset_prefix(wuji_cpu),
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="data_recorder_node",
                name="quest3_data_recorder",
                output="screen",
                prefix=_taskset_prefix(data_recorder_cpu),
                parameters=[
                    data_recorder_config_file,
                    {
                        "out_data_dir": out_data_dir,
                        "require_cameras": ParameterValue(
                            require_cameras,
                            value_type=bool,
                        ),
                    },
                ],
                condition=IfCondition(start_data_recorder),
            ),
        ]
    )
