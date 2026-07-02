from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _taskset_prefix(cpu):
    return PythonExpression(["'taskset -c ", cpu, "' if '", cpu, "' else ''"])


def _extra_controller_spawners(context, namespace, controllers_yaml):
    resolved_namespace = namespace.perform(context)
    resolved_controllers_yaml = controllers_yaml.perform(context)
    return [
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=resolved_namespace,
                    arguments=[
                        "cartesian_pose_command_controller",
                        "--inactive",
                        "--param-file",
                        resolved_controllers_yaml,
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    namespace=resolved_namespace,
                    arguments=[
                        "joint_position_controller",
                        "--inactive",
                        "--param-file",
                        resolved_controllers_yaml,
                        "--controller-manager-timeout",
                        "30",
                    ],
                    output="screen",
                ),
            ],
        ),
    ]


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    arm_prefix = LaunchConfiguration("arm_prefix")
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    ros2_control_cpu = LaunchConfiguration("ros2_control_cpu")
    franka_aux_cpu = LaunchConfiguration("franka_aux_cpu")
    http_server_cpu = LaunchConfiguration("http_server_cpu")
    watchdog_cpu = LaunchConfiguration("watchdog_cpu")
    start_rviz = LaunchConfiguration("start_rviz")
    start_impedance_controller = LaunchConfiguration("start_impedance_controller")
    server_host = LaunchConfiguration("server_host")
    server_port = LaunchConfiguration("server_port")
    base_frame = LaunchConfiguration("base_frame")
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
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("serl_franka_controllers_ros2"), "config", "serl_franka_controllers.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type", default_value="fr3", description="Franka robot type, e.g. panda or fr3"
            ),
            DeclareLaunchArgument(
                "arm_prefix", default_value="", description="Optional joint and frame prefix for the robot"
            ),
            DeclareLaunchArgument(
                "namespace", default_value="", description="Optional ROS namespace for the launched robot"
            ),
            DeclareLaunchArgument("robot_ip", description="Hostname or IP address of the Franka arm"),
            DeclareLaunchArgument(
                "load_gripper", default_value="false", description="Whether to launch the Franka gripper stack"
            ),
            DeclareLaunchArgument(
                "use_fake_hardware", default_value="false", description="Whether to use ros2_control fake hardware"
            ),
            DeclareLaunchArgument(
                "fake_sensor_commands",
                default_value="false",
                description="Whether fake hardware should accept sensor command interfaces",
            ),
            DeclareLaunchArgument(
                "joint_state_rate", default_value="30", description="Joint state publisher rate in Hz"
            ),
            DeclareLaunchArgument(
                "ros2_control_cpu",
                default_value="",
                description="Optional CPU list for ros2_control_node, e.g. '2' or '2-3'",
            ),
            DeclareLaunchArgument(
                "franka_aux_cpu",
                default_value="",
                description="Optional CPU list for non-control Franka helper processes",
            ),
            DeclareLaunchArgument(
                "http_server_cpu",
                default_value="",
                description="Optional CPU list for serl_franka_http_server.py",
            ),
            DeclareLaunchArgument(
                "watchdog_cpu",
                default_value="",
                description="Optional CPU list for franka_error_recovery_watchdog.py",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="false",
                description="Whether to launch RViz together with the HTTP server",
            ),
            DeclareLaunchArgument(
                "start_impedance_controller",
                default_value="false",
                description="Whether to activate Cartesian impedance immediately during HTTP launch",
            ),
            DeclareLaunchArgument(
                "server_host",
                default_value="0.0.0.0",
                description="Host interface used by the HTTP bridge server",
            ),
            DeclareLaunchArgument(
                "server_port",
                default_value="5000",
                description="TCP port used by the HTTP bridge server",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="fr3_link0",
                description="Base frame used in pose commands published by the HTTP bridge",
            ),
            DeclareLaunchArgument(
                "pose_fallback_to_ik",
                default_value="false",
                description="Enable /pose fallback via MoveIt IK plus PTP motion when impedance is inactive",
            ),
            DeclareLaunchArgument(
                "pose_auto_activate_impedance",
                default_value="false",
                description="Automatically activate Cartesian impedance when /pose is called while impedance is inactive",
            ),
            DeclareLaunchArgument(
                "pose_ik_timeout_sec",
                default_value="5.0",
                description="Timeout for the compute_ik service used by the HTTP pose fallback",
            ),
            DeclareLaunchArgument(
                "pose_fallback_goal_tolerance",
                default_value="0.005",
                description="Goal tolerance for the PTP motion used by the HTTP pose fallback",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="controller_manager",
                description="Controller manager node name. Relative names are resolved inside namespace.",
            ),
            DeclareLaunchArgument(
                "auto_clear_error",
                default_value="false",
                description="Whether the HTTP bridge should run Franka error recovery once after startup",
            ),
            DeclareLaunchArgument(
                "auto_start_impedance",
                default_value="false",
                description="Whether the HTTP bridge should activate Cartesian impedance after startup",
            ),
            DeclareLaunchArgument(
                "auto_start_delay_sec",
                default_value="8.0",
                description="Delay before automatic error recovery or impedance activation",
            ),
            DeclareLaunchArgument(
                "auto_recover_after_reflex",
                default_value="false",
                description="Whether to clear Franka reflex errors and restart impedance automatically",
            ),
            DeclareLaunchArgument(
                "recovery_watchdog_cooldown_sec",
                default_value="1.5",
                description="Minimum seconds between automatic reflex recovery attempts",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("serl_franka_controllers_ros2"), "launch", "impedance.launch.py"]
                    )
                ),
                launch_arguments={
                    "robot_type": robot_type,
                    "arm_prefix": arm_prefix,
                    "namespace": namespace,
                    "robot_ip": robot_ip,
                    "load_gripper": load_gripper,
                    "use_fake_hardware": use_fake_hardware,
                    "fake_sensor_commands": fake_sensor_commands,
                    "joint_state_rate": joint_state_rate,
                    "ros2_control_cpu": ros2_control_cpu,
                    "franka_aux_cpu": franka_aux_cpu,
                    "start_rviz": start_rviz,
                    "start_impedance_controller": start_impedance_controller,
                }.items(),
            ),
            OpaqueFunction(
                function=lambda context: _extra_controller_spawners(
                    context, namespace, controllers_yaml
                )
            ),
            Node(
                package="serl_franka_controllers_ros2",
                executable="serl_franka_http_server.py",
                namespace=namespace,
                output="screen",
                prefix=_taskset_prefix(http_server_cpu),
                parameters=[
                    controllers_yaml,
                    {
                        "host": server_host,
                        "port": server_port,
                        "base_frame": base_frame,
                        "robot_type": robot_type,
                        "arm_prefix": arm_prefix,
                        "load_gripper": load_gripper,
                        "controller_manager": controller_manager,
                        "impedance_controller": "cartesian_impedance_controller",
                        "joint_controller": "joint_position_controller",
                        "current_pose_topic": "franka_robot_state_broadcaster/current_pose",
                        "stiffness_wrench_topic": "franka_robot_state_broadcaster/external_wrench_in_stiffness_frame",
                        "measured_joint_states_topic": "franka_robot_state_broadcaster/measured_joint_states",
                        "franka_state_topic": "franka_robot_state_broadcaster/robot_state",
                        "jacobian_topic": "cartesian_impedance_controller/franka_jacobian",
                        "equilibrium_pose_topic": "cartesian_impedance_controller/equilibrium_pose",
                        "error_recovery_action": "action_server/error_recovery",
                        "collision_behavior_service": "service_server/set_full_collision_behavior",
                        "auto_clear_error": auto_clear_error,
                        "auto_start_impedance": auto_start_impedance,
                        "auto_start_delay_sec": auto_start_delay_sec,
                        "auto_start_wait_timeout_sec": 20.0,
                        "pose_fallback_to_ik": pose_fallback_to_ik,
                        "pose_auto_activate_impedance": pose_auto_activate_impedance,
                        "pose_ik_timeout_sec": pose_ik_timeout_sec,
                        "pose_fallback_goal_tolerance": pose_fallback_goal_tolerance,
                    }
                ],
            ),
            Node(
                package="serl_franka_controllers_ros2",
                executable="franka_error_recovery_watchdog.py",
                namespace=namespace,
                output="screen",
                prefix=_taskset_prefix(watchdog_cpu),
                parameters=[
                    {
                        "enabled": ParameterValue(auto_recover_after_reflex, value_type=bool),
                        "controller_manager": controller_manager,
                        "error_recovery_action": "action_server/error_recovery",
                        "franka_state_topic": "franka_robot_state_broadcaster/robot_state",
                        "impedance_controller": "cartesian_impedance_controller",
                        "deactivate_controllers_on_restart": [
                            "cartesian_pose_command_controller",
                            "joint_position_controller",
                            "fr3_arm_controller",
                        ],
                        "deactivate_before_recovery": True,
                        "restart_impedance_after_recovery": True,
                        "cooldown_sec": ParameterValue(recovery_watchdog_cooldown_sec, value_type=float),
                    }
                ],
                condition=IfCondition(auto_recover_after_reflex),
            ),
        ]
    )
