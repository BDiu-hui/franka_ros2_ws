import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
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
    base_frame = LaunchConfiguration("base_frame")
    controller_manager = LaunchConfiguration("controller_manager")
    auto_clear_error = LaunchConfiguration("auto_clear_error")
    auto_start_impedance = LaunchConfiguration("auto_start_impedance")
    auto_start_delay_sec = LaunchConfiguration("auto_start_delay_sec")
    pose_fallback_to_ik = LaunchConfiguration("pose_fallback_to_ik")
    pose_auto_activate_impedance = LaunchConfiguration("pose_auto_activate_impedance")
    pose_ik_timeout_sec = LaunchConfiguration("pose_ik_timeout_sec")
    pose_fallback_goal_tolerance = LaunchConfiguration("pose_fallback_goal_tolerance")

    franka_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.urdf.xacro",
    )
    robot_description_config = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            franka_xacro_file,
            " hand:=",
            load_gripper,
            " robot_ip:=",
            robot_ip,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " ros2_control:=true",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_config, value_type=str)}

    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3.srdf.xacro",
    )
    robot_description_semantic_config = Command(
        [FindExecutable(name="xacro"), " ", franka_semantic_xacro_file, " hand:=", load_gripper]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_config, value_type=str)
    }

    kinematics_yaml = load_yaml("franka_fr3_moveit_config", "config/kinematics.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml("franka_fr3_moveit_config", "config/fr3_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("serl_franka_controllers_ros2"), "config", "serl_franka_controllers.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="fr3",
                description="Franka robot type. This launch is intended for fr3 Cartesian precise control.",
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
                default_value="3.0",
                description="Delay before automatic error recovery or impedance activation",
            ),
            DeclareLaunchArgument(
                "pose_fallback_to_ik",
                default_value="false",
                description="Keep the older IK-plus-PTP fallback disabled by default.",
            ),
            DeclareLaunchArgument(
                "pose_auto_activate_impedance",
                default_value="false",
                description="Keep /pose bound to impedance mode only unless explicitly enabled.",
            ),
            DeclareLaunchArgument(
                "pose_ik_timeout_sec",
                default_value="5.0",
                description="Timeout for the older compute_ik fallback path.",
            ),
            DeclareLaunchArgument(
                "pose_fallback_goal_tolerance",
                default_value="0.005",
                description="Goal tolerance for the older IK-plus-PTP fallback path.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("serl_franka_controllers_ros2"), "launch", "http_control.launch.py"]
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
                    "start_rviz": start_rviz,
                    "start_impedance_controller": start_impedance_controller,
                    "server_host": server_host,
                    "server_port": server_port,
                    "base_frame": base_frame,
                    "controller_manager": controller_manager,
                    "auto_clear_error": auto_clear_error,
                    "auto_start_impedance": auto_start_impedance,
                    "auto_start_delay_sec": auto_start_delay_sec,
                    "pose_fallback_to_ik": pose_fallback_to_ik,
                    "pose_auto_activate_impedance": pose_auto_activate_impedance,
                    "pose_ik_timeout_sec": pose_ik_timeout_sec,
                    "pose_fallback_goal_tolerance": pose_fallback_goal_tolerance,
                }.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "fr3_arm_controller",
                    "--inactive",
                    "--param-file",
                    controllers_yaml,
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                namespace=namespace,
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    ompl_planning_pipeline_config,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor_parameters,
                ],
                remappings=[("joint_states", "franka/joint_states")],
            ),
        ]
    )
