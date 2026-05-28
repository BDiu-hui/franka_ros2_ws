from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    start_franka = LaunchConfiguration("start_franka")
    start_rviz = LaunchConfiguration("start_rviz")
    publish_world_to_base_tf = LaunchConfiguration("publish_world_to_base_tf")

    mock = LaunchConfiguration("mock")
    quest_ip_address = LaunchConfiguration("quest_ip_address")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    base_frame = LaunchConfiguration("base_frame")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    trigger_threshold = LaunchConfiguration("trigger_threshold")
    align_controller_frame_to_tcp = LaunchConfiguration("align_controller_frame_to_tcp")
    teleop_motion_mode = LaunchConfiguration("teleop_motion_mode")
    translation_scale = LaunchConfiguration("translation_scale")
    translation_x_sign = LaunchConfiguration("translation_x_sign")
    translation_y_sign = LaunchConfiguration("translation_y_sign")
    translation_z_sign = LaunchConfiguration("translation_z_sign")
    translation_deadband = LaunchConfiguration("translation_deadband_m")
    rotation_deadband = LaunchConfiguration("rotation_deadband_rad")
    delta_filter_alpha = LaunchConfiguration("delta_filter_alpha")
    max_tcp_delta_body = LaunchConfiguration("max_tcp_delta_body_m")
    max_tcp_delta_rotvec = LaunchConfiguration("max_tcp_delta_rotvec_rad")
    position_max_tcp_offset = LaunchConfiguration("position_max_tcp_offset_m")
    position_max_tcp_rotvec = LaunchConfiguration("position_max_tcp_rotvec_rad")
    position_reanchor_when_stationary = LaunchConfiguration("position_reanchor_when_stationary")
    position_stationary_translation = LaunchConfiguration("position_stationary_translation_m")
    position_stationary_rotation = LaunchConfiguration("position_stationary_rotation_rad")
    position_stationary_hold = LaunchConfiguration("position_stationary_hold_sec")
    position_resume_translation = LaunchConfiguration("position_resume_translation_m")
    position_resume_rotation = LaunchConfiguration("position_resume_rotation_rad")
    rotation_scale = LaunchConfiguration("rotation_scale")
    target_lead_time = LaunchConfiguration("target_lead_time_sec")
    max_controller_angle = LaunchConfiguration("max_controller_angle_rad")
    roll_sign = LaunchConfiguration("roll_sign")
    pitch_sign = LaunchConfiguration("pitch_sign")
    yaw_sign = LaunchConfiguration("yaw_sign")
    pose_log_enabled = LaunchConfiguration("pose_log_enabled")
    pose_log_rate_hz = LaunchConfiguration("pose_log_rate_hz")
    pose_log_dir = LaunchConfiguration("pose_log_dir")
    external_tcp_pose_timeout = LaunchConfiguration("external_tcp_pose_timeout_sec")
    gripper_buttons_enabled = LaunchConfiguration("gripper_buttons_enabled")
    gripper_command_topic = LaunchConfiguration("gripper_command_topic")
    gripper_open_button_name = LaunchConfiguration("gripper_open_button_name")
    gripper_close_button_name = LaunchConfiguration("gripper_close_button_name")

    current_pose_topic = LaunchConfiguration("current_pose_topic")
    target_pose_topic = LaunchConfiguration("target_pose_topic")
    enabled_topic = LaunchConfiguration("enabled_topic")
    enable_gripper = LaunchConfiguration("enable_gripper")
    gripper_close_width = LaunchConfiguration("gripper_close_width_m")
    gripper_speed = LaunchConfiguration("gripper_speed_mps")
    gripper_close_use_grasp = LaunchConfiguration("gripper_close_use_grasp")
    gripper_force = LaunchConfiguration("gripper_force_n")
    max_linear_velocity = LaunchConfiguration("max_linear_velocity_mps")
    max_angular_velocity = LaunchConfiguration("max_angular_velocity_radps")
    max_linear_acceleration = LaunchConfiguration("max_linear_acceleration_mps2")
    max_angular_acceleration = LaunchConfiguration("max_angular_acceleration_radps2")
    max_initial_target_distance = LaunchConfiguration("max_initial_target_distance_m")
    max_initial_target_angle = LaunchConfiguration("max_initial_target_angle_rad")
    automatic_error_recovery = LaunchConfiguration("automatic_error_recovery")
    error_recovery_cooldown = LaunchConfiguration("error_recovery_cooldown_sec")
    post_error_recovery_hold = LaunchConfiguration("post_error_recovery_hold_sec")
    relative_dynamics_factor = LaunchConfiguration("relative_dynamics_factor")
    stop_relative_dynamics_factor = LaunchConfiguration("stop_relative_dynamics_factor")
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    control_command_mode = LaunchConfiguration("control_command_mode")
    velocity_command_duration = LaunchConfiguration("velocity_command_duration_sec")
    command_target_lookahead = LaunchConfiguration("command_target_lookahead_sec")
    enabled_timeout = LaunchConfiguration("enabled_timeout_sec")
    stop_on_disable = LaunchConfiguration("stop_on_disable")
    ik_iterations_per_tick = LaunchConfiguration("ik_iterations_per_tick")
    ik_damping = LaunchConfiguration("ik_damping")
    ik_orientation_weight = LaunchConfiguration("ik_orientation_weight")
    ik_max_joint_step_rad = LaunchConfiguration("ik_max_joint_step_rad")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "rviz", "quest3_oculus.rviz"]
    )
    panda_urdf = (
        get_package_share_directory("moveit_resources_panda_description")
        + "/urdf/panda.urdf"
    )
    with open(panda_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="172.16.0.3"),
            DeclareLaunchArgument("start_franka", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("publish_world_to_base_tf", default_value="true"),
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("base_frame", default_value="panda_link0"),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/quest3_franka/robot_description",
            ),
            DeclareLaunchArgument("trigger_threshold", default_value="0.5"),
            DeclareLaunchArgument("align_controller_frame_to_tcp", default_value="false"),
            DeclareLaunchArgument("teleop_motion_mode", default_value="velocity"),
            DeclareLaunchArgument("translation_scale", default_value="1.0"),
            DeclareLaunchArgument("translation_x_sign", default_value="1.0"),
            DeclareLaunchArgument("translation_y_sign", default_value="1.0"),
            DeclareLaunchArgument("translation_z_sign", default_value="1.0"),
            DeclareLaunchArgument("translation_deadband_m", default_value="0.0015"),
            DeclareLaunchArgument("rotation_deadband_rad", default_value="0.004"),
            DeclareLaunchArgument("delta_filter_alpha", default_value="0.45"),
            DeclareLaunchArgument("max_tcp_delta_body_m", default_value="0.025"),
            DeclareLaunchArgument("max_tcp_delta_rotvec_rad", default_value="0.035"),
            DeclareLaunchArgument("position_max_tcp_offset_m", default_value="0.35"),
            DeclareLaunchArgument("position_max_tcp_rotvec_rad", default_value="0.50"),
            DeclareLaunchArgument("position_reanchor_when_stationary", default_value="false"),
            DeclareLaunchArgument("position_stationary_translation_m", default_value="0.0010"),
            DeclareLaunchArgument("position_stationary_rotation_rad", default_value="0.0030"),
            DeclareLaunchArgument("position_stationary_hold_sec", default_value="0.12"),
            DeclareLaunchArgument("position_resume_translation_m", default_value="0.0030"),
            DeclareLaunchArgument("position_resume_rotation_rad", default_value="0.0200"),
            DeclareLaunchArgument("rotation_scale", default_value="1.0"),
            DeclareLaunchArgument("target_lead_time_sec", default_value="0.25"),
            DeclareLaunchArgument("max_controller_angle_rad", default_value="0.9"),
            DeclareLaunchArgument("roll_sign", default_value="1.0"),
            DeclareLaunchArgument("pitch_sign", default_value="1.0"),
            DeclareLaunchArgument("yaw_sign", default_value="1.0"),
            DeclareLaunchArgument("pose_log_enabled", default_value="true"),
            DeclareLaunchArgument("pose_log_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("pose_log_dir", default_value=""),
            DeclareLaunchArgument("external_tcp_pose_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("gripper_buttons_enabled", default_value="true"),
            DeclareLaunchArgument(
                "gripper_command_topic",
                default_value="/quest3/right_teleop/gripper_command",
            ),
            DeclareLaunchArgument("gripper_open_button_name", default_value="A"),
            DeclareLaunchArgument("gripper_close_button_name", default_value="B"),
            DeclareLaunchArgument(
                "current_pose_topic",
                default_value="/franka_franky/current_pose",
            ),
            DeclareLaunchArgument("target_pose_topic", default_value="/franka_sim/tcp_target_pose"),
            DeclareLaunchArgument("enabled_topic", default_value="/quest3/right_teleop/enabled"),
            DeclareLaunchArgument("enable_gripper", default_value="true"),
            DeclareLaunchArgument("gripper_close_width_m", default_value="0.0"),
            DeclareLaunchArgument("gripper_speed_mps", default_value="0.04"),
            DeclareLaunchArgument("gripper_close_use_grasp", default_value="false"),
            DeclareLaunchArgument("gripper_force_n", default_value="30.0"),
            DeclareLaunchArgument("max_linear_velocity_mps", default_value="0.04"),
            DeclareLaunchArgument("max_angular_velocity_radps", default_value="0.25"),
            DeclareLaunchArgument("max_linear_acceleration_mps2", default_value="0.25"),
            DeclareLaunchArgument("max_angular_acceleration_radps2", default_value="1.0"),
            DeclareLaunchArgument("max_initial_target_distance_m", default_value="0.08"),
            DeclareLaunchArgument("max_initial_target_angle_rad", default_value="0.6"),
            DeclareLaunchArgument("automatic_error_recovery", default_value="true"),
            DeclareLaunchArgument("error_recovery_cooldown_sec", default_value="1.0"),
            DeclareLaunchArgument("post_error_recovery_hold_sec", default_value="0.60"),
            DeclareLaunchArgument("relative_dynamics_factor", default_value="0.05"),
            DeclareLaunchArgument("stop_relative_dynamics_factor", default_value="-1.0"),
            DeclareLaunchArgument("command_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("control_command_mode", default_value="pose"),
            DeclareLaunchArgument("velocity_command_duration_sec", default_value="0.15"),
            DeclareLaunchArgument("command_target_lookahead_sec", default_value="0.20"),
            DeclareLaunchArgument("enabled_timeout_sec", default_value="0.10"),
            DeclareLaunchArgument("stop_on_disable", default_value="true"),
            DeclareLaunchArgument("ik_iterations_per_tick", default_value="8"),
            DeclareLaunchArgument("ik_damping", default_value="0.045"),
            DeclareLaunchArgument("ik_orientation_weight", default_value="0.65"),
            DeclareLaunchArgument("ik_max_joint_step_rad", default_value="0.06"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="panda_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("robot_description", robot_description_topic)],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_franka_base",
                arguments=["0", "0", "0", "0", "0", "0", "world", base_frame],
                condition=IfCondition(publish_world_to_base_tf),
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
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "world_frame": base_frame,
                        "right_frame": "quest3_right_controller_raw",
                        "left_frame": "quest3_left_controller_raw",
                    }
                ],
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="right_hand_teleop_sim_node",
                name="right_hand_teleop_sim",
                output="screen",
                parameters=[
                    {
                        "world_frame": base_frame,
                        "trigger_threshold": ParameterValue(trigger_threshold, value_type=float),
                        "align_controller_frame_to_tcp": ParameterValue(
                            align_controller_frame_to_tcp,
                            value_type=bool,
                        ),
                        "teleop_motion_mode": teleop_motion_mode,
                        "translation_scale": ParameterValue(translation_scale, value_type=float),
                        "translation_x_sign": ParameterValue(translation_x_sign, value_type=float),
                        "translation_y_sign": ParameterValue(translation_y_sign, value_type=float),
                        "translation_z_sign": ParameterValue(translation_z_sign, value_type=float),
                        "translation_deadband_m": ParameterValue(translation_deadband, value_type=float),
                        "rotation_deadband_rad": ParameterValue(rotation_deadband, value_type=float),
                        "delta_filter_alpha": ParameterValue(delta_filter_alpha, value_type=float),
                        "max_tcp_delta_body_m": ParameterValue(max_tcp_delta_body, value_type=float),
                        "max_tcp_delta_rotvec_rad": ParameterValue(max_tcp_delta_rotvec, value_type=float),
                        "position_max_tcp_offset_m": ParameterValue(
                            position_max_tcp_offset,
                            value_type=float,
                        ),
                        "position_max_tcp_rotvec_rad": ParameterValue(
                            position_max_tcp_rotvec,
                            value_type=float,
                        ),
                        "position_reanchor_when_stationary": ParameterValue(
                            position_reanchor_when_stationary,
                            value_type=bool,
                        ),
                        "position_stationary_translation_m": ParameterValue(
                            position_stationary_translation,
                            value_type=float,
                        ),
                        "position_stationary_rotation_rad": ParameterValue(
                            position_stationary_rotation,
                            value_type=float,
                        ),
                        "position_stationary_hold_sec": ParameterValue(
                            position_stationary_hold,
                            value_type=float,
                        ),
                        "position_resume_translation_m": ParameterValue(
                            position_resume_translation,
                            value_type=float,
                        ),
                        "position_resume_rotation_rad": ParameterValue(
                            position_resume_rotation,
                            value_type=float,
                        ),
                        "rotation_scale": ParameterValue(rotation_scale, value_type=float),
                        "target_lead_time_sec": ParameterValue(target_lead_time, value_type=float),
                        "max_controller_angle_rad": ParameterValue(max_controller_angle, value_type=float),
                        "roll_sign": ParameterValue(roll_sign, value_type=float),
                        "pitch_sign": ParameterValue(pitch_sign, value_type=float),
                        "yaw_sign": ParameterValue(yaw_sign, value_type=float),
                        "pose_log_enabled": ParameterValue(pose_log_enabled, value_type=bool),
                        "pose_log_rate_hz": ParameterValue(pose_log_rate_hz, value_type=float),
                        "pose_log_dir": pose_log_dir,
                        "external_tcp_pose_topic": current_pose_topic,
                        "sync_external_tcp_when_idle": True,
                        "external_tcp_pose_timeout_sec": ParameterValue(
                            external_tcp_pose_timeout,
                            value_type=float,
                        ),
                        "gripper_buttons_enabled": ParameterValue(
                            gripper_buttons_enabled,
                            value_type=bool,
                        ),
                        "gripper_command_topic": gripper_command_topic,
                        "gripper_open_button_name": gripper_open_button_name,
                        "gripper_close_button_name": gripper_close_button_name,
                    }
                ],
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="franka_sim_ik_node",
                name="franka_sim_ik",
                output="screen",
                parameters=[
                    {
                        "target_pose_topic": target_pose_topic,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "iterations_per_tick": ParameterValue(ik_iterations_per_tick, value_type=int),
                        "damping": ParameterValue(ik_damping, value_type=float),
                        "orientation_weight": ParameterValue(ik_orientation_weight, value_type=float),
                        "max_joint_step_rad": ParameterValue(ik_max_joint_step_rad, value_type=float),
                    }
                ],
                condition=UnlessCondition(start_franka),
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="franky_cartesian_pose_node",
                name="franky_cartesian_pose",
                output="screen",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "target_pose_topic": target_pose_topic,
                        "enabled_topic": enabled_topic,
                        "current_pose_topic": current_pose_topic,
                        "base_frame": base_frame,
                        "enable_gripper": ParameterValue(enable_gripper, value_type=bool),
                        "gripper_command_topic": gripper_command_topic,
                        "gripper_close_width_m": ParameterValue(gripper_close_width, value_type=float),
                        "gripper_speed_mps": ParameterValue(gripper_speed, value_type=float),
                        "gripper_close_use_grasp": ParameterValue(
                            gripper_close_use_grasp,
                            value_type=bool,
                        ),
                        "gripper_force_n": ParameterValue(gripper_force, value_type=float),
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "command_rate_hz": ParameterValue(command_rate_hz, value_type=float),
                        "control_command_mode": control_command_mode,
                        "velocity_command_duration_sec": ParameterValue(
                            velocity_command_duration,
                            value_type=float,
                        ),
                        "command_target_lookahead_sec": ParameterValue(
                            command_target_lookahead,
                            value_type=float,
                        ),
                        "enabled_timeout_sec": ParameterValue(enabled_timeout, value_type=float),
                        "max_linear_velocity_mps": ParameterValue(max_linear_velocity, value_type=float),
                        "max_angular_velocity_radps": ParameterValue(max_angular_velocity, value_type=float),
                        "max_linear_acceleration_mps2": ParameterValue(
                            max_linear_acceleration,
                            value_type=float,
                        ),
                        "max_angular_acceleration_radps2": ParameterValue(
                            max_angular_acceleration,
                            value_type=float,
                        ),
                        "max_initial_target_distance_m": ParameterValue(
                            max_initial_target_distance,
                            value_type=float,
                        ),
                        "max_initial_target_angle_rad": ParameterValue(
                            max_initial_target_angle,
                            value_type=float,
                        ),
                        "automatic_error_recovery": ParameterValue(
                            automatic_error_recovery,
                            value_type=bool,
                        ),
                        "error_recovery_cooldown_sec": ParameterValue(
                            error_recovery_cooldown,
                            value_type=float,
                        ),
                        "post_error_recovery_hold_sec": ParameterValue(
                            post_error_recovery_hold,
                            value_type=float,
                        ),
                        "relative_dynamics_factor": ParameterValue(
                            relative_dynamics_factor,
                            value_type=float,
                        ),
                        "stop_relative_dynamics_factor": ParameterValue(
                            stop_relative_dynamics_factor,
                            value_type=float,
                        ),
                        "stop_on_disable": ParameterValue(stop_on_disable, value_type=bool),
                    }
                ],
                condition=IfCondition(start_franka),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                condition=IfCondition(start_rviz),
            ),
        ]
    )
