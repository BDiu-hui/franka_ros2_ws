import re

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def prefixed_panda_urdf(prefix: str) -> str:
    panda_urdf = (
        get_package_share_directory("moveit_resources_panda_description")
        + "/urdf/panda.urdf"
    )
    with open(panda_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    robot_description = re.sub(r'(<robot\s+name=")[^"]+(")', rf"\1{prefix}_panda\2", robot_description)
    robot_description = re.sub(r'(name|link|joint)="panda_', rf'\1="{prefix}_panda_', robot_description)
    return robot_description


def hand_teleop_node(
    *,
    hand: str,
    base_frame,
    controller_pose_topic: str,
    tcp_frame: str,
    controller_frame: str,
    attitude_frame: str,
    target_pose_topic: str,
    enabled_topic: str,
    debug_topic: str,
    twist_topic: str,
    marker_topic: str,
    current_pose_topic: str,
    trigger_button_name: str,
    trigger_value_name: str,
    trigger_threshold,
    align_controller_frame_to_tcp,
    teleop_motion_mode,
    translation_scale,
    translation_x_sign,
    translation_y_sign,
    translation_z_sign,
    translation_deadband,
    rotation_deadband,
    delta_filter_alpha,
    max_tcp_delta_body,
    max_tcp_delta_rotvec,
    position_max_tcp_offset,
    position_max_tcp_rotvec,
    rotation_scale,
    target_lead_time,
    max_controller_angle,
    roll_sign,
    pitch_sign,
    yaw_sign,
    pose_log_enabled,
    pose_log_rate_hz,
    pose_log_dir,
    external_tcp_pose_timeout,
    publish_rate_hz,
    gripper_buttons_enabled,
    gripper_command_topic: str,
    gripper_open_button_name,
    gripper_close_button_name,
) -> Node:
    return Node(
        package="quest3_oculus_rviz",
        executable="hand_teleop_sim_node",
        name=f"{hand}_hand_teleop",
        output="screen",
        parameters=[
            {
                "hand_label": hand,
                "world_frame": base_frame,
                "tcp_frame": tcp_frame,
                "controller_control_frame": controller_frame,
                "attitude_frame": attitude_frame,
                "controller_pose_topic": controller_pose_topic,
                "buttons_topic": "quest3/buttons",
                "twist_topic": twist_topic,
                "enabled_topic": enabled_topic,
                "debug_topic": debug_topic,
                "target_pose_topic": target_pose_topic,
                "marker_topic": marker_topic,
                "trigger_button_name": trigger_button_name,
                "trigger_value_name": trigger_value_name,
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
                "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                "gripper_buttons_enabled": ParameterValue(
                    gripper_buttons_enabled,
                    value_type=bool,
                ),
                "gripper_command_topic": gripper_command_topic,
                "gripper_open_button_name": gripper_open_button_name,
                "gripper_close_button_name": gripper_close_button_name,
            }
        ],
    )


def franky_node(
    *,
    hand: str,
    robot_ip,
    base_frame,
    target_pose_topic: str,
    enabled_topic: str,
    current_pose_topic: str,
    debug_topic: str,
    joint_state_topic: str,
    publish_rate_hz,
    command_rate_hz,
    control_command_mode,
    velocity_command_duration,
    command_target_lookahead,
    enabled_timeout,
    max_linear_velocity,
    max_angular_velocity,
    max_linear_acceleration,
    max_angular_acceleration,
    max_initial_target_distance,
    max_initial_target_angle,
    automatic_error_recovery,
    error_recovery_cooldown,
    post_error_recovery_hold,
    relative_dynamics_factor,
    stop_on_disable,
    enable_gripper,
    gripper_command_topic: str,
    gripper_close_width,
    gripper_speed,
    gripper_close_use_grasp,
    gripper_force,
    condition,
) -> Node:
    return Node(
        package="quest3_oculus_rviz",
        executable="franky_cartesian_pose_node",
        name=f"{hand}_franky_cartesian_pose",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "target_pose_topic": target_pose_topic,
                "enabled_topic": enabled_topic,
                "current_pose_topic": current_pose_topic,
                "debug_topic": debug_topic,
                "joint_state_topic": joint_state_topic,
                "joint_names": [
                    f"{hand}_panda_joint1",
                    f"{hand}_panda_joint2",
                    f"{hand}_panda_joint3",
                    f"{hand}_panda_joint4",
                    f"{hand}_panda_joint5",
                    f"{hand}_panda_joint6",
                    f"{hand}_panda_joint7",
                ],
                "finger_joint_names": [
                    f"{hand}_panda_finger_joint1",
                    f"{hand}_panda_finger_joint2",
                ],
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
                "stop_on_disable": ParameterValue(stop_on_disable, value_type=bool),
            }
        ],
        condition=IfCondition(condition),
    )


def generate_launch_description():
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    start_left_franka = LaunchConfiguration("start_left_franka")
    start_right_franka = LaunchConfiguration("start_right_franka")
    start_rviz = LaunchConfiguration("start_rviz")
    publish_world_to_base_tf = LaunchConfiguration("publish_world_to_base_tf")

    mock = LaunchConfiguration("mock")
    quest_ip_address = LaunchConfiguration("quest_ip_address")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    world_frame = LaunchConfiguration("world_frame")
    left_base_frame = LaunchConfiguration("left_base_frame")
    right_base_frame = LaunchConfiguration("right_base_frame")
    left_base_y = LaunchConfiguration("left_base_y")
    right_base_y = LaunchConfiguration("right_base_y")
    left_robot_description_topic = LaunchConfiguration("left_robot_description_topic")
    right_robot_description_topic = LaunchConfiguration("right_robot_description_topic")

    trigger_threshold = LaunchConfiguration("trigger_threshold")
    align_controller_frame_to_tcp = LaunchConfiguration("align_controller_frame_to_tcp")
    teleop_motion_mode = LaunchConfiguration("teleop_motion_mode")
    translation_scale = LaunchConfiguration("translation_scale")
    translation_x_sign = LaunchConfiguration("translation_x_sign")
    translation_y_sign = LaunchConfiguration("translation_y_sign")
    translation_z_sign = LaunchConfiguration("translation_z_sign")
    right_translation_x_sign = LaunchConfiguration("right_translation_x_sign")
    right_translation_y_sign = LaunchConfiguration("right_translation_y_sign")
    right_translation_z_sign = LaunchConfiguration("right_translation_z_sign")
    translation_deadband = LaunchConfiguration("translation_deadband_m")
    rotation_deadband = LaunchConfiguration("rotation_deadband_rad")
    delta_filter_alpha = LaunchConfiguration("delta_filter_alpha")
    max_tcp_delta_body = LaunchConfiguration("max_tcp_delta_body_m")
    max_tcp_delta_rotvec = LaunchConfiguration("max_tcp_delta_rotvec_rad")
    position_max_tcp_offset = LaunchConfiguration("position_max_tcp_offset_m")
    position_max_tcp_rotvec = LaunchConfiguration("position_max_tcp_rotvec_rad")
    rotation_scale = LaunchConfiguration("rotation_scale")
    target_lead_time = LaunchConfiguration("target_lead_time_sec")
    max_controller_angle = LaunchConfiguration("max_controller_angle_rad")
    roll_sign = LaunchConfiguration("roll_sign")
    pitch_sign = LaunchConfiguration("pitch_sign")
    yaw_sign = LaunchConfiguration("yaw_sign")
    right_roll_sign = LaunchConfiguration("right_roll_sign")
    right_pitch_sign = LaunchConfiguration("right_pitch_sign")
    right_yaw_sign = LaunchConfiguration("right_yaw_sign")
    pose_log_enabled = LaunchConfiguration("pose_log_enabled")
    pose_log_rate_hz = LaunchConfiguration("pose_log_rate_hz")
    pose_log_dir = LaunchConfiguration("pose_log_dir")
    external_tcp_pose_timeout = LaunchConfiguration("external_tcp_pose_timeout_sec")
    left_gripper_buttons_enabled = LaunchConfiguration("left_gripper_buttons_enabled")
    right_gripper_buttons_enabled = LaunchConfiguration("right_gripper_buttons_enabled")
    gripper_open_button_name = LaunchConfiguration("gripper_open_button_name")
    gripper_close_button_name = LaunchConfiguration("gripper_close_button_name")

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
    command_rate_hz = LaunchConfiguration("command_rate_hz")
    control_command_mode = LaunchConfiguration("control_command_mode")
    velocity_command_duration = LaunchConfiguration("velocity_command_duration_sec")
    command_target_lookahead = LaunchConfiguration("command_target_lookahead_sec")
    enabled_timeout = LaunchConfiguration("enabled_timeout_sec")
    stop_on_disable = LaunchConfiguration("stop_on_disable")
    enable_gripper = LaunchConfiguration("enable_gripper")
    gripper_close_width = LaunchConfiguration("gripper_close_width_m")
    gripper_speed = LaunchConfiguration("gripper_speed_mps")
    gripper_close_use_grasp = LaunchConfiguration("gripper_close_use_grasp")
    gripper_force = LaunchConfiguration("gripper_force_n")

    left_target_pose_topic = "/left_franka/tcp_target_pose"
    right_target_pose_topic = "/right_franka/tcp_target_pose"
    left_enabled_topic = "/quest3/left_teleop/enabled"
    right_enabled_topic = "/quest3/right_teleop/enabled"
    left_current_pose_topic = "/left_franka_franky/current_pose"
    right_current_pose_topic = "/right_franka_franky/current_pose"
    left_debug_topic = "/left_franka_franky/debug"
    right_debug_topic = "/right_franka_franky/debug"
    left_joint_state_topic = "/left_franka/joint_states"
    right_joint_state_topic = "/right_franka/joint_states"
    left_gripper_command_topic = "/quest3/left_teleop/gripper_command"
    right_gripper_command_topic = "/quest3/right_teleop/gripper_command"

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "rviz", "quest3_oculus_dual.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("left_robot_ip", default_value="172.16.0.2"),
            DeclareLaunchArgument("right_robot_ip", default_value="172.16.0.3"),
            DeclareLaunchArgument("start_left_franka", default_value="true"),
            DeclareLaunchArgument("start_right_franka", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("publish_world_to_base_tf", default_value="true"),
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("quest_ip_address", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("left_base_frame", default_value="left_panda_link0"),
            DeclareLaunchArgument("right_base_frame", default_value="right_panda_link0"),
            DeclareLaunchArgument("left_base_y", default_value="0.45"),
            DeclareLaunchArgument("right_base_y", default_value="-0.45"),
            DeclareLaunchArgument(
                "left_robot_description_topic",
                default_value="/left_franka/robot_description",
            ),
            DeclareLaunchArgument(
                "right_robot_description_topic",
                default_value="/right_franka/robot_description",
            ),
            DeclareLaunchArgument("trigger_threshold", default_value="0.5"),
            DeclareLaunchArgument("align_controller_frame_to_tcp", default_value="false"),
            DeclareLaunchArgument("teleop_motion_mode", default_value="velocity"),
            DeclareLaunchArgument("translation_scale", default_value="2.0"),
            DeclareLaunchArgument("translation_x_sign", default_value="1.0"),
            DeclareLaunchArgument("translation_y_sign", default_value="1.0"),
            DeclareLaunchArgument("translation_z_sign", default_value="1.0"),
            DeclareLaunchArgument("right_translation_x_sign", default_value="1.0"),
            DeclareLaunchArgument("right_translation_y_sign", default_value="1.0"),
            DeclareLaunchArgument("right_translation_z_sign", default_value="1.0"),
            DeclareLaunchArgument("translation_deadband_m", default_value="0.0015"),
            DeclareLaunchArgument("rotation_deadband_rad", default_value="0.004"),
            DeclareLaunchArgument("delta_filter_alpha", default_value="0.45"),
            DeclareLaunchArgument("max_tcp_delta_body_m", default_value="0.025"),
            DeclareLaunchArgument("max_tcp_delta_rotvec_rad", default_value="0.035"),
            DeclareLaunchArgument("position_max_tcp_offset_m", default_value="0.35"),
            DeclareLaunchArgument("position_max_tcp_rotvec_rad", default_value="0.50"),
            DeclareLaunchArgument("rotation_scale", default_value="1.0"),
            DeclareLaunchArgument("target_lead_time_sec", default_value="0.30"),
            DeclareLaunchArgument("max_controller_angle_rad", default_value="0.9"),
            DeclareLaunchArgument("roll_sign", default_value="1.0"),
            DeclareLaunchArgument("pitch_sign", default_value="1.0"),
            DeclareLaunchArgument("yaw_sign", default_value="1.0"),
            DeclareLaunchArgument("right_roll_sign", default_value="1.0"),
            DeclareLaunchArgument("right_pitch_sign", default_value="1.0"),
            DeclareLaunchArgument("right_yaw_sign", default_value="1.0"),
            DeclareLaunchArgument("pose_log_enabled", default_value="true"),
            DeclareLaunchArgument("pose_log_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("pose_log_dir", default_value=""),
            DeclareLaunchArgument("external_tcp_pose_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("left_gripper_buttons_enabled", default_value="false"),
            DeclareLaunchArgument("right_gripper_buttons_enabled", default_value="true"),
            DeclareLaunchArgument("gripper_open_button_name", default_value="A"),
            DeclareLaunchArgument("gripper_close_button_name", default_value="B"),
            DeclareLaunchArgument("max_linear_velocity_mps", default_value="0.12"),
            DeclareLaunchArgument("max_angular_velocity_radps", default_value="0.50"),
            DeclareLaunchArgument("max_linear_acceleration_mps2", default_value="0.25"),
            DeclareLaunchArgument("max_angular_acceleration_radps2", default_value="1.0"),
            DeclareLaunchArgument("max_initial_target_distance_m", default_value="0.08"),
            DeclareLaunchArgument("max_initial_target_angle_rad", default_value="0.6"),
            DeclareLaunchArgument("automatic_error_recovery", default_value="true"),
            DeclareLaunchArgument("error_recovery_cooldown_sec", default_value="1.0"),
            DeclareLaunchArgument("post_error_recovery_hold_sec", default_value="0.60"),
            DeclareLaunchArgument("relative_dynamics_factor", default_value="0.08"),
            DeclareLaunchArgument("command_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("control_command_mode", default_value="pose"),
            DeclareLaunchArgument("velocity_command_duration_sec", default_value="0.15"),
            DeclareLaunchArgument("command_target_lookahead_sec", default_value="0.20"),
            DeclareLaunchArgument("enabled_timeout_sec", default_value="0.10"),
            DeclareLaunchArgument("stop_on_disable", default_value="true"),
            DeclareLaunchArgument("enable_gripper", default_value="true"),
            DeclareLaunchArgument("gripper_close_width_m", default_value="0.0"),
            DeclareLaunchArgument("gripper_speed_mps", default_value="0.04"),
            DeclareLaunchArgument("gripper_close_use_grasp", default_value="false"),
            DeclareLaunchArgument("gripper_force_n", default_value="30.0"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="left_panda_robot_state_publisher",
                parameters=[{"robot_description": prefixed_panda_urdf("left")}],
                remappings=[
                    ("joint_states", left_joint_state_topic),
                    ("robot_description", left_robot_description_topic),
                ],
                condition=IfCondition(start_rviz),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="right_panda_robot_state_publisher",
                parameters=[{"robot_description": prefixed_panda_urdf("right")}],
                remappings=[
                    ("joint_states", right_joint_state_topic),
                    ("robot_description", right_robot_description_topic),
                ],
                condition=IfCondition(start_rviz),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_left_franka_base",
                arguments=["0", left_base_y, "0", "0", "0", "0", world_frame, left_base_frame],
                condition=IfCondition(publish_world_to_base_tf),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_right_franka_base",
                arguments=["0", right_base_y, "0", "0", "0", "0", world_frame, right_base_frame],
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
                        "world_frame": world_frame,
                        "right_frame": "quest3_right_controller_raw",
                        "left_frame": "quest3_left_controller_raw",
                    }
                ],
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="franka_sim_ik_node",
                name="left_franka_sim_ik",
                output="screen",
                parameters=[
                    {
                        "target_pose_topic": left_target_pose_topic,
                        "joint_state_topic": left_joint_state_topic,
                        "joint_name_prefix": "left_",
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                    }
                ],
                condition=UnlessCondition(start_left_franka),
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="franka_sim_ik_node",
                name="right_franka_sim_ik",
                output="screen",
                parameters=[
                    {
                        "target_pose_topic": right_target_pose_topic,
                        "joint_state_topic": right_joint_state_topic,
                        "joint_name_prefix": "right_",
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                    }
                ],
                condition=UnlessCondition(start_right_franka),
            ),
            hand_teleop_node(
                hand="left",
                base_frame=left_base_frame,
                controller_pose_topic="quest3/left_controller/pose",
                tcp_frame="left_franka_sim_tcp",
                controller_frame="quest3_left_controller",
                attitude_frame="quest3_left_attitude",
                target_pose_topic=left_target_pose_topic,
                enabled_topic=left_enabled_topic,
                debug_topic="/quest3/left_teleop/debug",
                twist_topic="/quest3/left_teleop/twist",
                marker_topic="/left_franka/tcp_markers",
                current_pose_topic=left_current_pose_topic,
                trigger_button_name="LTr",
                trigger_value_name="leftTrig",
                trigger_threshold=trigger_threshold,
                align_controller_frame_to_tcp=align_controller_frame_to_tcp,
                teleop_motion_mode=teleop_motion_mode,
                translation_scale=translation_scale,
                translation_x_sign=translation_x_sign,
                translation_y_sign=translation_y_sign,
                translation_z_sign=translation_z_sign,
                translation_deadband=translation_deadband,
                rotation_deadband=rotation_deadband,
                delta_filter_alpha=delta_filter_alpha,
                max_tcp_delta_body=max_tcp_delta_body,
                max_tcp_delta_rotvec=max_tcp_delta_rotvec,
                position_max_tcp_offset=position_max_tcp_offset,
                position_max_tcp_rotvec=position_max_tcp_rotvec,
                rotation_scale=rotation_scale,
                target_lead_time=target_lead_time,
                max_controller_angle=max_controller_angle,
                roll_sign=roll_sign,
                pitch_sign=pitch_sign,
                yaw_sign=yaw_sign,
                pose_log_enabled=pose_log_enabled,
                pose_log_rate_hz=pose_log_rate_hz,
                pose_log_dir=pose_log_dir,
                external_tcp_pose_timeout=external_tcp_pose_timeout,
                publish_rate_hz=publish_rate_hz,
                gripper_buttons_enabled=left_gripper_buttons_enabled,
                gripper_command_topic=left_gripper_command_topic,
                gripper_open_button_name=gripper_open_button_name,
                gripper_close_button_name=gripper_close_button_name,
            ),
            hand_teleop_node(
                hand="right",
                base_frame=right_base_frame,
                controller_pose_topic="quest3/right_controller/pose",
                tcp_frame="right_franka_sim_tcp",
                controller_frame="quest3_right_controller",
                attitude_frame="quest3_right_attitude",
                target_pose_topic=right_target_pose_topic,
                enabled_topic=right_enabled_topic,
                debug_topic="/quest3/right_teleop/debug",
                twist_topic="/quest3/right_teleop/twist",
                marker_topic="/right_franka/tcp_markers",
                current_pose_topic=right_current_pose_topic,
                trigger_button_name="RTr",
                trigger_value_name="rightTrig",
                trigger_threshold=trigger_threshold,
                align_controller_frame_to_tcp=align_controller_frame_to_tcp,
                teleop_motion_mode=teleop_motion_mode,
                translation_scale=translation_scale,
                translation_x_sign=right_translation_x_sign,
                translation_y_sign=right_translation_y_sign,
                translation_z_sign=right_translation_z_sign,
                translation_deadband=translation_deadband,
                rotation_deadband=rotation_deadband,
                delta_filter_alpha=delta_filter_alpha,
                max_tcp_delta_body=max_tcp_delta_body,
                max_tcp_delta_rotvec=max_tcp_delta_rotvec,
                position_max_tcp_offset=position_max_tcp_offset,
                position_max_tcp_rotvec=position_max_tcp_rotvec,
                rotation_scale=rotation_scale,
                target_lead_time=target_lead_time,
                max_controller_angle=max_controller_angle,
                roll_sign=right_roll_sign,
                pitch_sign=right_pitch_sign,
                yaw_sign=right_yaw_sign,
                pose_log_enabled=pose_log_enabled,
                pose_log_rate_hz=pose_log_rate_hz,
                pose_log_dir=pose_log_dir,
                external_tcp_pose_timeout=external_tcp_pose_timeout,
                publish_rate_hz=publish_rate_hz,
                gripper_buttons_enabled=right_gripper_buttons_enabled,
                gripper_command_topic=right_gripper_command_topic,
                gripper_open_button_name=gripper_open_button_name,
                gripper_close_button_name=gripper_close_button_name,
            ),
            franky_node(
                hand="left",
                robot_ip=left_robot_ip,
                base_frame=left_base_frame,
                target_pose_topic=left_target_pose_topic,
                enabled_topic=left_enabled_topic,
                current_pose_topic=left_current_pose_topic,
                debug_topic=left_debug_topic,
                joint_state_topic=left_joint_state_topic,
                publish_rate_hz=publish_rate_hz,
                command_rate_hz=command_rate_hz,
                control_command_mode=control_command_mode,
                velocity_command_duration=velocity_command_duration,
                command_target_lookahead=command_target_lookahead,
                enabled_timeout=enabled_timeout,
                max_linear_velocity=max_linear_velocity,
                max_angular_velocity=max_angular_velocity,
                max_linear_acceleration=max_linear_acceleration,
                max_angular_acceleration=max_angular_acceleration,
                max_initial_target_distance=max_initial_target_distance,
                max_initial_target_angle=max_initial_target_angle,
                automatic_error_recovery=automatic_error_recovery,
                error_recovery_cooldown=error_recovery_cooldown,
                post_error_recovery_hold=post_error_recovery_hold,
                relative_dynamics_factor=relative_dynamics_factor,
                stop_on_disable=stop_on_disable,
                enable_gripper=enable_gripper,
                gripper_command_topic=left_gripper_command_topic,
                gripper_close_width=gripper_close_width,
                gripper_speed=gripper_speed,
                gripper_close_use_grasp=gripper_close_use_grasp,
                gripper_force=gripper_force,
                condition=start_left_franka,
            ),
            franky_node(
                hand="right",
                robot_ip=right_robot_ip,
                base_frame=right_base_frame,
                target_pose_topic=right_target_pose_topic,
                enabled_topic=right_enabled_topic,
                current_pose_topic=right_current_pose_topic,
                debug_topic=right_debug_topic,
                joint_state_topic=right_joint_state_topic,
                publish_rate_hz=publish_rate_hz,
                command_rate_hz=command_rate_hz,
                control_command_mode=control_command_mode,
                velocity_command_duration=velocity_command_duration,
                command_target_lookahead=command_target_lookahead,
                enabled_timeout=enabled_timeout,
                max_linear_velocity=max_linear_velocity,
                max_angular_velocity=max_angular_velocity,
                max_linear_acceleration=max_linear_acceleration,
                max_angular_acceleration=max_angular_acceleration,
                max_initial_target_distance=max_initial_target_distance,
                max_initial_target_angle=max_initial_target_angle,
                automatic_error_recovery=automatic_error_recovery,
                error_recovery_cooldown=error_recovery_cooldown,
                post_error_recovery_hold=post_error_recovery_hold,
                relative_dynamics_factor=relative_dynamics_factor,
                stop_on_disable=stop_on_disable,
                enable_gripper=enable_gripper,
                gripper_command_topic=right_gripper_command_topic,
                gripper_close_width=gripper_close_width,
                gripper_speed=gripper_speed,
                gripper_close_use_grasp=gripper_close_use_grasp,
                gripper_force=gripper_force,
                condition=start_right_franka,
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
