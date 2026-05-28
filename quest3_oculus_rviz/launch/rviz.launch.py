from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock = LaunchConfiguration("mock")
    ip_address = LaunchConfiguration("ip_address")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    show_robot = LaunchConfiguration("show_robot")
    start_rviz = LaunchConfiguration("start_rviz")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    trigger_threshold = LaunchConfiguration("trigger_threshold")
    align_controller_frame_to_tcp = LaunchConfiguration("align_controller_frame_to_tcp")
    teleop_motion_mode = LaunchConfiguration("teleop_motion_mode")
    translation_scale = LaunchConfiguration("translation_scale")
    translation_x_sign = LaunchConfiguration("translation_x_sign")
    translation_y_sign = LaunchConfiguration("translation_y_sign")
    translation_z_sign = LaunchConfiguration("translation_z_sign")
    position_max_tcp_offset = LaunchConfiguration("position_max_tcp_offset_m")
    position_max_tcp_rotvec = LaunchConfiguration("position_max_tcp_rotvec_rad")
    rotation_scale = LaunchConfiguration("rotation_scale")
    max_controller_angle = LaunchConfiguration("max_controller_angle_rad")
    roll_sign = LaunchConfiguration("roll_sign")
    pitch_sign = LaunchConfiguration("pitch_sign")
    yaw_sign = LaunchConfiguration("yaw_sign")
    pose_log_enabled = LaunchConfiguration("pose_log_enabled")
    pose_log_rate_hz = LaunchConfiguration("pose_log_rate_hz")
    pose_log_dir = LaunchConfiguration("pose_log_dir")
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
            DeclareLaunchArgument("mock", default_value="false"),
            DeclareLaunchArgument("ip_address", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("show_robot", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/quest3_franka/robot_description",
            ),
            DeclareLaunchArgument("trigger_threshold", default_value="0.5"),
            DeclareLaunchArgument("align_controller_frame_to_tcp", default_value="false"),
            DeclareLaunchArgument("teleop_motion_mode", default_value="velocity"),
            DeclareLaunchArgument("translation_scale", default_value="1.0"),
            DeclareLaunchArgument("translation_x_sign", default_value="-1.0"),
            DeclareLaunchArgument("translation_y_sign", default_value="-1.0"),
            DeclareLaunchArgument("translation_z_sign", default_value="1.0"),
            DeclareLaunchArgument("position_max_tcp_offset_m", default_value="0.35"),
            DeclareLaunchArgument("position_max_tcp_rotvec_rad", default_value="0.50"),
            DeclareLaunchArgument("rotation_scale", default_value="1.0"),
            DeclareLaunchArgument("max_controller_angle_rad", default_value="0.9"),
            DeclareLaunchArgument("roll_sign", default_value="-1.0"),
            DeclareLaunchArgument("pitch_sign", default_value="-1.0"),
            DeclareLaunchArgument("yaw_sign", default_value="1.0"),
            DeclareLaunchArgument("pose_log_enabled", default_value="true"),
            DeclareLaunchArgument("pose_log_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("pose_log_dir", default_value=""),
            DeclareLaunchArgument("ik_iterations_per_tick", default_value="8"),
            DeclareLaunchArgument("ik_damping", default_value="0.045"),
            DeclareLaunchArgument("ik_orientation_weight", default_value="0.65"),
            DeclareLaunchArgument("ik_max_joint_step_rad", default_value="0.06"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_panda_link0",
                arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
                condition=IfCondition(show_robot),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="panda_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[("robot_description", robot_description_topic)],
                condition=IfCondition(show_robot),
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="franka_sim_ik_node",
                name="franka_sim_ik",
                output="screen",
                parameters=[
                    {
                        "publish_rate_hz": 50.0,
                        "iterations_per_tick": ParameterValue(ik_iterations_per_tick, value_type=int),
                        "damping": ParameterValue(ik_damping, value_type=float),
                        "orientation_weight": ParameterValue(ik_orientation_weight, value_type=float),
                        "max_joint_step_rad": ParameterValue(ik_max_joint_step_rad, value_type=float),
                    }
                ],
                condition=IfCondition(show_robot),
            ),
            Node(
                package="quest3_oculus_rviz",
                executable="oculus_tf_node",
                name="quest3_oculus_tf",
                output="screen",
                parameters=[
                    {
                        "mock": ParameterValue(mock, value_type=bool),
                        "ip_address": ip_address,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
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
                        "position_max_tcp_offset_m": ParameterValue(
                            position_max_tcp_offset,
                            value_type=float,
                        ),
                        "position_max_tcp_rotvec_rad": ParameterValue(
                            position_max_tcp_rotvec,
                            value_type=float,
                        ),
                        "rotation_scale": ParameterValue(rotation_scale, value_type=float),
                        "max_controller_angle_rad": ParameterValue(max_controller_angle, value_type=float),
                        "roll_sign": ParameterValue(roll_sign, value_type=float),
                        "pitch_sign": ParameterValue(pitch_sign, value_type=float),
                        "yaw_sign": ParameterValue(yaw_sign, value_type=float),
                        "pose_log_enabled": ParameterValue(pose_log_enabled, value_type=bool),
                        "pose_log_rate_hz": ParameterValue(pose_log_rate_hz, value_type=float),
                        "pose_log_dir": pose_log_dir,
                    }
                ],
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
