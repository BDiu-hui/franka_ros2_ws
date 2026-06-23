from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type = LaunchConfiguration("robot_type")
    arm_prefix = LaunchConfiguration("arm_prefix")
    namespace = LaunchConfiguration("namespace")
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    start_franka_robot_state_broadcaster = LaunchConfiguration("start_franka_robot_state_broadcaster")
    ros2_control_cpu = LaunchConfiguration("ros2_control_cpu")
    franka_aux_cpu = LaunchConfiguration("franka_aux_cpu")
    start_rviz = LaunchConfiguration("start_rviz")
    start_impedance_controller = LaunchConfiguration("start_impedance_controller")
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("serl_franka_controllers_ros2"), "config", "serl_franka_controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("serl_franka_controllers_ros2"), "rviz", "impedance_tuning.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type", default_value="panda", description="Franka robot type, e.g. panda or fr3"
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
                "start_franka_robot_state_broadcaster",
                default_value="true",
                description="Whether to spawn franka_robot_state_broadcaster",
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
                "start_rviz",
                default_value="true",
                description="Whether to launch RViz with the impedance tuning panel",
            ),
            DeclareLaunchArgument(
                "start_impedance_controller",
                default_value="true",
                description="Whether to activate the Cartesian impedance controller immediately",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"]
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
                    "start_franka_robot_state_broadcaster": start_franka_robot_state_broadcaster,
                    "ros2_control_cpu": ros2_control_cpu,
                    "franka_aux_cpu": franka_aux_cpu,
                    "controllers_yaml": controllers_yaml,
                }.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "cartesian_impedance_controller",
                    "--param-file",
                    controllers_yaml,
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
                condition=IfCondition(start_impedance_controller),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "cartesian_impedance_controller",
                    "--inactive",
                    "--param-file",
                    controllers_yaml,
                    "--controller-manager-timeout",
                    "30",
                ],
                output="screen",
                condition=UnlessCondition(start_impedance_controller),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_config],
                output="screen",
                condition=IfCondition(start_rviz),
            ),
        ]
    )
