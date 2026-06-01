from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    out_data_dir = LaunchConfiguration("out_data_dir")
    require_cameras = LaunchConfiguration("require_cameras")

    default_config = PathJoinSubstitution(
        [FindPackageShare("quest3_oculus_rviz"), "config", "data_recorder.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("out_data_dir", default_value="/tmp/quest3_recordings"),
            DeclareLaunchArgument("require_cameras", default_value="true"),
            Node(
                package="quest3_oculus_rviz",
                executable="data_recorder_node",
                name="quest3_data_recorder",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "out_data_dir": out_data_dir,
                        "require_cameras": ParameterValue(require_cameras, value_type=bool),
                    },
                ],
            ),
        ]
    )
