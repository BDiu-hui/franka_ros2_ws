from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = Path(get_package_share_directory("quest3_oculus_rviz"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(package_share / "launch" / "simple_dual_profile.launch.py")
                ),
                launch_arguments={"profile": "all_in_one"}.items(),
            )
        ]
    )
