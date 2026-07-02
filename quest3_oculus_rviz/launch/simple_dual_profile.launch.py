from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _normalize_launch_value(value: str) -> str:
    value = value.strip()
    if (
        (value.startswith('"') and value.endswith('"'))
        or (value.startswith("'") and value.endswith("'"))
    ):
        value = value[1:-1]
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered
    return value


def _read_profile_file(path: str) -> dict[str, dict[str, str]]:
    profiles: dict[str, dict[str, str]] = {}
    current_profile: str | None = None

    for line_number, raw_line in enumerate(Path(path).read_text().splitlines(), start=1):
        line_without_comment = raw_line.split("#", 1)[0].rstrip()
        if not line_without_comment.strip():
            continue

        indent = len(line_without_comment) - len(line_without_comment.lstrip())
        stripped = line_without_comment.strip()

        if indent == 0 and stripped.endswith(":"):
            current_profile = stripped[:-1].strip()
            if not current_profile:
                raise ValueError(f"Empty profile name in {path}:{line_number}")
            profiles[current_profile] = {}
            continue

        if current_profile is None or ":" not in stripped:
            raise ValueError(f"Expected 'key: value' under a profile in {path}:{line_number}")

        key, value = stripped.split(":", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"Empty launch argument name in {path}:{line_number}")
        profiles[current_profile][key] = _normalize_launch_value(value)

    return profiles


def _include_profile(context, *args, **kwargs):
    del args, kwargs
    package_share = Path(get_package_share_directory("quest3_oculus_rviz"))
    profile_file = LaunchConfiguration("profile_file").perform(context)
    profile = LaunchConfiguration("profile").perform(context)
    profiles = _read_profile_file(profile_file)
    if profile not in profiles:
        available = ", ".join(sorted(profiles)) or "(none)"
        raise ValueError(
            f"Profile '{profile}' was not found in {profile_file}. "
            f"Available profiles: {available}"
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_share / "launch" / "simple_dual_impedance_teleop.launch.py")
            ),
            launch_arguments=profiles[profile].items(),
        )
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("quest3_oculus_rviz"))
    default_profile_file = str(package_share / "config" / "simple_dual_split_launch.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile_file", default_value=default_profile_file),
            DeclareLaunchArgument("profile", default_value="franka_stack"),
            OpaqueFunction(function=_include_profile),
        ]
    )
