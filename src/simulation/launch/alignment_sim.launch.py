from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    """Launch the Phase 7 cart; alignment nodes stay separate until tested."""

    package_dir = Path(get_package_share_directory("simulation"))
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(package_dir / "launch" / "cart_only.launch.py")
        ))
    ])
