from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    """Run only the Phase 7 empty Gazebo world."""

    world = Path(get_package_share_directory("simulation")) / "worlds" / "empty.sdf"
    return LaunchDescription([ExecuteProcess(cmd=["gz", "sim", "-r", str(world)], output="screen")])
