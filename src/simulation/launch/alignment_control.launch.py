"""
Run Gazebo cart, fake optical alignment, rear kinematics, and ROS-Gazebo bridges.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Connect all Phase 7 test components; fake landmarks start separately."""

    package_dir = Path(get_package_share_directory("simulation"))
    bridge_arguments = [
        "/rear_left_steering_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        "/rear_right_steering_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        "/rear_left_wheel_speed_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        "/rear_right_wheel_speed_cmd@std_msgs/msg/Float64@gz.msgs.Double",
    ]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_dir / "launch" / "cart_only.launch.py"))),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=bridge_arguments, output="screen"),
        Node(package="rear_ackermann_controller", executable="rear_ackermann_node", output="screen"),
        Node(package="alignment", executable="alignment_node", output="screen"),
    ])
