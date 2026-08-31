"""D435 pose measurement; backend is selected in params_setting.json."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="vision", executable="realsense_pose", output="screen"),
    ])
