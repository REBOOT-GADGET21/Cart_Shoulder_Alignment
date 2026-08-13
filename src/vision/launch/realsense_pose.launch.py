"""D435/MediaPipe shoulder measurement; deliberately independent of RViz/robot control."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="vision", executable="realsense_mediapipe_pose", output="screen"),
    ])
