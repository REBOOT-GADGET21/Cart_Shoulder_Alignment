"""Run TRT in Gazebo against exactly one real D435/MediaPipe shoulder sample."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_dir = Path(get_package_share_directory("simulation"))
    config_path = Path.cwd() / "src" / "params_setting.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    pivot_to_front = config["body_length_m"] / 2.0 - config["rear_axle_x_m"]
    bridge = [
        "/rear_left_wheel_speed_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        "/rear_right_wheel_speed_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        "/model/rear_steer_cart/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
        "/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose",
    ]
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim_dir / "launch" / "cart_only.launch.py"))),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=bridge, output="screen"),
        Node(package="rear_ackermann_controller", executable="rear_ackermann_node", output="screen"),
        Node(package="shoulder_align_controller", executable="shoulder_align_node", parameters=[{
            "stop_distance_m": config["alignment_front_clearance_m"], "platform_length_m": pivot_to_front}], output="screen"),
        # Only supplies Gazebo odom/TF; it must not overwrite the frozen real target.
        Node(package="alignment", executable="gazebo_ground_truth_publisher", parameters=[{"publish_shoulders": False}], output="screen"),
        Node(
            package="vision", executable="realsense_mediapipe_pose",
            parameters=[{"show_preview": True, "color_width": 1280, "color_height": 720, "fps": 30}],
            output="screen",
        ),
        Node(package="alignment", executable="camera_snapshot_to_gazebo", output="screen"),
    ])
