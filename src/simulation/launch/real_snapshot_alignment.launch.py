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
    shoulder_align_parameters = {
        "stop_distance_m": config["alignment_front_clearance_m"],
        "platform_length_m": pivot_to_front,
        "pos_tolerance_m": min(
            config["alignment_lateral_tolerance_m"],
            config["alignment_longitudinal_tolerance_m"],
        ),
        "angle_tolerance_rad": config["alignment_yaw_tolerance_rad"],
        "max_valid_t_m": config["shoulder_align_max_valid_t_m"],
        "parallel_epsilon": config["shoulder_align_parallel_epsilon"],
        "k_v": config["alignment_position_kp"],
        "k_w": config["alignment_yaw_kp"],
        "max_v_mps": config["alignment_max_speed_mps"],
        "max_w_rad_s": config["alignment_max_yaw_rate_rad_s"],
        "debounce_s": config["shoulder_align_debounce_s"],
        "pos_hysteresis_m": config["shoulder_align_pos_hysteresis_m"],
        "angle_hysteresis_rad": config["shoulder_align_angle_hysteresis_rad"],
    }
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
        Node(
            package="shoulder_align_controller",
            executable="shoulder_align_node",
            parameters=[shoulder_align_parameters],
            output="screen",
        ),
        # Only supplies Gazebo odom/TF; it must not overwrite the frozen real target.
        Node(package="alignment", executable="gazebo_ground_truth_publisher", parameters=[{"publish_shoulders": False}], output="screen"),
        Node(
            package="vision", executable="realsense_mediapipe_pose",
            parameters=[{"show_preview": True, "color_width": 1280, "color_height": 720, "fps": 30}],
            output="screen",
        ),
        Node(package="alignment", executable="camera_snapshot_to_gazebo", output="screen"),
    ])
