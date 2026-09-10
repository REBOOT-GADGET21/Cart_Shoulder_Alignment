"""Run Gazebo cart control from live D435 custom-pose measurements."""

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
        "hybrid_alpha_gain": config["hybrid_alpha_gain"],
        "hybrid_beta_gain": config["hybrid_beta_gain"],
        "hybrid_min_approach_speed_mps": config["hybrid_min_approach_speed_mps"],
        "hybrid_alpha_full_speed_rad": config["hybrid_alpha_full_speed_rad"],
        "hybrid_alpha_stop_rad": config["hybrid_alpha_stop_rad"],
        "hybrid_final_heading_start_m": config["hybrid_final_heading_start_m"],
        "hybrid_final_heading_full_m": config["hybrid_final_heading_full_m"],
        "hybrid_target_position_tau_s": config["hybrid_target_position_tau_s"],
        "hybrid_target_heading_tau_s": config["hybrid_target_heading_tau_s"],
        "measurement_timeout_s": config["shoulder_align_measurement_timeout_s"],
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
        # Gazebo contributes only cart odometry.  It never publishes synthetic
        # shoulders, so /shoulder_line comes exclusively from the D435 node.
        Node(package="alignment", executable="gazebo_cart_odometry", output="screen"),
        Node(
            package="vision", executable="realsense_pose",
            parameters=[{"show_preview": True, "color_width": 1280, "color_height": 720, "fps": 30}],
            output="screen",
        ),
    ])
