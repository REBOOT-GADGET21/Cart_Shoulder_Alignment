"""
Run Gazebo cart, fake shoulder-line/odom inputs, C++ TRT alignment, and drive.
"""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Connect the C++ ShoulderAlignNode to Gazebo ground-truth test inputs."""

    package_dir = Path(get_package_share_directory("simulation"))
    config_file = Path.cwd() / "src" / "params_setting.json"
    if not config_file.exists():
        config_file = Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"
    config = json.loads(config_file.read_text(encoding="utf-8"))
    # The alignment node controls the rear-axle pivot.  Convert the actual SDF
    # geometry (base centre -> front edge) into pivot -> front edge distance.
    pivot_to_front_m = config["body_length_m"] / 2.0 - config["rear_axle_x_m"]
    # Keep controller inputs in the single JSON source.  Geometry-derived
    # platform_length_m intentionally remains derived here, so it cannot drift
    # from the cart model when body geometry changes.
    shoulder_align_parameters = {
        "stop_distance_m": config["alignment_front_clearance_m"],
        "platform_length_m": pivot_to_front_m,
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
    bridge_arguments = [
        "/rear_left_wheel_speed_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        "/rear_right_wheel_speed_cmd@std_msgs/msg/Float64@gz.msgs.Double",
        # PosePublisher is configured with use_pose_vector_msg=true, so this is
        # a gz.msgs.Pose_V, not a single gz.msgs.Pose.  Bridging it as Pose was
        # leaving fake perception at the spawn pose while the cart drove away.
        "/model/rear_steer_cart/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V",
    ]
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_dir / "launch" / "cart_only.launch.py"))),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=bridge_arguments, output="screen"),
        Node(package="rear_ackermann_controller", executable="rear_ackermann_node", output="screen"),
        Node(
            package="shoulder_align_controller",
            executable="shoulder_align_node",
            parameters=[shoulder_align_parameters],
            output="screen",
        ),
        Node(package="alignment", executable="gazebo_ground_truth_publisher", output="screen"),
        Node(package="vision", executable="shoulder_line_markers", condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", str(Path(get_package_share_directory("vision")) / "rviz" / "shoulder_alignment.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        ),
    ])
