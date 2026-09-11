"""실기 전용: IMU, 모터, D435/YOLO, 기존 정렬 제어기를 한 번에 실행한다."""

import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_path = Path.cwd() / "src" / "params_setting.json"
    if not params_path.exists():
        params_path = Path(get_package_share_directory("rear_ackermann_controller")) / "config" / "params_setting.json"
    config = json.loads(params_path.read_text(encoding="utf-8"))
    pivot_to_front_m = config["body_length_m"] / 2.0 - config["rear_axle_x_m"]
    controller_parameters = {
        "stop_distance_m": config["alignment_front_clearance_m"],
        "platform_length_m": pivot_to_front_m,
        "pos_tolerance_m": min(config["alignment_lateral_tolerance_m"], config["alignment_longitudinal_tolerance_m"]),
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
    zlac_launch = Path(get_package_share_directory("zlac8015d_driver")) / "launch" / "hardware_drive.launch.py"
    imu_launch = Path(get_package_share_directory("hwt9053_imu")) / "launch" / "hwt9053_imu.launch.py"
    return LaunchDescription([
        # require_safety_enable=true: 유효한 /alignment/drive_enabled 전에는 0x07 Stop 상태를 유지한다.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(zlac_launch)), launch_arguments={"require_safety_enable": "true"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(imu_launch))),
        Node(package="vision", executable="realsense_pose", output="screen"),
        Node(package="shoulder_align_controller", executable="shoulder_align_node",
             parameters=[controller_parameters], output="screen"),
    ])
