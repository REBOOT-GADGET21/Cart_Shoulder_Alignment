"""실제 ZLAC8015D 하드웨어 전용 구동 launch; Gazebo bridge와 함께 실행하지 않는다."""

import json
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def _matmul(left, right):
    return [[sum(left[row][index] * right[index][column] for index in range(3)) for column in range(3)] for row in range(3)]


def _quaternion_from_rotation(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((matrix[2][1] - matrix[1][2]) / scale, (matrix[0][2] - matrix[2][0]) / scale,
                (matrix[1][0] - matrix[0][1]) / scale, 0.25 * scale)
    diagonal = [matrix[0][0], matrix[1][1], matrix[2][2]]
    index = diagonal.index(max(diagonal))
    if index == 0:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return (0.25 * scale, (matrix[0][1] + matrix[1][0]) / scale, (matrix[0][2] + matrix[2][0]) / scale,
                (matrix[2][1] - matrix[1][2]) / scale)
    if index == 1:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return ((matrix[0][1] + matrix[1][0]) / scale, 0.25 * scale, (matrix[1][2] + matrix[2][1]) / scale,
                (matrix[0][2] - matrix[2][0]) / scale)
    scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return ((matrix[0][2] + matrix[2][0]) / scale, (matrix[1][2] + matrix[2][1]) / scale, 0.25 * scale,
            (matrix[1][0] - matrix[0][1]) / scale)


def _camera_optical_quaternion(config):
    """Return base_link <- D435 optical rotation used by geometry.frame_transform."""
    roll = config["camera_roll_rad"]
    pitch = -config["camera_pitch_down_rad"]
    yaw = config["camera_yaw_rad"]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    mount = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
             [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
             [-sp, cp * sr, cp * cr]]
    # D435 optical axes (right, down, forward) -> unpitched ROS camera axes (forward, left, up).
    optical_axes = [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
    return _quaternion_from_rotation(_matmul(mount, optical_axes))


def generate_launch_description() -> LaunchDescription:
    require_safety_enable = LaunchConfiguration("require_safety_enable")
    driver_config = get_package_share_directory("zlac8015d_driver") + "/config/zlac8015d.yaml"
    params_path = Path.cwd() / "src" / "params_setting.json"
    if not params_path.exists():
        params_path = Path(get_package_share_directory("rear_ackermann_controller")) / "config" / "params_setting.json"
    cart_params = json.loads(params_path.read_text(encoding="utf-8"))
    # 차체 기하와 IMU 입력은 params_setting.json을 단일 기준으로 사용한다.
    odometry_parameters = {
        "wheel_radius_m": cart_params["wheel_radius_m"],
        "wheel_track_m": cart_params["rear_track_m"],
        "odom_frame": cart_params["odom_frame"],
        "odom_child_frame": cart_params["rear_axle_frame"],
        "use_imu_yaw": cart_params["use_imu_yaw"],
        "imu_topic": cart_params["imu_topic"],
        "imu_timeout_sec": cart_params["imu_timeout_sec"],
        "imu_yaw_sign": cart_params["imu_yaw_sign"],
    }
    camera_q = _camera_optical_quaternion(cart_params)
    rear_to_base_x_m = -cart_params["rear_axle_x_m"]
    return LaunchDescription([
        # 단독 모터 시험은 false, real_alignment.launch.py는 true를 전달한다.
        DeclareLaunchArgument("require_safety_enable", default_value="false"),
        # Twist 우선순위와 wheel rad/s 변환은 기존 노드를 그대로 재사용한다.
        Node(package="rear_ackermann_controller", executable="rear_ackermann_node", output="screen"),
        # 이 노드는 wheel rad/s만 받아 RS485 Modbus RPM 명령으로 변환한다.
        Node(package="zlac8015d_driver", executable="zlac8015d_driver_node",
             parameters=[driver_config, {"require_safety_enable": ParameterValue(require_safety_enable, value_type=bool)}], output="screen"),
        # 바퀴 encoder feedback만으로 실제 차체의 /odom을 만든다.
        Node(package="zlac8015d_driver", executable="wheel_odometry_node",
             parameters=[odometry_parameters], output="screen"),
        # /imu/data quaternion을 사람이 확인하기 쉬운 roll, pitch, yaw [deg]로만 변환한다.
        Node(package="zlac8015d_driver", executable="imu_degrees_node",
             parameters=[{"imu_yaw_sign": cart_params["imu_yaw_sign"]}], output="screen"),
        # Wheel odometry is at the rear-wheel midpoint. Publish vehicle geometry separately.
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=[str(rear_to_base_x_m), "0", "0", "0", "0", "0", "1",
                        cart_params["rear_axle_frame"], cart_params["base_frame"]], output="screen"),
        # Use the existing calibrated camera translation and optical-frame rotation.
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=[str(cart_params["camera_to_base_x_m"]), str(cart_params["camera_to_base_y_m"]),
                        str(cart_params["camera_to_base_z_m"]), *(str(value) for value in camera_q),
                        cart_params["base_frame"], "camera_color_optical_frame"], output="screen"),
    ])
