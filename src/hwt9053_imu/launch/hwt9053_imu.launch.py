import json
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    enable_static_tf = LaunchConfiguration('enable_static_tf')
    params_path = Path.cwd() / 'src' / 'params_setting.json'
    if not params_path.exists():
        params_path = Path(get_package_share_directory('rear_ackermann_controller')) / 'config' / 'params_setting.json'
    cart_params = json.loads(params_path.read_text(encoding='utf-8'))
    # IMU 장착 위치와 방향은 차량 공통 설정인 params_setting.json에서만 읽는다.
    static_tf_parameters = {
        'base_frame': cart_params['base_frame'],
        'imu_frame': 'imu_link',
        'x': cart_params['base_to_imu_x_m'],
        'y': cart_params['base_to_imu_y_m'],
        'z': cart_params['base_to_imu_z_m'],
        'roll': cart_params['base_to_imu_roll_rad'],
        'pitch': cart_params['base_to_imu_pitch_rad'],
        'yaw': cart_params['base_to_imu_yaw_rad'],
    }
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('hwt9053_imu'), 'config', 'hwt9053_imu.yaml'])),
        DeclareLaunchArgument('enable_static_tf', default_value='true'),
        Node(
            package='hwt9053_imu', executable='hwt9053_imu_node', name='hwt9053_imu',
            parameters=[LaunchConfiguration('params_file')]),
        Node(
            package='hwt9053_imu', executable='imu_static_tf_node', name='imu_static_tf',
            condition=IfCondition(enable_static_tf),
            parameters=[static_tf_parameters]),
    ])
