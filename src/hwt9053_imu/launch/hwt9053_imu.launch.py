from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    enable_static_tf = LaunchConfiguration('enable_static_tf')
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
            parameters=[LaunchConfiguration('params_file')]),
    ])
