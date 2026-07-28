"""
카트만 띄우는 시뮬레이션 코드/ 움직임을 테스트 할 수 없음
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory("simulation")
    world_path = str(Path(pkg_dir) / "worlds" / "empty.sdf")
    models_dir = str(Path(pkg_dir) / "models")
    model_file = str(Path(pkg_dir) / "models" / "rear_steer_cart" / "model.sdf")
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join([models_dir, existing_resource_path]).rstrip(os.pathsep)

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=resource_path,
            ),
            ExecuteProcess(
                cmd=["gz", "sim", "-r", "-v", "4", world_path],
                output="screen",
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_cart",
                        arguments=[
                            "-world",
                            "default",
                            "-file",
                            model_file,
                            "-name",
                            "rear_steer_cart",
                            "-x",
                            "0.0",
                            "-y",
                            "0.0",
                            "-z",
                            "0.01",
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
