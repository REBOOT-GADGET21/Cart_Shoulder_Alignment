"""
카트만 띄우는 시뮬레이션 코드/ 움직임을 테스트 할 수 없음
"""

import json
import math
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
    patient_file = str(Path(pkg_dir) / "models" / "person_walking" / "model.sdf")
    left_marker_file = str(Path(pkg_dir) / "models" / "left_shoulder_marker" / "model.sdf")
    right_marker_file = str(Path(pkg_dir) / "models" / "right_shoulder_marker" / "model.sdf")
    config_file = Path.cwd() / "src" / "params_setting.json"
    if not config_file.exists():
        config_file = Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"
    config = json.loads(config_file.read_text(encoding="utf-8"))
    left_x, left_y = config["gazebo_fake_left_shoulder_x_m"], config["gazebo_fake_left_shoulder_y_m"]
    right_x, right_y = config["gazebo_fake_right_shoulder_x_m"], config["gazebo_fake_right_shoulder_y_m"]
    body_yaw = math.atan2(right_y - left_y, right_x - left_x) + math.pi / 2.0
    shoulder_center_x, shoulder_center_y = (left_x + right_x) / 2.0, (left_y + right_y) / 2.0
    # In person_walking/model.sdf, shoulder centre is local (-0.55, 0).
    patient_x = shoulder_center_x + 0.55 * math.cos(body_yaw)
    patient_y = shoulder_center_y + 0.55 * math.sin(body_yaw)
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
                    ),
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_patient",
                        arguments=["-world", "default", "-file", patient_file, "-name", "person_walking", "-x", str(patient_x), "-y", str(patient_y), "-z", "0.0", "-Y", str(body_yaw)],
                        output="screen",
                    ),
                    # Visible ground-truth landmarks: red = left, yellow = right.
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_left_shoulder_marker",
                        arguments=["-world", "default", "-file", left_marker_file, "-name", "left_shoulder_marker", "-x", str(left_x), "-y", str(left_y), "-z", "0.0"],
                        output="screen",
                    ),
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_right_shoulder_marker",
                        arguments=["-world", "default", "-file", right_marker_file, "-name", "right_shoulder_marker", "-x", str(right_x), "-y", str(right_y), "-z", "0.0"],
                        output="screen",
                    ),
                ],
            ),
        ]
    )
