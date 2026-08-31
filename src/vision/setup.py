from setuptools import find_packages, setup

package_name = "vision"
workspace_params = "../params_setting.json"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [workspace_params]),
        ("share/" + package_name + "/launch", ["launch/realsense_pose.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/shoulder_alignment.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="D435 pose pipeline with isolated MediaPipe and YOLO11n-pose backends",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "realsense_pose = vision.vision_node:main",
            "shoulder_line_markers = vision.shoulder_markers:main",
            "analyze_shoulder_measurements = vision.measurement_analysis:main",
        ],
    },
)
