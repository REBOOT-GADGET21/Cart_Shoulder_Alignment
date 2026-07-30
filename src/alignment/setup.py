from setuptools import find_packages, setup

package_name = "alignment"
workspace_params = "../params_setting.json"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [workspace_params]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Alignment logic for the CPR project",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "alignment_node=alignment.alignment_node:main",
            "fake_body_landmarks_publisher=alignment.fake_shoulder_publisher:main",
            "gazebo_ground_truth_publisher=alignment.gazebo_ground_truth_publisher:main",
        ],
    },
)
