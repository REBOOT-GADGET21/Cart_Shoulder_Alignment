from setuptools import setup
from pathlib import Path

package_name = "simulation"
package_root = Path(__file__).parent.resolve()


def asset_data_files(directory: str):
    """Install assets while preserving their paths below this package."""
    assets = sorted(path for path in (package_root / directory).rglob("*") if path.is_file())
    grouped_assets = {}
    for asset in assets:
        destination = Path("share") / package_name / asset.relative_to(package_root).parent
        grouped_assets.setdefault(str(destination), []).append(str(asset.relative_to(package_root)))
    return list(grouped_assets.items())

setup(
    name=package_name,
    version="0.0.1",
    packages=[],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Gazebo simulation assets for the CPR project",
    license="Apache-2.0",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [str(path.relative_to(package_root)) for path in (package_root / "launch").glob("*.py")]),
        ("share/" + package_name + "/urdf", [str(path.relative_to(package_root)) for path in (package_root / "urdf").glob("*") if path.is_file()]),
        *asset_data_files("worlds"),
        *asset_data_files("models"),
    ],
)
