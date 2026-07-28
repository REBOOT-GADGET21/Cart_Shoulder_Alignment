import os
import subprocess
from pathlib import Path


def test_xacro_file_generates_urdf():
    repo_root = Path(__file__).resolve().parents[3]
    xacro_path = repo_root / "src" / "simulation" / "urdf" / "rear_steer_cart.urdf.xacro"

    env = os.environ.copy()
    env["ROS_PACKAGE_PATH"] = str(repo_root / "src") + os.pathsep + env.get("ROS_PACKAGE_PATH", "")

    result = subprocess.run(
        ["xacro", str(xacro_path)],
        capture_output=True,
        text=True,
        check=False,
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "<robot" in result.stdout
    assert "base_link" in result.stdout
