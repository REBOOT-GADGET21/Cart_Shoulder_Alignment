"""Load camera calibration and final-alignment settings from the shared JSON."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from geometry.frame_transform import CameraExtrinsics

from .alignment_controller import AlignmentControlParams


@dataclass(frozen=True)
class AlignmentSettings:
    """Parameters needed between camera 3D points and a Twist command."""

    camera_extrinsics: CameraExtrinsics
    front_clearance_m: float
    body_length_m: float
    rear_axle_x_m: float
    shoulder_pelvis_parallel_tolerance_rad: float
    controller: AlignmentControlParams
    yaw_tolerance_rad: float
    lateral_tolerance_m: float
    longitudinal_tolerance_m: float
    required_stable_frames: int


def load_alignment_settings(config_path: str | None = None) -> AlignmentSettings:
    """Load shared settings; all camera offsets are measured from base_link."""

    if config_path:
        path = Path(config_path)
    else:
        workspace_path = Path.cwd() / "src" / "params_setting.json"
        if workspace_path.exists():
            path = workspace_path
        else:
            from ament_index_python.packages import get_package_share_directory

            path = Path(get_package_share_directory("alignment")) / "config" / "params_setting.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    return AlignmentSettings(
        camera_extrinsics=CameraExtrinsics(
            raw["camera_to_base_x_m"], raw["camera_to_base_y_m"], raw["camera_to_base_z_m"],
            raw["camera_roll_rad"], raw["camera_pitch_down_rad"], raw["camera_yaw_rad"],
        ),
        front_clearance_m=raw["alignment_front_clearance_m"],
        body_length_m=raw["body_length_m"],
        rear_axle_x_m=raw["rear_axle_x_m"],
        shoulder_pelvis_parallel_tolerance_rad=raw["shoulder_pelvis_parallel_tolerance_rad"],
        controller=AlignmentControlParams(
            raw["alignment_position_kp"], raw["alignment_yaw_kp"],
            raw["alignment_max_speed_mps"], raw["alignment_max_yaw_rate_rad_s"],
            min(raw["alignment_lateral_tolerance_m"], raw["alignment_longitudinal_tolerance_m"]), raw["alignment_yaw_tolerance_rad"],
        ),
        yaw_tolerance_rad=raw["alignment_yaw_tolerance_rad"],
        lateral_tolerance_m=raw["alignment_lateral_tolerance_m"],
        longitudinal_tolerance_m=raw["alignment_longitudinal_tolerance_m"],
        required_stable_frames=int(raw["alignment_required_stable_frames"]),
    )
