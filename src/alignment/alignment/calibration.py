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
    lateral_offset_m: float
    shoulder_pelvis_parallel_tolerance_rad: float
    controller: AlignmentControlParams


def load_alignment_settings(config_path: str | None = None) -> AlignmentSettings:
    """Load shared settings; all camera offsets are measured from base_link."""

    path = Path(config_path) if config_path else Path(__file__).resolve().parents[3] / "params_setting.json"
    raw = json.loads(path.read_text(encoding="utf-8"))
    return AlignmentSettings(
        camera_extrinsics=CameraExtrinsics(
            raw["camera_to_base_x_m"], raw["camera_to_base_y_m"], raw["camera_to_base_z_m"],
            raw["camera_roll_rad"], raw["camera_pitch_down_rad"], raw["camera_yaw_rad"],
        ),
        lateral_offset_m=raw["alignment_lateral_offset_m"],
        shoulder_pelvis_parallel_tolerance_rad=raw["shoulder_pelvis_parallel_tolerance_rad"],
        controller=AlignmentControlParams(
            raw["alignment_position_kp"], raw["alignment_yaw_kp"],
            raw["alignment_max_speed_mps"], raw["alignment_max_yaw_rate_rad_s"],
            raw["alignment_longitudinal_tolerance_m"], raw["alignment_yaw_tolerance_rad"],
        ),
    )
