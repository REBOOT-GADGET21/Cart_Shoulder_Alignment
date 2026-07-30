"""Pure calibration-to-command pipeline used later by fake, Gazebo, and D435 modes."""

from __future__ import annotations

from dataclasses import dataclass

from geometry.body_reference import estimate_body_line_yaw_rad
from geometry.frame_transform import transform_optical_point_to_base
from geometry.shoulder_geometry import compute_shoulder_center
from geometry.vector_math import Point3D

from .alignment_controller import VelocityCommand, compute_final_alignment_command
from .calibration import AlignmentSettings
from .target_pose import TargetPose2D, compute_front_alignment_target


@dataclass(frozen=True)
class BodyLandmarksOptical:
    """D435 optical-frame points; pelvis points may be absent."""

    left_shoulder: Point3D
    right_shoulder: Point3D
    left_pelvis: Point3D | None = None
    right_pelvis: Point3D | None = None


@dataclass(frozen=True)
class AlignmentPipelineResult:
    """Result with transformed target for debugging and a safe command."""

    target: TargetPose2D | None
    command: VelocityCommand
    valid: bool
    body_line_source: str


def compute_camera_alignment(landmarks: BodyLandmarksOptical, settings: AlignmentSettings) -> AlignmentPipelineResult:
    """Transform D435 landmarks, validate body line, and calculate final approach command."""

    transform = lambda point: transform_optical_point_to_base(point, settings.camera_extrinsics) if point else None
    left_shoulder, right_shoulder = transform(landmarks.left_shoulder), transform(landmarks.right_shoulder)
    body_line = estimate_body_line_yaw_rad(
        left_shoulder, right_shoulder, transform(landmarks.left_pelvis), transform(landmarks.right_pelvis),
        settings.shoulder_pelvis_parallel_tolerance_rad,
    )
    if not body_line.valid:
        return AlignmentPipelineResult(None, VelocityCommand(0.0, 0.0), False, body_line.source)
    center = compute_shoulder_center(left_shoulder, right_shoulder)
    target = compute_front_alignment_target(
        center, body_line.yaw_rad, settings.body_length_m, settings.front_clearance_m,
    )
    return AlignmentPipelineResult(target, compute_final_alignment_command(target, settings.controller), True, body_line.source)
