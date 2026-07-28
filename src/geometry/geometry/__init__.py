"""Geometry helpers for the CPR alignment project."""

from .alignment_error import AlignmentError, compute_alignment_error
from .shoulder_geometry import compute_shoulder_center, compute_shoulder_yaw_rad
from .vector_math import Point3D, vector_between_points, wrap_angle_radians
from .validation import is_finite_point
from .body_reference import BodyLineEstimate, estimate_body_line_yaw_rad
from .frame_transform import CameraExtrinsics, transform_optical_point_to_base

__all__ = [
    "AlignmentError",
    "Point3D",
    "compute_alignment_error",
    "compute_shoulder_center",
    "compute_shoulder_yaw_rad",
    "vector_between_points",
    "wrap_angle_radians",
    "is_finite_point",
    "BodyLineEstimate",
    "estimate_body_line_yaw_rad",
    "CameraExtrinsics",
    "transform_optical_point_to_base",
]
