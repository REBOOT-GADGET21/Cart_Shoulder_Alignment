"""Choose a reliable transverse body line from shoulders and pelvis."""

from __future__ import annotations

from dataclasses import dataclass

from .shoulder_geometry import compute_shoulder_yaw_rad
from .vector_math import Point3D, wrap_angle_radians


@dataclass(frozen=True)
class BodyLineEstimate:
    """A transverse body-line yaw with its source and validity."""

    yaw_rad: float
    source: str
    valid: bool


def _parallel_difference_rad(first_yaw_rad: float, second_yaw_rad: float) -> float:
    """Return the smallest difference between two unoriented lines."""

    difference_rad = wrap_angle_radians(first_yaw_rad - second_yaw_rad)
    if difference_rad > 1.5707963267948966:
        difference_rad -= 3.141592653589793
    elif difference_rad <= -1.5707963267948966:
        difference_rad += 3.141592653589793
    return difference_rad


def estimate_body_line_yaw_rad(
    left_shoulder: Point3D | None,
    right_shoulder: Point3D | None,
    left_pelvis: Point3D | None,
    right_pelvis: Point3D | None,
    max_shoulder_pelvis_difference_rad: float,
) -> BodyLineEstimate:
    """Prefer shoulders, use pelvis as a consistency check or safe fallback."""

    shoulder_yaw_rad = None
    pelvis_yaw_rad = None
    try:
        if left_shoulder is not None and right_shoulder is not None:
            shoulder_yaw_rad = compute_shoulder_yaw_rad(left_shoulder, right_shoulder)
    except ValueError:
        pass
    try:
        if left_pelvis is not None and right_pelvis is not None:
            pelvis_yaw_rad = compute_shoulder_yaw_rad(left_pelvis, right_pelvis)
    except ValueError:
        pass

    if shoulder_yaw_rad is not None and pelvis_yaw_rad is not None:
        if abs(_parallel_difference_rad(shoulder_yaw_rad, pelvis_yaw_rad)) <= max_shoulder_pelvis_difference_rad:
            return BodyLineEstimate(shoulder_yaw_rad, "shoulders_verified_by_pelvis", True)
        return BodyLineEstimate(0.0, "shoulder_pelvis_disagree", False)
    if shoulder_yaw_rad is not None:
        return BodyLineEstimate(shoulder_yaw_rad, "shoulders_only", True)
    if pelvis_yaw_rad is not None:
        return BodyLineEstimate(pelvis_yaw_rad, "pelvis_fallback", True)
    return BodyLineEstimate(0.0, "no_valid_body_line", False)
