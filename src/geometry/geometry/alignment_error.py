"""어깨선 형상을 정렬 오차 추정치로 변환하기 위한 보조 도구"""

from __future__ import annotations
from dataclasses import dataclass
from .vector_math import Point3D, wrap_angle_radians


@dataclass(frozen=True)
class AlignmentError:
    """Alignment error between the vehicle heading and the shoulder line."""

    yaw_error_rad: float
    lateral_error_m: float
    valid: bool


def compute_alignment_error(
    left_shoulder: Point3D,
    right_shoulder: Point3D,
    vehicle_yaw_rad: float,
    desired_lateral_offset_m: float = 0.0,
) -> AlignmentError:
    """Compute heading and lateral alignment error from shoulder points.

    The shoulder line yaw is compared with the vehicle yaw. A positive yaw error
    means the shoulder line is rotated counter-clockwise relative to the vehicle.
    """

    from .shoulder_geometry import compute_shoulder_center, compute_shoulder_yaw_rad

    shoulder_center = compute_shoulder_center(left_shoulder, right_shoulder)
    shoulder_yaw_rad = compute_shoulder_yaw_rad(left_shoulder, right_shoulder)
    # A line has no arrow: headings separated by pi are equally parallel.
    yaw_error_rad = wrap_angle_radians(shoulder_yaw_rad - vehicle_yaw_rad)
    if yaw_error_rad > 1.5707963267948966:
        yaw_error_rad -= 3.141592653589793
    elif yaw_error_rad <= -1.5707963267948966:
        yaw_error_rad += 3.141592653589793

    lateral_error_m = shoulder_center.y_m - desired_lateral_offset_m

    return AlignmentError(
        yaw_error_rad=yaw_error_rad,
        lateral_error_m=lateral_error_m,
        valid=True,
    )
