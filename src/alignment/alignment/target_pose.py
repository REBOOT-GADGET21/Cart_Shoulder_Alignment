"""Calculate the desired base_link pose from a person's transverse body line."""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry.vector_math import Point3D, wrap_angle_radians


@dataclass(frozen=True)
class TargetPose2D:
    """Desired vehicle origin and yaw, expressed in the current base_link frame."""

    x_m: float
    y_m: float
    yaw_rad: float


def closest_parallel_yaw_rad(line_yaw_rad: float, reference_yaw_rad: float = 0.0) -> float:
    """Choose the parallel line direction nearest to a reference yaw, never ±pi away."""

    first = wrap_angle_radians(line_yaw_rad)
    second = wrap_angle_radians(line_yaw_rad + math.pi)
    return min((first, second), key=lambda yaw_rad: abs(wrap_angle_radians(yaw_rad - reference_yaw_rad)))


def compute_side_alignment_target(
    body_center: Point3D,
    body_line_yaw_rad: float,
    lateral_offset_m: float,
) -> TargetPose2D:
    """Place base_link at a signed normal offset while keeping it parallel to body line."""

    yaw_rad = closest_parallel_yaw_rad(body_line_yaw_rad)
    normal_x_m, normal_y_m = -math.sin(yaw_rad), math.cos(yaw_rad)
    return TargetPose2D(
        x_m=body_center.x_m + lateral_offset_m * normal_x_m,
        y_m=body_center.y_m + lateral_offset_m * normal_y_m,
        yaw_rad=yaw_rad,
    )
