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


def compute_front_alignment_target(
    body_center: Point3D,
    body_line_yaw_rad: float,
    cart_body_length_m: float,
    front_clearance_m: float,
) -> TargetPose2D:
    """Place the cart in front of the shoulder line with parallel body axes.

    ``body_line_yaw_rad`` is the shoulder-to-shoulder line, i.e. the person's
    width.  The cart length is parallel to the person's body direction (90
    degrees from that line).  The target is measured from the cart's *front
    edge*, not its centre: half the cart length is included automatically.
    """

    yaw_rad = closest_parallel_yaw_rad(body_line_yaw_rad + math.pi / 2.0)
    front_to_shoulder_m = cart_body_length_m / 2.0 + front_clearance_m
    return TargetPose2D(
        x_m=body_center.x_m - front_to_shoulder_m * math.cos(yaw_rad),
        y_m=body_center.y_m - front_to_shoulder_m * math.sin(yaw_rad),
        yaw_rad=yaw_rad,
    )
