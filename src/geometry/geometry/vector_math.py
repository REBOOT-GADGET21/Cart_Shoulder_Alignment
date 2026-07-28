"""기하학 계산을 위한 기본적인 벡터 및 각도 보조 도구"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Point3D:
    """A 3D point expressed in meters."""

    x_m: float
    y_m: float
    z_m: float


def vector_between_points(start_point: Point3D, end_point: Point3D) -> list[float]:
    """Return the vector from start_point to end_point."""

    return [
        end_point.x_m - start_point.x_m,
        end_point.y_m - start_point.y_m,
        end_point.z_m - start_point.z_m,
    ]


def wrap_angle_radians(angle_rad: float) -> float:
    """Wrap any angle into the range [-pi, pi)."""

    wrapped_angle = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    if wrapped_angle <= -math.pi:
        wrapped_angle += 2.0 * math.pi
    return wrapped_angle
