"""Small validation helpers shared by pure geometry functions."""

from __future__ import annotations

import math

from .vector_math import Point3D


def is_finite_point(point: Point3D) -> bool:
    """Return whether all coordinates of a point in metres are finite."""

    return all(math.isfinite(value) for value in (point.x_m, point.y_m, point.z_m))
