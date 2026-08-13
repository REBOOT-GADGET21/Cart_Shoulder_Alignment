"""Depth sampling and pinhole deprojection in camera_optical_frame coordinates."""

from __future__ import annotations

import math
from typing import Sequence

from .camera_types import CameraIntrinsics, Point3D


def robust_depth_m(depth_image, u: int, v: int, depth_scale: float, radius_px: int = 2) -> float | None:
    """Return the median non-zero depth around a landmark pixel."""
    height, width = depth_image.shape[:2]
    values = []
    for y in range(max(0, v - radius_px), min(height, v + radius_px + 1)):
        for x in range(max(0, u - radius_px), min(width, u + radius_px + 1)):
            raw = float(depth_image[y, x])
            if raw > 0.0:
                values.append(raw * depth_scale)
    if not values:
        return None
    values.sort()
    return values[len(values) // 2]


def deproject_pixel(intrinsics: CameraIntrinsics, u: float, v: float, depth_m: float) -> Point3D | None:
    """Map a color-aligned pixel to ROS optical coordinates (x right, y down, z forward)."""
    if depth_m <= 0.0 or not math.isfinite(depth_m):
        return None
    return Point3D((u - intrinsics.ppx) * depth_m / intrinsics.fx,
                   (v - intrinsics.ppy) * depth_m / intrinsics.fy, depth_m)
