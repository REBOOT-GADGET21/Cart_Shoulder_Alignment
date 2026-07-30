"""Camera optical-frame to vehicle-base-frame calibration helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .vector_math import Point3D


@dataclass(frozen=True)
class CameraExtrinsics:
    """Measured pose of camera_link relative to base_link, in metres/radians."""

    translation_x_m: float
    translation_y_m: float
    translation_z_m: float
    roll_rad: float = 0.0
    pitch_down_rad: float = 0.0
    yaw_rad: float = 0.0


def transform_optical_point_to_base(point_optical_m: Point3D, extrinsics: CameraExtrinsics) -> Point3D:
    """Transform a D435 optical point into ROS ``base_link`` coordinates.

    Optical axes are x right, y down, z forward.  base_link axes are x forward,
    y left, z up. ``pitch_down_rad`` is positive when the camera looks downward.
    """

    # optical -> unpitched camera_link: (forward, left, up) = (z, -x, -y)
    camera_x_m = point_optical_m.z_m
    camera_y_m = -point_optical_m.x_m
    camera_z_m = -point_optical_m.y_m

    roll, pitch, yaw = extrinsics.roll_rad, -extrinsics.pitch_down_rad, extrinsics.yaw_rad
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Rz(yaw) * Ry(pitch) * Rx(roll)
    base_x_m = (cy * cp) * camera_x_m + (cy * sp * sr - sy * cr) * camera_y_m + (cy * sp * cr + sy * sr) * camera_z_m
    base_y_m = (sy * cp) * camera_x_m + (sy * sp * sr + cy * cr) * camera_y_m + (sy * sp * cr - cy * sr) * camera_z_m
    base_z_m = -sp * camera_x_m + cp * sr * camera_y_m + cp * cr * camera_z_m
    return Point3D(
        base_x_m + extrinsics.translation_x_m,
        base_y_m + extrinsics.translation_y_m,
        base_z_m + extrinsics.translation_z_m,
    )


def transform_base_point_to_optical(point_base_m: Point3D, extrinsics: CameraExtrinsics) -> Point3D:
    """Inverse of ``transform_optical_point_to_base`` for Gazebo fake perception."""

    base_x_m = point_base_m.x_m - extrinsics.translation_x_m
    base_y_m = point_base_m.y_m - extrinsics.translation_y_m
    base_z_m = point_base_m.z_m - extrinsics.translation_z_m
    roll, pitch, yaw = extrinsics.roll_rad, -extrinsics.pitch_down_rad, extrinsics.yaw_rad
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Transpose of Rz(yaw) * Ry(pitch) * Rx(roll).
    camera_x_m = (cy * cp) * base_x_m + (sy * cp) * base_y_m - sp * base_z_m
    camera_y_m = (cy * sp * sr - sy * cr) * base_x_m + (sy * sp * sr + cy * cr) * base_y_m + cp * sr * base_z_m
    camera_z_m = (cy * sp * cr + sy * sr) * base_x_m + (sy * sp * cr - cy * sr) * base_y_m + cp * cr * base_z_m
    return Point3D(-camera_y_m, -camera_z_m, camera_x_m)
