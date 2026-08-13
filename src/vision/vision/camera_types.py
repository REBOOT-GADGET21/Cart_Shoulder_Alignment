"""Small ROS-independent value types used by the D435 perception pipeline."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float
