"""Temporal filtering for individual 3-D landmarks."""

from __future__ import annotations

from collections import deque
from statistics import median

from .camera_types import Point3D


class LandmarkFilter:
    """EMA with a median fallback when a new point is implausibly far away."""

    def __init__(self, alpha: float = 0.35, median_window: int = 7, outlier_distance_m: float = 0.20):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.outlier_distance_m = outlier_distance_m
        self.history: deque[Point3D] = deque(maxlen=median_window)
        self.ema: Point3D | None = None

    def update(self, point: Point3D) -> Point3D:
        if self.history:
            med = Point3D(*(median([getattr(p, axis) for p in self.history]) for axis in ("x", "y", "z")))
            distance = ((point.x-med.x)**2 + (point.y-med.y)**2 + (point.z-med.z)**2) ** 0.5
            if distance > self.outlier_distance_m:
                point = med
        self.history.append(point)
        if self.ema is None:
            self.ema = point
        else:
            a = self.alpha
            self.ema = Point3D(a * point.x + (1-a) * self.ema.x,
                               a * point.y + (1-a) * self.ema.y,
                               a * point.z + (1-a) * self.ema.z)
        return self.ema

    def reset(self) -> None:
        self.history.clear()
        self.ema = None
