"""Safe, persistent completion decision for final platform alignment."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AlignmentThresholds:
    """Errors that must all be met before alignment may stop."""

    yaw_tolerance_rad: float
    lateral_tolerance_m: float
    longitudinal_tolerance_m: float
    required_stable_frames: int


@dataclass(frozen=True)
class AlignmentMeasurement:
    """Current target-relative errors in metres/radians."""

    yaw_error_rad: float
    lateral_error_m: float
    longitudinal_error_m: float
    landmarks_valid: bool


class AlignmentCompletionChecker:
    """Require consecutive valid in-tolerance frames before declaring ALIGNED."""

    def __init__(self, thresholds: AlignmentThresholds) -> None:
        self._thresholds = thresholds
        self._stable_frames = 0

    def update(self, measurement: AlignmentMeasurement) -> bool:
        """Record one measurement and return whether it is safely aligned."""

        within_tolerance = (
            measurement.landmarks_valid
            and abs(measurement.yaw_error_rad) <= self._thresholds.yaw_tolerance_rad
            and abs(measurement.lateral_error_m) <= self._thresholds.lateral_tolerance_m
            and abs(measurement.longitudinal_error_m) <= self._thresholds.longitudinal_tolerance_m
        )
        self._stable_frames = self._stable_frames + 1 if within_tolerance else 0
        return self._stable_frames >= self._thresholds.required_stable_frames
