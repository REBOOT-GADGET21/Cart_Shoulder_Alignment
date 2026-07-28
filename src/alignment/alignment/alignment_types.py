"""Minimal data structures for alignment state."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ShoulderPoseInput:
    """A simple shoulder pose input received from a publisher."""

    left_x_m: float
    left_y_m: float
    right_x_m: float
    right_y_m: float
