"""Gazebo 가짜 어깨선의 중심·폭·각도 파라미터를 좌표로 변환한다."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from geometry.vector_math import Point3D


@dataclass(frozen=True)
class FakeShoulderLine:
    """Gazebo world(=odom) 좌표계의 좌·우 어깨 점이다."""

    left: Point3D
    right: Point3D


def fake_shoulder_line_from_config(config: Mapping[str, object]) -> FakeShoulderLine:
    """중심·폭·반시계 양수 각도 정의를 기존 left/right 순서로 변환한다."""

    center_x_m = float(config["gazebo_fake_shoulder_center_x_m"])
    center_y_m = float(config["gazebo_fake_shoulder_center_y_m"])
    width_m = float(config["gazebo_fake_shoulder_width_m"])
    angle_deg = float(config["gazebo_fake_shoulder_angle_deg"])
    if not math.isfinite(width_m) or width_m < 0.05:
        raise ValueError("gazebo_fake_shoulder_width_m must be at least 0.05 m")
    if not math.isfinite(angle_deg):
        raise ValueError("gazebo_fake_shoulder_angle_deg must be finite")

    # angle=0이면 left는 +X, right는 -X이며 양수는 Gazebo x-y 평면 반시계 회전이다.
    angle_rad = math.radians(angle_deg)
    half_width_m = width_m * 0.5
    offset_x_m = half_width_m * math.cos(angle_rad)
    offset_y_m = half_width_m * math.sin(angle_rad)
    return FakeShoulderLine(
        left=Point3D(center_x_m + offset_x_m, center_y_m + offset_y_m, 0.25),
        right=Point3D(center_x_m - offset_x_m, center_y_m - offset_y_m, 0.25),
    )
