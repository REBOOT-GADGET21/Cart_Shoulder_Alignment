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


@dataclass(frozen=True)
class FakeBodyReference:
    """Synthetic head/pelvis centres derived from the configured shoulder line."""

    shoulder_line: FakeShoulderLine
    head: Point3D
    pelvis: Point3D


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


def fake_body_reference_from_config(config: Mapping[str, object]) -> FakeBodyReference:
    """Derive a directed body axis; positive sign puts the head on the CCW shoulder normal."""

    shoulder_line = fake_shoulder_line_from_config(config)
    center_x = (shoulder_line.left.x_m + shoulder_line.right.x_m) / 2.0
    center_y = (shoulder_line.left.y_m + shoulder_line.right.y_m) / 2.0
    dx = shoulder_line.right.x_m - shoulder_line.left.x_m
    dy = shoulder_line.right.y_m - shoulder_line.left.y_m
    length = math.hypot(dx, dy)
    normal_x, normal_y = -dy / length, dx / length
    head_sign = float(config.get("gazebo_fake_head_normal_sign", -1.0))
    body_offset_m = float(config.get("gazebo_fake_body_axis_offset_m", 0.55))
    if head_sign not in (-1.0, 1.0) or body_offset_m <= 0.0:
        raise ValueError("gazebo_fake_head_normal_sign must be -1 or 1 and body axis offset must be positive")
    head = Point3D(center_x + head_sign * body_offset_m * normal_x,
                   center_y + head_sign * body_offset_m * normal_y, shoulder_line.left.z_m)
    pelvis = Point3D(center_x - head_sign * body_offset_m * normal_x,
                     center_y - head_sign * body_offset_m * normal_y, shoulder_line.left.z_m)
    return FakeBodyReference(shoulder_line, head, pelvis)
