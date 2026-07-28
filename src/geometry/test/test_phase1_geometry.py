"""
mediapipe없이 어깨 선의 방향을 계산하는 코드
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry.alignment_error import AlignmentError, compute_alignment_error
from geometry.shoulder_geometry import (
    compute_shoulder_center,
    compute_shoulder_yaw_rad,
)
from geometry.vector_math import Point3D, vector_between_points, wrap_angle_radians
from geometry.body_reference import estimate_body_line_yaw_rad


def test_vector_between_points_returns_expected_component() -> None:
    start_point = Point3D(1.0, 2.0, 3.0)
    end_point = Point3D(4.0, 6.0, 8.0)

    vector = vector_between_points(start_point, end_point)

    assert vector == pytest.approx([3.0, 4.0, 5.0])


def test_wrap_angle_radians_keeps_range() -> None:
    wrapped_angle = wrap_angle_radians(7 * 3.141592653589793 / 2.0)

    assert wrapped_angle == pytest.approx(-3.141592653589793 / 2.0)


def test_shoulder_center_is_midpoint() -> None:
    left_shoulder = Point3D(0.0, 0.0, 0.0)
    right_shoulder = Point3D(2.0, 0.0, 0.0)

    center_point = compute_shoulder_center(left_shoulder, right_shoulder)

    assert center_point == Point3D(1.0, 0.0, 0.0)


def test_shoulder_yaw_is_zero_for_horizontal_points() -> None:
    left_shoulder = Point3D(0.0, 0.0, 0.0)
    right_shoulder = Point3D(1.0, 0.0, 0.0)

    yaw_rad = compute_shoulder_yaw_rad(left_shoulder, right_shoulder)

    assert yaw_rad == pytest.approx(0.0, abs=1e-9)


def test_shoulder_yaw_changes_when_point_order_is_reversed() -> None:
    left_shoulder = Point3D(0.0, 0.0, 0.0)
    right_shoulder = Point3D(1.0, 1.0, 0.0)

    forward_yaw = compute_shoulder_yaw_rad(left_shoulder, right_shoulder)
    reverse_yaw = compute_shoulder_yaw_rad(right_shoulder, left_shoulder)

    assert forward_yaw != pytest.approx(reverse_yaw)


def test_same_points_raise_value_error() -> None:
    left_shoulder = Point3D(0.0, 0.0, 0.0)
    right_shoulder = Point3D(0.0, 0.0, 0.0)

    with pytest.raises(ValueError):
        compute_shoulder_yaw_rad(left_shoulder, right_shoulder)


def test_alignment_error_is_zero_for_aligned_case() -> None:
    left_shoulder = Point3D(0.0, 0.0, 0.0)
    right_shoulder = Point3D(2.0, 0.0, 0.0)

    error = compute_alignment_error(left_shoulder, right_shoulder, vehicle_yaw_rad=0.0)

    assert isinstance(error, AlignmentError)
    assert error.valid is True
    assert error.yaw_error_rad == pytest.approx(0.0, abs=1e-9)
    assert error.lateral_error_m == pytest.approx(0.0, abs=1e-9)


def test_alignment_error_detects_heading_offset() -> None:
    left_shoulder = Point3D(0.0, 0.0, 0.0)
    right_shoulder = Point3D(2.0, 1.0, 0.0)

    error = compute_alignment_error(left_shoulder, right_shoulder, vehicle_yaw_rad=0.0)

    assert error.yaw_error_rad == pytest.approx(math.atan2(1.0, 2.0), abs=1e-9)


def test_pelvis_can_verify_shoulder_body_line() -> None:
    estimate = estimate_body_line_yaw_rad(
        Point3D(0.0, 0.0, 0.0), Point3D(1.0, 0.1, 0.0),
        Point3D(0.0, 1.0, 0.0), Point3D(1.0, 1.1, 0.0), 0.26,
    )

    assert estimate.valid is True
    assert estimate.source == "shoulders_verified_by_pelvis"
