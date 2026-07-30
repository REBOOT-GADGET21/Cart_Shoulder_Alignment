import math

import pytest

from alignment.alignment_controller import AlignmentControlParams
from alignment.alignment_pipeline import BodyLandmarksOptical, compute_camera_alignment
from alignment.calibration import AlignmentSettings
from geometry.frame_transform import CameraExtrinsics
from geometry.vector_math import Point3D


def test_optical_point_conversion_uses_expected_ros_axes() -> None:
    settings = AlignmentSettings(
        CameraExtrinsics(0.0, 0.0, 0.0), 0.30, 1.2, -0.5, 0.26,
        AlignmentControlParams(0.35, 0.8, 0.15, 0.3, 0.03, 0.035),
        0.035, 0.02, 0.03, 10,
    )
    result = compute_camera_alignment(
        BodyLandmarksOptical(Point3D(0.0, 0.0, 1.5), Point3D(0.0, 0.0, 2.5)), settings
    )

    assert result.valid is True
    assert result.target is not None
    # x/y are now the rear-axle pivot error, not a base_link-centre target.
    assert result.target.x_m == pytest.approx(2.5)
    assert result.target.y_m == pytest.approx(-1.4)
    assert result.target.yaw_rad == pytest.approx(math.pi / 2.0)


def test_parallel_line_does_not_choose_a_pi_rotation() -> None:
    settings = AlignmentSettings(
        CameraExtrinsics(0.0, 0.0, 0.0), 0.30, 1.2, -0.5, 0.26,
        AlignmentControlParams(0.35, 0.8, 0.15, 0.3, 0.03, 0.035),
        0.035, 0.02, 0.03, 10,
    )
    # Reversed shoulder order produces a pi line direction but must remain aligned.
    result = compute_camera_alignment(
        BodyLandmarksOptical(Point3D(0.0, 0.0, 2.5), Point3D(0.0, 0.0, 1.5)), settings
    )

    assert result.target is not None
    # The reversed transverse line may select either +90 or -90 degrees, but
    # must never request a 180-degree turn.
    assert abs(result.target.yaw_rad) == pytest.approx(math.pi / 2.0)
    # The cart first turns toward the front-of-shoulders target.
    assert result.command.angular_z_rad_s > 0.0
