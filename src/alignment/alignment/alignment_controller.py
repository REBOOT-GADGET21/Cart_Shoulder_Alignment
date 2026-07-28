"""Low-speed final approach controller, independent of perception hardware."""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry.vector_math import wrap_angle_radians
# 센서 데이터 처리 시 rad이나 도 값이 발생했을 때 [-pi, pi] 또는 [0, 2\pi]로 변환해주는 작업

from .target_pose import TargetPose2D


@dataclass(frozen=True)
class AlignmentControlParams:
    """Conservative final-alignment gains and limits."""

    position_kp: float
    yaw_kp: float
    max_speed_mps: float
    max_yaw_rate_rad_s: float
    position_tolerance_m: float
    yaw_tolerance_rad: float


@dataclass(frozen=True)
class VelocityCommand:
    """Base-frame velocity command for the rear steer-drive kinematics layer."""

    linear_x_mps: float
    angular_z_rad_s: float


def compute_final_alignment_command(target: TargetPose2D, params: AlignmentControlParams) -> VelocityCommand:
    """Drive to target position first, then rotate to final parallel orientation.

    The target is refreshed from each camera frame and is already expressed in
    the current base_link frame, whose origin/yaw are therefore (0, 0, 0).
    """

    distance_m = math.hypot(target.x_m, target.y_m)
    if distance_m > params.position_tolerance_m:
        heading_error_rad = wrap_angle_radians(math.atan2(target.y_m, target.x_m))
        yaw_rate_rad_s = max(-params.max_yaw_rate_rad_s, min(params.max_yaw_rate_rad_s, params.yaw_kp * heading_error_rad))
        # Do not translate until facing the target closely enough to avoid side slip.
        speed_mps = 0.0 if abs(heading_error_rad) > 0.20 else min(params.max_speed_mps, params.position_kp * distance_m)
        return VelocityCommand(speed_mps, yaw_rate_rad_s)

    final_yaw_error_rad = wrap_angle_radians(target.yaw_rad)
    if abs(final_yaw_error_rad) <= params.yaw_tolerance_rad:
        return VelocityCommand(0.0, 0.0)
    yaw_rate_rad_s = max(-params.max_yaw_rate_rad_s, min(params.max_yaw_rate_rad_s, params.yaw_kp * final_yaw_error_rad))
    return VelocityCommand(0.0, yaw_rate_rad_s)
