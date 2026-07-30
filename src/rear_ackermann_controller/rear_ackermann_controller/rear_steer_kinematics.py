"""Fixed-rear-wheel differential-drive kinematics.

The name is retained only for import compatibility with earlier phases.  The
cart no longer has steering joints: yaw is created by a left/right speed
difference.
"""

from __future__ import annotations
from dataclasses import dataclass
from .vehicle_params import VehicleParams


@dataclass(frozen=True)
class WheelCommand:
    """Signed wheel ground speed.  Steering is permanently zero."""

    steering_angle_rad: float
    wheel_speed_mps: float


@dataclass(frozen=True)
class RearSteerCommand:
    """Compatibility container for the left/right differential commands."""

    left: WheelCommand
    right: WheelCommand

def compute_rear_steer_command(
    linear_x_mps: float, angular_z_rad_s: float, params: VehicleParams,
) -> RearSteerCommand:
    """Convert ``Twist`` to fixed-wheel differential-drive speeds.

    Positive ``angular_z`` is counter-clockwise: the left wheel slows down and
    the right wheel speeds up.  With zero linear speed they have equal and
    opposite speeds, producing an in-place rotation.
    """

    angular_z_rad_s = max(-params.max_yaw_rate_rad_s, min(params.max_yaw_rate_rad_s, angular_z_rad_s))
    half_track_m = params.rear_track_m / 2.0
    left_speed_mps = linear_x_mps - angular_z_rad_s * half_track_m
    right_speed_mps = linear_x_mps + angular_z_rad_s * half_track_m

    def limited(speed_mps: float) -> WheelCommand:
        return WheelCommand(
            steering_angle_rad=0.0,
            wheel_speed_mps=max(-params.max_wheel_speed_mps, min(params.max_wheel_speed_mps, speed_mps)),
        )

    return RearSteerCommand(left=limited(left_speed_mps), right=limited(right_speed_mps))
