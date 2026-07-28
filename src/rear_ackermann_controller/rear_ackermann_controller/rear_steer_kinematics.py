"""Independent rear steer-drive kinematics for the CPR alignment cart.

The two powered rear modules may each steer and drive.  The front caster is
passive, so it is intentionally not commanded here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .vehicle_params import VehicleParams


@dataclass(frozen=True)
class WheelCommand:
    """One rear wheel command: steering radians and signed ground speed m/s."""

    steering_angle_rad: float
    wheel_speed_mps: float


@dataclass(frozen=True)
class RearSteerCommand:
    """Independent commands for left and right powered rear modules."""

    left: WheelCommand
    right: WheelCommand


def _nearest_steering_representation(
    heading_rad: float, speed_mps: float, max_angle_rad: float
) -> WheelCommand:
    """Use wheel reversal to keep a steering command within its allowed range."""

    candidates = (
        (heading_rad, speed_mps),
        (heading_rad + math.pi, -speed_mps),
        (heading_rad - math.pi, -speed_mps),
    )
    valid = [candidate for candidate in candidates if abs(candidate[0]) <= max_angle_rad + 1e-9]
    if not valid:
        # The requested instantaneous centre is unreachable.  Keep the wheel
        # command safe; a higher-level controller must observe the residual.
        return WheelCommand(
            math.copysign(max_angle_rad, heading_rad), speed_mps
        )
    angle_rad, signed_speed_mps = min(valid, key=lambda item: abs(item[0]))
    return WheelCommand(angle_rad, signed_speed_mps)


def compute_rear_steer_command(
    linear_x_mps: float,
    angular_z_rad_s: float,
    params: VehicleParams,
) -> RearSteerCommand:
    """Convert a base-frame Twist to independent rear steering and drive commands.

    ``linear_x_mps`` is forward and ``angular_z_rad_s`` is counter-clockwise.
    For zero linear speed and nonzero yaw rate this produces an in-place turn
    about ``base_link``; the passive front caster follows the resulting arc.
    """

    angular_z_rad_s = max(-params.max_yaw_rate_rad_s, min(params.max_yaw_rate_rad_s, angular_z_rad_s))
    if abs(linear_x_mps) < 1e-9 and abs(angular_z_rad_s) < 1e-9:
        stopped = WheelCommand(0.0, 0.0)
        return RearSteerCommand(stopped, stopped)

    def command_at(y_m: float) -> WheelCommand:
        velocity_x_mps = linear_x_mps - angular_z_rad_s * y_m
        velocity_y_mps = angular_z_rad_s * params.rear_axle_x_m
        speed_mps = math.hypot(velocity_x_mps, velocity_y_mps)
        heading_rad = math.atan2(velocity_y_mps, velocity_x_mps)
        command = _nearest_steering_representation(
            heading_rad, speed_mps, params.max_steering_angle_rad
        )
        limited_speed_mps = max(
            -params.max_wheel_speed_mps,
            min(params.max_wheel_speed_mps, command.wheel_speed_mps),
        )
        return WheelCommand(command.steering_angle_rad, limited_speed_mps)

    half_track_m = params.rear_track_m / 2.0
    return RearSteerCommand(left=command_at(half_track_m), right=command_at(-half_track_m))
