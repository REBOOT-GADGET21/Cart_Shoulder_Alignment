#pragma once

#include <algorithm>

namespace rear_ackermann_controller
{

struct VehicleParams
{
  double rear_track_m{0.5};
  double wheel_radius_m{0.2};
  double max_wheel_speed_mps{0.4};
  double max_yaw_rate_rad_s{0.5};
};

struct WheelSpeeds
{
  double left_mps{};
  double right_mps{};
};

inline WheelSpeeds compute_wheel_speeds(
  const double linear_x_mps, double angular_z_rad_s, const VehicleParams & params)
{
  angular_z_rad_s = std::clamp(
    angular_z_rad_s, -params.max_yaw_rate_rad_s, params.max_yaw_rate_rad_s);
  const double half_track_m = params.rear_track_m / 2.0;
  return {
    std::clamp(
      linear_x_mps - angular_z_rad_s * half_track_m,
      -params.max_wheel_speed_mps, params.max_wheel_speed_mps),
    std::clamp(
      linear_x_mps + angular_z_rad_s * half_track_m,
      -params.max_wheel_speed_mps, params.max_wheel_speed_mps)};
}

}  // namespace rear_ackermann_controller
