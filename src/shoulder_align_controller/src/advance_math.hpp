#pragma once
#include <algorithm>
#include <cmath>

namespace advance_math {
inline double speed(double previous, double dt, double maximum)
{
  return std::min({maximum, 0.05, previous + 0.05 * std::clamp(dt, 0.0, 0.05)});
}
inline double yaw_command(double target, double measured, double gain)
{
  const double error = std::atan2(std::sin(target - measured), std::cos(target - measured));
  const double correction = std::copysign(std::max(0.0, std::abs(error) - 3.14159265358979323846 / 360.0), error);
  return std::clamp(gain * correction, -0.05, 0.05);
}
}
