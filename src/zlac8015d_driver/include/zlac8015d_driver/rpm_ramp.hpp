#pragma once

#include <algorithm>
#include <cmath>

namespace zlac8015d_driver
{
// Keep fractional RPM internally; rounding each intermediate step would stall
// a slow ramp forever. Both wheels progress by the same fraction of the change.
struct RpmRamp
{
  double left{0.0}, right{0.0};

  void reset() { left = right = 0.0; }

  void step(double target_left, double target_right, double rate, double dt)
  {
    const double dl = target_left - left, dr = target_right - right;
    const double change = std::max(std::abs(dl), std::abs(dr));
    const double fraction = change > 0.0 ? std::min(1.0, rate * dt / change) : 1.0;
    left += fraction * dl;
    right += fraction * dr;
  }
};
}  // namespace zlac8015d_driver
