#pragma once
#include <algorithm>
#include <cmath>

// Fixed-target polar Lyapunov feedback. All distances are metres, angles radians.
namespace snapshot_math {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
struct Command { double v; double w; };
inline Command control(double rho, double alpha, double beta, double kr, double ka,
  double kb, double vmax, double wmax, int direction)
{
  const double c = std::cos(alpha);
  const double sinc = std::abs(alpha) < 1e-6 ? 1.0 : std::sin(alpha) / alpha;
  double v = kr * rho * c;
  double w = ka * alpha + kr * sinc * c * (alpha + kb * beta);
  // Scale both commands together: independent saturation destroys the ratio
  // used in the Lyapunov cancellation. Do not add a minimum-speed floor.
  const double scale = std::min({1.0, vmax / std::max(std::abs(v), 1e-12),
    wmax / std::max(std::abs(w), 1e-12)});
  return {direction * v * scale, w * scale};
}
}  // namespace snapshot_math
