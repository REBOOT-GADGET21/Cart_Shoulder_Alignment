#include <cassert>
#include <cmath>
#include "zlac8015d_driver/rpm_ramp.hpp"

int main()
{
  zlac8015d_driver::RpmRamp ramp;
  for (int i = 0; i < 20; ++i) {
    const double previous = ramp.right;
    ramp.step(-5.0, 5.0, 5.0, 0.05);
    assert(ramp.right - previous <= 0.250001);
    assert(std::abs(ramp.left + ramp.right) < 1e-9);
  }
  assert(std::abs(ramp.right - 5.0) < 1e-9);
  // Wheel reversal progresses through zero, without disturbing the other wheel.
  for (int i = 0; i < 40; ++i) {
    const double previous = ramp.left;
    ramp.step(5.0, 5.0, 5.0, 0.05);
    assert(std::abs(ramp.left - previous) <= 0.250001);
    assert(ramp.right == 5.0);
  }
  assert(std::abs(ramp.left - 5.0) < 1e-9);
  ramp.step(0.0, 0.0, 5.0, 0.05);
  assert(ramp.left > 0.0);  // Normal zero commands decelerate.
  ramp.reset();             // Safety stop clears remembered motion immediately.
  assert(ramp.left == 0.0 && ramp.right == 0.0);
  ramp.step(1.0, -1.0, 5.0, 0.05);
  assert(ramp.left == 0.25 && ramp.right == -0.25);
}
