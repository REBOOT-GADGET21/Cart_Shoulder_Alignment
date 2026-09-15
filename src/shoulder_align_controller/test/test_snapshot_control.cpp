#include <cassert>
#include <cmath>
#include "../src/snapshot_math.hpp"

int main()
{
  using namespace snapshot_math;
  // V=(rho^2+alpha^2+kb*beta^2)/2 must decrease in continuous polar dynamics,
  // including shared actuator saturation. Test away from angle wrap boundaries.
  for (double a : {-1.2, -0.4, 0.0, 0.4, 1.2}) {
    for (double b : {-0.5, 0.0, 0.5}) {
      const double r=0.6, kb=0.6;
      auto cmd=control(r,a,b,0.35,0.8,kb,0.3,0.1,1);
      const double bearing_dot=cmd.v/r*std::sin(a);
      const double dV=-r*cmd.v*std::cos(a)+a*(bearing_dot-cmd.w)+kb*b*bearing_dot;
      assert(dV < 0.0);
      assert(std::abs(cmd.v)<=0.3 && std::abs(cmd.w)<=0.100001);
    }
  }
  // Fixed goal: ideal unicycle must approach from both sides without a 180deg spin.
  for (int direction : {-1,1}) {
    double x=0, y=0, yaw=0;
    const double gx=direction*0.5, gy=0.03;
    for (int i=0;i<20000;++i) {
      const double rho=std::hypot(gx-x,gy-y);
      if (rho<0.02) break;
      const double bearing=std::atan2(gy-y,gx-x), offset=direction<0?pi:0;
      auto cmd=control(rho,wrap(bearing-yaw-offset),wrap(bearing-offset),
        0.35,0.8,0.6,0.3,0.1,direction);
      x+=cmd.v*std::cos(yaw)*0.01; y+=cmd.v*std::sin(yaw)*0.01;
      yaw+=cmd.w*0.01;
    }
    assert(std::hypot(gx-x,gy-y)<0.02);
  }
}
