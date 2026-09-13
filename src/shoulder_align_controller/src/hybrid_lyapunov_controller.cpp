#include "shoulder_align_controller/hybrid_lyapunov_controller.hpp"

#include <algorithm>
#include <cmath>

namespace shoulder_align_controller
{

double sinc_unnormalized(const double angle_rad)
{
  if (std::abs(angle_rad) < 1e-4) {
    return 1.0 - angle_rad * angle_rad / 6.0;
  }
  return std::sin(angle_rad) / angle_rad;
}

double heading_speed_gate(const double abs_alpha_rad, const double full_speed_rad, const double stop_rad)
{
  if (abs_alpha_rad <= full_speed_rad) {return 1.0;}
  if (abs_alpha_rad >= stop_rad) {return 0.0;}
  const double progress = (abs_alpha_rad - full_speed_rad) / (stop_rad - full_speed_rad);
  return 0.5 * (1.0 + std::cos(kPi * progress));
}

double final_heading_weight(const double rho_m, const double start_m, const double full_m)
{
  if (rho_m >= start_m) {return 0.0;}
  if (rho_m <= full_m) {return 1.0;}
  const double s = (start_m - rho_m) / (start_m - full_m);
  return 3.0 * s * s - 2.0 * s * s * s;
}

HybridControlResult compute_hybrid_control(
  const Pose2D & robot, const Pose2D & goal, HybridMode mode, const HybridControlParams & params)
{
  const double dx = goal.position.x - robot.position.x;
  const double dy = goal.position.y - robot.position.y;
  HybridControlResult result;
  result.rho_m = std::hypot(dx, dy);
  result.alpha_rad = normalize_angle(std::atan2(dy, dx) - robot.yaw_rad);
  result.beta_rad = normalize_angle(goal.yaw_rad - robot.yaw_rad - result.alpha_rad);
  result.heading_error_rad = normalize_angle(goal.yaw_rad - robot.yaw_rad);
  result.mode = mode;

  if (result.mode == HybridMode::POSITION && result.rho_m <= params.rho_enter_m) {
    result.mode = HybridMode::HEADING;
  } else if ((result.mode == HybridMode::HEADING || result.mode == HybridMode::HOLD) &&
    result.rho_m >= params.rho_exit_m) {
    result.mode = HybridMode::POSITION;
  } else if (result.mode == HybridMode::HEADING &&
    std::abs(result.heading_error_rad) <= params.theta_hold_enter_rad) {
    result.mode = HybridMode::HOLD;
  } else if (result.mode == HybridMode::HOLD &&
    std::abs(result.heading_error_rad) >= params.theta_hold_exit_rad) {
    result.mode = HybridMode::HEADING;
  }

  if (result.mode == HybridMode::POSITION) {
    const double gate = heading_speed_gate(
      std::abs(result.alpha_rad), params.alpha_full_speed_rad, params.alpha_stop_rad);
    result.translation_gate = gate;
    const double base_speed = std::min(
      params.max_v_mps, std::max(params.v_min_approach_mps, params.k_rho * result.rho_m));
    result.linear_mps = base_speed * std::cos(result.alpha_rad) * gate;
    const double heading_weight = final_heading_weight(
      result.rho_m, params.final_heading_start_m, params.final_heading_full_m);
    result.angular_rad_s = params.k_alpha * result.alpha_rad +
      params.k_rho * sinc_unnormalized(result.alpha_rad) * std::cos(result.alpha_rad) *
      (result.alpha_rad + heading_weight * params.k_beta * result.beta_rad);
  } else if (result.mode == HybridMode::HEADING) {
    result.angular_rad_s = params.k_heading * result.heading_error_rad;
  }
  result.linear_mps = std::clamp(result.linear_mps, -params.max_v_mps, params.max_v_mps);
  result.angular_rad_s = std::clamp(result.angular_rad_s, -params.max_w_rad_s, params.max_w_rad_s);
  return result;
}

}  // namespace shoulder_align_controller
