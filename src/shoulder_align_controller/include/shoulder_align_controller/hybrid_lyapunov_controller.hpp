#pragma once

#include "shoulder_align_controller/geometry_utils.hpp"

namespace shoulder_align_controller
{

enum class HybridMode {POSITION, HEADING, HOLD};

struct Pose2D
{
  Vec2 position;
  double yaw_rad{};
};

struct HybridControlParams
{
  double k_rho{};
  double k_alpha{};
  double k_beta{};
  double k_heading{};
  double max_v_mps{};
  double max_w_rad_s{};
  double rho_enter_m{};
  double rho_exit_m{};
  double theta_hold_enter_rad{};
  double theta_hold_exit_rad{};
  double v_min_approach_mps{};
  double alpha_full_speed_rad{};
  double alpha_stop_rad{};
  double final_heading_start_m{};
  double final_heading_full_m{};
};

struct HybridControlResult
{
  double linear_mps{};
  double angular_rad_s{};
  HybridMode mode{HybridMode::POSITION};
  double rho_m{};
  double alpha_rad{};
  double beta_rad{};
  double heading_error_rad{};
  // 0 means translation is intentionally held while the bearing is too large.
  // Exposed for runtime diagnostics; it is not an additional filter.
  double translation_gate{};
};

double sinc_unnormalized(double angle_rad);
double heading_speed_gate(double abs_alpha_rad, double full_speed_rad, double stop_rad);
double final_heading_weight(double rho_m, double start_m, double full_m);
HybridControlResult compute_hybrid_control(
  const Pose2D & robot, const Pose2D & goal, HybridMode mode, const HybridControlParams & params);

}  // namespace shoulder_align_controller
