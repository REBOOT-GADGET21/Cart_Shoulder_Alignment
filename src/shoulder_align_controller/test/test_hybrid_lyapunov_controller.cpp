#include <gtest/gtest.h>

#include "shoulder_align_controller/hybrid_lyapunov_controller.hpp"

namespace sac = shoulder_align_controller;

namespace
{
sac::HybridControlParams params()
{
  return {1.1, 2.8, 0.6, 2.6, 0.3, 1.0, 0.02, 0.04, 3.0 * sac::kPi / 180.0,
    6.0 * sac::kPi / 180.0, 0.04, 20.0 * sac::kPi / 180.0, 75.0 * sac::kPi / 180.0, 0.45, 0.12};
}
}  // namespace

TEST(HybridLyapunov, StopsTranslationForLargeBearingError)
{
  const auto result = sac::compute_hybrid_control({{0.0, 0.0}, sac::kPi / 2.0}, {{1.0, 0.0}, 0.0}, sac::HybridMode::POSITION, params());
  EXPECT_NEAR(result.linear_mps, 0.0, 1e-12);
  EXPECT_NEAR(result.translation_gate, 0.0, 1e-12);
  EXPECT_LT(result.angular_rad_s, 0.0);
}

TEST(HybridLyapunov, PreservesForwardMotionForSmallBearingError)
{
  const auto result = sac::compute_hybrid_control(
    {{0.0, 0.0}, 0.0}, {{1.0, 0.01}, 0.0}, sac::HybridMode::POSITION, params());
  EXPECT_GT(result.translation_gate, 0.99);
  EXPECT_GT(result.linear_mps, 0.0);
}

TEST(HybridLyapunov, UsesPositionHeadingAndHoldHysteresis)
{
  auto p = params();
  const sac::Pose2D robot{{0.0, 0.0}, 0.0};
  EXPECT_EQ(sac::compute_hybrid_control(robot, {{0.02, 0.0}, 0.2}, sac::HybridMode::POSITION, p).mode, sac::HybridMode::HEADING);
  EXPECT_EQ(sac::compute_hybrid_control(robot, {{0.04, 0.0}, 0.0}, sac::HybridMode::HEADING, p).mode, sac::HybridMode::POSITION);
  EXPECT_EQ(sac::compute_hybrid_control(robot, {{0.01, 0.0}, 0.01}, sac::HybridMode::HEADING, p).mode, sac::HybridMode::HOLD);
  EXPECT_EQ(sac::compute_hybrid_control(robot, {{0.01, 0.0}, 0.11}, sac::HybridMode::HOLD, p).mode, sac::HybridMode::HEADING);
}

TEST(HybridLyapunov, WrapsAnglesAndSaturatesOutput)
{
  auto p = params(); p.max_v_mps = 0.05; p.max_w_rad_s = 0.07;
  const auto result = sac::compute_hybrid_control({{0.0, 0.0}, 179.0 * sac::kPi / 180.0},
    {{2.0, 0.0}, -179.0 * sac::kPi / 180.0}, sac::HybridMode::POSITION, p);
  EXPECT_LE(std::abs(result.linear_mps), p.max_v_mps);
  EXPECT_LE(std::abs(result.angular_rad_s), p.max_w_rad_s);
  EXPECT_NEAR(result.heading_error_rad, 2.0 * sac::kPi / 180.0, 1e-12);
}
