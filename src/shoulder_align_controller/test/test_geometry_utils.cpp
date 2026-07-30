#include <gtest/gtest.h>

#include "shoulder_align_controller/geometry_utils.hpp"

namespace sac = shoulder_align_controller;

TEST(GeometryUtils, IntersectsHeadingAndNormalLines)
{
  const auto intersection = sac::intersect_lines({0.0, 0.0}, {1.0, 0.0}, {2.0, 3.0}, {0.0, 1.0}, 1e-6);
  ASSERT_TRUE(intersection.has_value());
  EXPECT_DOUBLE_EQ(intersection->point.x, 2.0);
  EXPECT_DOUBLE_EQ(intersection->point.y, 0.0);
  EXPECT_DOUBLE_EQ(intersection->first_t, 2.0);
}

TEST(GeometryUtils, ParallelLinesUseNoIntersectionFallback)
{
  const auto intersection = sac::intersect_lines({0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, 1e-6);
  EXPECT_FALSE(intersection.has_value());
}

TEST(GeometryUtils, NormalizesAngleToShortestRotation)
{
  EXPECT_NEAR(sac::normalize_angle(3.5), 3.5 - 2.0 * sac::kPi, 1e-12);
  EXPECT_NEAR(sac::normalize_angle(-3.5), -3.5 + 2.0 * sac::kPi, 1e-12);
}

TEST(GeometryUtils, ChoosesNormalOnRobotSide)
{
  const sac::Vec2 normal = sac::choose_robot_side_normal({1.0, 0.0}, {0.0, 0.0}, {0.0, -2.0});
  EXPECT_DOUBLE_EQ(normal.x, 0.0);
  EXPECT_DOUBLE_EQ(normal.y, -1.0);
}
