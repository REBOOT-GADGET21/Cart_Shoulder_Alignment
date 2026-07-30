#pragma once

#include <optional>

namespace shoulder_align_controller
{

inline constexpr double kPi = 3.14159265358979323846;

struct Vec2
{
  double x{};
  double y{};
};

struct LineIntersection
{
  Vec2 point;
  double first_t{};
  double second_t{};
};

double dot(const Vec2 & a, const Vec2 & b);
double cross(const Vec2 & a, const Vec2 & b);
double norm(const Vec2 & value);
Vec2 normalized(const Vec2 & value);
double normalize_angle(double angle_rad);
double angle_of(const Vec2 & direction);
Vec2 rotate_90_ccw(const Vec2 & direction);
Vec2 choose_robot_side_normal(const Vec2 & shoulder_direction, const Vec2 & midpoint, const Vec2 & robot_position);
std::optional<LineIntersection> intersect_lines(
  const Vec2 & first_origin, const Vec2 & first_direction,
  const Vec2 & second_origin, const Vec2 & second_direction, double epsilon);

}  // namespace shoulder_align_controller
