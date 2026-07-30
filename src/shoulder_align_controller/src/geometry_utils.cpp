#include "shoulder_align_controller/geometry_utils.hpp"

#include <cmath>
#include <stdexcept>

namespace shoulder_align_controller
{
double dot(const Vec2 & a, const Vec2 & b) {return a.x * b.x + a.y * b.y;}
double cross(const Vec2 & a, const Vec2 & b) {return a.x * b.y - a.y * b.x;}
double norm(const Vec2 & value) {return std::hypot(value.x, value.y);}

Vec2 normalized(const Vec2 & value)
{
  const double length = norm(value);
  if (length < 1e-12) {
    throw std::invalid_argument("Cannot normalize a zero-length vector");
  }
  return {value.x / length, value.y / length};
}

double normalize_angle(double angle_rad)
{
  while (angle_rad > kPi) {angle_rad -= 2.0 * kPi;}
  while (angle_rad <= -kPi) {angle_rad += 2.0 * kPi;}
  return angle_rad;
}

double angle_of(const Vec2 & direction) {return std::atan2(direction.y, direction.x);}
Vec2 rotate_90_ccw(const Vec2 & direction) {return {-direction.y, direction.x};}

Vec2 choose_robot_side_normal(
  const Vec2 & shoulder_direction, const Vec2 & midpoint, const Vec2 & robot_position)
{
  Vec2 normal = normalized(rotate_90_ccw(shoulder_direction));
  const Vec2 robot_from_midpoint{robot_position.x - midpoint.x, robot_position.y - midpoint.y};
  if (dot(normal, robot_from_midpoint) < 0.0) {
    normal.x = -normal.x;
    normal.y = -normal.y;
  }
  return normal;
}

std::optional<LineIntersection> intersect_lines(
  const Vec2 & first_origin, const Vec2 & first_direction,
  const Vec2 & second_origin, const Vec2 & second_direction, const double epsilon)
{
  const double denominator = cross(first_direction, second_direction);
  if (std::abs(denominator) < epsilon) {
    return std::nullopt;
  }
  const Vec2 delta{second_origin.x - first_origin.x, second_origin.y - first_origin.y};
  const double first_t = cross(delta, second_direction) / denominator;
  const double second_t = cross(delta, first_direction) / denominator;
  return LineIntersection{{
    first_origin.x + first_t * first_direction.x,
    first_origin.y + first_t * first_direction.y}, first_t, second_t};
}
}  // namespace shoulder_align_controller
