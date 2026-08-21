#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace hwt9053_imu
{
struct Quaternion
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

// Return a unit quaternion, or identity for a zero-length input.
inline Quaternion normalize(const Quaternion & q)
{
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  return n == 0.0 ? Quaternion{} : Quaternion{q.x/n, q.y/n, q.z/n, q.w/n};
}

// Return the conjugate of a quaternion.
inline Quaternion conjugate(const Quaternion & q)
{
  return {-q.x, -q.y, -q.z, q.w};
}

// Compose rotations represented by quaternions a and b.
inline Quaternion multiply(const Quaternion & a, const Quaternion & b)
{
  return {a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
          a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
          a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
          a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z};
}

// Compute inverse(reference) multiplied by current.
inline Quaternion relative_orientation(const Quaternion & reference, const Quaternion & current)
{
  return normalize(multiply(conjugate(normalize(reference)), normalize(current)));
}

// Convert ROS-convention roll, pitch, yaw values in radians to a quaternion.
inline Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
{
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  return normalize({sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                    cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy});
}

// Convert a quaternion to roll, pitch, yaw values in radians.
inline std::array<double, 3> quaternion_to_rpy(const Quaternion & input)
{
  const auto q = normalize(input);
  const double sinr = 2.0 * (q.w*q.x + q.y*q.z);
  const double cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y);
  const double sinp = 2.0 * (q.w*q.y - q.z*q.x);
  const double roll = std::atan2(sinr, cosr);
  const double pitch = std::copysign(M_PI / 2.0, sinp) * (std::abs(sinp) >= 1.0) +
    std::asin(sinp) * (std::abs(sinp) < 1.0);
  const double yaw = std::atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z));
  return {roll, pitch, yaw};
}

class RelativeOrientation
{
public:
  // Capture the first input as reference, then return the relative orientation.
  Quaternion apply(const Quaternion & absolute)
  {
  if (!has_reference_) {
    reference_ = normalize(absolute);
    has_reference_ = true;
  }
    return relative_orientation(reference_, absolute);
  }
  // Discard the reference so the next sample establishes a new one.
  void reset()
  {
    has_reference_ = false;
  }

  // Report whether an orientation reference has been captured.
  bool has_reference() const
  {
    return has_reference_;
  }
private:
  Quaternion reference_{};
  bool has_reference_{false};
};

// Calculate the standard Modbus RTU CRC-16 value.
inline uint16_t modbus_crc(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : crc >> 1;
    }
  }
  return crc;
}

// Build a Modbus function 0x03 request, including low-byte-first CRC.
inline std::vector<uint8_t> modbus_read_request(uint8_t id, uint16_t address, uint16_t count)
{
  std::vector<uint8_t> request{id, 0x03, static_cast<uint8_t>(address >> 8),
    static_cast<uint8_t>(address), static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count)};
  const uint16_t crc = modbus_crc(request.data(), request.size());
  request.push_back(static_cast<uint8_t>(crc));
  request.push_back(static_cast<uint8_t>(crc >> 8));
  return request;
}
}  // namespace hwt9053_imu
