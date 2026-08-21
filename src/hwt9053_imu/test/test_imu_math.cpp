#include <gtest/gtest.h>
#include "hwt9053_imu/imu_math.hpp"

using namespace hwt9053_imu;

TEST(ModbusCrc, StandardRequest) {
  const auto request = modbus_read_request(0x50, 0x0034, 3);
  ASSERT_EQ(request.size(), 8u);
  EXPECT_EQ(modbus_crc(request.data(), 6), 0x8449);  // sent low byte first: 0x49, 0x84
  EXPECT_EQ(request[6], 0x49); EXPECT_EQ(request[7], 0x84);
}

TEST(Quaternion, RelativeOrientationUsesQuaternionProduct) {
  const auto reference = rpy_to_quaternion(0.3, -0.2, 2.9);
  const auto current = rpy_to_quaternion(0.3, -0.2, -2.9);
  const auto q = relative_orientation(reference, current);
  const auto identity = relative_orientation(reference, reference);
  EXPECT_NEAR(identity.x, 0.0, 1e-12); EXPECT_NEAR(identity.y, 0.0, 1e-12);
  EXPECT_NEAR(identity.z, 0.0, 1e-12); EXPECT_NEAR(identity.w, 1.0, 1e-12);
  EXPECT_NEAR(std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w), 1.0, 1e-12);
}

TEST(RelativeReference, CapturesThenResets) {
  RelativeOrientation filter;
  const auto first = rpy_to_quaternion(0.1, -0.2, 1.0);
  const auto second = rpy_to_quaternion(0.1, -0.2, 1.4);
  auto out = filter.apply(first); EXPECT_NEAR(out.w, 1.0, 1e-12); EXPECT_TRUE(filter.has_reference());
  out = filter.apply(second); EXPECT_LT(out.w, 1.0);
  filter.reset(); EXPECT_FALSE(filter.has_reference());
  out = filter.apply(second); EXPECT_NEAR(out.x, 0.0, 1e-12); EXPECT_NEAR(out.y, 0.0, 1e-12); EXPECT_NEAR(out.z, 0.0, 1e-12); EXPECT_NEAR(out.w, 1.0, 1e-12);
}
