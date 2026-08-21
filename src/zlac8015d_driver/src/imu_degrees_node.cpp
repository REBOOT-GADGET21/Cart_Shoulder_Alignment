#include <algorithm>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuDegreesNode : public rclcpp::Node
{
public:
  ImuDegreesNode()
  : Node("imu_degrees")
  {
    publisher_ = create_publisher<geometry_msgs::msg::Vector3>("/imu/deg", 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr message) {on_imu(*message);});
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    const auto & q = message.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!std::isfinite(norm) || norm < 1.0e-12) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "유효하지 않은 IMU quaternion을 받았습니다");
      return;
    }

    // 입력 quaternion을 정규화해 수치 오차가 Euler 각도에 영향을 주지 않게 한다.
    const double x = q.x / norm;
    const double y = q.y / norm;
    const double z = q.z / norm;
    const double w = q.w / norm;
    constexpr double kRadToDeg = 180.0 / M_PI;

    geometry_msgs::msg::Vector3 degrees;
    // /imu/deg의 x, y, z는 각각 roll, pitch, yaw [deg]다.
    degrees.x = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * kRadToDeg;
    const double pitch_sin = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
    degrees.y = std::asin(pitch_sin) * kRadToDeg;
    degrees.z = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * kRadToDeg;
    publisher_->publish(degrees);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuDegreesNode>());
  rclcpp::shutdown();
  return 0;
}
