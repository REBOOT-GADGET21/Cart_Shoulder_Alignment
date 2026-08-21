#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

class ImuStaticTfNode : public rclcpp::Node
{
public:
  ImuStaticTfNode()
  : Node("imu_static_tf")
  {
    const std::string base_frame = declare_parameter<std::string>("base_frame", "base_link");
    const std::string imu_frame = declare_parameter<std::string>("imu_frame", "imu_link");
    const double x_m = declare_parameter<double>("x", 0.0);
    const double y_m = declare_parameter<double>("y", 0.0);
    const double z_m = declare_parameter<double>("z", 0.0);
    const double roll_rad = declare_parameter<double>("roll", 0.0);
    const double pitch_rad = declare_parameter<double>("pitch", 0.0);
    const double yaw_rad = declare_parameter<double>("yaw", 0.0);

    // IMU는 차체에 고정돼 있으므로 장착 위치와 방향을 /tf_static에 한 번만 발행한다.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = base_frame;
    transform.child_frame_id = imu_frame;
    transform.transform.translation.x = x_m;
    transform.transform.translation.y = y_m;
    transform.transform.translation.z = z_m;
    set_quaternion_from_rpy(roll_rad, pitch_rad, yaw_rad, transform);

    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    broadcaster_->sendTransform(transform);
    RCLCPP_INFO(get_logger(), "정적 TF 발행: %s -> %s (x=%.3f, y=%.3f, z=%.3f m)",
      base_frame.c_str(), imu_frame.c_str(), x_m, y_m, z_m);
  }

private:
  static void set_quaternion_from_rpy(
    const double roll_rad, const double pitch_rad, const double yaw_rad,
    geometry_msgs::msg::TransformStamped & transform)
  {
    // ROS 표준 roll-pitch-yaw 순서로 고정 장착 회전을 quaternion으로 바꾼다.
    const double half_roll = roll_rad * 0.5;
    const double half_pitch = pitch_rad * 0.5;
    const double half_yaw = yaw_rad * 0.5;
    const double cr = std::cos(half_roll);
    const double sr = std::sin(half_roll);
    const double cp = std::cos(half_pitch);
    const double sp = std::sin(half_pitch);
    const double cy = std::cos(half_yaw);
    const double sy = std::sin(half_yaw);
    transform.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    transform.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    transform.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    transform.transform.rotation.z = cr * cp * sy - sr * sp * cy;
  }

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuStaticTfNode>());
  rclcpp::shutdown();
  return 0;
}
