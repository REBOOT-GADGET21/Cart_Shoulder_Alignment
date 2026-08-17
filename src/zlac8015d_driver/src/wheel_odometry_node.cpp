#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odometry")
  {
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.0);
    wheel_track_m_ = declare_parameter<double>("wheel_track_m", 0.0);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    if (wheel_radius_m_ <= 0.0 || wheel_track_m_ <= 0.0) {
      throw std::invalid_argument("wheel_radius_m과 wheel_track_m은 실제 측정값으로 설정해야 합니다");
    }
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/zlac8015d/wheel_joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {on_wheel_state(*message);});
  }

private:
  void on_wheel_state(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.size() != 2 || message.position.size() != 2 ||
      message.name[0] != "left_wheel_joint" || message.name[1] != "right_wheel_joint")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "wheel_joint_states 형식이 예상과 다릅니다");
      return;
    }
    rclcpp::Time stamp(message.header.stamp);
    if (message.header.stamp.nanosec == 0 && message.header.stamp.sec == 0) {
      stamp = now();
    }
    if (!received_baseline_) {
      previous_left_angle_rad_ = message.position[0];
      previous_right_angle_rad_ = message.position[1];
      previous_stamp_ = stamp;
      received_baseline_ = true;
      return;
    }
    const double delta_left_m = (message.position[0] - previous_left_angle_rad_) * wheel_radius_m_;
    const double delta_right_m = (message.position[1] - previous_right_angle_rad_) * wheel_radius_m_;
    const double delta_s_m = (delta_left_m + delta_right_m) * 0.5;
    const double delta_yaw_rad = (delta_right_m - delta_left_m) / wheel_track_m_;
    const double yaw_mid_rad = yaw_rad_ + delta_yaw_rad * 0.5;
    x_m_ += delta_s_m * std::cos(yaw_mid_rad);
    y_m_ += delta_s_m * std::sin(yaw_mid_rad);
    yaw_rad_ = std::atan2(std::sin(yaw_rad_ + delta_yaw_rad), std::cos(yaw_rad_ + delta_yaw_rad));
    const double dt_sec = (stamp - previous_stamp_).seconds();
    publish_odometry(stamp, dt_sec > 0.0 ? delta_s_m / dt_sec : 0.0, dt_sec > 0.0 ? delta_yaw_rad / dt_sec : 0.0);
    previous_left_angle_rad_ = message.position[0];
    previous_right_angle_rad_ = message.position[1];
    previous_stamp_ = stamp;
  }

  void publish_odometry(const rclcpp::Time & stamp, const double linear_x_mps, const double angular_z_rad_s)
  {
    const double half_yaw = yaw_rad_ * 0.5;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_m_;
    odom.pose.pose.position.y = y_m_;
    odom.pose.pose.orientation.z = std::sin(half_yaw);
    odom.pose.pose.orientation.w = std::cos(half_yaw);
    odom.twist.twist.linear.x = linear_x_mps;
    odom.twist.twist.angular.z = angular_z_rad_s;
    odom_publisher_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_m_;
    transform.transform.translation.y = y_m_;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  double wheel_radius_m_{}, wheel_track_m_{};
  std::string odom_frame_, base_frame_;
  bool received_baseline_{false};
  double previous_left_angle_rad_{}, previous_right_angle_rad_{}, x_m_{}, y_m_{}, yaw_rad_{};
  rclcpp::Time previous_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
