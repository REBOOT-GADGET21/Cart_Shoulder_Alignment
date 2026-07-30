#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "shoulder_align_controller/geometry_utils.hpp"

namespace sac = shoulder_align_controller;
using namespace std::chrono_literals;

class ShoulderAlignNode : public rclcpp::Node
{
public:
  ShoulderAlignNode() : Node("shoulder_align_node")
  {
    stop_distance_m_ = declare_parameter<double>("stop_distance_m", 0.9);
    platform_length_m_ = declare_parameter<double>("platform_length_m", 0.5);
    pos_tolerance_m_ = declare_parameter<double>("pos_tolerance_m", 0.03);
    angle_tolerance_rad_ = declare_parameter<double>("angle_tolerance_rad", 0.035);
    max_valid_t_m_ = declare_parameter<double>("max_valid_t_m", 8.0);
    parallel_epsilon_ = declare_parameter<double>("parallel_epsilon", 1e-5);
    k_v_ = declare_parameter<double>("k_v", 0.35);
    k_w_ = declare_parameter<double>("k_w", 0.8);
    max_v_mps_ = declare_parameter<double>("max_v_mps", 0.20);
    max_w_rad_s_ = declare_parameter<double>("max_w_rad_s", 0.40);
    debounce_s_ = declare_parameter<double>("debounce_s", 0.20);
    pos_hysteresis_m_ = declare_parameter<double>("pos_hysteresis_m", 0.01);
    angle_hysteresis_rad_ = declare_parameter<double>("angle_hysteresis_rad", 0.01);

    shoulder_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/shoulder_line", 10, [this](geometry_msgs::msg::PoseArray::SharedPtr message) {
        on_shoulders(*message);});
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 20, [this](nav_msgs::msg::Odometry::SharedPtr message) {on_odom(*message);});
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(50ms, [this]() {control_step();});
    state_entered_ = now();
    RCLCPP_INFO(get_logger(), "ShoulderAlignNode ready: waiting for /shoulder_line and /odom");
  }

private:
  enum class State {TRANSLATE_TO_R, ROTATE_TO_NORMAL, TRANSLATE_FINAL, DONE, FALLBACK_ROTATE_TO_R};

  struct RobotPose {sac::Vec2 position; double yaw_rad{}; std::string frame_id;};
  struct ShoulderLine {sac::Vec2 left; sac::Vec2 right; std::string frame_id;};

  void on_shoulders(const geometry_msgs::msg::PoseArray & message)
  {
    if (message.poses.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "/shoulder_line needs pose[0] and pose[1]");
      return;
    }
    shoulders_ = ShoulderLine{{message.poses[0].position.x, message.poses[0].position.y},
      {message.poses[1].position.x, message.poses[1].position.y}, message.header.frame_id};
    // A static perception publisher repeats the same line continuously.  Once
    // DONE is reached, never treat those repeats as a new alignment request.
    // A later explicit enable/reset topic will be responsible for re-arming.
  }

  void on_odom(const nav_msgs::msg::Odometry & message)
  {
    const auto & q = message.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    robot_ = RobotPose{{message.pose.pose.position.x, message.pose.pose.position.y}, yaw, message.header.frame_id};
  }

  void control_step()
  {
    if (!shoulders_ || !robot_) {publish_stop(); return;}
    if (!shoulders_->frame_id.empty() && !robot_->frame_id.empty() && shoulders_->frame_id != robot_->frame_id) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Frame mismatch: shoulder_line and odom must use the same frame");
      publish_stop();
      return;
    }
    const sac::Vec2 shoulder_vector{shoulders_->right.x - shoulders_->left.x, shoulders_->right.y - shoulders_->left.y};
    if (sac::norm(shoulder_vector) < 0.05) {
      publish_stop(); 
      return;
    }
    const sac::Vec2 midpoint{
      (shoulders_->left.x + shoulders_->right.x) / 2.0,
      (shoulders_->left.y + shoulders_->right.y) / 2.0
    };
    const sac::Vec2 shoulder_direction = sac::normalized(shoulder_vector);
    const sac::Vec2 normal = sac::choose_robot_side_normal(shoulder_direction, midpoint, robot_->position);
    const sac::Vec2 heading{std::cos(robot_->yaw_rad), std::sin(robot_->yaw_rad)};

    if (state_ == State::DONE) {
      publish_stop(); 
      return;
    }
    if (state_ == State::TRANSLATE_TO_R) {
      translate_to_intersection(midpoint, normal, heading); 
      return;
    }
    if (state_ == State::FALLBACK_ROTATE_TO_R) {
      fallback_rotate_to_normal_line(midpoint, normal); 
      return;
    }
    if (state_ == State::ROTATE_TO_NORMAL) {
      rotate_to_angle(sac::angle_of({-normal.x, -normal.y}), State::TRANSLATE_FINAL);
      return;
    }
    translate_final(midpoint, normal);
  }

  void translate_to_intersection(const sac::Vec2 & midpoint, const sac::Vec2 & normal, const sac::Vec2 & heading)
  {
    const auto intersection = sac::intersect_lines(robot_->position, heading, midpoint, normal, parallel_epsilon_);
    if (!intersection || std::abs(intersection->first_t) > max_valid_t_m_) {
      transition_to(State::FALLBACK_ROTATE_TO_R);
      return;
    }
    if (std::abs(intersection->first_t) <= pos_tolerance_m_ + pos_hysteresis_m_ && debounced()) {
      transition_to(State::ROTATE_TO_NORMAL);
      return;
    }
    publish_velocity(std::clamp(k_v_ * intersection->first_t, -max_v_mps_, max_v_mps_), 0.0);
  }

  void fallback_rotate_to_normal_line(const sac::Vec2 & midpoint, const sac::Vec2 & normal)
  {
    // A parallel, offset pair has no R. Rotate toward the closest point on the
    // normal line, then STATE 1 can translate to the line without strafing.
    const double along_normal = sac::dot({robot_->position.x - midpoint.x, robot_->position.y - midpoint.y}, normal);
    const sac::Vec2 closest{midpoint.x + along_normal * normal.x, midpoint.y + along_normal * normal.y};
    const sac::Vec2 toward_line{closest.x - robot_->position.x, closest.y - robot_->position.y};
    if (sac::norm(toward_line) <= pos_tolerance_m_ + pos_hysteresis_m_) {
      transition_to(State::ROTATE_TO_NORMAL);
      return;
    }
    rotate_to_angle(sac::angle_of(toward_line), State::TRANSLATE_TO_R);
  }

  void rotate_to_angle(const double target_yaw, const State next_state)
  {
    const double error = sac::normalize_angle(target_yaw - robot_->yaw_rad);
    if (std::abs(error) <= angle_tolerance_rad_ + angle_hysteresis_rad_ && debounced()) {
      transition_to(next_state);
      return;
    }
    publish_velocity(0.0, std::clamp(k_w_ * error, -max_w_rad_s_, max_w_rad_s_));
  }

  void translate_final(const sac::Vec2 & midpoint, const sac::Vec2 & normal)
  {
    const double pivot_distance_m = sac::dot(
      {robot_->position.x - midpoint.x, robot_->position.y - midpoint.y}, normal);
    const double desired_distance_m = stop_distance_m_ + platform_length_m_;
    if (pivot_distance_m <= desired_distance_m + pos_tolerance_m_ + pos_hysteresis_m_ && debounced()) {
      transition_to(State::DONE);
      publish_stop();
      return;
    }
    publish_velocity(std::clamp(k_v_ * (pivot_distance_m - desired_distance_m), 0.0, max_v_mps_), 0.0);
  }

  bool debounced() const {return (now() - state_entered_).seconds() >= debounce_s_;}
  void transition_to(const State next) {state_ = next; state_entered_ = now();}
  void publish_velocity(const double v, const double w)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = v;
    command.angular.z = w;
    cmd_publisher_->publish(command);
  }
  void publish_stop() {publish_velocity(0.0, 0.0);}

  double stop_distance_m_{}, platform_length_m_{}, pos_tolerance_m_{}, angle_tolerance_rad_{};
  double max_valid_t_m_{}, parallel_epsilon_{}, k_v_{}, k_w_{}, max_v_mps_{}, max_w_rad_s_{};
  double debounce_s_{}, pos_hysteresis_m_{}, angle_hysteresis_rad_{};
  State state_{State::TRANSLATE_TO_R};
  rclcpp::Time state_entered_;
  std::optional<RobotPose> robot_;
  std::optional<ShoulderLine> shoulders_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr shoulder_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShoulderAlignNode>());
  rclcpp::shutdown();
  return 0;
}
