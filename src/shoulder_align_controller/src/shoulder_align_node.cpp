// 통합 노드

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "shoulder_align_controller/geometry_utils.hpp"
#include "shoulder_align_controller/hybrid_lyapunov_controller.hpp"

namespace sac = shoulder_align_controller;
using namespace std::chrono_literals;

class ShoulderAlignNode : public rclcpp::Node
{
public:
  ShoulderAlignNode() : Node("shoulder_align_node")
  {
    stop_distance_m_ = declare_parameter<double>("stop_distance_m", 0.9);
    platform_length_m_ = declare_parameter<double>("platform_length_m", 0.5);
    params_.rho_enter_m = declare_parameter<double>("pos_tolerance_m", 0.03);
    params_.theta_hold_enter_rad = declare_parameter<double>("angle_tolerance_rad", 0.02);
    params_.k_rho = declare_parameter<double>("k_v", 0.35);
    params_.k_alpha = declare_parameter<double>("hybrid_alpha_gain", 0.8);
    params_.k_beta = declare_parameter<double>("hybrid_beta_gain", 0.6);
    params_.k_heading = declare_parameter<double>("k_w", 0.8);
    params_.max_v_mps = declare_parameter<double>("max_v_mps", 0.20);
    params_.max_w_rad_s = declare_parameter<double>("max_w_rad_s", 0.40);
    params_.rho_exit_m = params_.rho_enter_m + declare_parameter<double>("pos_hysteresis_m", 0.01);
    params_.theta_hold_exit_rad = params_.theta_hold_enter_rad + declare_parameter<double>("angle_hysteresis_rad", 0.01);
    hold_confirm_s_ = declare_parameter<double>("debounce_s", 0.20);
    params_.v_min_approach_mps = declare_parameter<double>("hybrid_min_approach_speed_mps", 0.04);
    params_.alpha_full_speed_rad = declare_parameter<double>("hybrid_alpha_full_speed_rad", 0.3490658504);
    params_.alpha_stop_rad = declare_parameter<double>("hybrid_alpha_stop_rad", 1.3089969390);
    params_.final_heading_start_m = declare_parameter<double>("hybrid_final_heading_start_m", 0.45);
    params_.final_heading_full_m = declare_parameter<double>("hybrid_final_heading_full_m", 0.12);
    target_position_tau_s_ = declare_parameter<double>("hybrid_target_position_tau_s", 0.18);
    target_heading_tau_s_ = declare_parameter<double>("hybrid_target_heading_tau_s", 0.30);
    measurement_timeout_s_ = declare_parameter<double>("measurement_timeout_s", 0.50);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.25);

    shoulder_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>("/shoulder_line", 10,
      [this](geometry_msgs::msg::PoseArray::SharedPtr message) {on_landmarks(*message);});
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 20,
      [this](nav_msgs::msg::Odometry::SharedPtr message) {on_odom(*message);});
    // Automatic alignment must not masquerade as manual /cmd_vel input.
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/alignment_cmd", 10);
    angle_error_publisher_ = create_publisher<std_msgs::msg::Float32>("/alignment/error_angle_deg", 10);
    aligned_publisher_ = create_publisher<std_msgs::msg::Bool>("/alignment/aligned", 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>("/shoulder_align/state", 10);
    debug_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("/shoulder_align/debug_markers", 10);
    timer_ = create_wall_timer(50ms, [this]() {control_step();});
  }

private:
  struct Landmarks {sac::Vec2 left; sac::Vec2 right; sac::Vec2 head; sac::Vec2 pelvis; std::string frame_id;};
  struct RobotPose {sac::Pose2D pose; std::string frame_id;};
  static bool finite(const sac::Vec2 & p) {return std::isfinite(p.x) && std::isfinite(p.y);}

  void on_landmarks(const geometry_msgs::msg::PoseArray & message)
  {
    // pose[0..3]: left shoulder, right shoulder, head centre, pelvis centre; odom frame and metres.
    if (message.poses.size() < 4) {landmarks_.reset(); return;}
    Landmarks value{{message.poses[0].position.x, message.poses[0].position.y},
      {message.poses[1].position.x, message.poses[1].position.y},
      {message.poses[2].position.x, message.poses[2].position.y},
      {message.poses[3].position.x, message.poses[3].position.y}, message.header.frame_id};
    if (!finite(value.left) || !finite(value.right) || !finite(value.head) || !finite(value.pelvis) ||
      sac::norm({value.right.x - value.left.x, value.right.y - value.left.y}) < 0.05) {landmarks_.reset(); return;}
    landmarks_ = value;
    last_landmark_time_ = now();
  }

  void on_odom(const nav_msgs::msg::Odometry & message)
  {
    const auto & q = message.pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    robot_ = RobotPose{{{message.pose.pose.position.x, message.pose.pose.position.y}, yaw}, message.header.frame_id};
    last_odom_time_ = now();
  }

  std::optional<sac::Pose2D> raw_goal() const
  {
    if (!landmarks_) {return std::nullopt;}
    const sac::Vec2 mid{(landmarks_->left.x + landmarks_->right.x) / 2.0,
      (landmarks_->left.y + landmarks_->right.y) / 2.0};
    const sac::Vec2 shoulder_line = sac::normalized({landmarks_->right.x - landmarks_->left.x, landmarks_->right.y - landmarks_->left.y});
    sac::Vec2 head_normal = sac::rotate_90_ccw(shoulder_line);
    const sac::Vec2 body_axis{landmarks_->head.x - landmarks_->pelvis.x, landmarks_->head.y - landmarks_->pelvis.y};
    if (sac::dot(head_normal, body_axis) < 0.0) {head_normal = {-head_normal.x, -head_normal.y};}
    const sac::Vec2 front{mid.x - landmarks_->head.x, mid.y - landmarks_->head.y};
    if (sac::norm(front) < 1e-6) {return std::nullopt;}
    const double distance_m = stop_distance_m_ + platform_length_m_;
    return sac::Pose2D{{mid.x + distance_m * head_normal.x, mid.y + distance_m * head_normal.y}, sac::angle_of(front)};
  }

  sac::Pose2D filter_goal(const sac::Pose2D & raw)
  {
    const auto current = now();
    if (!filtered_goal_) {filtered_goal_ = raw;} else {
      const double dt = std::max(0.0, (current - last_goal_filter_time_).seconds());
      const double position_lambda = dt / (target_position_tau_s_ + dt);
      const double heading_lambda = dt / (target_heading_tau_s_ + dt);
      filtered_goal_->position.x += position_lambda * (raw.position.x - filtered_goal_->position.x);
      filtered_goal_->position.y += position_lambda * (raw.position.y - filtered_goal_->position.y);
      filtered_goal_->yaw_rad = sac::normalize_angle(filtered_goal_->yaw_rad + heading_lambda * sac::normalize_angle(raw.yaw_rad - filtered_goal_->yaw_rad));
    }
    last_goal_filter_time_ = current;
    return *filtered_goal_;
  }

  void control_step()
  {
    if (!landmarks_ || !robot_ || (now() - last_landmark_time_).seconds() > measurement_timeout_s_ ||
      (now() - last_odom_time_).seconds() > odom_timeout_s_ ||
      (!landmarks_->frame_id.empty() && !robot_->frame_id.empty() && landmarks_->frame_id != robot_->frame_id)) {
      publish_stop(); publish_state(); publish_aligned_status(); return;
    }
    const auto raw = raw_goal();
    if (!raw) {publish_stop(); return;}
    const auto goal = filter_goal(*raw);
    const auto previous = mode_;
    auto result = sac::compute_hybrid_control(robot_->pose, goal, mode_, params_);
    if (mode_ == sac::HybridMode::HEADING && result.mode == sac::HybridMode::HOLD) {
      if (hold_candidate_since_.nanoseconds() == 0) {hold_candidate_since_ = now();}
      if ((now() - hold_candidate_since_).seconds() < hold_confirm_s_) {
        result = sac::compute_hybrid_control(robot_->pose, goal, sac::HybridMode::HEADING, params_);
        result.mode = sac::HybridMode::HEADING;
      }
    } else if (result.mode != sac::HybridMode::HOLD) {hold_candidate_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());}
    mode_ = result.mode;
    if (mode_ != previous) {
      RCLCPP_INFO(get_logger(), "Hybrid state %s -> %s (rho=%.3f m, heading=%.2f deg)", state_name(previous).c_str(), state_name(mode_).c_str(), result.rho_m, result.heading_error_rad * 180.0 / sac::kPi);
    }
    publish_velocity(result.linear_mps, result.angular_rad_s);
    publish_angle_error_deg(result.heading_error_rad);
    publish_debug(goal, result);
    publish_state(); publish_aligned_status();
  }

  static std::string state_name(const sac::HybridMode state)
  {
    if (state == sac::HybridMode::POSITION) {return "POSITION";}
    if (state == sac::HybridMode::HEADING) {return "HEADING";}
    return "HOLD";
  }
  void publish_angle_error_deg(const double e) {std_msgs::msg::Float32 m; m.data = static_cast<float>(e * 180.0 / sac::kPi); angle_error_publisher_->publish(m);}
  void publish_aligned_status() {std_msgs::msg::Bool m; m.data = mode_ == sac::HybridMode::HOLD; aligned_publisher_->publish(m);}
  void publish_state() {std_msgs::msg::String m; m.data = state_name(mode_); state_publisher_->publish(m);}
  void publish_debug(const sac::Pose2D & goal, const sac::HybridControlResult & result)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker target;
    target.header.frame_id = landmarks_->frame_id; target.header.stamp = now(); target.ns = "shoulder_alignment"; target.id = 0;
    target.type = visualization_msgs::msg::Marker::ARROW; target.action = visualization_msgs::msg::Marker::ADD;
    target.pose.position.x = goal.position.x; target.pose.position.y = goal.position.y;
    target.pose.orientation.z = std::sin(goal.yaw_rad / 2.0); target.pose.orientation.w = std::cos(goal.yaw_rad / 2.0);
    target.scale.x = 0.25; target.scale.y = target.scale.z = 0.06; target.color.g = target.color.a = 1.0; markers.markers.push_back(target);
    debug_publisher_->publish(markers);
    (void)result;
  }
  void publish_velocity(const double v, const double w) {geometry_msgs::msg::Twist m; m.linear.x = v; m.angular.z = w; cmd_publisher_->publish(m);}
  void publish_stop() {publish_velocity(0.0, 0.0);}

  double stop_distance_m_{}, platform_length_m_{}, hold_confirm_s_{}, target_position_tau_s_{}, target_heading_tau_s_{}, measurement_timeout_s_{}, odom_timeout_s_{};
  sac::HybridControlParams params_;
  sac::HybridMode mode_{sac::HybridMode::POSITION};
  std::optional<Landmarks> landmarks_; std::optional<RobotPose> robot_; std::optional<sac::Pose2D> filtered_goal_;
  rclcpp::Time last_landmark_time_{0, 0, RCL_ROS_TIME}, last_odom_time_{0, 0, RCL_ROS_TIME}, last_goal_filter_time_{0, 0, RCL_ROS_TIME}, hold_candidate_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr shoulder_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aligned_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ShoulderAlignNode>()); rclcpp::shutdown(); return 0;
}
