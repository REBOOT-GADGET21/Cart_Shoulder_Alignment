// Snapshot-based shoulder alignment for the real cart.
//
// The cart stays stopped while it measures a person for a short interval.
// A robust median snapshot is then converted to one fixed target pose. Drive
// control never consumes subsequent YOLO landmarks, so frame-to-frame pose
// jitter cannot reverse the target or the steering command during motion.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include "snapshot_math.hpp"
#include "advance_math.hpp"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
struct Vec2 { double x{}; double y{}; };
struct RobotPose { Vec2 position; double yaw_rad{}; std::string frame_id; };
struct LandmarkPair { Vec2 left; Vec2 right; std::string frame_id; Vec2 body_axis; };
struct SnapshotGoal { Vec2 shoulder_mid; Vec2 position; double yaw_rad{}; };

double norm(const Vec2 & value) { return std::hypot(value.x, value.y); }
double dot(const Vec2 & first, const Vec2 & second) { return first.x * second.x + first.y * second.y; }
double normalize_angle(double angle)
{
  while (angle > kPi) { angle -= 2.0 * kPi; }
  while (angle <= -kPi) { angle += 2.0 * kPi; }
  return angle;
}
Vec2 normalized(const Vec2 & value)
{
  const double length = norm(value);
  return {value.x / length, value.y / length};
}
double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  return values.size() % 2 == 0 ? (values[middle - 1] + values[middle]) * 0.5 : values[middle];
}
}  // namespace

class ShoulderAlignNode : public rclcpp::Node
{
public:
  ShoulderAlignNode() : Node("shoulder_align_node")
  {
    front_clearance_m_ = declare_parameter<double>("stop_distance_m", 0.70);
    pivot_to_front_m_ = declare_parameter<double>("platform_length_m", 0.50);
    capture_duration_s_ = declare_parameter<double>("snapshot_capture_duration_s", 5.0);
    capture_min_samples_ = declare_parameter<int>("snapshot_min_samples", 30);
    landmark_timeout_s_ = declare_parameter<double>("snapshot_landmark_timeout_s", 0.50);
    min_shoulder_width_m_ = declare_parameter<double>("snapshot_min_shoulder_width_m", 0.10);
    max_shoulder_width_m_ = declare_parameter<double>("snapshot_max_shoulder_width_m", 0.90);
    position_enter_m_ = declare_parameter<double>("pos_tolerance_m", 0.02);
    position_exit_m_ = position_enter_m_ + declare_parameter<double>("pos_hysteresis_m", 0.03);
    heading_enter_rad_ = declare_parameter<double>("angle_tolerance_rad", 0.035);
    heading_exit_rad_ = heading_enter_rad_ + declare_parameter<double>("angle_hysteresis_rad", 0.035);
    hold_confirm_s_ = declare_parameter<double>("debounce_s", 0.50);
    position_kp_ = declare_parameter<double>("k_v", 0.35);
    heading_kp_ = declare_parameter<double>("k_w", 0.80);
    alpha_gain_ = declare_parameter<double>("lyapunov_alpha_gain", 0.8);
    beta_gain_ = declare_parameter<double>("lyapunov_beta_gain", 0.6);
    shoulder_width_m_ = declare_parameter<double>("body_shoulder_width_m", 0.17);
    eye_pelvis_m_ = declare_parameter<double>("body_eye_pelvis_m", 0.56);
    shoulder_pelvis_m_ = declare_parameter<double>("body_shoulder_pelvis_m", 0.31);
    length_tolerance_m_ = declare_parameter<double>("body_length_tolerance_m", 0.08);
    max_speed_mps_ = declare_parameter<double>("max_v_mps", 0.30);
    max_yaw_rate_rad_s_ = declare_parameter<double>("max_w_rad_s", 0.10);
    // A zero default preserves the former behaviour: stop after final heading.
    post_hold_advance_distance_m_ = declare_parameter<double>("post_hold_advance_distance_m", 0.0);
    odom_timeout_s_ = declare_parameter<double>("odom_timeout_s", 0.25);
    if (front_clearance_m_ < 0.0 || pivot_to_front_m_ <= 0.0 || capture_duration_s_ <= 0.0 ||
      capture_min_samples_ <= 0 || landmark_timeout_s_ <= 0.0 || min_shoulder_width_m_ <= 0.0 ||
      max_shoulder_width_m_ <= min_shoulder_width_m_ || position_enter_m_ <= 0.0 ||
      position_exit_m_ < position_enter_m_ || heading_enter_rad_ <= 0.0 ||
      heading_exit_rad_ < heading_enter_rad_ || position_kp_ <= 0.0 || heading_kp_ <= 0.0 ||
      alpha_gain_ <= 0.0 || beta_gain_ <= 0.0 || max_speed_mps_ <= 0.0 ||
      shoulder_width_m_ <= 0.0 || eye_pelvis_m_ <= 0.0 || shoulder_pelvis_m_ <= 0.0 || length_tolerance_m_ <= 0.0 ||
      max_yaw_rate_rad_s_ <= 0.0 || post_hold_advance_distance_m_ < 0.0 || odom_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("Invalid snapshot alignment parameters");
    }
    shoulder_subscription_ = create_subscription<geometry_msgs::msg::PoseArray>("/shoulder_line", 10,
      [this](geometry_msgs::msg::PoseArray::SharedPtr message) { on_landmarks(*message); });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 20,
      [this](nav_msgs::msg::Odometry::SharedPtr message) { on_odom(*message); });
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/alignment_cmd", 10);
    angle_error_publisher_ = create_publisher<std_msgs::msg::Float32>("/alignment/error_angle_deg", 10);
    aligned_publisher_ = create_publisher<std_msgs::msg::Bool>("/alignment/aligned", 10);
    drive_enabled_publisher_ = create_publisher<std_msgs::msg::Bool>("/alignment/drive_enabled", 10);
    state_publisher_ = create_publisher<std_msgs::msg::String>("/shoulder_align/state", 10);
    // Reliable, volatile event: do not replay an old start to a restarted receiver.
    motor_start_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/motor_start", rclcpp::QoS(1).reliable().durability_volatile());
    timer_ = create_wall_timer(50ms, [this] { control_step(); });
  }

private:
  enum class State { CAPTURE, DRIVE, FINAL_HEADING, HOLD, POST_HOLD_ADVANCE, COMPLETE, ABORTED };

  void on_landmarks(const geometry_msgs::msg::PoseArray & message)
  {
    // RGB-D contract: left shoulder, right shoulder, eye midpoint, pelvis midpoint.
    // Validate physical lengths in 3-D BEFORE projecting the body axis onto the floor.
    if (message.poses.size() < 4) { return; }
    for (const auto & p : message.poses) {
      if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
        !std::isfinite(p.position.z)) { return; }
    }
    const auto & eye = message.poses[2].position;
    const auto & pelvis = message.poses[3].position;
    const auto & left = message.poses[0].position;
    const auto & right = message.poses[1].position;
    const auto distance = [](double x, double y, double z) { return std::sqrt(x*x + y*y + z*z); };
    const double width3 = distance(left.x-right.x, left.y-right.y, left.z-right.z);
    const double body3 = distance(eye.x-pelvis.x, eye.y-pelvis.y, eye.z-pelvis.z);
    const double torso3 = distance((left.x+right.x)/2-pelvis.x,
      (left.y+right.y)/2-pelvis.y, (left.z+right.z)/2-pelvis.z);
    const Vec2 axis{eye.x-pelvis.x, eye.y-pelvis.y};
    if (std::abs(width3-shoulder_width_m_) > length_tolerance_m_ ||
      std::abs(body3-eye_pelvis_m_) > length_tolerance_m_ ||
      std::abs(torso3-shoulder_pelvis_m_) > length_tolerance_m_ || norm(axis) < 0.20) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Capture rejects geometry: shoulder=%.3f eye-pelvis=%.3f shoulder-pelvis=%.3f m",
        width3, body3, torso3);
      return;
    }
    LandmarkPair sample{{message.poses[0].position.x, message.poses[0].position.y},
      {message.poses[1].position.x, message.poses[1].position.y}, message.header.frame_id, normalized(axis)};
    const double width = norm({sample.right.x - sample.left.x, sample.right.y - sample.left.y});
    if (!std::isfinite(sample.left.x) || !std::isfinite(sample.left.y) ||
      !std::isfinite(sample.right.x) || !std::isfinite(sample.right.y) ||
      width < min_shoulder_width_m_ || width > max_shoulder_width_m_) { return; }
    latest_landmarks_ = sample;
    last_landmark_time_ = now();
    ++landmark_sequence_;
  }

  void on_odom(const nav_msgs::msg::Odometry & message)
  {
    const auto & q = message.pose.pose.orientation;
    if (!std::isfinite(message.pose.pose.position.x) || !std::isfinite(message.pose.pose.position.y) ||
      !std::isfinite(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) ||
      q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1e-6) { robot_.reset(); return; }
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    robot_ = RobotPose{{message.pose.pose.position.x, message.pose.pose.position.y}, yaw, message.header.frame_id};
    last_odom_time_ = now();
    // Require fresh measured motion to remain small for 0.5 s. Repeated timer
    // ticks alone must never count as evidence that the cart has stopped.
    if (state_ == State::HOLD) {
      const double v = message.twist.twist.linear.x;
      const double w = message.twist.twist.angular.z;
      if (std::isfinite(v) && std::isfinite(w) && std::abs(v) < 0.005 && std::abs(w) < 0.01) {
        if (!stopped_since_) { stopped_since_ = now(); }
        stopped_confirmed_ = (now() - *stopped_since_).seconds() >= 0.5;
      } else {
        stopped_since_.reset();
        stopped_confirmed_ = false;
      }
    }
  }

  bool odom_is_fresh() const
  {
    return robot_.has_value() && (now() - last_odom_time_).seconds() <= odom_timeout_s_;
  }

  bool capture_input_is_valid() const
  {
    return latest_landmarks_.has_value() && odom_is_fresh() &&
      (now() - last_landmark_time_).seconds() <= landmark_timeout_s_ &&
      (latest_landmarks_->frame_id.empty() || robot_->frame_id.empty() ||
      latest_landmarks_->frame_id == robot_->frame_id);
  }

  void start_capture()
  {
    capture_samples_.clear();
    capture_started_ = true;
    capture_started_at_ = now();
    captured_sequence_ = landmark_sequence_ - 1;
    RCLCPP_INFO(get_logger(), "Snapshot capture started: keep the cart and person still for %.1f s", capture_duration_s_);
  }

  void add_new_capture_sample()
  {
    if (captured_sequence_ == landmark_sequence_) { return; }
    captured_sequence_ = landmark_sequence_;
    capture_samples_.push_back(*latest_landmarks_);
  }

  bool finalize_snapshot()
  {
    if (static_cast<int>(capture_samples_.size()) < capture_min_samples_) { return false; }
    std::vector<double> left_x, left_y, right_x, right_y, axis_x, axis_y;
    left_x.reserve(capture_samples_.size()); left_y.reserve(capture_samples_.size());
    right_x.reserve(capture_samples_.size()); right_y.reserve(capture_samples_.size());
    for (const auto & sample : capture_samples_) {
      left_x.push_back(sample.left.x); left_y.push_back(sample.left.y);
      right_x.push_back(sample.right.x); right_y.push_back(sample.right.y);
      axis_x.push_back(sample.body_axis.x); axis_y.push_back(sample.body_axis.y);
    }
    const Vec2 left{median(left_x), median(left_y)};
    const Vec2 right{median(right_x), median(right_y)};
    const Vec2 midpoint{(left.x + right.x) * 0.5, (left.y + right.y) * 0.5};
    const Vec2 shoulder{right.x - left.x, right.y - left.y};
    if (norm(shoulder) < min_shoulder_width_m_) { return false; }

    // Floor-projected pelvis -> eyes supplies body heading. Shoulder angle is
    // never used. Pick the cart's existing side once, then freeze this goal.
    const Vec2 body{median(axis_x), median(axis_y)};
    if (norm(body) < 0.5) { return false; }  // inconsistent samples: recapture
    Vec2 normal = normalized(body);
    const Vec2 robot_from_midpoint{robot_->position.x - midpoint.x, robot_->position.y - midpoint.y};
    if (dot(normal, robot_from_midpoint) < 0.0) { normal = {-normal.x, -normal.y}; }
    const double rear_pivot_standoff_m = front_clearance_m_ + pivot_to_front_m_;
    const Vec2 target{midpoint.x + normal.x * rear_pivot_standoff_m,
      midpoint.y + normal.y * rear_pivot_standoff_m};
    // At the final point the cart faces the shoulder midpoint along the body axis.
    snapshot_goal_ = SnapshotGoal{midpoint, target, std::atan2(midpoint.y - target.y, midpoint.x - target.x)};
    // Choose a virtual forward/reverse heading ONCE, avoiding sign chatter.
    const double bearing = std::atan2(target.y-robot_->position.y, target.x-robot_->position.x);
    travel_direction_ = std::cos(bearing-robot_->yaw_rad) >= 0.0 ? 1 : -1;
    state_ = State::DRIVE;
    hold_candidate_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "Snapshot locked from %zu samples: target=(%.3f, %.3f), rear standoff=%.3f m",
      capture_samples_.size(), target.x, target.y, rear_pivot_standoff_m);
    return true;
  }

  void reset_capture(const char * reason)
  {
    if (state_ != State::CAPTURE) { RCLCPP_WARN(get_logger(), "Snapshot discarded: %s", reason); }
    state_ = State::CAPTURE;
    snapshot_goal_.reset();
    capture_samples_.clear();
    capture_started_ = false;
    hold_candidate_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void control_step()
  {
    if (state_ == State::COMPLETE) { complete_step(); return; }
    if (state_ == State::ABORTED) {
      publish_stop(false);
      return;
    }
    // Odom is required throughout motion. Landmarks are required only for
    // CAPTURE, then deliberately ignored until a future capture starts.
    if (!odom_is_fresh()) {
      // COMPLETE is terminal; a later odometry outage must not start a new capture.
      if (state_ == State::HOLD || state_ == State::POST_HOLD_ADVANCE) {
        // Never repeat the final advance automatically after feedback loss.
        state_ = State::ABORTED;
        RCLCPP_ERROR(get_logger(), "Final advance aborted: odometry is stale; restart required");
      } else { reset_capture("odometry is stale"); }
      publish_stop(false);
      return;
    }
    if (state_ == State::CAPTURE) { capture_step(); return; }
    drive_step();
    if (state_ == State::COMPLETE) { complete_step(); }
  }

  void complete_step()
  {
    // Keep publishing stop while DDS discovers the waiting receiver and for
    // three seconds after the one-shot event. No sleep blocks the executor.
    publish_stop(false);
    if (!motor_start_sent_at_) {
      if (motor_start_publisher_->get_subscription_count() == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "COMPLETE: waiting for /motor_start subscriber before publishing");
        return;
      }
      motor_start_publisher_->publish(std_msgs::msg::Bool().set__data(true));
      motor_start_sent_at_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(get_logger(), "COMPLETE: /motor_start=true published once; exiting in 3 seconds");
    } else if (std::chrono::steady_clock::now() - *motor_start_sent_at_ >= 3s) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Shoulder alignment controller finished normally");
      rclcpp::shutdown();
    }
  }

  void capture_step()
  {
    publish_stop(false);
    if (!capture_input_is_valid()) {
      if (capture_started_) { reset_capture("waiting for matching /shoulder_line and /odom frames"); }
      return;
    }
    if (!capture_started_) { start_capture(); }
    add_new_capture_sample();
    if ((now() - capture_started_at_).seconds() < capture_duration_s_) { return; }
    if (!finalize_snapshot()) {
      RCLCPP_WARN(get_logger(), "Snapshot rejected: only %zu valid shoulder samples; retrying", capture_samples_.size());
      capture_started_ = false;
      capture_samples_.clear();
    }
  }

  void drive_step()
  {
    const Vec2 delta{snapshot_goal_->position.x - robot_->position.x,
      snapshot_goal_->position.y - robot_->position.y};
    const double rho_m = norm(delta);
    const double alpha_rad = normalize_angle(std::atan2(delta.y, delta.x) - robot_->yaw_rad);
    const double heading_error_rad = normalize_angle(snapshot_goal_->yaw_rad - robot_->yaw_rad);
    double linear_mps = 0.0;
    double angular_rad_s = 0.0;

    if (state_ == State::DRIVE) {
      if (rho_m <= position_enter_m_) {
        state_ = State::FINAL_HEADING;
      } else {
        const double bearing = std::atan2(delta.y, delta.x);
        const double offset = travel_direction_ < 0 ? kPi : 0.0;
        const double a = normalize_angle(bearing - robot_->yaw_rad - offset);
        const double b = normalize_angle(bearing - snapshot_goal_->yaw_rad - offset);
        const auto cmd = snapshot_math::control(rho_m, a, b, position_kp_,
          alpha_gain_, beta_gain_, max_speed_mps_, max_yaw_rate_rad_s_, travel_direction_);
        linear_mps = cmd.v;
        angular_rad_s = cmd.w;
      }
    }

    if (state_ == State::FINAL_HEADING) {
      if (rho_m >= position_exit_m_) {
        state_ = State::DRIVE;
        hold_candidate_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        publish_stop(true);
        return;
      }
      if (std::abs(heading_error_rad) <= heading_enter_rad_) {
        if (hold_candidate_since_.nanoseconds() == 0) { hold_candidate_since_ = now(); }
        if ((now() - hold_candidate_since_).seconds() >= hold_confirm_s_) {
          state_ = State::HOLD;
          stopped_since_.reset();
          stopped_confirmed_ = false;
          RCLCPP_INFO(get_logger(), "HOLD: waiting for measured stop for 0.5 s");
        }
      } else {
        hold_candidate_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        angular_rad_s = heading_kp_ * heading_error_rad;
      }
    }

    if (state_ == State::HOLD) {
      // Alignment is terminal. Do not reopen the old target after this point.
      if (post_hold_advance_distance_m_ == 0.0) {
        state_ = State::COMPLETE;
        RCLCPP_INFO(get_logger(), "Alignment complete: no post-hold advance requested");
      } else if (stopped_confirmed_) {
        post_hold_start_ = robot_->position;
        post_hold_heading_rad_ = robot_->yaw_rad;
        advance_speed_mps_ = 0.0;
        advance_tick_ = std::chrono::steady_clock::now();
        state_ = State::POST_HOLD_ADVANCE;
        RCLCPP_INFO(get_logger(), "Post-hold advance started: %.3f m straight ahead",
          post_hold_advance_distance_m_);
      }
    }

    if (state_ == State::POST_HOLD_ADVANCE) {
      // Count displacement along the departure yaw; retain that yaw throughout
      // the advance to correct asymmetric wheel startup without a new snapshot.
      const Vec2 displacement{robot_->position.x - post_hold_start_.x,
        robot_->position.y - post_hold_start_.y};
      const Vec2 forward{std::cos(post_hold_heading_rad_), std::sin(post_hold_heading_rad_)};
      const double advanced_m = dot(displacement, forward);
      if (advanced_m >= post_hold_advance_distance_m_) {
        state_ = State::COMPLETE;
        RCLCPP_INFO(get_logger(), "Post-hold advance complete: %.3f m", advanced_m);
      } else {
        const auto tick = std::chrono::steady_clock::now();
        const double dt = std::clamp(std::chrono::duration<double>(tick - advance_tick_).count(), 0.0, 0.05);
        advance_tick_ = tick;
        advance_speed_mps_ = advance_math::speed(advance_speed_mps_, dt, max_speed_mps_);
        linear_mps = advance_speed_mps_;
        const double yaw_error = normalize_angle(post_hold_heading_rad_ - robot_->yaw_rad);
        // Continuous deadband: ignore <0.5 degrees, cap correction at 0.05 rad/s.
        angular_rad_s = advance_math::yaw_command(post_hold_heading_rad_, robot_->yaw_rad, heading_kp_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
          "Advance: distance=%.3f/%.3f m yaw_error=%.2f deg v=%.3f w=%.3f",
          advanced_m, post_hold_advance_distance_m_, yaw_error * 180.0 / kPi, linear_mps, angular_rad_s);
      }
    }

    linear_mps = std::clamp(linear_mps, -max_speed_mps_, max_speed_mps_);
    angular_rad_s = std::clamp(angular_rad_s, -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);
    const bool enabled = state_ == State::DRIVE || state_ == State::FINAL_HEADING ||
      state_ == State::HOLD || state_ == State::POST_HOLD_ADVANCE;
    log_control(rho_m, alpha_rad, heading_error_rad, linear_mps, angular_rad_s);
    publish_velocity(linear_mps, angular_rad_s, enabled, heading_error_rad);
  }

  static const char * state_name(State state)
  {
    switch (state) {
      case State::CAPTURE: return "CAPTURE";
      case State::DRIVE: return "DRIVE";
      case State::FINAL_HEADING: return "FINAL_HEADING";
      case State::HOLD: return "HOLD";
      case State::POST_HOLD_ADVANCE: return "POST_HOLD_ADVANCE";
      case State::COMPLETE: return "COMPLETE";
      case State::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
  }

  void log_control(double rho_m, double alpha_rad, double heading_error_rad, double linear_mps, double angular_rad_s)
  {
    const double shoulder_range_m = norm({snapshot_goal_->shoulder_mid.x - robot_->position.x,
      snapshot_goal_->shoulder_mid.y - robot_->position.y});
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "state=%s rho_err=%.3f m shoulder_range=%.3f m target_rear_range=%.3f m "
      "alpha=%.2f deg heading_err=%.2f deg cmd=(v=%.3f m/s,w=%.3f rad/s)",
      state_name(state_), rho_m, shoulder_range_m, front_clearance_m_ + pivot_to_front_m_,
      alpha_rad * 180.0 / kPi, heading_error_rad * 180.0 / kPi, linear_mps, angular_rad_s);
  }

  void publish_velocity(double linear_mps, double angular_rad_s, bool drive_enabled, double heading_error_rad)
  {
    geometry_msgs::msg::Twist command;
    command.linear.x = linear_mps;
    command.angular.z = angular_rad_s;
    cmd_publisher_->publish(command);
    drive_enabled_publisher_->publish(std_msgs::msg::Bool().set__data(drive_enabled));
    aligned_publisher_->publish(std_msgs::msg::Bool().set__data(
      state_ == State::COMPLETE));
    state_publisher_->publish(std_msgs::msg::String().set__data(state_name(state_)));
    angle_error_publisher_->publish(std_msgs::msg::Float32().set__data(
      static_cast<float>(heading_error_rad * 180.0 / kPi)));
  }

  void publish_stop(bool drive_enabled) { publish_velocity(0.0, 0.0, drive_enabled, 0.0); }

  double front_clearance_m_{}, pivot_to_front_m_{}, capture_duration_s_{}, landmark_timeout_s_{};
  int capture_min_samples_{};
  double min_shoulder_width_m_{}, max_shoulder_width_m_{};
  double position_enter_m_{}, position_exit_m_{}, heading_enter_rad_{}, heading_exit_rad_{}, hold_confirm_s_{};
  double position_kp_{}, heading_kp_{}, alpha_gain_{}, beta_gain_{}, max_speed_mps_{}, max_yaw_rate_rad_s_{}, odom_timeout_s_{};
  double post_hold_advance_distance_m_{}, post_hold_heading_rad_{};
  double shoulder_width_m_{}, eye_pelvis_m_{}, shoulder_pelvis_m_{}, length_tolerance_m_{};
  int travel_direction_{1};
  State state_{State::CAPTURE};
  std::optional<LandmarkPair> latest_landmarks_;
  std::optional<RobotPose> robot_;
  std::optional<SnapshotGoal> snapshot_goal_;
  std::vector<LandmarkPair> capture_samples_;
  uint64_t landmark_sequence_{}, captured_sequence_{};
  bool capture_started_{false};
  rclcpp::Time capture_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_landmark_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time hold_candidate_since_{0, 0, RCL_ROS_TIME};
  Vec2 post_hold_start_{};
  std::optional<rclcpp::Time> stopped_since_;
  bool stopped_confirmed_{false};
  double advance_speed_mps_{0.0};
  std::chrono::steady_clock::time_point advance_tick_{};
  std::optional<std::chrono::steady_clock::time_point> motor_start_sent_at_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr shoulder_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr aligned_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_start_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_enabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShoulderAlignNode>());
  if (rclcpp::ok()) { rclcpp::shutdown(); }
  return 0;
}
