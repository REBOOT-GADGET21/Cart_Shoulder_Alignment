#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "hwt9053_imu/imu_math.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace hwt9053_imu
{
class Hwt9053ImuNode : public rclcpp::Node
{
public:
  // Declare parameters, open the serial device, and create ROS interfaces.
  Hwt9053ImuNode()
  : Node("hwt9053_imu")
  {
    port_                 = declare_parameter<std::string>("port", "/dev/imu_usb");
    baudrate_             = declare_parameter<int>("baudrate", 115200);
    protocol_             = declare_parameter<std::string>("protocol", "RS485_HIGH");
    modbus_id_            = declare_parameter<int>("modbus_id", 80);
    frame_id_             = declare_parameter<std::string>("frame_id", "imu_link");
    relative_orientation_ = declare_parameter<bool>("relative_orientation", true);
    timeout_ms_           = declare_parameter<int>("response_timeout_ms", 50);

    if (protocol_ != "RS485_HIGH") {
      throw std::runtime_error("Only protocol=RS485_HIGH is supported");
    }
    if (modbus_id_ < 1 || modbus_id_ > 247) {
      throw std::runtime_error("modbus_id must be 1..247");
    }

    open_serial();
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/imu/rpy", rclcpp::SensorDataQoS());
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/imu/reset_reference",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        reference_.reset();
        response->success = true;
        response->message = "Reference will be captured from the next valid IMU sample.";
      });
    timer_ = create_wall_timer(2ms, std::bind(&Hwt9053ImuNode::poll, this));

    RCLCPP_INFO(
      get_logger(), "Opened %s at %d baud (Modbus ID 0x%02X)",
      
      port_.c_str(), baudrate_, modbus_id_);
  }

  // Close the serial file descriptor when the ROS node is destroyed.
  ~Hwt9053ImuNode() override
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  // Convert the configured baud rate to the POSIX termios value.
  static speed_t baud(int value)
  {
    switch (value) {
      case 9600:
        return B9600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:
        throw std::runtime_error("Supported baudrates: 9600, 115200, 230400");
    }
  }

  // Open and configure an 8N1 non-blocking serial port.
  void open_serial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Cannot open " + port_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      throw std::runtime_error("tcgetattr failed");
    }

    cfsetispeed(&tty, baud(baudrate_));
    cfsetospeed(&tty, baud(baudrate_));
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag = 0;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      throw std::runtime_error("tcsetattr failed");
    }
  }

  // Perform one non-blocking Modbus transaction step.
  void poll()
  {
    read_serial();
    parse_buffer();

    const auto now_time = std::chrono::steady_clock::now();
    if (waiting_ && now_time - sent_at_ > std::chrono::milliseconds(timeout_ms_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Modbus response timeout");
      waiting_ = false;
      advance_request();
    }

    if (!waiting_) {
      send_request();
    }
  }

  // Request the next HWT9053 data register block.
  void send_request()
  {
    current_address_ = addresses_[request_index_];
    const uint16_t register_count = current_address_ == 0x3d ? 6 : 3;
    const auto request = modbus_read_request(
      static_cast<uint8_t>(modbus_id_), current_address_, register_count);
    const auto written = ::write(fd_, request.data(), request.size());

    if (written == static_cast<ssize_t>(request.size())) {
      waiting_ = true;
      sent_at_ = std::chrono::steady_clock::now();
    }
  }

  // Move to the next register after a response or timeout.
  void advance_request()
  {
    request_index_ = (request_index_ + 1) % addresses_.size();
  }

  // Copy currently available serial bytes into the receive buffer.
  void read_serial()
  {
    uint8_t data[256];
    const auto bytes_read = ::read(fd_, data, sizeof(data));
    if (bytes_read > 0) {
      buffer_.insert(buffer_.end(), data, data + bytes_read);
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Serial read error: %s", std::strerror(errno));
    }
  }

  // Resynchronize, validate CRC, and dispatch complete Modbus response frames.
  void parse_buffer()
  {
    while (buffer_.size() >= 5) {
      if (buffer_[0] != modbus_id_ || buffer_[1] != 0x03) {
        buffer_.erase(buffer_.begin());
        continue;
      }

      const size_t frame_length = 5 + buffer_[2];
      if (buffer_.size() < frame_length) {
        return;
      }

      const uint16_t received_crc = static_cast<uint16_t>(buffer_[frame_length - 2]) |
        (static_cast<uint16_t>(buffer_[frame_length - 1]) << 8);
      if (modbus_crc(buffer_.data(), frame_length - 2) != received_crc) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Discarded Modbus frame with invalid CRC");
        buffer_.erase(buffer_.begin());
        continue;
      }

      std::vector<uint8_t> frame(buffer_.begin(), buffer_.begin() + frame_length);
      buffer_.erase(buffer_.begin(), buffer_.begin() + frame_length);
      handle_frame(frame);
      waiting_ = false;
      advance_request();
    }
  }

  // Decode a Modbus big-endian signed 16-bit integer.
  static int16_t be_i16(const uint8_t * data)
  {
    return static_cast<int16_t>((data[0] << 8) | data[1]);
  }

  // Decode the vendor-specific high-precision 32-bit angle encoding.
  static int32_t high_precision_angle(const uint8_t * data)
  {
    return static_cast<int32_t>(
      (data[2] << 24) | (data[3] << 16) | (data[0] << 8) | data[1]);
  }

  // Store an HWT9053 register response; publish after the angle response.
  void handle_frame(const std::vector<uint8_t> & frame)
  {
    const uint8_t * data = frame.data() + 3;
    const size_t byte_count = frame[2];

    if (current_address_ == 0x3d) {
      if (byte_count != 12) {
        RCLCPP_WARN(get_logger(), "Unexpected angle response length: %zu", byte_count);
        return;
      }
      angles_ = {
        high_precision_angle(data),
        high_precision_angle(data + 4),
        high_precision_angle(data + 8)};
      publish();
      return;
    }

    if (byte_count != 6) {
      RCLCPP_WARN(get_logger(), "Unexpected vector response length: %zu", byte_count);
      return;
    }

    const std::array<int16_t, 3> values{
      be_i16(data),
      be_i16(data + 2),
      be_i16(data + 4)};
    if (current_address_ == 0x34) {
      acceleration_ = values;
    } else if (current_address_ == 0x37) {
      gyro_ = values;
    }
  }

  // Convert cached raw data to ROS messages and publish them.
  void publish()
  {
    constexpr double gravity = 9.80665;
    constexpr double deg_to_rad = M_PI / 180.0;
    const auto absolute_orientation = rpy_to_quaternion(
      angles_[0] / 1000.0 * deg_to_rad,
      angles_[1] / 1000.0 * deg_to_rad,
      angles_[2] / 1000.0 * deg_to_rad);
    const auto orientation = relative_orientation_ ?
      reference_.apply(absolute_orientation) : absolute_orientation;
    const auto rpy = quaternion_to_rpy(orientation);
    const auto stamp = now();

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = frame_id_;
    imu.orientation.x = orientation.x;
    imu.orientation.y = orientation.y;
    imu.orientation.z = orientation.z;
    imu.orientation.w = orientation.w;
    imu.angular_velocity.x = gyro_[0] * (2000.0 / 32768.0) * deg_to_rad;
    imu.angular_velocity.y = gyro_[1] * (2000.0 / 32768.0) * deg_to_rad;
    imu.angular_velocity.z = gyro_[2] * (2000.0 / 32768.0) * deg_to_rad;
    imu.linear_acceleration.x = acceleration_[0] * (16.0 / 32768.0) * gravity;
    imu.linear_acceleration.y = acceleration_[1] * (16.0 / 32768.0) * gravity;
    imu.linear_acceleration.z = acceleration_[2] * (16.0 / 32768.0) * gravity;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);

    geometry_msgs::msg::Vector3Stamped debug;
    debug.header = imu.header;
    debug.vector.x = rpy[0];
    debug.vector.y = rpy[1];
    debug.vector.z = rpy[2];
    rpy_pub_->publish(debug);
  }

  int fd_{-1};
  int baudrate_;
  int modbus_id_;
  int timeout_ms_;
  std::string port_;
  std::string protocol_;
  std::string frame_id_;
  bool relative_orientation_;
  bool waiting_{false};
  std::array<uint16_t, 4> addresses_{{0x34, 0x37, 0x3a, 0x3d}};
  size_t request_index_{0};
  uint16_t current_address_{0};
  std::chrono::steady_clock::time_point sent_at_{};
  std::vector<uint8_t> buffer_;
  std::array<int16_t, 3> acceleration_{{0, 0, 0}};
  std::array<int16_t, 3> gyro_{{0, 0, 0}};
  std::array<int32_t, 3> angles_{{0, 0, 0}};
  RelativeOrientation reference_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpy_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace hwt9053_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hwt9053_imu::Hwt9053ImuNode>());
  } 
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hwt9053_imu"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
