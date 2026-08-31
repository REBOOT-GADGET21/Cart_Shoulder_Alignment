#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "hwt9053_imu/imu_math.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

extern "C" {
#include "wit_c_sdk.h"
}

using namespace std::chrono_literals;

namespace hwt9053_imu
{
class Hwt9053ImuNode : public rclcpp::Node
{
public:
  Hwt9053ImuNode() : Node("hwt9053_imu")
  {
    port_ = declare_parameter<std::string>("port", "/dev/imu_usb");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    protocol_ = declare_parameter<std::string>("protocol", "RS485_HIGH");
    modbus_id_ = declare_parameter<int>("modbus_id", 80);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    relative_orientation_ = declare_parameter<bool>("relative_orientation", true);
    timeout_ms_ = declare_parameter<int>("response_timeout_ms", 50);
    if (protocol_ != "RS485_HIGH") throw std::runtime_error("Only protocol=RS485_HIGH is supported");
    if (modbus_id_ < 1 || modbus_id_ > 247) throw std::runtime_error("modbus_id must be 1..247");

    open_serial();
    sdk_instance_ = this;
    initialize_sdk();
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS());
    rpy_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/rpy", rclcpp::SensorDataQoS());
    reset_service_ = create_service<std_srvs::srv::Trigger>("/imu/reset_reference",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        reference_.reset();
        response->success = true;
        response->message = "Reference will be captured from the next valid IMU sample.";
      });
    request_axis6_status();
    timer_ = create_wall_timer(2ms, [this] { poll(); });
    RCLCPP_INFO(get_logger(), "Opened %s at %d baud (Modbus ID 0x%02X)",
      port_.c_str(), baudrate_, modbus_id_);
  }

  ~Hwt9053ImuNode() override
  {
    if (sdk_instance_ == this) { sdk_instance_ = nullptr; WitDeInit(); }
    if (fd_ >= 0) ::close(fd_);
  }

private:
  enum class State { kVerifyAxis6, kWriteAxis6, kVerifySavedAxis6, kPolling };

  static speed_t baud(int value)
  {
    switch (value) {
      case 9600: return B9600;
      case 115200: return B115200;
      case 230400: return B230400;
      default: throw std::runtime_error("Supported baudrates: 9600, 115200, 230400");
    }
  }

  void open_serial()
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("Cannot open " + port_ + ": " + std::strerror(errno));
    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(errno)));
    cfsetispeed(&tty, baud(baudrate_));
    cfsetospeed(&tty, baud(baudrate_));
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag = 0; tty.c_oflag = 0; tty.c_lflag = 0;
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) throw std::runtime_error("tcsetattr failed: " + std::string(std::strerror(errno)));
  }

  void initialize_sdk()
  {
    if (WitInit(WIT_PROTOCOL_905x_MODBUS, static_cast<uint8_t>(modbus_id_)) != WIT_HAL_OK ||
      WitSerialWriteRegister(&Hwt9053ImuNode::sdk_write) != WIT_HAL_OK ||
      WitRegisterCallBack(&Hwt9053ImuNode::sdk_update) != WIT_HAL_OK)
    {
      throw std::runtime_error("Failed to initialize WITMOTION SDK");
    }
  }

  static void sdk_write(uint8_t * data, uint32_t size)
  {
    if (sdk_instance_) sdk_instance_->write_bytes(data, size);
  }
  static void sdk_update(uint32_t first_register, uint32_t register_count)
  {
    if (sdk_instance_) sdk_instance_->on_sdk_update(first_register, register_count);
  }

  void write_bytes(const uint8_t * data, uint32_t size)
  {
    size_t offset = 0;
    while (offset < size) {
      const ssize_t written = ::write(fd_, data + offset, size - offset);
      if (written > 0) { offset += static_cast<size_t>(written); continue; }
      if (written < 0 && errno == EINTR) continue;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial write error: %s",
        written < 0 ? std::strerror(errno) : "zero-byte write");
      return;
    }
  }

  void read_serial()
  {
    uint8_t data[256];
    const ssize_t count = ::read(fd_, data, sizeof(data));
    if (count > 0) {
      for (ssize_t i = 0; i < count; ++i) WitSerialDataIn(data[i]);
    } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read error: %s", std::strerror(errno));
    }
  }

  void request_axis6_status()
  {
    waiting_ = WitReadReg(AXIS6, 1) == WIT_HAL_OK;
    sent_at_ = std::chrono::steady_clock::now();
  }
  void request_data()
  {
    // Official high-precision ROS example reads one coherent 0x34..0x42 block.
    waiting_ = WitReadReg(AX, 15) == WIT_HAL_OK;
    sent_at_ = std::chrono::steady_clock::now();
  }

  void poll()
  {
    read_serial();
    if (waiting_ && std::chrono::steady_clock::now() - sent_at_ > std::chrono::milliseconds(timeout_ms_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Modbus response timeout");
      waiting_ = false;
      initialize_sdk();  // clear official SDK's partial receive buffer before retrying
    }
    if (waiting_) return;
    switch (state_) {
      case State::kVerifyAxis6:
      case State::kVerifySavedAxis6: request_axis6_status(); break;
      case State::kWriteAxis6: configure_axis6(); break;
      case State::kPolling: request_data(); break;
    }
  }

  void configure_axis6()
  {
    // Sensor-side 6-axis configuration: KEY=0xB588, AXIS6=1, SAVE=0.
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) {
      RCLCPP_ERROR(get_logger(), "Failed to send WITMOTION KEY unlock command"); return;
    }
    std::this_thread::sleep_for(20ms);
    if (WitWriteReg(AXIS6, ALGRITHM6) != WIT_HAL_OK || WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK) {
      RCLCPP_ERROR(get_logger(), "Failed to send WITMOTION 6-axis configuration commands"); return;
    }
    // The SDK parses read (0x03) frames only; remove write acknowledgements before verification.
    initialize_sdk();
    state_ = State::kVerifySavedAxis6;
    std::this_thread::sleep_for(20ms);
  }

  void on_sdk_update(uint32_t first_register, uint32_t register_count)
  {
    waiting_ = false;
    if (first_register == AXIS6 && register_count == 1) {
      if (sReg[AXIS6] == ALGRITHM6) {
        state_ = State::kPolling;
        RCLCPP_INFO(get_logger(), "Confirmed HWT9073 6-axis algorithm (AXIS6=0x0001)");
      } else {
        state_ = State::kWriteAxis6;
        RCLCPP_WARN(get_logger(), "AXIS6=0x%04X; configuring 6-axis algorithm", static_cast<uint16_t>(sReg[AXIS6]));
      }
    } else if (first_register == AX && register_count == 15) {
      publish_from_sdk();
    }
  }

  static int32_t high_precision_angle(size_t axis)
  {
    const uint32_t raw = (static_cast<uint32_t>(static_cast<uint16_t>(sReg[HRoll + 2 * axis])) << 16) |
      static_cast<uint16_t>(sReg[LRoll + 2 * axis]);
    int32_t signed_raw{};
    static_assert(sizeof(signed_raw) == sizeof(raw), "Expected a 32-bit signed angle");
    std::memcpy(&signed_raw, &raw, sizeof(raw));
    return signed_raw;
  }

  void publish_from_sdk()
  {
    constexpr double gravity = 9.80665;
    constexpr double deg_to_rad = M_PI / 180.0;
    const auto absolute = rpy_to_quaternion(high_precision_angle(0) / 1000.0 * deg_to_rad,
      high_precision_angle(1) / 1000.0 * deg_to_rad, high_precision_angle(2) / 1000.0 * deg_to_rad);
    const auto orientation = relative_orientation_ ? reference_.apply(absolute) : absolute;
    const auto rpy = quaternion_to_rpy(orientation);
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = frame_id_;
    imu.orientation.x = orientation.x; imu.orientation.y = orientation.y;
    imu.orientation.z = orientation.z; imu.orientation.w = orientation.w;
    imu.angular_velocity.x = sReg[GX] * (2000.0 / 32768.0) * deg_to_rad;
    imu.angular_velocity.y = sReg[GY] * (2000.0 / 32768.0) * deg_to_rad;
    imu.angular_velocity.z = sReg[GZ] * (2000.0 / 32768.0) * deg_to_rad;
    imu.linear_acceleration.x = sReg[AX] * (16.0 / 32768.0) * gravity;
    imu.linear_acceleration.y = sReg[AY] * (16.0 / 32768.0) * gravity;
    imu.linear_acceleration.z = sReg[AZ] * (16.0 / 32768.0) * gravity;
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity_covariance[0] = -1.0;
    imu.linear_acceleration_covariance[0] = -1.0;
    imu_pub_->publish(imu);
    geometry_msgs::msg::Vector3Stamped debug;
    debug.header = imu.header;
    debug.vector.x = rpy[0]; debug.vector.y = rpy[1]; debug.vector.z = rpy[2];
    rpy_pub_->publish(debug);
  }

  inline static Hwt9053ImuNode * sdk_instance_{nullptr};
  int fd_{-1}, baudrate_{}, modbus_id_{}, timeout_ms_{};
  std::string port_, protocol_, frame_id_;
  bool relative_orientation_{}, waiting_{false};
  State state_{State::kVerifyAxis6};
  std::chrono::steady_clock::time_point sent_at_{};
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
  try { rclcpp::spin(std::make_shared<hwt9053_imu::Hwt9053ImuNode>()); }
  catch (const std::exception & error) { RCLCPP_FATAL(rclcpp::get_logger("hwt9053_imu"), "%s", error.what()); }
  rclcpp::shutdown();
  return 0;
}
