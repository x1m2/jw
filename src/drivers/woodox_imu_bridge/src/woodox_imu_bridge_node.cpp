// Copyright 2026 Bucar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "rcl/context.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "woodox_ros2_interface/msg/imu.hpp"

namespace
{

constexpr double kGravity = 9.80665;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

uint64_t stableHash(const std::string & value)
{
  uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : value) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::string lockPath(size_t domain_id, const std::string & output_topic)
{
  std::ostringstream digest;
  digest << std::hex << std::setfill('0') << std::setw(16)
         << stableHash(std::to_string(domain_id) + ":" + output_topic);
  return "/tmp/woodox-imu-bridge-" + std::to_string(getuid()) + "-" +
         digest.str() + ".lock";
}

}  // namespace

class WoodoxImuBridge : public rclcpp::Node
{
public:
  WoodoxImuBridge()
  : Node("woodox_imu_bridge")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/imu");
    const auto output_topic =
      declare_parameter<std::string>("output_topic", "/imu/data");
    const auto gyro_unit = declare_parameter<std::string>("gyro_unit", "rad_s");
    const auto acc_unit = declare_parameter<std::string>("acc_unit", "m_s2");
    frame_id_ = declare_parameter<std::string>("frame_id", "woodox_imu");
    orientation_format_ =
      declare_parameter<std::string>("orientation_format", "none");
    const auto input_reliability =
      declare_parameter<std::string>("input_reliability", "reliable");
    input_timeout_s_ = declare_parameter<double>("input_timeout_s", 5.0);

    if (gyro_unit != "rad_s" && gyro_unit != "deg_s") {
      throw std::invalid_argument("gyro_unit must be 'rad_s' or 'deg_s'");
    }
    gyro_scale_ = gyro_unit == "deg_s" ? kDegreesToRadians : 1.0;

    if (acc_unit != "m_s2" && acc_unit != "g") {
      throw std::invalid_argument("acc_unit must be 'm_s2' or 'g'");
    }
    acc_scale_ = acc_unit == "g" ? kGravity : 1.0;

    if (frame_id_.empty()) {
      throw std::invalid_argument("frame_id must be a non-empty string");
    }
    if (orientation_format_ != "none" && orientation_format_ != "wxyz" &&
      orientation_format_ != "xyzw")
    {
      throw std::invalid_argument(
              "orientation_format must be 'none', 'wxyz' or 'xyzw'");
    }
    if (input_reliability != "reliable" &&
      input_reliability != "best_effort")
    {
      throw std::invalid_argument(
              "input_reliability must be 'reliable' or 'best_effort'");
    }
    if (!std::isfinite(input_timeout_s_) || input_timeout_s_ <= 0.0) {
      throw std::invalid_argument("input_timeout_s must be finite and positive");
    }

    const auto node_base = get_node_base_interface();
    input_topic_ =
      node_base->resolve_topic_or_service_name(input_topic, false, false);
    output_topic_ =
      node_base->resolve_topic_or_service_name(output_topic, false, false);
    if (input_topic_ == output_topic_) {
      throw std::invalid_argument("Input and output topics must differ");
    }
    acquireInstanceLock();

    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    output_qos.reliable().durability_volatile();
    auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100));
    if (input_reliability == "reliable") {
      input_qos.reliable();
    } else {
      input_qos.best_effort();
    }
    input_qos.durability_volatile();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, output_qos);
    subscription_ = create_subscription<woodox_ros2_interface::msg::IMU>(
      input_topic, input_qos,
      std::bind(&WoodoxImuBridge::onImu, this, std::placeholders::_1));

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(input_timeout_s_));
    input_timer_ = create_wall_timer(timeout, [this]() {checkInput();});

    RCLCPP_INFO(
      get_logger(),
      "Forwarding %s to %s; gyro_unit=%s, acc_unit=%s, frame_id=%s, "
      "orientation=%s. Timestamps use this host clock when each callback "
      "receives the reading.",
      input_topic_.c_str(), output_topic_.c_str(), gyro_unit.c_str(),
      acc_unit.c_str(), frame_id_.c_str(), orientation_format_.c_str());
  }

  ~WoodoxImuBridge() override
  {
    if (lock_fd_ >= 0) {
      close(lock_fd_);
    }
  }

private:
  void acquireInstanceLock()
  {
    size_t domain_id = 0;
    const auto context = get_node_base_interface()->get_context();
    if (rcl_context_get_domain_id(context->get_rcl_context().get(), &domain_id) !=
      RCL_RET_OK)
    {
      throw std::runtime_error("Failed to determine ROS domain ID");
    }

    const auto path = lockPath(domain_id, output_topic_);
    lock_fd_ = open(
      path.c_str(),
      O_RDWR | O_CREAT | O_NOFOLLOW | O_CLOEXEC, 0600);
    if (lock_fd_ < 0) {
      throw std::runtime_error(
              "Failed to open instance lock " + path + ": " +
              std::strerror(errno));
    }
    if (flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      close(lock_fd_);
      lock_fd_ = -1;
      throw std::runtime_error(
              "A bridge for " + output_topic_ +
              " is already running on this host.");
    }
    if (ftruncate(lock_fd_, 0) != 0) {
      throw std::runtime_error("Failed to truncate instance lock " + path);
    }
    const auto pid = std::to_string(getpid());
    if (write(lock_fd_, pid.data(), pid.size()) < 0) {
      throw std::runtime_error("Failed to write instance lock " + path);
    }
  }

  void warnInvalid(const std::string & message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!invalid_warning_emitted_ ||
      now - last_invalid_warning_ >= std::chrono::seconds(5))
    {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      last_invalid_warning_ = now;
      invalid_warning_emitted_ = true;
    }
  }

  void checkInput()
  {
    const auto now = std::chrono::steady_clock::now();
    if (!received_valid_input_) {
      if (!reported_waiting_) {
        RCLCPP_WARN(
          get_logger(), "Waiting for valid IMU readings on %s.",
          input_topic_.c_str());
        reported_waiting_ = true;
      }
      return;
    }
    if (std::chrono::duration<double>(now - last_received_).count() >
      input_timeout_s_)
    {
      RCLCPP_WARN(
        get_logger(), "No valid IMU readings for %gs on %s.",
        input_timeout_s_, input_topic_.c_str());
    }
  }

  void onImu(const woodox_ros2_interface::msg::IMU::SharedPtr source)
  {
    for (const float value : source->gyroscope) {
      if (!std::isfinite(value)) {
        warnInvalid("Ignoring non-finite IMU reading.");
        return;
      }
    }
    for (const float value : source->accelerometer) {
      if (!std::isfinite(value)) {
        warnInvalid("Ignoring non-finite IMU reading.");
        return;
      }
    }

    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;

    if (orientation_format_ != "none") {
      std::array<double, 4> quaternion{};
      for (size_t index = 0; index < quaternion.size(); ++index) {
        quaternion[index] = static_cast<double>(source->quaternion[index]);
      }
      const double norm = std::sqrt(
        quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
      bool valid = std::isfinite(norm) && norm > 1e-12;
      for (const double value : quaternion) {
        valid = valid && std::isfinite(value);
      }
      if (valid) {
        if (orientation_format_ == "wxyz") {
          message.orientation.x = quaternion[1] / norm;
          message.orientation.y = quaternion[2] / norm;
          message.orientation.z = quaternion[3] / norm;
          message.orientation.w = quaternion[0] / norm;
        } else {
          message.orientation.x = quaternion[0] / norm;
          message.orientation.y = quaternion[1] / norm;
          message.orientation.z = quaternion[2] / norm;
          message.orientation.w = quaternion[3] / norm;
        }
        message.orientation_covariance[0] = 0.0;
      } else {
        warnInvalid(
          "Invalid quaternion; publishing angular velocity/acceleration "
          "without orientation.");
      }
    }

    message.angular_velocity.x = source->gyroscope[0] * gyro_scale_;
    message.angular_velocity.y = source->gyroscope[1] * gyro_scale_;
    message.angular_velocity.z = source->gyroscope[2] * gyro_scale_;
    message.linear_acceleration.x = source->accelerometer[0] * acc_scale_;
    message.linear_acceleration.y = source->accelerometer[1] * acc_scale_;
    message.linear_acceleration.z = source->accelerometer[2] * acc_scale_;
    publisher_->publish(message);

    if (!received_valid_input_ || reported_waiting_) {
      RCLCPP_INFO(
        get_logger(),
        "IMU readings received; publishing standard messages on %s.",
        output_topic_.c_str());
      reported_waiting_ = false;
    }
    received_valid_input_ = true;
    last_received_ = std::chrono::steady_clock::now();
  }

  double gyro_scale_{1.0};
  double acc_scale_{1.0};
  double input_timeout_s_{5.0};
  std::string frame_id_;
  std::string orientation_format_;
  std::string input_topic_;
  std::string output_topic_;
  int lock_fd_{-1};
  bool received_valid_input_{false};
  bool reported_waiting_{false};
  bool invalid_warning_emitted_{false};
  std::chrono::steady_clock::time_point last_received_{};
  std::chrono::steady_clock::time_point last_invalid_warning_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<woodox_ros2_interface::msg::IMU>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr input_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<WoodoxImuBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("woodox_imu_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
