#include "imu_orientation_converter.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <utility>

namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
}  // namespace

ImuOrientationConverter::ImuOrientationConverter(
    rclcpp::Node *node, StairsStateCallback stairs_state_callback)
    : node_(node), stairs_state_callback_(std::move(stairs_state_callback)) {
  alpha_ = node_->declare_parameter<double>("imu_orientation.alpha", 0.1);
  queue_size_ = node_->declare_parameter<int>("imu_orientation.queue_size", 10);
  verbose_logging_ =
      node_->declare_parameter<bool>("imu_orientation.verbose_logging", false);
  pitch_baseline_ =
      node_->declare_parameter<double>("imu_orientation.pitch_baseline", 0.0);
  stairs_threshold_ = node_->declare_parameter<double>(
      "imu_orientation.stairs_threshold", 14.0);

  imu_topic_ = node_->declare_parameter<std::string>(
      "imu_orientation.imu_topic", "/imu/data");
  euler_topic_ = node_->declare_parameter<std::string>(
      "imu_orientation.euler_topic", "/imu/data_orientation");
  pitch_deviation_topic_ = node_->declare_parameter<std::string>(
      "imu_orientation.pitch_deviation_topic", "/imu/pitch_deviation");
  stairs_state_topic_ = node_->declare_parameter<std::string>(
      "imu_orientation.stairs_state_topic", "/stairs_state");

  if (queue_size_ <= 0) {
    RCLCPP_WARN(node_->get_logger(),
                "imu_orientation.queue_size <= 0, reset to 10");
    queue_size_ = 10;
  }
  if (alpha_ <= 0.0 || alpha_ > 1.0) {
    RCLCPP_WARN(node_->get_logger(),
                "imu_orientation.alpha should be in (0, 1], reset to 0.1");
    alpha_ = 0.1;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));
  auto imu_qos =
      rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size_)));
  imu_qos.best_effort();
  imu_qos.durability_volatile();

  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      std::bind(&ImuOrientationConverter::imuCallback, this,
                std::placeholders::_1));

  euler_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      euler_topic_, qos);
  pitch_deviation_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
      pitch_deviation_topic_, qos);
  stairs_state_pub_ =
      node_->create_publisher<std_msgs::msg::Bool>(stairs_state_topic_, qos);

  filtered_euler_deg_.x = 0.0;
  filtered_euler_deg_.y = 0.0;
  filtered_euler_deg_.z = 0.0;

  RCLCPP_INFO(node_->get_logger(),
              "IMU orientation converter embedded in fast_lio: %s -> %s, "
              "baseline=%.1f deg, threshold=%.1f deg, stairs_state=%s",
              imu_topic_.c_str(), euler_topic_.c_str(), pitch_baseline_,
              stairs_threshold_, stairs_state_topic_.c_str());
}

void ImuOrientationConverter::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  // 将 IMU 四元数转成欧拉角角度制，再做低通滤波。
  const auto euler_angles_rad = quaternionToEuler(msg->orientation);

  geometry_msgs::msg::Vector3 euler_angles_deg;
  euler_angles_deg.x = euler_angles_rad.x * kRadToDeg;
  euler_angles_deg.y = euler_angles_rad.y * kRadToDeg;
  euler_angles_deg.z = euler_angles_rad.z * kRadToDeg;

  const auto filtered_euler_deg = applyLowPassFilter(euler_angles_deg);

  // 楼梯检测只看 pitch 偏离平地基准的幅度。
  const double pitch_deviation =
      std::abs(filtered_euler_deg.y - pitch_baseline_);

  std_msgs::msg::Float32 deviation_msg;
  deviation_msg.data = static_cast<float>(pitch_deviation);
  pitch_deviation_pub_->publish(deviation_msg);

  const bool current_stairs_state = pitch_deviation > stairs_threshold_;

  std_msgs::msg::Bool stairs_state_msg;
  stairs_state_msg.data = current_stairs_state;
  stairs_state_pub_->publish(stairs_state_msg);

  // 只在状态变化时通知 FAST-LIO，避免每帧 IMU 都重置 z 约束。
  if (current_stairs_state != last_stairs_state_) {
    last_stairs_state_ = current_stairs_state;
    if (stairs_state_callback_) {
      stairs_state_callback_(current_stairs_state);
    }

    RCLCPP_INFO(node_->get_logger(),
                "[STAIRS_DETECT] state=%s, pitch_deviation=%.2f deg, "
                "threshold=%.1f deg",
                current_stairs_state ? "stairs" : "flat", pitch_deviation,
                stairs_threshold_);
  }

  geometry_msgs::msg::Vector3Stamped euler_msg;
  euler_msg.header = msg->header;
  euler_msg.vector = filtered_euler_deg;
  euler_pub_->publish(euler_msg);

  if (verbose_logging_) {
    RCLCPP_INFO(node_->get_logger(),
                "[IMU_ORIENTATION] roll=%.2f pitch=%.2f yaw=%.2f, "
                "deviation=%.2f, stairs=%d",
                filtered_euler_deg.x, filtered_euler_deg.y,
                filtered_euler_deg.z, pitch_deviation, current_stairs_state);
  }
}

geometry_msgs::msg::Vector3 ImuOrientationConverter::quaternionToEuler(
    const geometry_msgs::msg::Quaternion &quat) const {
  tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf_quat.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

  geometry_msgs::msg::Vector3 result;
  result.x = std::remainder(roll, 2.0 * M_PI);
  result.y = pitch;
  result.z = std::remainder(yaw, 2.0 * M_PI);

  return result;
}

geometry_msgs::msg::Vector3 ImuOrientationConverter::applyLowPassFilter(
    const geometry_msgs::msg::Vector3 &new_euler_deg) {
  if (first_data_) {
    filtered_euler_deg_ = new_euler_deg;
    first_data_ = false;
    return filtered_euler_deg_;
  }

  geometry_msgs::msg::Vector3 diff;
  diff.x = new_euler_deg.x - filtered_euler_deg_.x;
  diff.y = new_euler_deg.y - filtered_euler_deg_.y;
  diff.z = new_euler_deg.z - filtered_euler_deg_.z;

  // 角度差按最短路径滤波，避免跨 +/-180 度时突跳。
  diff.x = std::remainder(diff.x, 360.0);
  diff.y = std::remainder(diff.y, 360.0);
  diff.z = std::remainder(diff.z, 360.0);

  filtered_euler_deg_.x += alpha_ * diff.x;
  filtered_euler_deg_.y += alpha_ * diff.y;
  filtered_euler_deg_.z += alpha_ * diff.z;

  filtered_euler_deg_.x = std::remainder(filtered_euler_deg_.x, 360.0);
  while (filtered_euler_deg_.y > 90.0)
    filtered_euler_deg_.y = 180.0 - filtered_euler_deg_.y;
  while (filtered_euler_deg_.y < -90.0)
    filtered_euler_deg_.y = -180.0 - filtered_euler_deg_.y;
  filtered_euler_deg_.z = std::remainder(filtered_euler_deg_.z, 360.0);

  return filtered_euler_deg_;
}
