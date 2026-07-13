#ifndef FAST_LIO_IMU_ORIENTATION_CONVERTER_HPP
#define FAST_LIO_IMU_ORIENTATION_CONVERTER_HPP

#include <functional>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

class ImuOrientationConverter {
 public:
  // 只在平地/楼梯状态变化时通知 FAST-LIO。
  using StairsStateCallback = std::function<void(bool is_stairs)>;

  ImuOrientationConverter(rclcpp::Node *node,
                          StairsStateCallback stairs_state_callback);

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  geometry_msgs::msg::Vector3 quaternionToEuler(
      const geometry_msgs::msg::Quaternion &quat) const;
  geometry_msgs::msg::Vector3 applyLowPassFilter(
      const geometry_msgs::msg::Vector3 &new_euler_deg);

  rclcpp::Node *node_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_deviation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stairs_state_pub_;
  StairsStateCallback stairs_state_callback_;

  double alpha_ = 0.1;
  bool first_data_ = true;
  geometry_msgs::msg::Vector3 filtered_euler_deg_;

  double pitch_baseline_ = 0.0;
  double stairs_threshold_ = 14.0;
  bool last_stairs_state_ = false;

  int queue_size_ = 10;
  bool verbose_logging_ = false;

  std::string imu_topic_;
  std::string euler_topic_;
  std::string pitch_deviation_topic_;
  std::string stairs_state_topic_;
};

#endif  // FAST_LIO_IMU_ORIENTATION_CONVERTER_HPP
