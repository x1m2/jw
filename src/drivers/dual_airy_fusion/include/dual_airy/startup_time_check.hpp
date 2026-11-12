#pragma once
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <set>
#include <std_msgs/msg/string.hpp>

#include "dual_airy/core.hpp"

namespace dual_airy {
// 挂在融合节点上的启动阶段模块；不是独立 ROS 节点，不执行点云融合。
// poll() 由节点默认回调组调用，四路观测使用单独的 IMU/点云回调组。
class StartupTimeCheck {
 public:
  enum class State { Checking, Ready, Failed };
  StartupTimeCheck(rclcpp::Node &node, Calibration config, std::string output,
                   double timeout, std::array<std::string, 4> topics,
                   const std::string &reliability,
                   rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status);
  State poll();
  // 仅在 poll() 返回 Ready 后读取；此时观测已停止，delta 在本会话冻结。
  const Calibration &calibration() const { return config_; }

 private:
  void observe(size_t i, const builtin_interfaces::msg::Time &stamp,
               const rclcpp::MessageInfo &info);
  rclcpp::Node &node_;
  Calibration config_;
  std::string output_;
  double timeout_, started_ = 0, last_poll_ = 0;
  State state_ = State::Checking;
  HostTimeAlignment alignment_;
  std::set<std::string> sources_;
  std::mutex mutex_;
  std::array<std::string, 4> topics_;
  std::array<rclcpp::SubscriptionBase::SharedPtr, 4> subscriptions_;
  std::array<uint64_t, 4> received_{};
  std::array<double, 4> last_callback_{}, last_header_{}, middleware_age_{};
  rclcpp::CallbackGroup::SharedPtr imu_group_, cloud_group_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
}  // namespace dual_airy
