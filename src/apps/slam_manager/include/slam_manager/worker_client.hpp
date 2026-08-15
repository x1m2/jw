#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager_interfaces/srv/set_worker_state.hpp"

namespace slam_manager {

// 封装 Worker 启停服务的异步调用，供各业务控制器复用。
class WorkerClient {
 public:
  using SetWorkerState = slam_manager_interfaces::srv::SetWorkerState;
  using Completion = std::function<void(bool, const std::string &)>;

  WorkerClient(rclcpp::Node *node, const std::string &service_name,
               rclcpp::CallbackGroup::SharedPtr callback_group);

  void start(uint64_t session_id, Completion completion);
  void start(uint64_t session_id, const std::string &resource_id,
             const std::string &resource_path, Completion completion);
  void start(uint64_t session_id, const std::string &resource_id,
             const std::string &resource_path, bool use_initial_pose,
             Completion completion);
  void stop(uint64_t session_id, Completion completion);

 private:
  // 发送统一命令，并通过 completion 返回执行结果。
  void request(uint8_t command, uint64_t session_id,
               const std::string &resource_id,
               const std::string &resource_path, bool use_initial_pose,
               Completion completion);

  std::string service_name_;
  rclcpp::Client<SetWorkerState>::SharedPtr client_;
};

}  // namespace slam_manager
