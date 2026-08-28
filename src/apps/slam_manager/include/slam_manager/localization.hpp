#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager/worker_runtime.hpp"

namespace slam_manager {

// 负责基于地图定位 Worker 的启动和停止。
class LocalizationController {
 public:
  using Completion = std::function<void(bool, const std::string &)>;

  LocalizationController(rclcpp::Node *node,
                         rclcpp::CallbackGroup::SharedPtr callback_group);

  void start(uint64_t session_id, const WorkerStartContext &context,
             Completion completion);
  void stop(uint64_t session_id, Completion completion);

 private:
  WorkerClient fastlio_localization_worker_;
};

}  // namespace slam_manager
