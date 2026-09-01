#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager/worker_runtime.hpp"

namespace slam_manager {

// 负责建图相关 Worker 的有序启动、停止和失败回滚。
class MappingController {
 public:
  using Completion = std::function<void(bool, const std::string &)>;

  MappingController(rclcpp::Node *node,
                    rclcpp::CallbackGroup::SharedPtr callback_group);

  void start(uint64_t session_id, const std::string &map_id,
             std::shared_ptr<WorkerOperationToken> operation_token,
             Completion completion);
  void finish(uint64_t session_id, const std::string &map_save_path,
              Completion completion);
  void cancel(uint64_t session_id, Completion completion);

 private:
  // PGO 先启动，FastLIO 后启动；停止时顺序相反。
  WorkerClient pgo_worker_;
  WorkerClient fast_lio_worker_;
};

}  // namespace slam_manager
