#include "slam_manager/localization.hpp"

#include <utility>

namespace slam_manager {

LocalizationController::LocalizationController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : localization_worker_(
          node, node->declare_parameter<std::string>(
                    "localization.worker_service",
                    "/fastlio_localization_worker/set_state"), callback_group) {}

void LocalizationController::start(uint64_t session_id,
                                   const WorkerStartContext &context,
                                   Completion completion) {
  // 定位 Worker 内部包含前端跟踪和基于地图的重定位流程。
  localization_worker_.start(session_id, context, std::move(completion));
}

void LocalizationController::stop(uint64_t session_id,
                                  Completion completion) {
  localization_worker_.stop(session_id, std::move(completion));
}

}  // namespace slam_manager
