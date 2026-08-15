#include "slam_manager/localization_controller.hpp"

#include <utility>

namespace slam_manager {

LocalizationController::LocalizationController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : fastlio_localization_worker_(
          node, "/fastlio_localization_worker/set_state", callback_group) {}

void LocalizationController::start(uint64_t session_id,
                                   const std::string &map_id,
                                   const std::string &map_path,
                                   bool use_initial_pose,
                                   Completion completion) {
  // fastlio_localization 内部包含定位前端和基于地图的重定位流程。
  fastlio_localization_worker_.start(
      session_id, map_id, map_path, use_initial_pose, std::move(completion));
}

void LocalizationController::stop(uint64_t session_id,
                                  Completion completion) {
  fastlio_localization_worker_.stop(session_id, std::move(completion));
}

}  // namespace slam_manager
