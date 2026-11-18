#include "slam_manager/navigation.hpp"

#include <chrono>
#include <future>
#include <stdexcept>
#include <sstream>
#include <utility>

namespace slam_manager {

NavigationController::NavigationController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group,
    const std::string &profile)
    : obstacle_stop_worker_(node, "/obstacle_stop_worker/set_state",
                            callback_group),
      global_planner_worker_(node, "/global_path_planner_worker/set_state",
                             callback_group),
      path_smoother_worker_(node, "/path_smoother_worker/set_state",
                            callback_group),
      scan_planner_worker_(node, "/scan_planner_worker/set_state",
                           callback_group),
      terrain_filter_worker_(node, "/terrain_filter_worker/set_state",
                             callback_group),
      costmap_worker_(node, "/costmap_worker/set_state", callback_group),
      pure_pursuit_worker_(node, "/pure_pursuit_worker/set_state",
                           callback_group) {
  if (profile == "3d") {
    start_order_ = {&obstacle_stop_worker_, &global_planner_worker_,
                    &path_smoother_worker_, &scan_planner_worker_};
    stop_order_ = {&scan_planner_worker_, &path_smoother_worker_,
                   &global_planner_worker_, &obstacle_stop_worker_};
    pause_client_ = node->create_client<std_srvs::srv::SetBool>(
        "/scan_planner/set_paused", rmw_qos_profile_services_default,
        callback_group);
  } else if (profile == "2d") {
    start_order_ = {&obstacle_stop_worker_, &global_planner_worker_,
                    &path_smoother_worker_, &terrain_filter_worker_,
                    &costmap_worker_, &pure_pursuit_worker_};
    stop_order_ = {&pure_pursuit_worker_, &costmap_worker_,
                   &terrain_filter_worker_, &path_smoother_worker_,
                   &global_planner_worker_, &obstacle_stop_worker_};
    pause_client_ = node->create_client<std_srvs::srv::SetBool>(
        "/pure_pursuit/set_paused", rmw_qos_profile_services_default,
        callback_group);
  } else {
    throw std::invalid_argument("navigation.profile must be '2d' or '3d'");
  }
}

void NavigationController::start(uint64_t session_id,
                                 const std::string &topology_file,
                                 const std::string &map_directory,
                                 std::shared_ptr<WorkerOperationToken>
                                     operation_token,
                                 Completion completion) {
  // 从启动顺序的第一个 Worker 开始异步串行启动。
  startNext(session_id, 0U, topology_file, map_directory, operation_token,
            std::move(completion));
}

void NavigationController::startNext(uint64_t session_id, std::size_t index,
                                     const std::string &topology_file,
                                     const std::string &map_directory,
                                     std::shared_ptr<WorkerOperationToken>
                                         operation_token,
                                     Completion completion) {
  if (!operation_token->active()) {
    completion(false, "navigation start cancelled");
    return;
  }
  if (index == start_order_.size()) {
    completion(true, "navigation workers started");
    return;
  }

  WorkerClient *worker = start_order_[index];
  auto worker_completion =
      [this, session_id, index, operation_token,
       topology_file, map_directory,
       completion = std::move(completion)](
          bool success, const std::string &message) mutable {
        if (!operation_token->active()) {
          completion(false, "navigation start cancelled after Worker response");
          return;
        }
        if (!success) {
          completion(false, "navigation start failed: " + message);
          return;
        }
        startNext(session_id, index + 1U, topology_file, map_directory,
                  operation_token, std::move(completion));
      };

  // 全局规划器和停障节点加载同一份运行时拓扑。规划器额外需要地图目录；
  // 停障节点从拓扑文件的同级目录自行解析区域ID对应的YAML。
  if (worker == &global_planner_worker_ || worker == &obstacle_stop_worker_) {
    WorkerStartContext worker_context;
    worker_context.resource_id = topology_file;
    if (worker == &global_planner_worker_) {
      worker_context.resource_path = map_directory;
    }
    worker->start(session_id, worker_context, std::move(worker_completion));
  } else {
    worker->start(session_id, std::move(worker_completion));
  }
}

void NavigationController::stop(uint64_t session_id, Completion completion) {
  // 停止过程继续执行到最后，单个模块失败不会阻断后续清理。
  stopNext(session_id, 0U,
           std::make_shared<std::vector<std::string>>(),
           std::move(completion));
}

bool NavigationController::setPaused(bool paused, double timeout_sec,
                                     std::string &message) {
  const auto timeout = std::chrono::duration<double>(timeout_sec);
  if (!pause_client_->wait_for_service(timeout)) {
    message = "pure pursuit pause service is unavailable";
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = paused;
  auto future = pause_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    message = "pure pursuit pause request timed out";
    return false;
  }

  try {
    const auto response = future.get();
    message = response->message;
    return response->success;
  } catch (const std::exception &error) {
    message = std::string("pure pursuit pause request failed: ") + error.what();
    return false;
  }
}

void NavigationController::stopNext(
    uint64_t session_id, std::size_t index,
    std::shared_ptr<std::vector<std::string>> errors,
    Completion completion) {
  if (index == stop_order_.size()) {
    if (errors->empty()) {
      completion(true, "navigation workers stopped");
      return;
    }
    std::ostringstream message;
    message << "navigation stop failed:";
    for (const auto &error : *errors) message << " " << error << ";";
    completion(false, message.str());
    return;
  }

  stop_order_[index]->stop(
      session_id,
      [this, session_id, index, errors,
       completion = std::move(completion)](
          bool success, const std::string &message) mutable {
        if (!success) errors->push_back(message);
        stopNext(session_id, index + 1U, errors, std::move(completion));
      });
}

}  // namespace slam_manager
