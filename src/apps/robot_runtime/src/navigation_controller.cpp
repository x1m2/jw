#include "robot_runtime/navigation_controller.hpp"

#include <chrono>
#include <future>
#include <sstream>
#include <utility>

namespace robot_runtime {

NavigationController::NavigationController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : costmap_worker_(node, "/costmap_worker/set_state", callback_group),
      global_planner_worker_(node, "/global_path_planner_worker/set_state",
                             callback_group),
      path_smoother_worker_(node, "/path_smoother_worker/set_state",
                            callback_group),
      pure_pursuit_worker_(node, "/pure_pursuit_worker/set_state",
                           callback_group),
      // 纯跟踪最后启动，避免依赖模块未就绪时提前输出速度。
      start_order_{&costmap_worker_, &global_planner_worker_,
                   &path_smoother_worker_, &pure_pursuit_worker_},
      // 停止时优先关闭运动控制，确保机器人先停下来。
      stop_order_{&pure_pursuit_worker_, &path_smoother_worker_,
                  &global_planner_worker_, &costmap_worker_} {
  pause_client_ = node->create_client<std_srvs::srv::SetBool>(
      "/pure_pursuit/set_paused", rmw_qos_profile_services_default,
      callback_group);
}

void NavigationController::start(uint64_t session_id,
                                 const std::string &topology_file,
                                 const std::string &map_directory,
                                 Completion completion) {
  // 从启动顺序的第一个 Worker 开始异步串行启动。
  startNext(session_id, 0U, topology_file, map_directory,
            std::make_shared<WorkerList>(), std::move(completion));
}

void NavigationController::startNext(uint64_t session_id, std::size_t index,
                                     const std::string &topology_file,
                                     const std::string &map_directory,
                                     std::shared_ptr<WorkerList> started,
                                     Completion completion) {
  if (index == start_order_.size()) {
    completion(true, "navigation workers started");
    return;
  }

  WorkerClient *worker = start_order_[index];
  auto worker_completion =
      [this, session_id, index, worker, started,
       topology_file, map_directory,
       completion = std::move(completion)](
          bool success, const std::string &message) mutable {
        if (!success) {
          // 保留已启动列表，失败后按相反顺序回滚。
          rollbackStart(session_id, started, "navigation start failed: " +
                                                   message,
                        std::move(completion));
          return;
        }
        started->push_back(worker);
        startNext(session_id, index + 1U, topology_file, map_directory,
                  started, std::move(completion));
      };

  // 全局规划器按当前map_id动态加载task拓扑和map可行域点云。
  if (worker == &global_planner_worker_) {
    worker->start(session_id, topology_file, map_directory,
                  std::move(worker_completion));
  } else {
    worker->start(session_id, std::move(worker_completion));
  }
}

void NavigationController::rollbackStart(uint64_t session_id,
                                         std::shared_ptr<WorkerList> started,
                                         std::string failure,
                                         Completion completion) {
  if (started->empty()) {
    completion(false, failure);
    return;
  }

  WorkerClient *worker = started->back();
  started->pop_back();
  worker->stop(
      session_id,
      [this, session_id, started, failure = std::move(failure),
       completion = std::move(completion)](
          bool, const std::string &) mutable {
        rollbackStart(session_id, started, std::move(failure),
                      std::move(completion));
      });
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

}  // namespace robot_runtime
