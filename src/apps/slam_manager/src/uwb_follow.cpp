#include "slam_manager/uwb_follow.hpp"
#include "slam_manager/slam_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace slam_manager {

namespace {
constexpr double kUwbWorkerTimeoutSec = 5.0;
}

UwbFollowController::UwbFollowController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : fast_lio_worker_(node, "/fast_lio_worker/set_state", callback_group),
      terrain_filter_worker_(node, "/terrain_filter_worker/set_state",
                             callback_group),
      obstacle_stop_worker_(node, "/obstacle_stop_worker/set_state",
                            callback_group),
      costmap_worker_(node, "/costmap_worker/set_state", callback_group),
      path_smoother_worker_(node, "/path_smoother_worker/set_state",
                            callback_group),
      pure_pursuit_worker_(node, "/pure_pursuit_worker/set_state",
                           callback_group),
      uwb_follow_planner_worker_(
          node, "/uwb_follow_planner_worker/set_state", callback_group),
      start_order_{&fast_lio_worker_, &terrain_filter_worker_,
                   &obstacle_stop_worker_, &costmap_worker_,
                   &path_smoother_worker_, &pure_pursuit_worker_,
                   &uwb_follow_planner_worker_},
      stop_order_{&pure_pursuit_worker_, &uwb_follow_planner_worker_,
                  &path_smoother_worker_, &costmap_worker_,
                  &obstacle_stop_worker_, &terrain_filter_worker_,
                  &fast_lio_worker_} {}

bool UwbFollowController::active() const {
  return std::any_of(attempted_.begin(), attempted_.end(),
                     [](bool attempted) { return attempted; });
}

bool UwbFollowController::wasAttempted(const WorkerClient *worker) const {
  const auto entry = std::find(start_order_.begin(), start_order_.end(), worker);
  return entry != start_order_.end() &&
         attempted_[static_cast<std::size_t>(entry - start_order_.begin())];
}

bool UwbFollowController::ready(std::string &message) const {
  // Check every service before changing the Manager mode or starting Workers.
  // The follow planner is first because it may be absent from an older OTA.
  if (!uwb_follow_planner_worker_.serviceReady()) {
    message = "UWB follow planner Worker is unavailable";
    return false;
  }
  for (WorkerClient *worker : start_order_) {
    if (worker == &uwb_follow_planner_worker_) continue;
    if (!worker->serviceReady()) {
      message = "UWB follow Worker is unavailable";
      return false;
    }
  }
  message = "UWB follow Workers are available";
  return true;
}

bool UwbFollowController::start(
    uint64_t session_id, const std::shared_ptr<WorkerOperationToken> &token,
    double timeout_sec, std::string &message) {
  session_id_ = session_id;
  for (std::size_t index = 0; index < start_order_.size(); ++index) {
    WorkerClient *worker = start_order_[index];
    if (!token || !token->active()) {
      message = "follow startup superseded";
      return false;
    }
    WorkerStartContext context;
    if (worker == &pure_pursuit_worker_ || worker == &costmap_worker_ ||
        worker == &path_smoother_worker_ || worker == &obstacle_stop_worker_) {
      context.algorithm_profile = "uwb_follow";
    } else if (worker == &uwb_follow_planner_worker_) {
      context.algorithm_profile = "managed_follow";
    }
    // A timed-out START can still finish inside the Worker. Remember the
    // attempt before sending it, then use the same session for every STOP.
    attempted_[index] = true;
    const auto result = awaitWorkerCommand(
        "UWB follow worker start", timeout_sec, token,
        [worker, session_id, context, token](WorkerCompletion completion) {
          worker->start(
              session_id, context,
              [worker, session_id, token,
               completion = std::move(completion)](
                  bool success, const std::string &response) {
                // A START response can arrive after timeout and cleanup.
                // Its session-scoped STOP cannot stop a later APP session.
                if (success && !token->active()) {
                  worker->stopConfirmed(
                      session_id, [](bool, const std::string &) {});
                }
                completion(success, response);
              });
        });
    if (!result.success()) {
      // A completed failure cannot own an algorithm in this session. A
      // timeout is uncertain and must remain in the cleanup set.
      if (result.status == WorkerCommandStatus::kFailed) {
        attempted_[index] = false;
      }
      message = result.message;
      return false;
    }
  }
  message = "UWB follow workers started";
  return true;
}

bool UwbFollowController::stop(double timeout_sec, std::string &message) {
  if (!active()) {
    message = "already stopped";
    return true;
  }
  bool all_stopped = true;
  message.clear();
  for (WorkerClient *worker : stop_order_) {
    const auto entry = std::find(start_order_.begin(), start_order_.end(), worker);
    const auto index = static_cast<std::size_t>(entry - start_order_.begin());
    if (!attempted_[index]) continue;
    const auto result = awaitWorkerCommand(
        "UWB follow worker stop", timeout_sec,
        std::make_shared<WorkerOperationToken>(),
        [worker, session_id = session_id_](WorkerCompletion completion) {
          worker->stopConfirmed(session_id, std::move(completion));
        });
    all_stopped = all_stopped && result.success();
    if (result.success()) attempted_[index] = false;
    if (!message.empty()) message += "; ";
    message += result.message;
  }
  if (all_stopped) {
    session_id_ = 0U;
  }
  return all_stopped;
}

bool SlamManagerNode::sendUwbPlannerCommand(int8_t command) {
  for (int attempt = 0; attempt < 40; ++attempt) {
    if (uwb_planner_command_pub_->get_subscription_count() != 0U) {
      std_msgs::msg::Int8 message;
      message.data = command;
      uwb_planner_command_pub_->publish(message);
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_WARN(get_logger(), "UWB planner command subscriber unavailable");
  return false;
}

bool SlamManagerNode::stopUwbFollowWorkers(std::string &message) {
  std::string ignored;
  setRobotMode(0, ignored);
  if (uwb_follow_controller_->plannerAttempted()) sendUwbPlannerCommand(2);
  return uwb_follow_controller_->stop(kUwbWorkerTimeoutSec, message);
}

void SlamManagerNode::uwbFollowCommandCallback(
    const std_msgs::msg::Int8::ConstSharedPtr cmd) {
  const int value = static_cast<int>(cmd->data);
  RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] BLE command=%d mode=%s", value,
              std::string(toString(state_machine_.mode())).c_str());
  if (value == 0) {
    const auto mode = state_machine_.mode();
    if (mode == SlamMode::kFollowPaused) {
      resumeUwbFollow();
    } else if (mode == SlamMode::kIdle) {
      // Localization can leave the Manager idle while its APP Worker is active.
      // Reject before beginLifecycleCommand cancels an APP operation.
      if (active_localization_session_id_.load() != 0U) {
        RCLCPP_WARN(get_logger(),
                    "UWB follow rejected: APP localization is active");
        return;
      }
      std::string readiness_message;
      if (!uwb_follow_controller_->ready(readiness_message)) {
        RCLCPP_WARN(get_logger(), "UWB follow rejected: %s",
                    readiness_message.c_str());
        return;
      }
      const auto generation = beginLifecycleCommand("start_uwb_follow");
      startUwbFollow(generation);
    } else if (mode != SlamMode::kFollowing) {
      RCLCPP_WARN(get_logger(), "UWB follow start rejected in mode=%s",
                  std::string(toString(mode)).c_str());
    }
  } else if (value == 2) {
    const auto mode = state_machine_.mode();
    if (mode == SlamMode::kStartingFollow || mode == SlamMode::kFollowing ||
        mode == SlamMode::kFollowPaused) {
      beginLifecycleCommand("stop_uwb_follow");
      stopUwbFollow();
    } else if (mode == SlamMode::kError) {
      // A failed teardown may leave UWB Workers running. The stop method
      // checks ownership under the lifecycle lock, so an unrelated APP error
      // cannot be cleared by a BLE command.
      stopUwbFollow();
    }
  } else if (value == 1) {
    const auto mode = state_machine_.mode();
    if (mode == SlamMode::kFollowing) {
      pauseUwbFollow();
    } else if (mode == SlamMode::kFollowPaused) {
      // Pause is idempotent; only 0 resumes.
      sendUwbPlannerCommand(1);
    } else {
      RCLCPP_WARN(get_logger(), "UWB pause ignored in mode=%s",
                  std::string(toString(mode)).c_str());
    }
  } else {
    RCLCPP_WARN(get_logger(), "Unsupported BLE follow command=%d", value);
  }
}

bool SlamManagerNode::startUwbFollow(uint64_t generation) {
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const auto token = installLifecycleToken(generation);
  if (!token || state_machine_.mode() != SlamMode::kIdle) return false;

  std::string message;
  if (!isLifecycleCommandActive(generation, token) ||
      !state_machine_.beginFollow()) return false;
  relocalization_succeeded_.store(false);
  publishMode();

  // Pure Pursuit starts paused in the follow profile. The UWB planner alone
  // decides when fresh odometry and target data allow it to resume.
  std::string mode_message;
  bool success = robotStateAllowsNavigation(message) &&
                 setRobotMode(0, mode_message);
  if (success) {
    ++session_id_;
    success = uwb_follow_controller_->start(
        session_id_, token, kUwbWorkerTimeoutSec, message);
  }
  if (success && isLifecycleCommandActive(generation, token)) {
    success = sendUwbPlannerCommand(0);
    if (!success) message = "UWB planner command subscriber unavailable";
  }
  if (success && isLifecycleCommandActive(generation, token)) {
    success = robotStateAllowsNavigation(message);
  }
  if (success && isLifecycleCommandActive(generation, token)) {
    success = setRobotMode(1, mode_message);
    if (!success) message = "SetMode(1) failed: " + mode_message;
  }
  if (success && isLifecycleCommandActive(generation, token) &&
      state_machine_.followStarted()) {
    publishMode();
    RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] started, session=%llu",
                static_cast<unsigned long long>(session_id_));
    return true;
  }

  // Roll back even if a newer command cancelled this startup. Never leave a
  // partially started path controller running.
  std::string stop_message;
  const bool stopped = stopUwbFollowWorkers(stop_message);
  if (stopped) {
    state_machine_.resetToIdleAfterTeardown();
    publishMode();
  } else {
    enterError(RuntimeState::NAVIGATION_ERROR,
               "UWB follow rollback failed: " + stop_message);
  }
  RCLCPP_ERROR(get_logger(), "UWB follow start failed: %s; cleanup: %s",
               message.c_str(), stop_message.c_str());
  return false;
}

bool SlamManagerNode::pauseUwbFollow() {
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (state_machine_.mode() != SlamMode::kFollowing) return false;
  if (!sendUwbPlannerCommand(1)) {
    // No planner can own PP pause in this state. Stop the follow stack.
    std::string stop_message;
    const bool stopped = stopUwbFollowWorkers(stop_message);
    if (stopped) {
      state_machine_.resetToIdleAfterTeardown();
      publishMode();
    } else {
      enterError(RuntimeState::NAVIGATION_ERROR,
                 "UWB pause fallback stop failed: " + stop_message);
    }
    RCLCPP_ERROR(get_logger(), "UWB pause command could not reach planner");
    return false;
  }
  state_machine_.pauseFollow();
  publishMode();
  RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] paused; FAST-LIO remains active");
  return true;
}

bool SlamManagerNode::resumeUwbFollow() {
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (state_machine_.mode() != SlamMode::kFollowPaused) return false;
  std::string message;
  if (!robotStateAllowsNavigation(message)) {
    RCLCPP_WARN(get_logger(), "UWB follow resume rejected: %s",
                message.c_str());
    return false;
  }
  // The planner keeps PP paused until its own pose and UWB checks pass.
  if (!sendUwbPlannerCommand(0)) {
    RCLCPP_WARN(get_logger(), "UWB follow resume command could not reach planner");
    return false;
  }
  state_machine_.resumeFollow();
  publishMode();
  RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] resumed");
  return true;
}

bool SlamManagerNode::stopUwbFollow() {
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const bool retry_after_error = state_machine_.mode() == SlamMode::kError;
  if (retry_after_error && !uwb_follow_controller_->active()) return false;
  if (!state_machine_.beginStopFollow()) return false;
  // Keep the published error until every outstanding Worker is stopped.
  if (!retry_after_error) publishMode();
  std::string message;
  const bool stopped = stopUwbFollowWorkers(message);
  if (stopped && state_machine_.followStopped()) {
    publishMode();
    RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] stopped");
    return true;
  }
  enterError(RuntimeState::NAVIGATION_ERROR,
             "UWB follow stop failed: " + message);
  return false;
}

}  // namespace slam_manager
