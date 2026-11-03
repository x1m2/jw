#include "slam_manager/uwb_follow.hpp"
#include "slam_manager/slam_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>
#include <utility>

namespace slam_manager {

namespace {
constexpr double kUwbWorkerStartTimeoutSec = 30.0;
constexpr double kUwbWorkerStopTimeoutSec = 5.0;
constexpr std::array<const char *, 5> kUwbWorkerNames{
    "FAST-LIO", "obstacle_stop",
    "path_smoother", "pure_pursuit", "uwb_follow_planner"};
}

// Keep the response across individual wait timeouts. Only the lifecycle
// thread replaces the waiter; the ROS response callback completes it once.
struct UwbFollowController::PendingStart {
  void wait(WorkerCompletion completion) {
    bool success;
    std::string message;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!completed_) {
        waiter_ = std::move(completion);
        return;
      }
      success = success_;
      message = message_;
    }
    completion(success, message);
  }

  void complete(bool success, const std::string &message) {
    WorkerCompletion waiter;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      completed_ = true;
      success_ = success;
      message_ = message;
      waiter = std::move(waiter_);
    }
    if (waiter) waiter(success, message);
  }

 private:
  std::mutex mutex_;
  bool completed_{false};
  bool success_{false};
  std::string message_;
  WorkerCompletion waiter_;
};

UwbFollowController::UwbFollowController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : fast_lio_worker_(node, "/fast_lio_worker/set_state", callback_group),
      obstacle_stop_worker_(node, "/obstacle_stop_worker/set_state",
                            callback_group),
      path_smoother_worker_(node, "/path_smoother_worker/set_state",
                            callback_group),
      pure_pursuit_worker_(node, "/pure_pursuit_worker/set_state",
                           callback_group),
      uwb_follow_planner_worker_(
          node, "/uwb_follow_planner_worker/set_state", callback_group),
      start_order_{&fast_lio_worker_, &obstacle_stop_worker_,
                   &path_smoother_worker_, &pure_pursuit_worker_,
                   &uwb_follow_planner_worker_},
      stop_order_{&pure_pursuit_worker_, &uwb_follow_planner_worker_,
                  &path_smoother_worker_, &obstacle_stop_worker_,
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

bool UwbFollowController::startFastLio(
    uint64_t session_id, const std::shared_ptr<WorkerOperationToken> &token,
    double timeout_sec, std::string &message) {
  return startThrough(session_id, token, timeout_sec, 1U, message);
}

bool UwbFollowController::start(
    uint64_t session_id, const std::shared_ptr<WorkerOperationToken> &token,
    double timeout_sec, std::string &message) {
  return startThrough(session_id, token, timeout_sec, start_order_.size(),
                      message);
}

bool UwbFollowController::startThrough(
    uint64_t session_id, const std::shared_ptr<WorkerOperationToken> &token,
    double timeout_sec, std::size_t worker_count, std::string &message) {
  if (session_id_ != 0U && session_id_ != session_id) {
    message = "previous UWB follow session has not been stopped";
    return false;
  }
  session_id_ = session_id;
  for (std::size_t index = 0; index < worker_count; ++index) {
    WorkerClient *worker = start_order_[index];
    if (!token || !token->active()) {
      message = "follow startup superseded";
      return false;
    }
    if (started_[index]) continue;
    const bool send_start = !pending_starts_[index];
    if (send_start && !worker->serviceReady()) {
      message = std::string(kUwbWorkerNames[index]) + " Worker is unavailable";
      return false;
    }
    WorkerStartContext context;
    if (worker == &pure_pursuit_worker_ ||
        worker == &path_smoother_worker_ || worker == &obstacle_stop_worker_) {
      context.algorithm_profile = "uwb_follow";
    } else if (worker == &uwb_follow_planner_worker_) {
      context.algorithm_profile = "managed_follow";
    }
    if (send_start) {
      // A timed-out START can still finish inside the Worker. Keep both
      // ownership and the response state until it resolves or teardown.
      attempted_[index] = true;
      pending_starts_[index] = std::make_shared<PendingStart>();
    }
    const auto pending = pending_starts_[index];
    const auto result = awaitWorkerCommand(
        "UWB follow worker start", timeout_sec,
        std::make_shared<WorkerOperationToken>(),
        [worker, session_id, context, token, pending,
         send_start](WorkerCompletion completion) {
          pending->wait(std::move(completion));
          if (!send_start) return;
          worker->start(
              session_id, context,
              [worker, session_id, token, pending](
                  bool success, const std::string &response) {
                // Only an explicit session cancellation (STOP or APP takeover)
                // stops a late success. An individual retry timeout does not.
                if (success && !token->active()) {
                  worker->stopConfirmed(
                      session_id, [](bool, const std::string &) {});
                }
                pending->complete(success, response);
              });
        },
        [token]() { return token && token->active() && rclcpp::ok(); });
    if (!result.success()) {
      if (result.status == WorkerCommandStatus::kFailed) {
        // An explicit failure (or send exception) permits a new attempt.
        // A timeout leaves the original request pending and never resends it.
        pending_starts_[index].reset();
      }
      // Retain uncertain ownership across retries for explicit teardown.
      message = std::string(kUwbWorkerNames[index]) + ": " + result.message;
      if (result.status == WorkerCommandStatus::kTimedOut) {
        message += "; waiting for the outstanding START response (not resent)";
      }
      return false;
    }
    pending_starts_[index].reset();
    started_[index] = true;
  }
  message = "UWB follow workers started";
  return true;
}

bool UwbFollowController::stop(double timeout_sec, std::string &message) {
  if (!active()) {
    session_id_ = 0U;
    started_.fill(false);
    pending_starts_.fill(nullptr);
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
    if (result.success()) {
      attempted_[index] = false;
      started_[index] = false;
      // Late callbacks retain their response state and cancelled session
      // token independently; they cannot stop a newer APP session.
      pending_starts_[index].reset();
    }
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
  return uwb_follow_controller_->stop(kUwbWorkerStopTimeoutSec, message);
}

void SlamManagerNode::uwbFollowCommandCallback(
    const std_msgs::msg::Int8::ConstSharedPtr cmd) {
  const int value = static_cast<int>(cmd->data);
  RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] BLE command=%d mode=%s", value,
              std::string(toString(state_machine_.mode())).c_str());
  if (value == 0) {
    RCLCPP_WARN(get_logger(),
                "[uwb_follow_disabled] UWB follow is unavailable: SCAN adaptation was withdrawn");
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

  ++session_id_;
  const uint64_t follow_session = session_id_;

  // Pure Pursuit starts paused in the follow profile. The UWB planner alone
  // decides when fresh odometry and target data allow it to resume.
  while (rclcpp::ok() && !shutting_down_.load() &&
         isLifecycleCommandActive(generation, token)) {
    // FAST-LIO has no motion output. Bring it up even while the robot gate or
    // the rest of the follow stack is unavailable, and retain it on retries.
    bool success = uwb_follow_controller_->startFastLio(
        follow_session, token, kUwbWorkerStartTimeoutSec, message);
    std::string mode_message;
    if (success && isLifecycleCommandActive(generation, token)) {
      success = robotStateAllowsNavigation(message);
    }
    if (success && isLifecycleCommandActive(generation, token)) {
      success = setRobotMode(0, mode_message);
      if (!success) message = "SetMode(0) failed: " + mode_message;
    }
    if (success && isLifecycleCommandActive(generation, token)) {
      success = uwb_follow_controller_->start(
          follow_session, token, kUwbWorkerStartTimeoutSec, message);
    }
    if (success && isLifecycleCommandActive(generation, token) &&
        uwb_planner_command_pub_->get_subscription_count() == 0U) {
      success = false;
      message = "UWB planner command subscriber unavailable";
    }
    if (success && isLifecycleCommandActive(generation, token)) {
      success = robotStateAllowsNavigation(message);
    }
    if (success && isLifecycleCommandActive(generation, token)) {
      success = setRobotMode(1, mode_message);
      if (!success) message = "SetMode(1) failed: " + mode_message;
    }
    if (success && isLifecycleCommandActive(generation, token)) {
      // Release the planner (and therefore PP) only after the whole stack
      // and robot gate are ready. Missing DDS discovery is retried above.
      success = sendUwbPlannerCommand(0);
      if (!success) {
        message = "UWB planner command subscriber unavailable";
        setRobotMode(0, mode_message);
      }
    }
    if (success && isLifecycleCommandActive(generation, token) &&
        state_machine_.followStarted()) {
      publishMode();
      RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] started, session=%llu",
                  static_cast<unsigned long long>(follow_session));
      return true;
    }
    if (!isLifecycleCommandActive(generation, token)) break;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[UWB_FOLLOW] startup pending, retaining Workers and retrying: %s",
        message.c_str());
    for (int attempt = 0; attempt < 10 && rclcpp::ok() &&
         !shutting_down_.load() &&
         isLifecycleCommandActive(generation, token); ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // Retryable startup failures keep the session and FAST-LIO. Only STOP,
  // APP takeover, or shutdown tears down this session.
  token->cancel();
  std::string stop_message;
  const bool stopped = stopUwbFollowWorkers(stop_message);
  if (stopped) {
    state_machine_.resetToIdleAfterTeardown();
    publishMode();
  } else {
    enterError(RuntimeState::NAVIGATION_ERROR,
               "UWB follow rollback failed: " + stop_message);
  }
  RCLCPP_INFO(get_logger(), "[UWB_FOLLOW] startup cancelled; cleanup: %s",
              stop_message.c_str());
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
