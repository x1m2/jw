#include "slam_manager/worker_runtime.hpp"
#include "slam_manager_interfaces/msg/worker_status.hpp"

#include <chrono>
#include <condition_variable>
#include <exception>
#include <memory>
#include <mutex>
#include <utility>

namespace slam_manager {

namespace {

struct CommandCompletionState {
  std::mutex mutex;
  std::condition_variable condition;
  bool accepting{true};
  bool completed{false};
  bool success{false};
  std::string message;
};

WorkerCompletion makeCompletion(
    const std::shared_ptr<CommandCompletionState> &state) {
  return [state](bool success, const std::string &message) {
    {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (!state->accepting || state->completed) return;
      state->completed = true;
      state->success = success;
      state->message = message;
    }
    state->condition.notify_all();
  };
}

WorkerCommandResult waitForCommand(
    const std::string &name, double timeout_sec,
    const std::shared_ptr<WorkerOperationToken> &token,
    const AsyncWorkerCommand &command,
    const std::function<bool()> &is_current) {
  WorkerCommandResult result;
  if (timeout_sec <= 0.0) {
    result.message = name + " has an invalid timeout";
    return result;
  }

  auto state = std::make_shared<CommandCompletionState>();
  try {
    command(makeCompletion(state));
  } catch (const std::exception &error) {
    result.message = name + " threw an exception: " + error.what();
    return result;
  }

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
  std::unique_lock<std::mutex> lock(state->mutex);
  while (!state->completed) {
    if (is_current && !is_current()) {
      state->accepting = false;
      token->cancel();
      result.status = WorkerCommandStatus::kSuperseded;
      result.message = name + " superseded by a newer operation";
      return result;
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      state->accepting = false;
      token->cancel();
      result.status = WorkerCommandStatus::kTimedOut;
      result.message = name + " timed out";
      return result;
    }
    state->condition.wait_for(lock, std::chrono::milliseconds(50));
  }

  state->accepting = false;
  result.status = state->success ? WorkerCommandStatus::kSucceeded
                                 : WorkerCommandStatus::kFailed;
  result.message = state->message;
  return result;
}

}  // namespace

WorkerCommandResult awaitWorkerCommand(
    const std::string &name, double timeout_sec,
    const std::shared_ptr<WorkerOperationToken> &token,
    const AsyncWorkerCommand &command,
    const std::function<bool()> &is_current,
    const AsyncWorkerCommand &rollback, double rollback_timeout_sec) {
  const auto effective_token =
      token ? token : std::make_shared<WorkerOperationToken>();
  WorkerCommandResult result = waitForCommand(
      name, timeout_sec, effective_token, command, is_current);
  if (result.success() || !rollback) return result;

  effective_token->cancel();
  result.rollback_attempted = true;
  const double effective_rollback_timeout =
      rollback_timeout_sec > 0.0 ? rollback_timeout_sec : timeout_sec;
  const auto rollback_token = std::make_shared<WorkerOperationToken>();
  const WorkerCommandResult rollback_result = waitForCommand(
      name + " rollback", effective_rollback_timeout, rollback_token,
      rollback, {});
  result.rollback_succeeded = rollback_result.success();
  result.message += "; rollback: " + rollback_result.message;
  if (!rollback_result.success()) {
    result.status = WorkerCommandStatus::kRollbackFailed;
  }
  return result;
}

WorkerClient::WorkerClient(rclcpp::Node *node, const std::string &service_name,
                           rclcpp::CallbackGroup::SharedPtr callback_group)
    : service_name_(service_name) {
  client_ = node->create_client<SetWorkerState>(
      service_name_, rmw_qos_profile_services_default, callback_group);
}

bool WorkerClient::serviceReady() const {
  return client_->service_is_ready();
}

void WorkerClient::start(uint64_t session_id, Completion completion) {
  start(session_id, WorkerStartContext{}, std::move(completion));
}

void WorkerClient::start(uint64_t session_id,
                         const WorkerStartContext &context,
                         Completion completion) {
  request(SetWorkerState::Request::START, session_id, context,
          std::move(completion));
}

void WorkerClient::stop(uint64_t session_id, Completion completion) {
  request(SetWorkerState::Request::STOP, session_id, WorkerStartContext{},
          std::move(completion));
}

void WorkerClient::stopConfirmed(uint64_t session_id, Completion completion) {
  request(SetWorkerState::Request::STOP, session_id, WorkerStartContext{},
          std::move(completion), true);
}

void WorkerClient::save(uint64_t session_id, const std::string &resource_path,
                        Completion completion) {
  WorkerStartContext context;
  context.resource_path = resource_path;
  request(SetWorkerState::Request::SAVE, session_id, context,
          std::move(completion));
}

void WorkerClient::request(uint8_t command, uint64_t session_id,
                           const WorkerStartContext &context,
                           Completion completion, bool require_stopped) {
  // 服务未就绪时立即返回，避免控制流程无限等待。
  if (!client_->wait_for_service(std::chrono::seconds(3))) {
    if (command == SetWorkerState::Request::STOP) {
      completion(true, service_name_ + " is not available; treated as stopped");
      return;
    }
    completion(false, service_name_ + " is not ready");
    return;
  }

  auto request = std::make_shared<SetWorkerState::Request>();
  request->command = command;
  request->session_id = session_id;
  request->resource_id = context.resource_id;
  request->resource_path = context.resource_path;
  request->use_initial_pose = context.use_initial_pose;
  request->algorithm_profile = context.algorithm_profile;

  // 异步响应继续驱动控制器中的下一步启动或停止操作。
  client_->async_send_request(
      request, [completion = std::move(completion), require_stopped](
                   rclcpp::Client<SetWorkerState>::SharedFuture future) {
        try {
          const auto response = future.get();
          const bool stopped =
              response->actual_state ==
              slam_manager_interfaces::msg::WorkerStatus::STOPPED;
          const bool success = response->success &&
                               (!require_stopped || stopped);
          completion(success, success || !require_stopped
                                  ? response->message
                                  : response->message + "; Worker is not stopped");
        } catch (const std::exception &error) {
          completion(false, error.what());
        }
      });
}

}  // namespace slam_manager
