#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager_interfaces/srv/set_worker_state.hpp"

namespace slam_manager {

// Worker启动所需的统一上下文。控制器按业务填充，算法Worker按需读取。
struct WorkerStartContext {
  std::string resource_id;
  std::string resource_path;
  bool use_initial_pose{false};
  std::string algorithm_profile;
};

// 共享给等待方和异步Controller的取消令牌。超时或新任务替代后，旧的
// Controller回调必须停止继续启动后续Worker。
class WorkerOperationToken {
 public:
  void cancel() { active_.store(false); }
  bool active() const { return active_.load(); }

 private:
  std::atomic_bool active_{true};
};

enum class WorkerCommandStatus {
  kSucceeded,
  kFailed,
  kTimedOut,
  kSuperseded,
  kRollbackFailed,
};

struct WorkerCommandResult {
  WorkerCommandStatus status{WorkerCommandStatus::kFailed};
  std::string message;
  bool rollback_attempted{false};
  bool rollback_succeeded{false};

  bool success() const {
    return status == WorkerCommandStatus::kSucceeded;
  }
};

using WorkerCompletion = std::function<void(bool, const std::string &)>;
using AsyncWorkerCommand = std::function<void(WorkerCompletion)>;

// 统一执行异步Worker命令：等待完成、检查任务有效性，并在失败、超时或
// 被替代时取消旧异步链及执行回滚。
WorkerCommandResult awaitWorkerCommand(
    const std::string &name, double timeout_sec,
    const std::shared_ptr<WorkerOperationToken> &token,
    const AsyncWorkerCommand &command,
    const std::function<bool()> &is_current = {},
    const AsyncWorkerCommand &rollback = {}, double rollback_timeout_sec = 0.0);

// 封装 Worker 启停服务的异步调用，供各业务控制器复用。
class WorkerClient {
 public:
  using SetWorkerState = slam_manager_interfaces::srv::SetWorkerState;
  using Completion = std::function<void(bool, const std::string &)>;

  WorkerClient(rclcpp::Node *node, const std::string &service_name,
               rclcpp::CallbackGroup::SharedPtr callback_group);

  void start(uint64_t session_id, Completion completion);
  void start(uint64_t session_id, const WorkerStartContext &context,
             Completion completion);
  void stop(uint64_t session_id, Completion completion);
  void save(uint64_t session_id, const std::string &resource_path,
            Completion completion);

 private:
  // 发送统一命令，并通过 completion 返回执行结果。
  void request(uint8_t command, uint64_t session_id,
               const WorkerStartContext &context, Completion completion);

  std::string service_name_;
  rclcpp::Client<SetWorkerState>::SharedPtr client_;
};

}  // namespace slam_manager
