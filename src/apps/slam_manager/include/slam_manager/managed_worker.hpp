#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager_interfaces/msg/worker_status.hpp"
#include "slam_manager_interfaces/srv/set_worker_state.hpp"

namespace slam_manager {

// 管理单个算法对象的创建、运行和销毁，供 platform 模式按需启停算法。
class ManagedWorker : public rclcpp::Node {
 public:
  // 同时保存 ROS 节点、算法对象所有权和停止前的安全处理函数。
  struct AlgorithmInstance {
    std::shared_ptr<rclcpp::Node> node;
    std::shared_ptr<void> owner;
    std::function<void()> before_stop;
  };

  // 启动上下文由 SlamManager 生成，算法不会直接接收 APP 文件路径。
  struct StartContext {
    std::string resource_id;
    std::string resource_path;
    bool use_initial_pose{false};
  };

  using AlgorithmFactory =
      std::function<AlgorithmInstance(const StartContext &)>;
  using LegacyAlgorithmFactory = std::function<AlgorithmInstance()>;
  using WorkerStatus = slam_manager_interfaces::msg::WorkerStatus;
  using SetWorkerState = slam_manager_interfaces::srv::SetWorkerState;

  ManagedWorker(const std::string &worker_name, LegacyAlgorithmFactory factory,
                std::size_t executor_threads = 4U)
      : ManagedWorker(
            worker_name,
            [factory = std::move(factory)](const StartContext &) {
              return factory();
            },
            executor_threads) {}

  ManagedWorker(const std::string &worker_name, AlgorithmFactory factory,
                std::size_t executor_threads = 4U)
      : Node(worker_name),
        worker_name_(worker_name),
        factory_(std::move(factory)),
        executor_threads_(executor_threads) {
    runtime_profile_ =
        declare_parameter<std::string>("runtime_profile", "platform");
    if (runtime_profile_ != "test" && runtime_profile_ != "platform") {
      throw std::invalid_argument(
          "runtime_profile must be either 'test' or 'platform'");
    }

    // 每个 Worker 使用私有服务接收统一的启动/停止命令。
    state_service_ = create_service<SetWorkerState>(
        "~/set_state",
        std::bind(&ManagedWorker::stateCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
    status_pub_ = create_publisher<WorkerStatus>(
        "/slam_manager/worker_status", 10);
    status_timer_ = create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&ManagedWorker::publishStatus, this));

    RCLCPP_INFO(get_logger(), "Runtime profile: %s",
                runtime_profile_.c_str());

    // 测试模式保持原 launch 行为：节点启动后自动创建算法对象。
    if (runtime_profile_ == "test") {
      autostart_timer_ = create_wall_timer(
          std::chrono::milliseconds(100), [this]() {
            autostart_timer_->cancel();
            std::string message;
            if (!startAlgorithm(1U, {}, message)) {
              RCLCPP_ERROR(get_logger(), "Autostart failed: %s",
                           message.c_str());
            }
            publishStatus();
          });
    }
  }

  ~ManagedWorker() override { stopAlgorithm(); }

 private:
  // 处理上层管理节点发来的 Worker 状态切换请求。
  void stateCallback(const SetWorkerState::Request::SharedPtr request,
                     SetWorkerState::Response::SharedPtr response) {
    std::string message;
    if (request->command == SetWorkerState::Request::START) {
      const StartContext context{request->resource_id, request->resource_path,
                                 request->use_initial_pose};
      response->success =
          startAlgorithm(request->session_id, context, message);
    } else if (request->command == SetWorkerState::Request::STOP) {
      response->success = stopAlgorithm(request->session_id, &message);
    } else {
      response->success = false;
      message = "unknown command";
    }
    response->actual_state = state_.load();
    response->message = message;
    publishStatus();
  }

  bool startAlgorithm(uint64_t session_id, const StartContext &context,
                      std::string &message) {
    std::scoped_lock lock(mutex_);
    if (algorithm_instance_.node) {
      const bool same_session = session_id_.load() == session_id;
      message = same_session ? "algorithm is already running"
                             : "algorithm is running in another session";
      return same_session;
    }

    state_.store(WorkerStatus::STARTING);
    try {
      // 工厂只在收到启动命令时创建实际算法对象。
      algorithm_instance_ = factory_(context);
      if (!algorithm_instance_.node) {
        throw std::runtime_error("algorithm factory returned a null node");
      }
      // 算法节点在独立执行器线程中运行，不阻塞 Worker 控制服务。
      algorithm_executor_ =
          std::make_unique<rclcpp::executors::MultiThreadedExecutor>(
              rclcpp::ExecutorOptions(), executor_threads_);
      algorithm_executor_->add_node(algorithm_instance_.node);
      auto *executor = algorithm_executor_.get();
      algorithm_thread_ = std::thread([executor]() { executor->spin(); });

      session_id_.store(session_id);
      resource_id_ = context.resource_id;
      state_.store(WorkerStatus::RUNNING);
      message = "algorithm started";
      RCLCPP_INFO(get_logger(), "Algorithm started, session=%lu",
                  static_cast<unsigned long>(session_id));
      return true;
    } catch (const std::exception &error) {
      algorithm_instance_ = {};
      algorithm_executor_.reset();
      state_.store(WorkerStatus::ERROR);
      message = error.what();
      RCLCPP_ERROR(get_logger(), "Failed to start algorithm: %s",
                   error.what());
      return false;
    }
  }

  bool stopAlgorithm(uint64_t requested_session_id = 0U,
                     std::string *message = nullptr) {
    std::unique_lock lock(mutex_);
    if (!algorithm_instance_.node) {
      state_.store(WorkerStatus::STOPPED);
      if (message) *message = "algorithm is already stopped";
      return true;
    }

    const uint64_t running_session_id = session_id_.load();
    if (requested_session_id != 0U &&
        requested_session_id != running_session_id) {
      if (message) {
        *message = "stale stop ignored: requested_session=" +
                   std::to_string(requested_session_id) +
                   ", running_session=" +
                   std::to_string(running_session_id);
      }
      RCLCPP_WARN(get_logger(),
                  "Ignoring stale STOP: requested_session=%lu, "
                  "running_session=%lu",
                  static_cast<unsigned long>(requested_session_id),
                  static_cast<unsigned long>(running_session_id));
      return true;
    }

    state_.store(WorkerStatus::STOPPING);
    bool stop_hook_ok = true;
    std::string stop_hook_error;
    // 先执行安全停止钩子，例如取消控制定时器并发布零速度。
    if (algorithm_instance_.before_stop) {
      try {
        algorithm_instance_.before_stop();
      } catch (const std::exception &error) {
        stop_hook_ok = false;
        stop_hook_error = error.what();
        RCLCPP_ERROR(get_logger(), "Algorithm stop hook failed: %s",
                     error.what());
      }
    }
    // 退出算法执行器后再释放对象，避免回调访问已销毁资源。
    algorithm_executor_->cancel();
    lock.unlock();
    if (algorithm_thread_.joinable()) algorithm_thread_.join();
    lock.lock();

    algorithm_executor_->remove_node(algorithm_instance_.node);
    algorithm_instance_.before_stop = {};
    algorithm_instance_.owner.reset();
    algorithm_instance_.node.reset();
    algorithm_executor_.reset();
    session_id_.store(0U);
    resource_id_.clear();
    state_.store(WorkerStatus::STOPPED);
    if (message) {
      *message = stop_hook_ok ? "algorithm stopped"
                              : "algorithm stopped, stop hook failed: " +
                                    stop_hook_error;
    }
    RCLCPP_INFO(get_logger(), "Algorithm stopped");
    return stop_hook_ok;
  }

  void publishStatus() {
    // 周期发布 Worker 状态，方便上层应用监控算法是否就绪。
    WorkerStatus status;
    status.stamp = now();
    status.worker_name = worker_name_;
    status.state = state_.load();
    status.session_id = session_id_.load();
    status.ready = status.state == WorkerStatus::RUNNING;
    status.error_code = status.state == WorkerStatus::ERROR ? 1 : 0;
    status.message = resource_id_.empty()
                         ? runtime_profile_
                         : runtime_profile_ + ":" + resource_id_;
    status_pub_->publish(status);
  }

  std::string worker_name_;
  std::string runtime_profile_;
  std::string resource_id_;
  AlgorithmFactory factory_;
  std::size_t executor_threads_;
  std::mutex mutex_;
  std::atomic<uint8_t> state_{WorkerStatus::STOPPED};
  std::atomic<uint64_t> session_id_{0U};
  AlgorithmInstance algorithm_instance_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor>
      algorithm_executor_;
  std::thread algorithm_thread_;
  rclcpp::Service<SetWorkerState>::SharedPtr state_service_;
  rclcpp::Publisher<WorkerStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace slam_manager
