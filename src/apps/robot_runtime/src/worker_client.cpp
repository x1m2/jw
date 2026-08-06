#include "robot_runtime/worker_client.hpp"

#include <exception>
#include <memory>
#include <utility>

namespace robot_runtime {

WorkerClient::WorkerClient(
    rclcpp::Node *node, const std::string &service_name,
    rclcpp::CallbackGroup::SharedPtr callback_group)
    : service_name_(service_name) {
  client_ = node->create_client<SetWorkerState>(
      service_name_, rmw_qos_profile_services_default, callback_group);
}

void WorkerClient::start(uint64_t session_id, Completion completion) {
  start(session_id, {}, {}, false, std::move(completion));
}

void WorkerClient::start(uint64_t session_id, const std::string &resource_id,
                         const std::string &resource_path,
                         Completion completion) {
  start(session_id, resource_id, resource_path, false, std::move(completion));
}

void WorkerClient::start(uint64_t session_id, const std::string &resource_id,
                         const std::string &resource_path,
                         bool use_initial_pose,
                         Completion completion) {
  request(SetWorkerState::Request::START, session_id, resource_id,
          resource_path, use_initial_pose, std::move(completion));
}

void WorkerClient::stop(uint64_t session_id, Completion completion) {
  request(SetWorkerState::Request::STOP, session_id, {}, {}, false,
          std::move(completion));
}

void WorkerClient::request(uint8_t command, uint64_t session_id,
                           const std::string &resource_id,
                           const std::string &resource_path,
                           bool use_initial_pose,
                           Completion completion) {
  // 服务未就绪时立即返回，避免控制流程无限等待。
  if (!client_->service_is_ready()) {
    completion(false, service_name_ + " is not ready");
    return;
  }

  auto request = std::make_shared<SetWorkerState::Request>();
  request->command = command;
  request->session_id = session_id;
  request->resource_id = resource_id;
  request->resource_path = resource_path;
  request->use_initial_pose = use_initial_pose;

  // 异步响应继续驱动控制器中的下一步启动或停止操作。
  client_->async_send_request(
      request,
      [completion = std::move(completion)](
          rclcpp::Client<SetWorkerState>::SharedFuture future) {
        try {
          const auto response = future.get();
          completion(response->success, response->message);
        } catch (const std::exception &error) {
          completion(false, error.what());
        }
      });
}

}  // namespace robot_runtime
