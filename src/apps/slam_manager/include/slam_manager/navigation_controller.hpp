#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "slam_manager/worker_client.hpp"

namespace slam_manager {

// 统一编排导航链路中的各个 Worker，并处理启动失败回滚。
class NavigationController {
 public:
  using Completion = std::function<void(bool, const std::string &)>;

  NavigationController(rclcpp::Node *node,
                       rclcpp::CallbackGroup::SharedPtr callback_group);

  void start(uint64_t session_id, const std::string &topology_file,
             const std::string &map_directory, Completion completion);
  void stop(uint64_t session_id, Completion completion);
  // 暂停只控制底盘速度输出，不停止或析构导航算法。
  bool setPaused(bool paused, double timeout_sec, std::string &message);

 private:
  using WorkerList = std::vector<WorkerClient *>;

  // 按 start_order_ 逐个启动，确保上游模块先就绪。
  void startNext(uint64_t session_id, std::size_t index,
                 const std::string &topology_file,
                 const std::string &map_directory,
                 std::shared_ptr<WorkerList> started,
                 Completion completion);
  // 启动中途失败时，逆序停止本次已经启动的 Worker。
  void rollbackStart(uint64_t session_id,
                     std::shared_ptr<WorkerList> started,
                     std::string failure, Completion completion);
  // 按 stop_order_ 逐个停止，并汇总所有停止错误。
  void stopNext(uint64_t session_id, std::size_t index,
                std::shared_ptr<std::vector<std::string>> errors,
                Completion completion);

  WorkerClient obstacle_stop_worker_;
  WorkerClient costmap_worker_;
  WorkerClient global_planner_worker_;
  WorkerClient path_smoother_worker_;
  WorkerClient pure_pursuit_worker_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr pause_client_;
  // 启停顺序集中维护，便于明确导航模块之间的依赖关系。
  WorkerList start_order_;
  WorkerList stop_order_;
};

}  // namespace slam_manager
