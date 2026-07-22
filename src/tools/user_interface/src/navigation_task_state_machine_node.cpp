#include <locale.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "robot_runtime/managed_worker.hpp"
#include "user_interface/navigation_task_state_machine.hpp"

/**
 * @brief 信号处理函数
 */
void signalHandler(int signum) {
  RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
              "收到信号 %d，正在关闭导航任务状态机节点...", signum);

  rclcpp::shutdown();
}

/**
 * @brief 导航任务状态机节点主函数
 */
int main(int argc, char** argv) {
  // 设置中文编码
  setlocale(LC_ALL, "");

  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 注册信号处理
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
              "启动导航任务状态机节点");

  try {
    auto worker = std::make_shared<robot_runtime::ManagedWorker>(
        "navigation_task_worker", []() {
          auto state_machine =
              std::make_shared<user_interface::NavigationTaskStateMachine>();
          if (!state_machine->initialize()) {
            throw std::runtime_error("导航任务状态机初始化失败");
          }
          robot_runtime::ManagedWorker::AlgorithmInstance instance;
          instance.node = state_machine;
          return instance;
        });
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("navigation_task_state_machine_node"),
                 "节点运行异常: %s", e.what());

    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
              "导航任务状态机节点正常退出");

  return 0;
}
