#include <locale.h>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>

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
    // 创建状态机节点
    auto state_machine =
        std::make_shared<user_interface::NavigationTaskStateMachine>();

    // 初始化
    if (!state_machine->initialize()) {
      RCLCPP_ERROR(rclcpp::get_logger("navigation_task_state_machine_node"),
                   "导航任务状态机初始化失败");

      return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "导航任务状态机节点初始化完成");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "等待导航任务信号...");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "任务信号说明(std_msgs/msg/String):");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "  <task_name>       - 执行指定任务");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "  LOOP:<task_name>  - 循环执行指定任务");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "  STOP              - 停止任务");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "  CONTINUE          - 继续执行任务");

    RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
                "  GO_HOME           - 回家");

    // ROS2 spin
    rclcpp::spin(state_machine);

    rclcpp::shutdown();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("navigation_task_state_machine_node"),
                 "节点运行异常: %s", e.what());

    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("navigation_task_state_machine_node"),
              "导航任务状态机节点正常退出");

  return 0;
}