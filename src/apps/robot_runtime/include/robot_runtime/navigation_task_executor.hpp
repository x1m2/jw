#pragma once

#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_interfaces/action/start_navigation.hpp"

#include "robot_runtime/navigation_controller.hpp"
#include "robot_runtime/robot_state_machine.hpp"
#include "robot_runtime/topology_file_writer.hpp"

namespace robot_runtime {

// 执行APP导航任务：拓扑生成、逐点下发、到达等待、取消、超时和反馈。
class NavigationTaskExecutor {
 public:
  using StartNavigation = robot_interfaces::action::StartNavigation;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StartNavigation>;

  struct Config {
    double control_timeout_sec{3.0};
    double worker_timeout_sec{30.0};
    double goal_timeout_sec{300.0};
  };

  struct Callbacks {
    std::function<void()> publish_mode;
    std::function<void(uint8_t)> publish_runtime_state;
    std::function<void(int32_t, const std::string &)> enter_error;
  };

  NavigationTaskExecutor(rclcpp::Node *node,
                         rclcpp::CallbackGroup::SharedPtr worker_callback_group,
                         RobotStateMachine *state_machine,
                         TopologyFileWriter *topology_writer, Config config,
                         Callbacks callbacks);
  ~NavigationTaskExecutor();

  bool validateGoal(const StartNavigation::Goal &goal,
                    std::string &message) const;
  bool validatePose(const geometry_msgs::msg::Pose &pose,
                    std::string &message) const;

  bool reserveOperation(std::string &message);
  // load_map成功后配置当前map_id专属的任务目录和地图目录。
  void configureMap(const std::filesystem::path &task_directory,
                    const std::filesystem::path &map_directory);
  void startAction(const std::shared_ptr<GoalHandle> &goal_handle,
                   uint64_t session_id);
  void cancel();

  bool navigateToPose(const geometry_msgs::msg::PoseStamped &current_pose,
                      const geometry_msgs::msg::Pose &target_pose,
                      uint64_t session_id, std::string &message);

  void onNavigationState(bool reached);
  bool setPaused(bool paused, std::string &message);

 private:
  void executeAction(const std::shared_ptr<GoalHandle> &goal_handle,
                     uint64_t session_id);
  bool startWorkers(uint64_t session_id, std::string &message);
  bool stopWorkers(uint64_t session_id, std::string &message);
  bool waitForGoal(const std::shared_ptr<GoalHandle> &goal_handle,
                   double timeout_sec);
  void resetGoalState();
  void releaseOperation();

  rclcpp::Node *node_;
  RobotStateMachine *state_machine_;
  TopologyFileWriter *topology_writer_;
  NavigationController controller_;
  Config config_;
  Callbacks callbacks_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_pose_pub_;

  std::mutex operation_mutex_;
  bool operation_active_{false};
  std::mutex result_mutex_;
  std::condition_variable result_cv_;
  bool goal_armed_{false};
  bool goal_reached_{false};
  bool cancel_requested_{false};
  bool shutting_down_{false};
  std::thread execution_thread_;
  std::filesystem::path task_directory_;
  std::filesystem::path topology_file_;
  std::filesystem::path map_directory_;
};

}  // namespace robot_runtime
