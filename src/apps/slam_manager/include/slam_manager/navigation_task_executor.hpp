#pragma once

#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "slam_manager_interfaces/action/navigate_to_pose.hpp"
#include "slam_manager_interfaces/action/start_navigation.hpp"
#include "slam_manager_interfaces/msg/navigation_node.hpp"

#include "slam_manager/navigation_controller.hpp"
#include "slam_manager/slam_state_machine.hpp"
#include "slam_manager/task_topology.hpp"

namespace slam_manager {

// 执行APP导航任务：拓扑生成、逐点下发、到达等待、取消、超时和反馈。
class NavigationTaskExecutor {
 public:
  using StartNavigation = slam_manager_interfaces::action::StartNavigation;
  using StartNavigationGoalHandle =
      rclcpp_action::ServerGoalHandle<StartNavigation>;
  using NavigateToPose = slam_manager_interfaces::action::NavigateToPose;
  using NavigateToPoseGoalHandle =
      rclcpp_action::ServerGoalHandle<NavigateToPose>;

  struct Config {
    double control_timeout_sec{3.0};
    double worker_timeout_sec{30.0};
    double goal_timeout_sec{300.0};
  };

  struct Callbacks {
    std::function<void()> publish_mode;
    std::function<void(uint8_t)> publish_runtime_state;
    std::function<void(int32_t, const std::string &)> enter_error;
    std::function<bool(geometry_msgs::msg::PoseStamped &)> get_current_pose;
  };

  NavigationTaskExecutor(rclcpp::Node *node,
                         rclcpp::CallbackGroup::SharedPtr worker_callback_group,
                         SlamStateMachine *state_machine,
                         TopologyFileWriter *topology_writer, Config config,
                         Callbacks callbacks);
  ~NavigationTaskExecutor();

  bool validateGoal(const StartNavigation::Goal &goal,
                    std::string &message) const;
  bool validateGoal(const NavigateToPose::Goal &goal,
                    std::string &message) const;

  bool reserveOperation(std::string &message);
  // load_map成功后配置当前map_id专属的任务目录和地图目录。
  void configureMap(const std::filesystem::path &task_directory,
                    const std::filesystem::path &map_directory);
  void startPatrolAction(
      const std::shared_ptr<StartNavigationGoalHandle> &goal_handle,
      uint64_t session_id);
  void startNavigateToPoseAction(
      const std::shared_ptr<NavigateToPoseGoalHandle> &goal_handle,
      const geometry_msgs::msg::PoseStamped &current_pose,
      uint64_t session_id);
  void cancel();

  void onNavigationState(bool reached);
  bool setPaused(bool paused, std::string &message);

 private:
  void executePatrolAction(
      const std::shared_ptr<StartNavigationGoalHandle> &goal_handle,
      uint64_t session_id);
  void executeNavigateToPoseAction(
      const std::shared_ptr<NavigateToPoseGoalHandle> &goal_handle,
      geometry_msgs::msg::PoseStamped current_pose, uint64_t session_id);
  bool validatePose(const geometry_msgs::msg::Pose &pose,
                    std::string &message) const;
  bool validatePoseFields(double x, double y, double z, double theta,
                          std::string &message) const;
  static slam_manager_interfaces::msg::NavigationNode nodeFromPose(
      uint32_t node_id, const geometry_msgs::msg::Pose &pose);
  bool startWorkers(uint64_t session_id, std::string &message);
  bool stopWorkers(uint64_t session_id, std::string &message);
  bool waitForGoal(double timeout_sec,
                   const std::function<bool()> &is_canceling);
  bool waitForStopDuration(double duration_sec,
                           const std::function<bool()> &is_canceling);
  void fillCurrentPoseFeedback(StartNavigation::Feedback &feedback) const;
  void resetGoalState();
  void releaseOperation();

  rclcpp::Node *node_;
  SlamStateMachine *state_machine_;
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

}  // namespace slam_manager
