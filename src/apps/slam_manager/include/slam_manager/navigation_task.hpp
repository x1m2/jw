#pragma once

#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <slam_manager_interfaces/msg/task_target.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "slam_manager/navigation.hpp"
#include "slam_manager/slam_mode.hpp"
#include "slam_manager_interfaces/action/navigate_to_pose.hpp"
#include "slam_manager_interfaces/action/start_navigation.hpp"
#include "slam_manager_interfaces/msg/navigation_node.hpp"

namespace slam_manager {

struct TaskEdgePolicy {
  double linear_velocity{0.5};
  double angular_velocity{0.5};
  int8_t motion_direction{1};
  bool obstacle_stop_enabled{true};
  bool obstacle_avoidance_enabled{false};
  double obstacle_avoidance_distance_m{0.5};
};

struct TaskTopologyEdge {
  uint32_t id{0U};
  uint32_t from_id{0U};
  uint32_t to_id{0U};
  std::string edge_type{"straight"};
  bool bidirectional{true};
  TaskEdgePolicy policy;
  bool has_bezier_control_points{false};
  std::array<std::array<double, 3>, 2> bezier_control_points{};
};

struct TaskRegionParameters {
  std::optional<double> linear_velocity;
  std::optional<double> angular_velocity;
  std::optional<double> obstacle_avoidance_distance_m;
};

struct TaskTopologyRegion {
  uint32_t id{0U};
  std::string region_type;
  bool enabled{true};
  std::vector<std::array<double, 2>> polygon;
  TaskRegionParameters parameters;
};

struct TaskTopologyGraph {
  std::vector<slam_manager_interfaces::msg::NavigationNode> nodes;
  std::vector<TaskTopologyEdge> edges;
  std::vector<TaskTopologyRegion> regions;
};

using DirectedEdgeIndex = std::unordered_map<uint64_t, std::size_t>;

uint64_t makeDirectedEdgeKey(uint32_t from_id, uint32_t to_id);
bool parseTaskJson(const std::filesystem::path &path, TaskTopologyGraph &graph,
                   DirectedEdgeIndex &directed_edge_indexes,
                   std::string &message,
                   const TaskEdgePolicy &default_edge_policy =
                       TaskEdgePolicy{});
const TaskTopologyEdge *findDirectEdge(
    const TaskTopologyGraph &graph,
    const DirectedEdgeIndex &directed_edge_indexes, uint32_t from_id,
    uint32_t to_id);

// 将合并后的任务拓扑原子保存为全局规划器使用的runtime JSON。
class TopologyFileWriter {
 public:
  TopologyFileWriter() = default;

  void setOutputFile(std::filesystem::path output_file);
  const std::filesystem::path &outputFile() const;
  bool write(
      const std::vector<slam_manager_interfaces::msg::NavigationNode> &nodes,
      std::string &message,
      const TaskEdgePolicy &policy = TaskEdgePolicy{}) const;
  bool write(const TaskTopologyGraph &graph, std::string &message) const;

 private:
  std::filesystem::path output_file_;
};

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
    bool enable_skip_first_target{true};
    double skip_first_target_threshold{0.8};
    TaskEdgePolicy default_edge_policy;
  };

  struct Callbacks {
    std::function<void()> publish_mode;
    std::function<void(uint8_t)> publish_runtime_state;
    std::function<void(int32_t, const std::string &)> enter_error;
    std::function<bool()> relocalization_succeeded;
    std::function<bool(std::string &)> navigation_allowed;
    std::function<bool(int32_t, std::string &)> set_robot_mode;
    std::function<bool(geometry_msgs::msg::PoseStamped &)> get_current_pose;
    std::function<bool(const slam_manager_interfaces::msg::NavigationNode &,
                       const std::function<bool()> &, std::string &)>
        relocalize;
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
      const geometry_msgs::msg::PoseStamped &current_pose, uint64_t session_id);
  void cancel();
  bool stopAndWait(uint64_t session_id, std::string &message);

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
  bool beginNavigationIfLocalized();
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

  rclcpp::Publisher<slam_manager_interfaces::msg::TaskTarget>::SharedPtr
      task_target_pub_;

  std::mutex operation_mutex_;
  bool operation_active_{false};
  std::mutex result_mutex_;
  std::condition_variable result_cv_;
  bool goal_armed_{false};
  bool goal_reached_{false};
  bool cancel_requested_{false};
  bool shutting_down_{false};
  std::mutex execution_mutex_;
  std::thread execution_thread_;
  std::filesystem::path task_directory_;
  std::filesystem::path topology_file_;
  std::filesystem::path map_directory_;
};

}  // namespace slam_manager
