#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "slam_manager_interfaces/action/navigate_to_pose.hpp"
#include "slam_manager_interfaces/action/start_navigation.hpp"
#include "slam_manager_interfaces/msg/map_keyframe.hpp"
#include "slam_manager_interfaces/msg/localization_pose.hpp"
#include "slam_manager_interfaces/msg/navigation_node.hpp"
#include "slam_manager_interfaces/msg/robot_pose.hpp"
#include "slam_manager_interfaces/msg/runtime_state.hpp"
#include "slam_manager_interfaces/srv/delete_map.hpp"
#include "slam_manager_interfaces/srv/load_map.hpp"
#include "slam_manager_interfaces/srv/start_localization.hpp"
#include "slam_manager_interfaces/srv/start_mapping.hpp"

#include "slam_manager/localization_controller.hpp"
#include "slam_manager/bbs_voxel_generator.hpp"
#include "slam_manager/map_storage.hpp"
#include "slam_manager/mapping_controller.hpp"
#include "slam_manager/navigation_task_executor.hpp"
#include "slam_manager/slam_state_machine.hpp"
#include "slam_manager/task_topology.hpp"

namespace slam_manager {

// 严格按照APP协议提供机器人控制接口，算法私有Topic和Worker服务不对APP暴露。
class SlamManagerNode : public rclcpp::Node {
 public:
  SlamManagerNode();
  ~SlamManagerNode() override;

 private:
  using Trigger = std_srvs::srv::Trigger;
  using DeleteMap = slam_manager_interfaces::srv::DeleteMap;
  using LoadMap = slam_manager_interfaces::srv::LoadMap;
  using LocalizationPose = slam_manager_interfaces::msg::LocalizationPose;
  using MapKeyframe = slam_manager_interfaces::msg::MapKeyframe;
  using NavigateToPose = slam_manager_interfaces::action::NavigateToPose;
  using RobotPose = slam_manager_interfaces::msg::RobotPose;
  using RuntimeState = slam_manager_interfaces::msg::RuntimeState;
  using StartLocalization = slam_manager_interfaces::srv::StartLocalization;
  using StartMapping = slam_manager_interfaces::srv::StartMapping;
  using StartNavigation = slam_manager_interfaces::action::StartNavigation;
  using StartNavigationGoalHandle =
      rclcpp_action::ServerGoalHandle<StartNavigation>;
  using NavigateToPoseGoalHandle =
      rclcpp_action::ServerGoalHandle<NavigateToPose>;

  // 协议2.1～2.5：地图生命周期和重定位Service入口。
  void startMappingCallback(const StartMapping::Request::SharedPtr request,
                            StartMapping::Response::SharedPtr response);
  void finishMappingCallback(const Trigger::Request::SharedPtr request,
                             Trigger::Response::SharedPtr response);
  void deleteMapCallback(const DeleteMap::Request::SharedPtr request,
                         DeleteMap::Response::SharedPtr response);
  void loadMapCallback(const LoadMap::Request::SharedPtr request,
                       LoadMap::Response::SharedPtr response);
  void startLocalizationCallback(
      const StartLocalization::Request::SharedPtr request,
      StartLocalization::Response::SharedPtr response);
  // 协议2.8～2.9：暂停和继续当前导航Action。
  void pauseNavigationCallback(const Trigger::Request::SharedPtr request,
                               Trigger::Response::SharedPtr response);
  void resumeNavigationCallback(const Trigger::Request::SharedPtr request,
                                Trigger::Response::SharedPtr response);

  // 协议2.6：巡检导航Action的接收、取消和异步执行入口。
  rclcpp_action::GoalResponse handleNavigationGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const StartNavigation::Goal> goal);
  rclcpp_action::CancelResponse handleNavigationCancel(
      const std::shared_ptr<StartNavigationGoalHandle> goal_handle);
  void handleNavigationAccepted(
      const std::shared_ptr<StartNavigationGoalHandle> goal_handle);

  // 协议2.7：单点导航Action的接收、取消和异步执行入口。
  rclcpp_action::GoalResponse handleNavigateToPoseGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handleNavigateToPoseCancel(
      const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle);
  void handleNavigateToPoseAccepted(
      const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle);

  // 算法内部结果及协议2.10～2.12数据Topic的转发回调。
  void relocalizationResultCallback(
      const std_msgs::msg::Bool::SharedPtr message);
  void navigationStateCallback(const std_msgs::msg::Bool::SharedPtr message);
  void keyframeCloudCallback(const MapKeyframe::SharedPtr message);
  void currentPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr message);
  void localizationPoseCallback(const LocalizationPose::SharedPtr message);
  void publishCurrentPose(const geometry_msgs::msg::PoseStamped &pose,
                          float match_score);
  void bodyCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr message);

  // 协议2.13：内部状态机到RuntimeState消息的统一出口。
  void publishMode();
  void publishRuntimeState(uint8_t state, int32_t error_code = 0,
                           const std::string &error_message = "");
  void enterError(int32_t error_code, const std::string &error_message);

  SlamStateMachine state_machine_;
  uint64_t session_id_{0U};
  double start_mapping_timeout_sec_{30.0};
  double finish_mapping_timeout_sec_{300.0};
  double relocalization_timeout_sec_{60.0};
  double localization_stop_timeout_sec_{10.0};
  std::string active_map_id_;
  std::filesystem::path active_map_staging_path_;
  std::string selected_map_id_;

  std::mutex localization_command_mutex_;
  std::mutex localization_result_mutex_;
  std::condition_variable localization_result_cv_;
  uint64_t localization_attempt_session_id_{0U};
  bool localization_attempt_active_{false};
  bool localization_result_ready_{false};
  bool localization_result_success_{false};
  std::string localization_result_message_;

  std::mutex current_pose_mutex_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_current_pose_{false};

  std::mutex runtime_state_mutex_;
  int32_t error_code_{RuntimeState::NO_ERROR};
  std::string error_message_;

  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::CallbackGroup::SharedPtr localization_control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr localization_result_callback_group_;
  rclcpp::CallbackGroup::SharedPtr navigation_result_callback_group_;
  std::unique_ptr<MappingController> mapping_controller_;
  std::unique_ptr<LocalizationController> localization_controller_;
  std::unique_ptr<MapStorage> map_storage_;
  std::unique_ptr<BbsVoxelGenerator> bbs_voxel_generator_;
  std::unique_ptr<TopologyFileWriter> topology_file_writer_;
  std::unique_ptr<NavigationTaskExecutor> navigation_task_executor_;

  rclcpp::Service<StartMapping>::SharedPtr start_mapping_service_;
  rclcpp::Service<Trigger>::SharedPtr finish_mapping_service_;
  rclcpp::Service<LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<DeleteMap>::SharedPtr delete_map_service_;
  rclcpp::Service<StartLocalization>::SharedPtr start_localization_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_navigation_service_;
  rclcpp_action::Server<StartNavigation>::SharedPtr start_navigation_action_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_action_;

  rclcpp::Publisher<RuntimeState>::SharedPtr runtime_state_pub_;
  rclcpp::Publisher<MapKeyframe>::SharedPtr keyframe_cloud_pub_;
  rclcpp::Publisher<RobotPose>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr body_cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_pub_;

  rclcpp::Subscription<MapKeyframe>::SharedPtr keyframe_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      mapping_pose_sub_;
  rclcpp::Subscription<LocalizationPose>::SharedPtr localization_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      body_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      relocalization_result_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_state_sub_;
};

}  // namespace slam_manager
