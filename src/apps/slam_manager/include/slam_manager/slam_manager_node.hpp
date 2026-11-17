#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <Eigen/Core>
#include <filesystem>
#include <functional>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "slam_manager/localization.hpp"
#include "slam_manager/map_check.hpp"
#include "slam_manager/mapping.hpp"
#include "slam_manager/navigation_task.hpp"
#include "slam_manager/slam_mode.hpp"
#include "slam_manager/uwb_follow.hpp"
#include "slam_manager_interfaces/action/navigate_to_pose.hpp"
#include "slam_manager_interfaces/action/start_localization.hpp"
#include "slam_manager_interfaces/action/start_navigation.hpp"
#include "slam_manager_interfaces/msg/localization_pose.hpp"
#include "slam_manager_interfaces/msg/map_keyframe.hpp"
#include "slam_manager_interfaces/msg/navigation_obstacle_state.hpp"
#include "slam_manager_interfaces/msg/navigation_node.hpp"
#include "slam_manager_interfaces/msg/robot_pose.hpp"
#include "slam_manager_interfaces/msg/runtime_state.hpp"
#include "slam_manager_interfaces/srv/delete_map.hpp"
#include "slam_manager_interfaces/srv/calibrate_lidar_tilt.hpp"
#include "slam_manager_interfaces/srv/load_map.hpp"
#include "slam_manager_interfaces/srv/start_mapping.hpp"
#include "woodox_ros2_interface/srv/get_state.hpp"
#include "woodox_ros2_interface/srv/set_mode.hpp"

namespace slam_manager {

// 严格按照APP协议提供机器人控制接口，算法私有Topic和Worker服务不对APP暴露。
class SlamManagerNode : public rclcpp::Node {
 public:
  SlamManagerNode();
  ~SlamManagerNode() override;

 private:
  using Trigger = std_srvs::srv::Trigger;
  using CalibrateLidarTilt = slam_manager_interfaces::srv::CalibrateLidarTilt;
  using DeleteMap = slam_manager_interfaces::srv::DeleteMap;
  using LoadMap = slam_manager_interfaces::srv::LoadMap;
  using LocalizationPose = slam_manager_interfaces::msg::LocalizationPose;
  using MapKeyframe = slam_manager_interfaces::msg::MapKeyframe;
  using NavigateToPose = slam_manager_interfaces::action::NavigateToPose;
  using NavigationObstacleState =
      slam_manager_interfaces::msg::NavigationObstacleState;
  using RobotPose = slam_manager_interfaces::msg::RobotPose;
  using RuntimeState = slam_manager_interfaces::msg::RuntimeState;
  using StartLocalization =
      slam_manager_interfaces::action::StartLocalization;
  using StartMapping = slam_manager_interfaces::srv::StartMapping;
  using StartNavigation = slam_manager_interfaces::action::StartNavigation;
  using GetState = woodox_ros2_interface::srv::GetState;
  using SetMode = woodox_ros2_interface::srv::SetMode;
  using StartNavigationGoalHandle =
      rclcpp_action::ServerGoalHandle<StartNavigation>;
  using StartLocalizationGoalHandle =
      rclcpp_action::ServerGoalHandle<StartLocalization>;
  using NavigateToPoseGoalHandle =
      rclcpp_action::ServerGoalHandle<NavigateToPose>;

  enum class RelocalizationStatus {
    kSucceeded,
    kFailed,
    kTimedOut,
    kCancelled,
    kSuperseded,
  };

  struct RelocalizationOptions {
    std::string source;
    std::string map_id;
    std::filesystem::path map_path;
    bool use_initial_pose{false};
    std::string algorithm_profile{"initial"};
    geometry_msgs::msg::Pose initial_pose;
    uint64_t generation{0U};
    std::shared_ptr<WorkerOperationToken> operation_token;
    bool stop_existing_worker{false};
    bool preserve_navigation_state{false};
    std::function<bool()> is_canceling;
    std::function<void(const std::string &)> publish_feedback;
  };

  struct RelocalizationResult {
    RelocalizationStatus status{RelocalizationStatus::kFailed};
    uint64_t session_id{0U};
    std::string message;

    bool success() const {
      return status == RelocalizationStatus::kSucceeded;
    }
  };

  // 协议2.1～2.5：地图生命周期Service入口。
  void startMappingCallback(const StartMapping::Request::SharedPtr request,
                            StartMapping::Response::SharedPtr response);
  void finishMappingCallback(const Trigger::Request::SharedPtr request,
                             Trigger::Response::SharedPtr response);
  void cancelMappingCallback(const Trigger::Request::SharedPtr request,
                             Trigger::Response::SharedPtr response);
  void deleteMapCallback(const DeleteMap::Request::SharedPtr request,
                         DeleteMap::Response::SharedPtr response);
  void loadMapCallback(const LoadMap::Request::SharedPtr request,
                       LoadMap::Response::SharedPtr response);
  RelocalizationResult runRelocalizationAttempt(
      const RelocalizationOptions &options,
      std::unique_lock<std::mutex> lifecycle_lock);
  bool relocalizeForMapConnection(
      const slam_manager_interfaces::msg::NavigationNode &node,
      const std::function<bool()> &is_canceling, std::string &message);

  // 协议2.6：重定位Action的接收、取消和异步执行入口。
  rclcpp_action::GoalResponse handleLocalizationGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const StartLocalization::Goal> goal);
  rclcpp_action::CancelResponse handleLocalizationCancel(
      const std::shared_ptr<StartLocalizationGoalHandle> goal_handle);
  void handleLocalizationAccepted(
      const std::shared_ptr<StartLocalizationGoalHandle> goal_handle);
  void executeLocalizationAction(
      const std::shared_ptr<StartLocalizationGoalHandle> goal_handle);
  // 协议2.8～2.9：暂停和继续当前导航Action。
  void pauseNavigationCallback(const Trigger::Request::SharedPtr request,
                               Trigger::Response::SharedPtr response);
  void resumeNavigationCallback(const Trigger::Request::SharedPtr request,
                                Trigger::Response::SharedPtr response);
  void uwbFollowCommandCallback(const std_msgs::msg::Int8::ConstSharedPtr cmd);
  bool startUwbFollow(uint64_t generation);
  bool pauseUwbFollow();
  bool resumeUwbFollow();
  bool stopUwbFollow();
  bool sendUwbPlannerCommand(int8_t command);
  bool stopUwbFollowWorkers(std::string &message);

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

  void calibrateLidarTiltCallback(
      const CalibrateLidarTilt::Request::SharedPtr request,
      CalibrateLidarTilt::Response::SharedPtr response);
  bool pauseNavigationLocked(std::string &message);
  bool robotStateAllowsNavigation(std::string &message);
  void pollRobotState();
  void handlePostureState(bool stopped);
  void setPostureStop(bool stopped);
  void obstacleHandlingStateCallback(
      const std_msgs::msg::UInt8::ConstSharedPtr message);
  void publishNavigationObstacleState();
  bool setRobotMode(int32_t mode, std::string &message);

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
  bool stopAllAlgorithms(std::string &message);
  bool clearRuntimeAfterTeardown(std::string &message);
  uint64_t beginLifecycleCommand(const char *command_name);
  bool isLatestCommand(uint64_t generation) const;
  std::shared_ptr<WorkerOperationToken> installLifecycleToken(
      uint64_t generation);
  bool isLifecycleCommandActive(
      uint64_t generation,
      const std::shared_ptr<WorkerOperationToken> &token) const;
  bool prepareForStartCommand(
      uint64_t generation,
      const std::shared_ptr<WorkerOperationToken> &token,
      std::string &message);
  std::string recoverLocalizationActionException(const std::string &message);
  void abortLocalizationActionAfterException(
      const std::shared_ptr<StartLocalizationGoalHandle> &goal_handle,
      const std::string &message) noexcept;

  SlamStateMachine state_machine_;
  uint64_t session_id_{0U};
  // 只允许一个生命周期命令执行“析构旧算法 -> 启动新算法”。
  std::mutex lifecycle_mutex_;
  std::atomic<uint64_t> command_generation_{0U};
  std::mutex lifecycle_token_mutex_;
  std::shared_ptr<WorkerOperationToken> active_lifecycle_token_;
  double start_mapping_timeout_sec_{30.0};
  double finish_mapping_timeout_sec_{300.0};
  double relocalization_timeout_sec_{60.0};
  double localization_tracking_pose_timeout_sec_{2.0};
  double localization_stop_timeout_sec_{10.0};
  std::string imu_calibration_topic_;
  bool imu_calibration_config_valid_{false};
  std::size_t imu_calibration_sample_count_{200U};
  double imu_calibration_timeout_sec_{5.0};
  double imu_calibration_max_gyro_rms_{0.05};
  double imu_calibration_max_accel_relative_std_{0.02};
  Eigen::Matrix3d imu_from_lidar_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d calibration_base_from_lidar_{Eigen::Matrix3d::Identity()};
  std::string calibration_rotation_type_;
  double calibration_yaw_rad_{0.0};
  bool enable_set_mode_{true};
  double body_cloud_publish_hz_{5.0};
  double last_body_cloud_publish_time_{-1.0};
  std::string active_map_id_;
  std::filesystem::path active_map_path_;
  bool active_map_directory_prepared_{false};
  std::string selected_map_id_;

  std::mutex localization_result_mutex_;
  std::condition_variable localization_result_cv_;
  std::atomic<uint64_t> active_localization_session_id_{0U};
  uint64_t localization_attempt_session_id_{0U};
  bool localization_attempt_active_{false};
  bool localization_result_armed_{false};
  bool localization_match_succeeded_{false};
  bool localization_result_ready_{false};
  bool localization_result_success_{false};
  uint64_t localization_pose_sequence_{0U};
  uint64_t localization_match_pose_sequence_{0U};
  std::string localization_result_message_;
  std::atomic_bool relocalization_succeeded_{false};
  std::atomic_bool localization_action_active_{false};
  std::atomic_bool shutting_down_{false};
  std::atomic_bool state_req_pending_{false};
  std::atomic<int64_t> state_req_started_ns_{0};
  std::atomic<uint64_t> state_req_seq_{0U};
  std::mutex localization_action_thread_mutex_;
  std::thread localization_action_thread_;

  std::mutex current_pose_mutex_;
  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_current_pose_{false};

  std::mutex runtime_state_mutex_;
  int32_t error_code_{RuntimeState::NO_ERROR};
  std::string error_message_;
  bool runtime_state_published_{false};
  uint8_t published_runtime_state_{RuntimeState::IDLE};
  int32_t published_runtime_error_code_{RuntimeState::NO_ERROR};
  std::string published_runtime_error_message_;

  std::mutex navigation_obstacle_state_mutex_;
  uint8_t obstacle_handling_state_{NavigationObstacleState::UNBLOCKED};
  int published_navigation_obstacle_state_{-1};

  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lifecycle_control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr localization_result_callback_group_;
  rclcpp::CallbackGroup::SharedPtr navigation_result_callback_group_;
  rclcpp::CallbackGroup::SharedPtr robot_control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr imu_calibration_callback_group_;
  std::unique_ptr<MappingController> mapping_controller_;
  std::unique_ptr<LocalizationController> localization_controller_;
  std::unique_ptr<UwbFollowController> uwb_follow_controller_;
  std::unique_ptr<MapStorage> map_storage_;
  std::unique_ptr<BbsVoxelGenerator> bbs_voxel_generator_;
  std::unique_ptr<TopologyFileWriter> topology_file_writer_;
  std::unique_ptr<NavigationTaskExecutor> navigation_task_executor_;

  rclcpp::Service<StartMapping>::SharedPtr start_mapping_service_;
  rclcpp::Service<Trigger>::SharedPtr finish_mapping_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_mapping_service_;
  rclcpp::Service<LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<DeleteMap>::SharedPtr delete_map_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_navigation_service_;
  rclcpp::Service<CalibrateLidarTilt>::SharedPtr
      calibrate_lidar_tilt_service_;
  rclcpp::Client<GetState>::SharedPtr get_robot_state_client_;
  rclcpp::Client<SetMode>::SharedPtr set_robot_mode_client_;
  rclcpp::TimerBase::SharedPtr robot_state_timer_;
  rclcpp_action::Server<StartNavigation>::SharedPtr start_navigation_action_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_to_pose_action_;
  rclcpp_action::Server<StartLocalization>::SharedPtr
      start_localization_action_;

  rclcpp::Publisher<RuntimeState>::SharedPtr runtime_state_pub_;
  rclcpp::Publisher<NavigationObstacleState>::SharedPtr
      navigation_obstacle_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr posture_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr uwb_planner_command_pub_;
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
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr
      obstacle_handling_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr uwb_command_sub_;
};

}  // namespace slam_manager
