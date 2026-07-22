#pragma once

#include <cstdint>
#include <cstddef>
#include <condition_variable>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "robot_interfaces/srv/delete_map.hpp"
#include "robot_interfaces/srv/load_map.hpp"
#include "robot_interfaces/srv/start_mapping.hpp"

#include "robot_runtime/localization_controller.hpp"
#include "robot_runtime/map_publisher.hpp"
#include "robot_runtime/mapping_controller.hpp"
#include "robot_runtime/navigation_controller.hpp"
#include "robot_runtime/robot_state_machine.hpp"

namespace robot_runtime {

// 对外提供机器人模式切换服务，并协调建图、定位和导航控制器。
class RobotRuntimeNode : public rclcpp::Node {
 public:
  RobotRuntimeNode();

 private:
  using Trigger = std_srvs::srv::Trigger;
  using DeleteMap = robot_interfaces::srv::DeleteMap;
  using LoadMap = robot_interfaces::srv::LoadMap;
  using StartMapping = robot_interfaces::srv::StartMapping;

  void startMappingCallback(const StartMapping::Request::SharedPtr request,
                            StartMapping::Response::SharedPtr response);
  void stopMappingCallback(const Trigger::Request::SharedPtr request,
                           Trigger::Response::SharedPtr response);
  void resetErrorCallback(const Trigger::Request::SharedPtr request,
                          Trigger::Response::SharedPtr response);
  void loadMapCallback(const LoadMap::Request::SharedPtr request,
                       LoadMap::Response::SharedPtr response);
  void startLocalizationCallback(const Trigger::Request::SharedPtr request,
                                 Trigger::Response::SharedPtr response);
  void stopLocalizationCallback(const Trigger::Request::SharedPtr request,
                                Trigger::Response::SharedPtr response);
  void relocalizationResultCallback(
      const std_msgs::msg::Bool::SharedPtr message);
  void deleteMapCallback(const DeleteMap::Request::SharedPtr request,
                         DeleteMap::Response::SharedPtr response);
  void startNavigationCallback(const Trigger::Request::SharedPtr request,
                               Trigger::Response::SharedPtr response);
  void stopNavigationCallback(const Trigger::Request::SharedPtr request,
                              Trigger::Response::SharedPtr response);
  void pauseNavigationCallback(const Trigger::Request::SharedPtr request,
                               Trigger::Response::SharedPtr response);
  void resumeNavigationCallback(const Trigger::Request::SharedPtr request,
                                Trigger::Response::SharedPtr response);
  bool validateMapId(const std::string &map_id, std::string &message) const;
  bool deleteMapDirectory(const std::string &map_id, std::string &message);
  bool countSavedMaps(std::size_t &count, std::string &message) const;
  bool finalizeActiveMap(std::string &message);
  std::filesystem::path mapPath(const std::string &map_id) const;
  void publishMode();

  // 状态机负责限制非法模式切换，session_id 用于区分每次任务。
  RobotStateMachine state_machine_;
  uint64_t session_id_{0U};
  std::string map_root_;
  std::string display_pcd_filename_;
  std::size_t max_maps_{5U};
  double finish_mapping_timeout_sec_{300.0};
  double relocalization_timeout_sec_{60.0};
  double localization_stop_timeout_sec_{10.0};
  double navigation_control_timeout_sec_{3.0};
  std::string active_map_id_;
  std::filesystem::path active_map_staging_path_;
  std::string selected_map_id_;
  std::filesystem::path selected_map_path_;
  std::unique_ptr<MapPublisher> map_publisher_;

  // start_localization Service等待算法异步重定位结果。
  std::mutex localization_command_mutex_;
  std::mutex localization_result_mutex_;
  std::condition_variable localization_result_cv_;
  uint64_t localization_attempt_session_id_{0U};
  bool localization_attempt_active_{false};
  bool localization_result_ready_{false};
  bool localization_result_success_{false};
  uint64_t localization_cancelled_session_id_{0U};
  std::string localization_result_message_;

  // Worker 服务回调使用可重入组，支持异步串行编排多个模块。
  rclcpp::CallbackGroup::SharedPtr worker_callback_group_;
  rclcpp::CallbackGroup::SharedPtr localization_control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr localization_result_callback_group_;
  std::unique_ptr<MappingController> mapping_controller_;
  std::unique_ptr<LocalizationController> localization_controller_;
  std::unique_ptr<NavigationController> navigation_controller_;
  rclcpp::Service<StartMapping>::SharedPtr start_mapping_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_mapping_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_error_service_;
  rclcpp::Service<LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<DeleteMap>::SharedPtr delete_map_service_;
  rclcpp::Service<Trigger>::SharedPtr start_localization_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_localization_service_;
  rclcpp::Service<Trigger>::SharedPtr start_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_navigation_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_navigation_service_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      relocalization_result_sub_;
};

}  // namespace robot_runtime
