#include "robot_runtime/robot_runtime_node.hpp"

#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>

namespace robot_runtime {

RobotRuntimeNode::RobotRuntimeNode() : Node("robot_runtime") {
  map_root_ = declare_parameter<std::string>(
      "map_storage.root_directory", "/robotfs/data/slam_data/map");
  const auto configured_max_maps =
      declare_parameter<int64_t>("map_storage.max_maps", 5);
  finish_mapping_timeout_sec_ = declare_parameter<double>(
      "map_storage.finish_mapping_timeout_sec", 300.0);
  relocalization_timeout_sec_ = declare_parameter<double>(
      "localization.relocalization_timeout_sec", 60.0);
  localization_stop_timeout_sec_ = declare_parameter<double>(
      "localization.stop_timeout_sec", 10.0);
  navigation_control_timeout_sec_ = declare_parameter<double>(
      "navigation.control_timeout_sec", 3.0);
  display_pcd_filename_ = declare_parameter<std::string>(
      "map_display.pcd_filename", "globalmap.pcd");
  const auto map_display_topic = declare_parameter<std::string>(
      "map_display.topic", "/robot/map/point_cloud");
  const auto map_display_frame =
      declare_parameter<std::string>("map_display.frame_id", "map");
  if (map_root_.empty() ||
      !std::filesystem::path(map_root_).is_absolute()) {
    throw std::invalid_argument(
        "map_storage.root_directory must be an absolute path");
  }
  if (configured_max_maps <= 0) {
    throw std::invalid_argument(
        "map_storage.max_maps must be greater than zero");
  }
  if (finish_mapping_timeout_sec_ <= 0.0) {
    throw std::invalid_argument(
        "map_storage.finish_mapping_timeout_sec must be greater than zero");
  }
  if (relocalization_timeout_sec_ <= 0.0 ||
      localization_stop_timeout_sec_ <= 0.0 ||
      navigation_control_timeout_sec_ <= 0.0) {
    throw std::invalid_argument(
        "localization and navigation timeout parameters must be greater than zero");
  }
  if (display_pcd_filename_.empty() ||
      std::filesystem::path(display_pcd_filename_).is_absolute() ||
      display_pcd_filename_.find('/') != std::string::npos ||
      map_display_topic.empty() || map_display_frame.empty()) {
    throw std::invalid_argument(
        "map_display parameters must contain a filename, topic and frame_id");
  }
  max_maps_ = static_cast<std::size_t>(configured_max_maps);

  map_publisher_ = std::make_unique<MapPublisher>(
      this, map_display_topic, map_display_frame);

  // 可重入回调组允许 Worker 的异步响应驱动下一步启停操作。
  worker_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 开始重定位会等待算法结果，停止服务必须能并发进入以主动取消等待。
  localization_control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // start_localization会同步等待结果，结果订阅必须位于独立回调组。
  localization_result_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  mapping_controller_ =
      std::make_unique<MappingController>(this, worker_callback_group_);
  localization_controller_ =
      std::make_unique<LocalizationController>(this, worker_callback_group_);
  navigation_controller_ =
      std::make_unique<NavigationController>(this, worker_callback_group_);

  // 使用 transient_local，使新订阅者可以立即获得最近一次模式。
  mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/robot/runtime_state",
      rclcpp::QoS(1).reliable().transient_local());

  // 对上层统一暴露机器人级控制服务，隐藏内部 Worker 编排细节。
  start_mapping_service_ = create_service<StartMapping>(
      "/robot/start_mapping",
      std::bind(&RobotRuntimeNode::startMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  stop_mapping_service_ = create_service<Trigger>(
      "/robot/stop_mapping",
      std::bind(&RobotRuntimeNode::stopMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  reset_error_service_ = create_service<Trigger>(
      "/robot/reset_error",
      std::bind(&RobotRuntimeNode::resetErrorCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  load_map_service_ = create_service<LoadMap>(
      "/robot/load_map",
      std::bind(&RobotRuntimeNode::loadMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  start_localization_service_ = create_service<Trigger>(
      "/robot/start_localization",
      std::bind(&RobotRuntimeNode::startLocalizationCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      localization_control_callback_group_);
  stop_localization_service_ = create_service<Trigger>(
      "/robot/stop_localization",
      std::bind(&RobotRuntimeNode::stopLocalizationCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      localization_control_callback_group_);
  start_navigation_service_ = create_service<Trigger>(
      "/robot/start_navigation",
      std::bind(&RobotRuntimeNode::startNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  stop_navigation_service_ = create_service<Trigger>(
      "/robot/stop_navigation",
      std::bind(&RobotRuntimeNode::stopNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  pause_navigation_service_ = create_service<Trigger>(
      "/robot/pause_navigation",
      std::bind(&RobotRuntimeNode::pauseNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  resume_navigation_service_ = create_service<Trigger>(
      "/robot/resume_navigation",
      std::bind(&RobotRuntimeNode::resumeNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // fastlio_localization 重定位成功后自动将运行模式切换为 ready。
  rclcpp::SubscriptionOptions reloc_subscription_options;
  reloc_subscription_options.callback_group =
      localization_result_callback_group_;
  relocalization_result_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/reloc_success", rclcpp::QoS(10).reliable(),
      std::bind(&RobotRuntimeNode::relocalizationResultCallback, this,
                std::placeholders::_1),
      reloc_subscription_options);

  // 删除结果必须明确返回APP，因此使用请求/响应Service。
  delete_map_service_ = create_service<DeleteMap>(
      "/robot/delete_map",
      std::bind(&RobotRuntimeNode::deleteMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  state_machine_.initialize();
  publishMode();
  RCLCPP_INFO(get_logger(), "Robot runtime ready");
}

void RobotRuntimeNode::startMappingCallback(
    const StartMapping::Request::SharedPtr request,
    StartMapping::Response::SharedPtr response) {
  std::string validation_message;
  if (!validateMapId(request->map_id, validation_message)) {
    response->success = false;
    response->message = validation_message;
    return;
  }

  std::error_code exists_error;
  const auto final_map_path = mapPath(request->map_id);
  const bool map_exists = std::filesystem::exists(final_map_path, exists_error);
  if (exists_error) {
    response->success = false;
    response->message = "failed to inspect map directory: " +
                        exists_error.message();
    return;
  }
  const bool complete_map_exists =
      map_exists && std::filesystem::exists(final_map_path / "globalmap.pcd",
                                            exists_error);
  if (exists_error) {
    response->success = false;
    response->message = "failed to inspect existing map: " +
                        exists_error.message();
    return;
  }
  if (!complete_map_exists) {
    std::size_t saved_map_count = 0U;
    std::string count_message;
    if (!countSavedMaps(saved_map_count, count_message)) {
      response->success = false;
      response->message = count_message;
      return;
    }
    if (saved_map_count >= max_maps_) {
      response->success = false;
      response->message = "map limit reached: " +
                          std::to_string(saved_map_count) + "/" +
                          std::to_string(max_maps_);
      return;
    }
  }

  // 只有空闲状态允许进入建图启动过程。
  if (!state_machine_.beginMapping()) {
    response->success = false;
    response->message =
        "start mapping rejected, current state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  // 同名地图复用统一删除逻辑，旧目录在新建图开始前整体移除。
  if (map_exists) {
    std::string delete_message;
    if (!deleteMapDirectory(request->map_id, delete_message)) {
      state_machine_.enterError();
      response->success = false;
      response->message =
          "failed to replace existing map: " + delete_message;
      publishMode();
      return;
    }
    RCLCPP_INFO(get_logger(), "%s", delete_message.c_str());
  }

  // 建图期间由PGO发布实时地图，先清除之前加载的静态地图。
  if (!selected_map_id_.empty()) {
    selected_map_id_.clear();
    selected_map_path_.clear();
    map_publisher_->clear();
  }

  // 每次新任务使用新的会话编号，避免混用旧任务命令。
  ++session_id_;
  const uint64_t session_id = session_id_;
  active_map_id_ = request->map_id;
  active_map_staging_path_ =
      std::filesystem::path(map_root_) / ".staging" /
      (active_map_id_ + "_" + std::to_string(session_id));

  std::error_code create_error;
  std::filesystem::create_directories(active_map_staging_path_, create_error);
  if (create_error) {
    state_machine_.enterError();
    response->success = false;
    response->message = "failed to create map staging directory: " +
                        create_error.message();
    publishMode();
    return;
  }
  publishMode();

  mapping_controller_->start(
      session_id, active_map_id_, active_map_staging_path_.string(),
      [this](bool success, const std::string &message) {
        if (success) {
          state_machine_.mappingStarted();
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        } else {
          std::error_code cleanup_error;
          std::filesystem::remove_all(active_map_staging_path_, cleanup_error);
          state_machine_.enterError();
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        }
        publishMode();
      });

  response->success = true;
  response->message = "start mapping accepted, map_id=" + active_map_id_;
}

void RobotRuntimeNode::stopMappingCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  if (!state_machine_.beginStopMapping()) {
    response->success = false;
    response->message =
        "stop mapping rejected, current state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  publishMode();
  // Service等待Worker停止、PGO保存和目录发布全部结束后再响应APP。
  using StopResult = std::pair<bool, std::string>;
  auto completion = std::make_shared<std::promise<StopResult>>();
  auto future = completion->get_future();
  mapping_controller_->stop(
      session_id_,
      [completion](bool success, const std::string &message) {
        completion->set_value({success, message});
      });

  const auto timeout = std::chrono::duration<double>(
      finish_mapping_timeout_sec_);
  if (future.wait_for(timeout) != std::future_status::ready) {
    state_machine_.enterError();
    publishMode();
    response->success = false;
    response->message = "finish mapping timed out, map_id=" + active_map_id_;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto [workers_stopped, stop_message] = future.get();
  if (!workers_stopped) {
    state_machine_.enterError();
    publishMode();
    response->success = false;
    response->message = "finish mapping failed: " + stop_message;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  std::string finalize_message;
  if (!finalizeActiveMap(finalize_message)) {
    state_machine_.enterError();
    publishMode();
    response->success = false;
    response->message = "map save failed: " + finalize_message;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  state_machine_.mappingStopped();
  publishMode();
  response->success = true;
  response->message = finalize_message;
  RCLCPP_INFO(get_logger(), "%s; %s", stop_message.c_str(),
              finalize_message.c_str());
}

void RobotRuntimeNode::resetErrorCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  response->success = state_machine_.resetError();
  response->message = response->success
                          ? "error state reset"
                          : "reset rejected, current state=" +
                                std::string(toString(state_machine_.mode()));
  if (response->success) publishMode();
}

void RobotRuntimeNode::loadMapCallback(
    const LoadMap::Request::SharedPtr request,
    LoadMap::Response::SharedPtr response) {
  if (state_machine_.mode() != RobotMode::kIdle) {
    response->success = false;
    response->message =
        "load map rejected, robot must be idle, current state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  std::string validation_message;
  if (!validateMapId(request->map_id, validation_message)) {
    response->success = false;
    response->message = validation_message;
    return;
  }

  const auto requested_map_path = mapPath(request->map_id);
  std::error_code exists_error;
  if (!std::filesystem::exists(requested_map_path / "globalmap.pcd",
                               exists_error) ||
      exists_error) {
    response->success = false;
    response->message = exists_error
                            ? "failed to inspect map: " + exists_error.message()
                            : "map is incomplete or does not exist: " +
                                  request->map_id;
    return;
  }

  const auto display_pcd_path =
      requested_map_path / display_pcd_filename_;
  std::error_code display_exists_error;
  if (!std::filesystem::exists(display_pcd_path, display_exists_error) ||
      display_exists_error) {
    response->success = false;
    response->message =
        display_exists_error
            ? "failed to inspect display map: " +
                  display_exists_error.message()
            : "display map does not exist: " + display_pcd_path.string();
    return;
  }

  // 只有PCD读取和发布均成功，才更新当前选中的地图。
  std::string publish_message;
  if (!map_publisher_->loadAndPublish(display_pcd_path, publish_message)) {
    response->success = false;
    response->message = publish_message;
    return;
  }

  selected_map_id_ = request->map_id;
  selected_map_path_ = requested_map_path;
  response->success = true;
  response->message = "map loaded: " + selected_map_id_ + "; " +
                      publish_message;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void RobotRuntimeNode::startLocalizationCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  if (selected_map_id_.empty() || selected_map_path_.empty()) {
    response->success = false;
    response->message = "no map selected; call /robot/load_map first";
    return;
  }

  std::error_code exists_error;
  if (!std::filesystem::exists(selected_map_path_ / "globalmap.pcd",
                               exists_error) ||
      exists_error) {
    response->success = false;
    response->message = exists_error
                            ? "failed to inspect selected map: " +
                                  exists_error.message()
                            : "selected map no longer exists: " +
                                  selected_map_id_;
    return;
  }

  // 只保护“进入状态、登记任务、发送START”这一小段；等待结果期间释放，
  // 使stop_localization能够并发进入并取消本次重定位。
  std::unique_lock<std::mutex> command_lock(localization_command_mutex_);
  if (!state_machine_.beginLocalization()) {
    response->success = false;
    response->message =
        "start localization rejected, current state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  ++session_id_;
  const uint64_t session_id = session_id_;
  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    localization_attempt_session_id_ = session_id;
    localization_attempt_active_ = true;
    localization_result_ready_ = false;
    localization_result_success_ = false;
    localization_result_message_.clear();
  }
  publishMode();

  localization_controller_->start(
      session_id, selected_map_id_, selected_map_path_.string(),
      [this, session_id](bool success, const std::string &message) {
        {
          std::lock_guard<std::mutex> lock(localization_result_mutex_);
          if (!localization_attempt_active_ ||
              localization_attempt_session_id_ != session_id ||
              localization_cancelled_session_id_ == session_id ||
              localization_result_ready_) {
            return;
          }
          if (!success) {
            localization_result_ready_ = true;
            localization_result_success_ = false;
            localization_result_message_ =
                "failed to start localization: " + message;
          }
        }
        if (success) {
          state_machine_.localizationStarted();
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        } else {
          state_machine_.localizationFailed();
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
          localization_result_cv_.notify_all();
        }
        publishMode();
      });
  command_lock.unlock();

  bool result_received = false;
  bool result_success = false;
  bool cancelled = false;
  std::string result_message;
  {
    std::unique_lock<std::mutex> lock(localization_result_mutex_);
    result_received = localization_result_cv_.wait_for(
        lock, std::chrono::duration<double>(relocalization_timeout_sec_),
        [this, session_id]() {
          return localization_cancelled_session_id_ == session_id ||
                 (localization_attempt_session_id_ == session_id &&
                  localization_result_ready_);
        });
    cancelled = localization_cancelled_session_id_ == session_id;
    if (cancelled) {
      result_message = "relocalization cancelled by stop request";
      result_success = false;
    } else if (result_received) {
      result_success = localization_result_success_;
      result_message = localization_result_message_;
    }
    if (localization_attempt_session_id_ == session_id) {
      localization_attempt_active_ = false;
    }
  }

  // stop_localization负责停止Worker、析构节点和完成状态流转，避免重复STOP。
  if (cancelled) {
    response->success = false;
    response->message = result_message;
    return;
  }

  if (result_received && result_success) {
    response->success = true;
    response->message = result_message;
    return;
  }

  const std::string failure_message =
      result_received
          ? result_message
          : "relocalization timed out after " +
                std::to_string(relocalization_timeout_sec_) +
                " seconds, map_id=" + selected_map_id_;

  // 启动失败或超时均发送STOP，确保可能残留的定位节点被析构。
  using StopResult = std::pair<bool, std::string>;
  auto stop_completion = std::make_shared<std::promise<StopResult>>();
  auto stop_future = stop_completion->get_future();
  localization_controller_->stop(
      session_id,
      [stop_completion](bool success, const std::string &message) {
        stop_completion->set_value({success, message});
      });

  const auto stop_timeout =
      std::chrono::duration<double>(localization_stop_timeout_sec_);
  const bool stop_finished =
      stop_future.wait_for(stop_timeout) == std::future_status::ready;
  const auto stop_result =
      stop_finished ? stop_future.get()
                    : StopResult{false, "localization stop timed out"};
  if (stop_result.first) {
    state_machine_.localizationFailed();
  } else {
    state_machine_.enterError();
  }
  publishMode();

  response->success = false;
  response->message = failure_message + "; " + stop_result.second;
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

void RobotRuntimeNode::stopLocalizationCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  using StopResult = std::pair<bool, std::string>;
  auto completion = std::make_shared<std::promise<StopResult>>();
  auto future = completion->get_future();
  uint64_t session_id = 0U;

  {
    // 与start_localization的启动片段串行，保证STOP不会早于START发送。
    std::scoped_lock command_lock(localization_command_mutex_);
    if (!state_machine_.beginStopLocalization()) {
      response->success = false;
      response->message =
          "stop localization rejected, current state=" +
          std::string(toString(state_machine_.mode()));
      return;
    }

    session_id = session_id_;
    {
      std::lock_guard<std::mutex> result_lock(localization_result_mutex_);
      if (localization_attempt_active_ &&
          localization_attempt_session_id_ == session_id) {
        localization_cancelled_session_id_ = session_id;
      }
    }
    localization_result_cv_.notify_all();
    publishMode();

    localization_controller_->stop(
        session_id, [completion](bool success, const std::string &message) {
          completion->set_value({success, message});
        });
  }

  // 等待Worker完成Executor退出和节点析构，再向APP确认停止成功。
  const auto timeout =
      std::chrono::duration<double>(localization_stop_timeout_sec_);
  if (future.wait_for(timeout) != std::future_status::ready) {
    state_machine_.enterError();
    publishMode();
    response->success = false;
    response->message = "stop localization timed out before node destruction";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto [success, message] = future.get();
  if (!success) {
    state_machine_.enterError();
    publishMode();
    response->success = false;
    response->message = "failed to destroy localization node: " + message;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  state_machine_.localizationStopped();
  publishMode();
  response->success = true;
  response->message = "localization node stopped and destroyed";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void RobotRuntimeNode::relocalizationResultCallback(
    const std_msgs::msg::Bool::SharedPtr message) {
  // false表示算法仍在尝试，不作为最终失败；最终失败由超时统一判定。
  if (!message->data) return;

  std::string result_message;
  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    if (!localization_attempt_active_ || localization_result_ready_ ||
        localization_cancelled_session_id_ ==
            localization_attempt_session_id_) {
      return;
    }
    localization_result_ready_ = true;
    localization_result_success_ = true;
    result_message = "relocalization succeeded, map_id=" + selected_map_id_;
    localization_result_message_ = result_message;
  }

  if (state_machine_.localizationReady()) {
    RCLCPP_INFO(get_logger(), "%s", result_message.c_str());
    publishMode();
  }
  localization_result_cv_.notify_all();
}

void RobotRuntimeNode::deleteMapCallback(
    const DeleteMap::Request::SharedPtr request,
    DeleteMap::Response::SharedPtr response) {
  const RobotMode current_mode = state_machine_.mode();
  if (current_mode != RobotMode::kIdle) {
    response->success = false;
    response->message =
        "delete map rejected, robot must be idle, current state=" +
        std::string(toString(current_mode));
    return;
  }

  response->success =
      deleteMapDirectory(request->map_id, response->message);
  if (response->success) {
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Delete map rejected: %s",
                response->message.c_str());
  }
}

void RobotRuntimeNode::startNavigationCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  // 导航必须在定位就绪状态下启动。
  if (!state_machine_.beginNavigation()) {
    response->success = false;
    response->message =
        "start navigation rejected, localization is not ready or current "
        "state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  ++session_id_;
  const uint64_t session_id = session_id_;
  publishMode();
  navigation_controller_->start(
      session_id,
      [this](bool success, const std::string &message) {
        if (success) {
          state_machine_.navigationStarted();
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        } else {
          state_machine_.enterError();
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        }
        publishMode();
      });

  response->success = true;
  response->message = "start navigation accepted, session=" +
                      std::to_string(session_id);
}

void RobotRuntimeNode::stopNavigationCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  if (!state_machine_.beginStopNavigation()) {
    response->success = false;
    response->message =
        "stop navigation rejected, current state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  publishMode();
  // 先进入停止中状态，完成后回到可再次导航的 ready 状态。
  navigation_controller_->stop(
      session_id_, [this](bool success, const std::string &message) {
        if (success) {
          state_machine_.navigationStopped();
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        } else {
          state_machine_.enterError();
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        }
        publishMode();
      });

  response->success = true;
  response->message = "stop navigation accepted";
}

void RobotRuntimeNode::pauseNavigationCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  if (state_machine_.mode() != RobotMode::kNavigating) {
    response->success = false;
    response->message = "pause navigation rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  std::string message;
  if (!navigation_controller_->setPaused(
          true, navigation_control_timeout_sec_, message)) {
    response->success = false;
    response->message = "pause navigation failed: " + message;
    return;
  }
  if (!state_machine_.pauseNavigation()) {
    response->success = false;
    response->message = "pause navigation state transition failed";
    return;
  }

  publishMode();
  response->success = true;
  response->message = message;
}

void RobotRuntimeNode::resumeNavigationCallback(
    const Trigger::Request::SharedPtr,
    Trigger::Response::SharedPtr response) {
  if (state_machine_.mode() != RobotMode::kNavigationPaused) {
    response->success = false;
    response->message = "resume navigation rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  std::string message;
  if (!navigation_controller_->setPaused(
          false, navigation_control_timeout_sec_, message)) {
    response->success = false;
    response->message = "resume navigation failed: " + message;
    return;
  }
  if (!state_machine_.resumeNavigation()) {
    response->success = false;
    response->message = "resume navigation state transition failed";
    return;
  }

  publishMode();
  response->success = true;
  response->message = message;
}

bool RobotRuntimeNode::validateMapId(const std::string &map_id,
                                     std::string &message) const {
  if (map_id.empty() || map_id.size() > 64U) {
    message = "map_id length must be between 1 and 64";
    return false;
  }
  if (!std::isalnum(static_cast<unsigned char>(map_id.front()))) {
    message = "map_id must start with a letter or digit";
    return false;
  }
  for (const char character : map_id) {
    const auto value = static_cast<unsigned char>(character);
    if (!std::isalnum(value) && character != '_' && character != '-') {
      message = "map_id may only contain letters, digits, '_' and '-'";
      return false;
    }
  }
  return true;
}

std::filesystem::path RobotRuntimeNode::mapPath(
    const std::string &map_id) const {
  return std::filesystem::path(map_root_) / map_id;
}

bool RobotRuntimeNode::deleteMapDirectory(const std::string &map_id,
                                          std::string &message) {
  if (!validateMapId(map_id, message)) return false;

  const auto target = mapPath(map_id);
  std::error_code filesystem_error;
  const bool exists = std::filesystem::exists(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect map directory: " +
              filesystem_error.message();
    return false;
  }
  if (!exists) {
    message = "map does not exist: " + map_id;
    return false;
  }
  if (!std::filesystem::is_directory(target, filesystem_error) ||
      filesystem_error) {
    message = "map target is not a valid directory: " + target.string();
    return false;
  }

  const auto removed_count =
      std::filesystem::remove_all(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to delete map directory: " +
              filesystem_error.message();
    return false;
  }
  if (selected_map_id_ == map_id) {
    selected_map_id_.clear();
    selected_map_path_.clear();
    map_publisher_->clear();
  }
  message = "map deleted: " + map_id + ", removed entries=" +
            std::to_string(removed_count);
  return true;
}

bool RobotRuntimeNode::countSavedMaps(std::size_t &count,
                                      std::string &message) const {
  count = 0U;
  const std::filesystem::path root(map_root_);
  std::error_code filesystem_error;
  if (!std::filesystem::exists(root, filesystem_error)) {
    if (filesystem_error) {
      message = "failed to inspect map root: " + filesystem_error.message();
      return false;
    }
    return true;
  }

  std::filesystem::directory_iterator iterator(root, filesystem_error);
  const std::filesystem::directory_iterator end;
  if (filesystem_error) {
    message = "failed to list map root: " + filesystem_error.message();
    return false;
  }
  while (iterator != end) {
    const auto &entry = *iterator;
    const std::string name = entry.path().filename().string();
    if (!name.empty() && name.front() != '.' &&
        entry.is_directory(filesystem_error) &&
        std::filesystem::exists(entry.path() / "globalmap.pcd",
                                filesystem_error)) {
      ++count;
    }
    if (filesystem_error) {
      message = "failed to inspect map entry: " + filesystem_error.message();
      return false;
    }
    iterator.increment(filesystem_error);
    if (filesystem_error) {
      message = "failed to iterate map root: " + filesystem_error.message();
      return false;
    }
  }
  return true;
}

bool RobotRuntimeNode::finalizeActiveMap(std::string &message) {
  if (active_map_id_.empty() || active_map_staging_path_.empty()) {
    message = "no active map task";
    return false;
  }

  const auto global_map = active_map_staging_path_ / "globalmap.pcd";
  std::error_code filesystem_error;
  if (!std::filesystem::exists(global_map, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect saved map: " +
                        filesystem_error.message()
                  : "PGO did not produce globalmap.pcd";
    return false;
  }

  std::ofstream manifest(active_map_staging_path_ / "manifest.yaml");
  if (!manifest.is_open()) {
    message = "failed to create manifest.yaml";
    return false;
  }
  manifest << "schema_version: 1\n"
           << "map_id: \"" << active_map_id_ << "\"\n"
           << "frame_id: \"map\"\n"
           << "status: \"ready\"\n"
           << "global_map: \"globalmap.pcd\"\n";
  manifest.close();
  if (!manifest) {
    message = "failed to write manifest.yaml";
    return false;
  }

  const auto final_path = mapPath(active_map_id_);
  const bool final_map_exists =
      std::filesystem::exists(final_path, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect existing map: " +
              filesystem_error.message();
    return false;
  }
  if (final_map_exists) {
    message = "map appeared after mapping started; publish rejected";
    return false;
  }
  std::size_t saved_map_count = 0U;
  if (!countSavedMaps(saved_map_count, message)) return false;
  if (saved_map_count >= max_maps_) {
    message = "map limit reached before finalization: " +
              std::to_string(saved_map_count) + "/" +
              std::to_string(max_maps_);
    return false;
  }

  std::filesystem::rename(active_map_staging_path_, final_path,
                          filesystem_error);
  if (filesystem_error) {
    message = "failed to publish saved map: " + filesystem_error.message();
    return false;
  }

  const std::string saved_map_id = active_map_id_;
  active_map_id_.clear();
  active_map_staging_path_.clear();
  message = "map saved: " + saved_map_id;
  return true;
}

void RobotRuntimeNode::publishMode() {
  // 对外发布当前机器人运行模式。
  std_msgs::msg::String message;
  message.data = std::string(toString(state_machine_.mode()));
  mode_pub_->publish(message);
}

}  // namespace robot_runtime
