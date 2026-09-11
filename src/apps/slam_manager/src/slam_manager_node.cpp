#include "slam_manager/slam_manager_node.hpp"

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include "slam_manager/pose_utils.hpp"

namespace slam_manager {

namespace {

constexpr char kRobotStateServiceName[] = "/GetState";
constexpr char kNavigationRequiredRobotState[] = "Locomotion";
constexpr double kRobotStateTimeoutSec = 3.0;
constexpr char kRobotModeServiceName[] = "/SetMode";
constexpr double kRobotModeTimeoutSec = 3.0;

bool finitePoseFields(double x, double y, double z, double theta) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         std::isfinite(theta);
}

class ScopeExit {
 public:
  explicit ScopeExit(std::function<void()> callback)
      : callback_(std::move(callback)) {}
  ~ScopeExit() { callback_(); }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit &operator=(const ScopeExit &) = delete;

 private:
  std::function<void()> callback_;
};

bool validMapId(const std::string &map_id, std::string &message) {
  constexpr std::size_t kMaximumIdLength = 64U;
  if (map_id.empty() || map_id.size() > kMaximumIdLength) {
    message = "map_id must contain 1 to " + std::to_string(kMaximumIdLength) +
              " characters";
    return false;
  }
  for (const char character : map_id) {
    const bool valid = (character >= 'a' && character <= 'z') ||
                       (character >= 'A' && character <= 'Z') ||
                       (character >= '0' && character <= '9') ||
                       character == '_' || character == '-';
    if (!valid) {
      message = "map_id may contain only letters, digits, '_' and '-'";
      return false;
    }
  }
  // root/log专用于运行日志，不能被同名地图覆盖或删除。
  if (map_id == "log") {
    message = "map_id 'log' is reserved for runtime logs";
    return false;
  }
  return true;
}

}  // namespace

SlamManagerNode::SlamManagerNode() : Node("slam_manager") {
  const auto map_root = declare_parameter<std::string>(
      "map_storage.root_directory", "/robotfs/data/slam_data");
  const auto configured_max_maps =
      declare_parameter<int64_t>("map_storage.max_maps", 5);
  if (configured_max_maps <= 0) {
    throw std::invalid_argument(
        "map_storage.max_maps must be greater than zero");
  }
  start_mapping_timeout_sec_ =
      declare_parameter<double>("map_storage.start_mapping_timeout_sec", 30.0);
  finish_mapping_timeout_sec_ = declare_parameter<double>(
      "map_storage.finish_mapping_timeout_sec", 300.0);
  relocalization_timeout_sec_ = declare_parameter<double>(
      "localization.relocalization_timeout_sec", 60.0);
  localization_stop_timeout_sec_ =
      declare_parameter<double>("localization.stop_timeout_sec", 10.0);
  body_cloud_publish_hz_ =
      declare_parameter<double>("publish.body_cloud_hz", 5.0);

  const auto bbs_saver_executable = declare_parameter<std::string>(
      "localization.bbs.saver_executable",
      "/home/jw-zkl/robotics_platform/3rdparty/3d_bbs/test/build/saver");
  const auto bbs_min_level_resolution =
      declare_parameter<double>("localization.bbs.min_level_resolution", 0.5);
  const auto bbs_max_level =
      declare_parameter<int64_t>("localization.bbs.max_level", 6);
  const auto bbs_generation_timeout_sec = declare_parameter<double>(
      "localization.bbs.generation_timeout_sec", 120.0);
  const auto navigation_control_timeout_sec =
      declare_parameter<double>("navigation.control_timeout_sec", 3.0);
  const auto navigation_worker_timeout_sec =
      declare_parameter<double>("navigation.worker_timeout_sec", 30.0);
  const auto navigation_goal_timeout_sec =
      declare_parameter<double>("navigation.goal_timeout_sec", 300.0);
  enable_set_mode_ =
      declare_parameter<bool>("navigation.enable_set_mode", true);
  const auto navigation_enable_skip_first_target =
      declare_parameter<bool>("navigation.enable_skip_first_target", true);
  const auto navigation_skip_first_target_threshold =
      declare_parameter<double>("navigation.skip_first_target_threshold", 0.8);
  TaskEdgePolicy default_edge_policy;
  default_edge_policy.obstacle_stop_enabled = true;
  default_edge_policy.obstacle_avoidance_enabled = declare_parameter<bool>(
      "navigation.default_edge_policy.obstacle_avoidance_enabled", false);
  default_edge_policy.obstacle_avoidance_distance_m = declare_parameter<double>(
      "navigation.default_edge_policy.obstacle_avoidance_distance_m", 0.5);
  map_storage_ = std::make_unique<MapStorage>(
      map_root, static_cast<std::size_t>(configured_max_maps));
  bbs_voxel_generator_ = std::make_unique<BbsVoxelGenerator>(
      BbsVoxelGenerator::Config{bbs_saver_executable, bbs_min_level_resolution,
                                bbs_max_level, bbs_generation_timeout_sec});
  topology_file_writer_ = std::make_unique<TopologyFileWriter>();

  // 可重入回调组允许 Worker 的异步响应驱动下一步启停操作。
  worker_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 生命周期入口必须可重入：新命令需要在旧命令等待Worker时进入，登记新的
  // generation并取消旧token；真正的启停过程仍由lifecycle_mutex_串行化。
  lifecycle_control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 重定位Action会异步等待结果，结果订阅必须位于独立回调组。
  localization_result_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  navigation_result_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 导航执行线程会同步等待底盘服务；独立可重入组保证响应可由执行器处理。
  robot_control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  get_robot_state_client_ = create_client<GetState>(
      kRobotStateServiceName, rmw_qos_profile_services_default,
      robot_control_callback_group_);
  set_robot_mode_client_ = create_client<SetMode>(
      kRobotModeServiceName, rmw_qos_profile_services_default,
      robot_control_callback_group_);
  mapping_controller_ =
      std::make_unique<MappingController>(this, worker_callback_group_);
  localization_controller_ =
      std::make_unique<LocalizationController>(this, worker_callback_group_);
  navigation_task_executor_ = std::make_unique<NavigationTaskExecutor>(
      this, worker_callback_group_, &state_machine_,
      topology_file_writer_.get(),
      NavigationTaskExecutor::Config{
          navigation_control_timeout_sec, navigation_worker_timeout_sec,
          navigation_goal_timeout_sec, navigation_enable_skip_first_target,
          navigation_skip_first_target_threshold, default_edge_policy},
      NavigationTaskExecutor::Callbacks{
          [this]() { publishMode(); },
          [this](uint8_t state) { publishRuntimeState(state); },
          [this](int32_t code, const std::string &message) {
            enterError(code, message);
          },
          [this]() { return relocalization_succeeded_.load(); },
          [this](std::string &message) {
            return robotStateAllowsNavigation(message);
          },
          [this](int32_t mode, std::string &message) {
            return setRobotMode(mode, message);
          },
          [this](geometry_msgs::msg::PoseStamped &pose) {
            std::lock_guard<std::mutex> lock(current_pose_mutex_);
            if (!has_current_pose_) {
              return false;
            }
            pose = current_pose_;
            return true;
          },
          [this](const slam_manager_interfaces::msg::NavigationNode &node,
                 const std::function<bool()> &is_canceling,
                 std::string &message) {
            return relocalizeForMapConnection(node, is_canceling, message);
          }});

  // 协议2.10～2.13：Runtime统一对APP发布位姿、实时点云、关键帧和运行状态。
  runtime_state_pub_ = create_publisher<RuntimeState>(
      "/robot/runtime_state", rclcpp::QoS(1).reliable().transient_local());
  keyframe_cloud_pub_ =
      create_publisher<MapKeyframe>("/robot/keyframe_cloud", 100);
  current_pose_pub_ = create_publisher<RobotPose>("/robot/current_pose", 20);
  body_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/robot/cloud_body", rclcpp::QoS(20).best_effort());

  // 算法内部Topic不直接暴露给APP，由Runtime转换为统一的 /robot/* Topic。
  keyframe_cloud_sub_ = create_subscription<MapKeyframe>(
      "/pgo/keyframe_cloud", 100,
      std::bind(&SlamManagerNode::keyframeCloudCallback, this,
                std::placeholders::_1));
  mapping_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose_stamped", 20,
      std::bind(&SlamManagerNode::currentPoseCallback, this,
                std::placeholders::_1));
  localization_pose_sub_ = create_subscription<LocalizationPose>(
      "/app/localization_pose", 20,
      std::bind(&SlamManagerNode::localizationPoseCallback, this,
                std::placeholders::_1));
  body_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_body_downsampled", rclcpp::QoS(20).best_effort(),
      std::bind(&SlamManagerNode::bodyCloudCallback, this,
                std::placeholders::_1));

  initial_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 1);

  // 协议2.1～2.5、2.8～2.9：只创建协议中定义的APP Service。
  start_mapping_service_ = create_service<StartMapping>(
      "/robot/start_mapping",
      std::bind(&SlamManagerNode::startMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);
  finish_mapping_service_ = create_service<Trigger>(
      "/robot/finish_mapping",
      std::bind(&SlamManagerNode::finishMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);
  cancel_mapping_service_ = create_service<Trigger>(
      "/robot/cancel_mapping",
      std::bind(&SlamManagerNode::cancelMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);
  load_map_service_ = create_service<LoadMap>(
      "/robot/load_map",
      std::bind(&SlamManagerNode::loadMapCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);
  delete_map_service_ = create_service<DeleteMap>(
      "/robot/delete_map",
      std::bind(&SlamManagerNode::deleteMapCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);
  pause_navigation_service_ = create_service<Trigger>(
      "/robot/pause_navigation",
      std::bind(&SlamManagerNode::pauseNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);
  resume_navigation_service_ = create_service<Trigger>(
      "/robot/resume_navigation",
      std::bind(&SlamManagerNode::resumeNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, lifecycle_control_callback_group_);

  // 协议2.6：巡检导航使用Action，支持逐点反馈、最终结果和任务取消。
  start_navigation_action_ = rclcpp_action::create_server<StartNavigation>(
      this, "/robot/start_navigation",
      std::bind(&SlamManagerNode::handleNavigationGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&SlamManagerNode::handleNavigationCancel, this,
                std::placeholders::_1),
      std::bind(&SlamManagerNode::handleNavigationAccepted, this,
                std::placeholders::_1));

  // 协议2.7：单点导航同样使用Action，支持状态反馈和GoalHandle取消。
  navigate_to_pose_action_ = rclcpp_action::create_server<NavigateToPose>(
      this, "/robot/navigate_to_pose",
      std::bind(&SlamManagerNode::handleNavigateToPoseGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&SlamManagerNode::handleNavigateToPoseCancel, this,
                std::placeholders::_1),
      std::bind(&SlamManagerNode::handleNavigateToPoseAccepted, this,
                std::placeholders::_1));

  // 协议2.6：重定位使用Action，执行期间支持反馈和取消。
  start_localization_action_ = rclcpp_action::create_server<StartLocalization>(
      this, "/robot/start_localization",
      std::bind(&SlamManagerNode::handleLocalizationGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&SlamManagerNode::handleLocalizationCancel, this,
                std::placeholders::_1),
      std::bind(&SlamManagerNode::handleLocalizationAccepted, this,
                std::placeholders::_1));

  // fastlio_localization 重定位成功后回到空闲态，并记录导航许可标志。
  rclcpp::SubscriptionOptions reloc_subscription_options;
  reloc_subscription_options.callback_group =
      localization_result_callback_group_;
  relocalization_result_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/reloc_success", 10,
      std::bind(&SlamManagerNode::relocalizationResultCallback, this,
                std::placeholders::_1),
      reloc_subscription_options);
  rclcpp::SubscriptionOptions navigation_subscription_options;
  navigation_subscription_options.callback_group =
      navigation_result_callback_group_;
  navigation_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav_state", 10,
      std::bind(&SlamManagerNode::navigationStateCallback, this,
                std::placeholders::_1),
      navigation_subscription_options);

  state_machine_.initialize();
  publishMode();
  RCLCPP_INFO(get_logger(), "Robot runtime ready");
}

SlamManagerNode::~SlamManagerNode() {
  shutting_down_.store(true);
  {
    std::lock_guard<std::mutex> lock(lifecycle_token_mutex_);
    if (active_lifecycle_token_) active_lifecycle_token_->cancel();
  }
  localization_result_cv_.notify_all();
  std::lock_guard<std::mutex> lock(localization_action_thread_mutex_);
  if (localization_action_thread_.joinable())
    localization_action_thread_.join();
}

uint64_t SlamManagerNode::beginLifecycleCommand(const char *command_name) {
  const uint64_t generation = command_generation_.fetch_add(1U) + 1U;
  // 新生命周期命令先让旧异步链停止，再等待旧命令释放执行锁。
  {
    std::lock_guard<std::mutex> lock(lifecycle_token_mutex_);
    if (active_lifecycle_token_) active_lifecycle_token_->cancel();
  }
  localization_result_cv_.notify_all();
  navigation_task_executor_->cancel();
  RCLCPP_INFO(get_logger(),
              "[LIFECYCLE] accepted command=%s generation=%llu; previous "
              "operation cancellation requested",
              command_name, static_cast<unsigned long long>(generation));
  return generation;
}

bool SlamManagerNode::isLatestCommand(uint64_t generation) const {
  return command_generation_.load() == generation;
}

std::shared_ptr<WorkerOperationToken> SlamManagerNode::installLifecycleToken(
    uint64_t generation) {
  std::lock_guard<std::mutex> lock(lifecycle_token_mutex_);
  if (!isLatestCommand(generation)) return {};
  active_lifecycle_token_ = std::make_shared<WorkerOperationToken>();
  return active_lifecycle_token_;
}

bool SlamManagerNode::isLifecycleCommandActive(
    uint64_t generation,
    const std::shared_ptr<WorkerOperationToken> &token) const {
  return token && token->active() && isLatestCommand(generation);
}

bool SlamManagerNode::prepareForStartCommand(
    uint64_t generation, const std::shared_ptr<WorkerOperationToken> &token,
    std::string &message) {
  if (!isLifecycleCommandActive(generation, token)) {
    message = "lifecycle command superseded before teardown";
    return false;
  }
  if (!stopAllAlgorithms(message)) {
    if (!isLifecycleCommandActive(generation, token)) {
      message = "lifecycle command superseded during teardown";
    } else {
      message = "failed to stop current algorithms: " + message;
    }
    return false;
  }
  RCLCPP_INFO(get_logger(), "Stopped current algorithms: %s", message.c_str());
  if (!isLifecycleCommandActive(generation, token)) {
    message = "lifecycle command superseded after teardown";
    return false;
  }
  if (!clearRuntimeAfterTeardown(message)) return false;
  publishMode();
  if (!isLifecycleCommandActive(generation, token)) {
    message = "lifecycle command superseded after runtime reset";
    return false;
  }
  return true;
}

std::string SlamManagerNode::recoverLocalizationActionException(
    const std::string &message) {
  RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  std::string result_message = message;
  try {
    const uint64_t generation =
        beginLifecycleCommand("recover_localization_action");
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    const auto operation_token = installLifecycleToken(generation);
    std::string cleanup_message;
    if (operation_token &&
        prepareForStartCommand(generation, operation_token, cleanup_message)) {
      result_message += "; runtime cleanup succeeded";
    } else {
      result_message += "; runtime cleanup failed: " + cleanup_message;
      if (isLifecycleCommandActive(generation, operation_token)) {
        enterError(RuntimeState::LOCALIZATION_ERROR, result_message);
      }
    }
  } catch (const std::exception &cleanup_error) {
    result_message += "; runtime cleanup exception: ";
    result_message += cleanup_error.what();
    enterError(RuntimeState::LOCALIZATION_ERROR, result_message);
  } catch (...) {
    result_message += "; unknown runtime cleanup exception";
    enterError(RuntimeState::LOCALIZATION_ERROR, result_message);
  }
  return result_message;
}

void SlamManagerNode::abortLocalizationActionAfterException(
    const std::shared_ptr<StartLocalizationGoalHandle> &goal_handle,
    const std::string &message) noexcept {
  try {
    const std::string result_message =
        recoverLocalizationActionException(message);
    auto result = std::make_shared<StartLocalization::Result>();
    result->success = false;
    result->message = result_message;
    if (goal_handle->is_active()) goal_handle->abort(result);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to recover or abort localization action: %s",
                 error.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(),
                 "Failed to recover or abort localization action");
  }
}

bool SlamManagerNode::stopAllAlgorithms(std::string &message) {
  bool success = true;
  std::string details;
  const auto append_result = [&success, &details](const std::string &name,
                                                  bool stopped,
                                                  const std::string &detail) {
    if (!details.empty()) details += "; ";
    details += name + "=" + detail;
    success = success && stopped;
  };

  std::string stop_message;
  const bool navigation_stopped =
      navigation_task_executor_->stopAndWait(0U, stop_message);
  append_result("navigation", navigation_stopped, stop_message);

  const auto localization_stop = awaitWorkerCommand(
      "localization stop", localization_stop_timeout_sec_,
      std::make_shared<WorkerOperationToken>(),
      [this](WorkerCompletion completion) {
        localization_controller_->stop(0U, std::move(completion));
      });
  if (localization_stop.success()) {
    active_localization_session_id_.store(0U);
  }
  append_result("localization", localization_stop.success(),
                localization_stop.message);

  const auto mapping_stop = awaitWorkerCommand(
      "mapping stop", finish_mapping_timeout_sec_,
      std::make_shared<WorkerOperationToken>(),
      [this](WorkerCompletion completion) {
        mapping_controller_->cancel(0U, std::move(completion));
      });
  append_result("mapping", mapping_stop.success(), mapping_stop.message);

  message = details;
  return success;
}

bool SlamManagerNode::clearRuntimeAfterTeardown(std::string &message) {
  if (active_map_directory_prepared_ && !active_map_id_.empty()) {
    std::string delete_message;
    if (!map_storage_->deleteMap(active_map_id_, delete_message)) {
      message = "failed to discard incomplete map '" + active_map_id_ +
                "': " + delete_message;
      return false;
    }
  }
  active_map_id_.clear();
  active_map_path_.clear();
  active_map_directory_prepared_ = false;
  relocalization_succeeded_.store(false);
  active_localization_session_id_.store(0U);
  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    localization_attempt_session_id_ = 0U;
    localization_attempt_active_ = false;
    localization_result_armed_ = false;
    localization_result_ready_ = false;
    localization_result_success_ = false;
    localization_result_message_.clear();
  }
  localization_result_cv_.notify_all();
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    current_pose_ = geometry_msgs::msg::PoseStamped();
    has_current_pose_ = false;
  }
  state_machine_.resetToIdleAfterTeardown();
  message = "runtime algorithms stopped and destroyed";
  return true;
}

void SlamManagerNode::startMappingCallback(
    const StartMapping::Request::SharedPtr request,
    StartMapping::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=start_mapping map_id='%s' mode=%s "
              "session=%llu selected_map='%s' reloc_success=%s",
              request->map_id.c_str(),
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_),
              selected_map_id_.c_str(),
              relocalization_succeeded_.load() ? "true" : "false");

  // 协议2.1：APP提供字符串map_id，该字符串直接作为目录名。
  const std::string &map_id = request->map_id;
  if (!validMapId(map_id, response->message)) {
    response->success = false;
    return;
  }

  std::error_code exists_error;
  const auto final_data_path = map_storage_->dataPath(map_id);
  const auto final_map_path = map_storage_->mapPath(map_id);
  const bool map_exists =
      std::filesystem::exists(final_data_path, exists_error);
  if (exists_error) {
    response->success = false;
    response->message =
        "failed to inspect map directory: " + exists_error.message();
    return;
  }
  const bool complete_map_exists =
      map_exists &&
      std::filesystem::exists(final_map_path / "globalmap.pcd", exists_error);
  if (exists_error) {
    response->success = false;
    response->message =
        "failed to inspect existing map: " + exists_error.message();
    return;
  }
  if (!complete_map_exists) {
    std::size_t saved_map_count = 0U;
    std::string count_message;
    if (!map_storage_->countSavedMaps(saved_map_count, count_message)) {
      response->success = false;
      response->message = count_message;
      return;
    }
    if (saved_map_count >= map_storage_->maxMaps()) {
      response->success = false;
      response->message =
          "map limit reached: " + std::to_string(saved_map_count) + "/" +
          std::to_string(map_storage_->maxMaps());
      return;
    }
  }

  const uint64_t generation = beginLifecycleCommand("start_mapping");
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const auto operation_token = installLifecycleToken(generation);
  if (!operation_token) {
    response->success = false;
    response->message = "mapping superseded by a newer start request";
    return;
  }

  std::string teardown_message;
  if (!prepareForStartCommand(generation, operation_token, teardown_message)) {
    if (!isLifecycleCommandActive(generation, operation_token)) {
      response->success = false;
      response->message = "mapping superseded by a newer start request";
      return;
    }
    response->success = false;
    response->message = teardown_message;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    return;
  }

  if (!state_machine_.beginMapping()) {
    response->success = false;
    response->message = "start mapping rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  // 开始新建图后，之前由load_map记录的地图选择失效。
  if (!selected_map_id_.empty()) {
    selected_map_id_.clear();
  }

  // 每次新任务使用新的会话编号，避免混用旧任务命令。
  ++session_id_;
  const uint64_t session_id = session_id_;
  active_map_id_ = map_id;
  active_map_path_ = map_storage_->mapPath(active_map_id_);
  active_map_directory_prepared_ = false;
  publishMode();

  // PGO建图期间不持有保存路径也不创建目录；随后启动FAST-LIO。
  // 两者节点级Ready后才响应APP。
  const auto start_result = awaitWorkerCommand(
      "mapping workers start", start_mapping_timeout_sec_, operation_token,
      [this, session_id, operation_token](WorkerCompletion completion) {
        mapping_controller_->start(session_id, active_map_id_, operation_token,
                                   std::move(completion));
      },
      [this, generation, operation_token]() {
        return isLifecycleCommandActive(generation, operation_token);
      },
      [this, session_id](WorkerCompletion completion) {
        mapping_controller_->cancel(session_id, std::move(completion));
      },
      finish_mapping_timeout_sec_);

  if (start_result.status == WorkerCommandStatus::kSuperseded ||
      !isLifecycleCommandActive(generation, operation_token)) {
    response->success = false;
    response->message = "mapping superseded by a newer start request";
    return;
  }
  if (!start_result.success()) {
    response->success = false;
    response->message = start_result.message;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    return;
  }

  state_machine_.mappingStarted();
  publishMode();
  response->success = true;
  response->message = "mapping started, map_id=" + active_map_id_;
}

void SlamManagerNode::finishMappingCallback(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=finish_mapping mode=%s session=%llu "
              "active_map='%s'",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_),
              active_map_id_.c_str());

  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);

  // 只有APP确认完成建图时才创建保存目录。同名地图也在此刻覆盖；准备
  // 失败时保持MAPPING和Worker运行，允许APP修复存储问题后重试。
  if (state_machine_.mode() != SlamMode::kMapping) {
    response->success = false;
    response->message = "finish mapping rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }
  std::string prepare_message;
  if (!map_storage_->prepareForSave(active_map_id_, prepare_message)) {
    response->success = false;
    response->message =
        "failed to prepare map save directory: " + prepare_message;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }
  active_map_directory_prepared_ = true;
  RCLCPP_INFO(get_logger(), "%s", prepare_message.c_str());

  // 协议2.2：停止数据生产、触发PGO保存并发布正式地图目录。
  if (!state_machine_.beginStopMapping()) {
    response->success = false;
    response->message = "finish mapping rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  publishMode();
  // Service等待Worker停止、PGO保存和目录发布全部结束后再响应APP。
  const auto stop_result = awaitWorkerCommand(
      "mapping workers stop", finish_mapping_timeout_sec_,
      std::make_shared<WorkerOperationToken>(),
      [this](WorkerCompletion completion) {
        mapping_controller_->finish(session_id_, active_map_path_.string(),
                                    std::move(completion));
      });
  if (!stop_result.success()) {
    response->success = false;
    response->message = "finish mapping failed: " + stop_result.message;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  // PGO停止时已将globalmap.pcd保存到本次finish_mapping创建的map目录。
  std::string finalize_message;
  if (!map_storage_->finalize(active_map_path_, active_map_id_,
                              finalize_message)) {
    response->success = false;
    response->message = "map save failed: " + finalize_message;
    enterError(RuntimeState::MAP_ERROR, response->message);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }
  active_map_id_.clear();
  active_map_path_.clear();
  active_map_directory_prepared_ = false;

  state_machine_.mappingStopped();
  publishMode();
  response->success = true;
  response->message = finalize_message;
  RCLCPP_INFO(get_logger(), "%s; %s", stop_result.message.c_str(),
              finalize_message.c_str());
}

void SlamManagerNode::cancelMappingCallback(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=cancel_mapping mode=%s session=%llu "
              "active_map='%s'",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_),
              active_map_id_.c_str());

  const SlamMode current_mode = state_machine_.mode();
  if (active_map_id_.empty() || current_mode != SlamMode::kMapping) {
    response->success = false;
    response->message = "cancel mapping rejected, current state=" +
                        std::string(toString(current_mode));
    return;
  }

  // 纳入生命周期命令序列，再串行执行Worker安全停止。
  const uint64_t generation = beginLifecycleCommand("cancel_mapping");
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const auto operation_token = installLifecycleToken(generation);
  if (!operation_token) {
    response->success = false;
    response->message = "cancel mapping superseded by a newer command";
    return;
  }

  if (active_map_id_.empty() || !state_machine_.beginStopMapping()) {
    response->success = false;
    response->message = "cancel mapping rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }
  publishMode();

  const auto cancel_result = awaitWorkerCommand(
      "mapping workers cancel", finish_mapping_timeout_sec_, operation_token,
      [this](WorkerCompletion completion) {
        mapping_controller_->cancel(session_id_, std::move(completion));
      },
      [this, generation, operation_token]() {
        return isLifecycleCommandActive(generation, operation_token);
      });
  if (cancel_result.status == WorkerCommandStatus::kSuperseded ||
      !isLifecycleCommandActive(generation, operation_token)) {
    response->success = false;
    response->message = "cancel mapping superseded by a newer command";
    return;
  }
  if (!cancel_result.success()) {
    response->success = false;
    response->message = cancel_result.message;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    return;
  }

  const std::string cancelled_map_id = active_map_id_;
  if (active_map_directory_prepared_) {
    std::string delete_message;
    if (!map_storage_->deleteMap(active_map_id_, delete_message)) {
      response->success = false;
      response->message = "failed to discard incomplete map: " + delete_message;
      enterError(RuntimeState::MAP_ERROR, response->message);
      return;
    }
  }
  active_map_id_.clear();
  active_map_path_.clear();
  active_map_directory_prepared_ = false;
  if (!state_machine_.mappingStopped()) {
    response->success = false;
    response->message = "failed to enter idle state after cancelling mapping";
    enterError(RuntimeState::SLAM_ERROR, response->message);
    return;
  }
  publishMode();
  response->success = true;
  response->message =
      "mapping cancelled without saving, map_id=" + cancelled_map_id;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void SlamManagerNode::loadMapCallback(const LoadMap::Request::SharedPtr request,
                                      LoadMap::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=load_map map_id='%s' mode=%s "
              "session=%llu selected_map='%s' reloc_success=%s",
              request->map_id.c_str(),
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_),
              selected_map_id_.c_str(),
              relocalization_succeeded_.load() ? "true" : "false");

  const std::string &map_id = request->map_id;
  if (!validMapId(map_id, response->message)) {
    response->success = false;
    return;
  }

  const auto global_map_path = map_storage_->mapPath(map_id) / "globalmap.pcd";
  std::error_code filesystem_error;
  const bool map_file_exists =
      std::filesystem::exists(global_map_path, filesystem_error);
  if (filesystem_error || !map_file_exists) {
    response->success = false;
    response->message =
        filesystem_error
            ? "failed to inspect map: " + filesystem_error.message()
            : "map is incomplete or does not exist: " + map_id;
    return;
  }

  const bool is_regular_file =
      std::filesystem::is_regular_file(global_map_path, filesystem_error);
  if (filesystem_error || !is_regular_file) {
    response->success = false;
    response->message =
        filesystem_error
            ? "failed to inspect map file: " + filesystem_error.message()
            : "globalmap.pcd is not a regular file: " +
                  global_map_path.string();
    return;
  }

  const uint64_t generation = beginLifecycleCommand("load_map");
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const auto operation_token = installLifecycleToken(generation);
  if (!operation_token) {
    response->success = false;
    response->message = "load map superseded by a newer lifecycle command";
    return;
  }

  // load_map也是替换型生命周期命令：先完整销毁建图、定位和导航算法，
  // 再进入地图加载状态，但它本身不启动定位算法。
  std::string teardown_message;
  if (!prepareForStartCommand(generation, operation_token, teardown_message)) {
    response->success = false;
    response->message = teardown_message;
    if (isLifecycleCommandActive(generation, operation_token)) {
      enterError(RuntimeState::MAP_ERROR, response->message);
    }
    return;
  }
  // 体素重新生成期间该地图尚不可用于定位；失败或被替代时也不能保留
  // 一个指向不完整voxelmaps_coords的选择结果。
  selected_map_id_.clear();
  navigation_task_executor_->configureMap({}, {});
  if (!state_machine_.beginLoadMap()) {
    response->success = false;
    response->message = "load map state transition failed, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  publishMode();
  const auto finish_loading = [this]() {
    if (!state_machine_.loadMapFinished()) {
      RCLCPP_ERROR(get_logger(), "load_map failed to leave loading_map state");
      state_machine_.enterError();
    }
    publishMode();
  };

  // 每次load_map都删除旧体素并基于当前globalmap.pcd重新生成，
  // 避免APP覆盖或拼接PCD后误用上一版BBS。
  std::string bbs_message;
  if (!bbs_voxel_generator_->generate(
          map_storage_->mapPath(map_id), map_id, bbs_message,
          [this, generation, operation_token]() {
            return !isLifecycleCommandActive(generation, operation_token);
          })) {
    response->success = false;
    response->message =
        isLifecycleCommandActive(generation, operation_token)
            ? "load map failed: 3D-BBS generation failed: " + bbs_message
            : "load map superseded by a newer lifecycle "
              "command";
    finish_loading();
    if (isLifecycleCommandActive(generation, operation_token)) {
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
    return;
  }
  if (!isLifecycleCommandActive(generation, operation_token)) {
    response->success = false;
    response->message = "load map superseded by a newer lifecycle command";
    finish_loading();
    return;
  }
  RCLCPP_INFO(get_logger(), "%s", bbs_message.c_str());

  // BBS准备完成后才选择地图，不读取/发布PCD，也不启动定位。
  selected_map_id_ = map_id;
  navigation_task_executor_->configureMap(map_storage_->taskPath(map_id),
                                          map_storage_->mapPath(map_id));
  {
    // 切换地图后旧地图位姿失效，等待新一轮定位输出map位姿。
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    current_pose_ = geometry_msgs::msg::PoseStamped();
    has_current_pose_ = false;
  }
  response->success = true;
  response->message = "map selected: " + selected_map_id_ + "; " + bbs_message;
  finish_loading();
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

rclcpp_action::GoalResponse SlamManagerNode::handleLocalizationGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const StartLocalization::Goal> goal) {
  RCLCPP_INFO(
      get_logger(),
      "[APP][ACTION_GOAL] command=start_localization "
      "use_initial_pose=%s pose=(%.6f,%.6f,%.6f,%.6f) mode=%s "
      "session=%llu selected_map='%s' reloc_success=%s",
      goal->use_initial_pose ? "true" : "false", goal->x, goal->y, goal->z,
      goal->theta, std::string(toString(state_machine_.mode())).c_str(),
      static_cast<unsigned long long>(session_id_), selected_map_id_.c_str(),
      relocalization_succeeded_.load() ? "true" : "false");

  if (goal->use_initial_pose &&
      !finitePoseFields(goal->x, goal->y, goal->z, goal->theta)) {
    RCLCPP_WARN(get_logger(),
                "Reject localization goal: initial pose contains non-finite "
                "values");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (shutting_down_.load()) {
    RCLCPP_WARN(get_logger(),
                "Reject localization goal: the node is shutting down");
    return rclcpp_action::GoalResponse::REJECT;
  }
  bool expected = false;
  if (!localization_action_active_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(get_logger(),
                "Reject localization goal: another localization action is "
                "active; cancel it before sending a new goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SlamManagerNode::handleLocalizationCancel(
    const std::shared_ptr<StartLocalizationGoalHandle>) {
  RCLCPP_INFO(get_logger(),
              "[APP][ACTION_CANCEL] command=start_localization mode=%s "
              "session=%llu",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_));
  localization_result_cv_.notify_all();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SlamManagerNode::handleLocalizationAccepted(
    const std::shared_ptr<StartLocalizationGoalHandle> goal_handle) {
  std::lock_guard<std::mutex> lock(localization_action_thread_mutex_);
  // active标志只在线程完全退出时释放，因此这里最多回收一个已经结束的线程，
  // 不会等待仍在执行的Action。
  if (localization_action_thread_.joinable())
    localization_action_thread_.join();
  try {
    localization_action_thread_ = std::thread([this, goal_handle]() {
      ScopeExit release_action(
          [this]() { localization_action_active_.store(false); });
      try {
        executeLocalizationAction(goal_handle);
      } catch (const std::exception &error) {
        abortLocalizationActionAfterException(
            goal_handle,
            std::string("localization action exception: ") + error.what());
      } catch (...) {
        abortLocalizationActionAfterException(
            goal_handle, "unknown localization action exception");
      }
    });
  } catch (const std::exception &error) {
    localization_action_active_.store(false);
    auto result = std::make_shared<StartLocalization::Result>();
    result->success = false;
    result->message =
        std::string("failed to start localization action thread: ") +
        error.what();
    goal_handle->abort(result);
  }
}

void SlamManagerNode::executeLocalizationAction(
    const std::shared_ptr<StartLocalizationGoalHandle> goal_handle) {
  const auto goal = goal_handle->get_goal();
  const auto started_at = std::chrono::steady_clock::now();
  const auto publish_feedback = [goal_handle,
                                 started_at](const std::string &message) {
    if (!goal_handle->is_active()) return;
    auto feedback = std::make_shared<StartLocalization::Feedback>();
    feedback->elapsed_time = std::chrono::duration<float>(
                                 std::chrono::steady_clock::now() - started_at)
                                 .count();
    feedback->message = message;
    goal_handle->publish_feedback(feedback);
  };
  const auto finish_action = [this, goal_handle](
                                 const RelocalizationResult &attempt_result) {
    auto result = std::make_shared<StartLocalization::Result>();
    result->success = attempt_result.success();
    result->message = attempt_result.message;
    if (goal_handle->is_active()) {
      if (attempt_result.status == RelocalizationStatus::kSucceeded) {
        goal_handle->succeed(result);
      } else if (attempt_result.status == RelocalizationStatus::kCancelled ||
                 goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    }
  };

  publish_feedback("starting localization");
  if (goal_handle->is_canceling() || shutting_down_.load()) {
    RelocalizationResult result;
    result.status = RelocalizationStatus::kCancelled;
    result.message = "app relocalization cancelled";
    finish_action(result);
    return;
  }

  const uint64_t generation = beginLifecycleCommand("start_localization");
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const auto operation_token = installLifecycleToken(generation);
  if (!operation_token) {
    RelocalizationResult result;
    result.status = RelocalizationStatus::kSuperseded;
    result.message = "relocalization superseded by a newer start request";
    finish_action(result);
    return;
  }

  std::string teardown_message;
  if (!prepareForStartCommand(generation, operation_token, teardown_message)) {
    RelocalizationResult result;
    if (!isLifecycleCommandActive(generation, operation_token)) {
      result.status = RelocalizationStatus::kSuperseded;
      result.message = "relocalization superseded by a newer start request";
      finish_action(result);
      return;
    }
    result.message = teardown_message;
    enterError(RuntimeState::LOCALIZATION_ERROR, result.message);
    finish_action(result);
    return;
  }

  // 运行时前置条件在统一生命周期锁内检查，避免load_map并发修改选择结果。
  if (selected_map_id_.empty()) {
    RelocalizationResult result;
    result.message = "no map selected; call /robot/load_map first";
    finish_action(result);
    return;
  }
  const std::string selected_map_id = selected_map_id_;
  const auto selected_map_path = map_storage_->mapPath(selected_map_id);
  std::error_code exists_error;
  if (!std::filesystem::exists(selected_map_path / "globalmap.pcd",
                               exists_error) ||
      exists_error) {
    RelocalizationResult result;
    result.message =
        exists_error
            ? "failed to inspect selected map: " + exists_error.message()
            : "selected map no longer exists: " + selected_map_id;
    finish_action(result);
    return;
  }

  geometry_msgs::msg::Pose initial_pose;
  initial_pose.orientation.w = 1.0;
  if (goal->use_initial_pose) {
    initial_pose =
        pose_utils::fromXyzTheta(goal->x, goal->y, goal->z, goal->theta);
  }

  RelocalizationOptions options;
  options.source = "app";
  options.map_id = selected_map_id;
  options.map_path = selected_map_path;
  options.use_initial_pose = goal->use_initial_pose;
  options.algorithm_profile = "initial";
  options.initial_pose = initial_pose;
  options.generation = generation;
  options.operation_token = operation_token;
  options.stop_existing_worker = false;
  options.is_canceling = [this, goal_handle]() {
    return goal_handle->is_canceling() || shutting_down_.load();
  };
  options.publish_feedback = publish_feedback;

  const auto attempt_result =
      runRelocalizationAttempt(options, std::move(lifecycle_lock));
  finish_action(attempt_result);
}

SlamManagerNode::RelocalizationResult SlamManagerNode::runRelocalizationAttempt(
    const RelocalizationOptions &options,
    std::unique_lock<std::mutex> lifecycle_lock) {
  RelocalizationResult result;
  const auto superseded = [this, &options]() {
    return !isLatestCommand(options.generation) ||
           (options.operation_token && !options.operation_token->active());
  };
  const auto cancelled = [&options]() {
    return options.is_canceling && options.is_canceling();
  };

  if (!lifecycle_lock.owns_lock()) {
    result.message = "relocalization lifecycle lock is not held";
    return result;
  }
  if (superseded()) {
    result.status = RelocalizationStatus::kSuperseded;
    result.message = "relocalization superseded by a newer start request";
    return result;
  }

  if (options.stop_existing_worker) {
    const uint64_t active_session_id = active_localization_session_id_.load();
    RCLCPP_INFO(get_logger(),
                "Stopping localization before %s: active_session=%llu",
                options.source.c_str(),
                static_cast<unsigned long long>(active_session_id));
    const auto stop_result = awaitWorkerCommand(
        "localization stop", localization_stop_timeout_sec_,
        options.operation_token ? options.operation_token
                                : std::make_shared<WorkerOperationToken>(),
        [this, active_session_id](WorkerCompletion completion) {
          localization_controller_->stop(active_session_id,
                                         std::move(completion));
        },
        [this, generation = options.generation,
         token = options.operation_token]() {
          return isLatestCommand(generation) && (!token || token->active());
        });
    if (superseded()) {
      result.status = RelocalizationStatus::kSuperseded;
      result.message = "relocalization superseded by a newer start request";
      return result;
    }
    if (!stop_result.success()) {
      result.message = "failed to stop localization before " + options.source +
                       ": " + stop_result.message;
      enterError(RuntimeState::LOCALIZATION_ERROR, result.message);
      return result;
    }
    active_localization_session_id_.store(0U);
  }

  if (cancelled()) {
    result.status = RelocalizationStatus::kCancelled;
    result.message = options.source + " relocalization cancelled";
    return result;
  }
  if (options.preserve_navigation_state) {
    if (state_machine_.mode() != SlamMode::kNavigating) {
      result.message = options.source +
                       " relocalization requires active navigation, current "
                       "state=" +
                       std::string(toString(state_machine_.mode()));
      return result;
    }
  } else {
    if (!state_machine_.beginLocalization()) {
      result.message = options.source +
                       " relocalization rejected, current state=" +
                       std::string(toString(state_machine_.mode()));
      enterError(RuntimeState::LOCALIZATION_ERROR, result.message);
      return result;
    }
    publishMode();
  }
  relocalization_succeeded_.store(false);

  ++session_id_;
  const uint64_t localization_session_id = session_id_;
  result.session_id = localization_session_id;
  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    localization_attempt_session_id_ = localization_session_id;
    localization_attempt_active_ = true;
    localization_result_armed_ = false;
    localization_result_ready_ = false;
    localization_result_success_ = false;
    localization_result_message_.clear();
  }

  const WorkerStartContext worker_context{
      options.map_id, options.map_path.string(), options.use_initial_pose,
      options.algorithm_profile};
  localization_controller_->start(
      localization_session_id, worker_context,
      [this, generation = options.generation,
       operation_token = options.operation_token, localization_session_id,
       source = options.source, use_initial_pose = options.use_initial_pose,
       algorithm_profile = options.algorithm_profile,
       preserve_navigation_state = options.preserve_navigation_state,
       initial_pose = options.initial_pose](bool success,
                                            const std::string &worker_message) {
        if (!isLatestCommand(generation) ||
            (operation_token && !operation_token->active())) {
          return;
        }
        {
          std::lock_guard<std::mutex> lock(localization_result_mutex_);
          if (!localization_attempt_active_ ||
              localization_attempt_session_id_ != localization_session_id ||
              localization_result_ready_) {
            return;
          }
        }

        if (!success) {
          {
            std::lock_guard<std::mutex> lock(localization_result_mutex_);
            if (!localization_attempt_active_ ||
                localization_attempt_session_id_ != localization_session_id ||
                localization_result_ready_) {
              return;
            }
            localization_result_ready_ = true;
            localization_result_success_ = false;
            localization_result_message_ =
                "failed to start localization: " + worker_message;
          }
          RCLCPP_ERROR(get_logger(), "%s", worker_message.c_str());
          localization_result_cv_.notify_all();
          return;
        }

        active_localization_session_id_.store(localization_session_id);

        if (!preserve_navigation_state &&
            !state_machine_.localizationStarted()) {
          {
            std::lock_guard<std::mutex> lock(localization_result_mutex_);
            if (localization_attempt_session_id_ == localization_session_id) {
              localization_result_ready_ = true;
              localization_result_success_ = false;
              localization_result_message_ =
                  "localization state changed while worker was starting";
            }
          }
          localization_result_cv_.notify_all();
          return;
        }

        {
          std::lock_guard<std::mutex> lock(localization_result_mutex_);
          if (!localization_attempt_active_ ||
              localization_attempt_session_id_ != localization_session_id ||
              localization_result_ready_) {
            return;
          }
          localization_result_armed_ = true;
        }
        if (use_initial_pose) {
          geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_message;
          initial_pose_message.header.stamp = now();
          initial_pose_message.header.frame_id = "map";
          initial_pose_message.pose.pose = initial_pose;
          initial_pose_pub_->publish(initial_pose_message);
        }
        RCLCPP_INFO(get_logger(),
                    "Relocalization worker started: source=%s, session=%llu, "
                    "profile=%s, use_initial_pose=%s, %s",
                    source.c_str(),
                    static_cast<unsigned long long>(localization_session_id),
                    algorithm_profile.c_str(),
                    use_initial_pose ? "true" : "false",
                    worker_message.c_str());
        publishMode();
      });
  lifecycle_lock.unlock();

  bool result_received = false;
  bool result_success = false;
  bool was_cancelled = false;
  bool was_superseded = false;
  std::string result_message;
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(relocalization_timeout_sec_);
  auto next_feedback =
      std::chrono::steady_clock::now() + std::chrono::seconds(1);
  {
    std::unique_lock<std::mutex> lock(localization_result_mutex_);
    while (localization_attempt_session_id_ == localization_session_id &&
           !localization_result_ready_) {
      was_superseded = superseded();
      was_cancelled = cancelled();
      if (was_superseded || was_cancelled ||
          std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      localization_result_cv_.wait_for(lock, std::chrono::milliseconds(100));
      const auto current_time = std::chrono::steady_clock::now();
      if (options.publish_feedback && current_time >= next_feedback) {
        const std::string feedback_message = localization_result_armed_
                                                 ? "relocalizing"
                                                 : "starting localization";
        lock.unlock();
        options.publish_feedback(feedback_message);
        lock.lock();
        next_feedback = current_time + std::chrono::seconds(1);
      }
    }
    if (localization_attempt_session_id_ != localization_session_id) {
      was_superseded = true;
    } else {
      result_received = localization_result_ready_;
      result_success = localization_result_success_;
      result_message = localization_result_message_;
      localization_attempt_active_ = false;
      localization_result_armed_ = false;
    }
  }

  if (was_superseded || superseded()) {
    result.status = RelocalizationStatus::kSuperseded;
    result.message = "relocalization superseded by a newer start request";
    return result;
  }

  const auto reacquire_if_current = [&]() {
    while (!lifecycle_lock.try_lock()) {
      if (superseded()) return false;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return !superseded();
  };
  if (!reacquire_if_current()) {
    result.status = RelocalizationStatus::kSuperseded;
    result.message = "relocalization superseded by a newer start request";
    return result;
  }

  if (result_received && result_success && !was_cancelled && !cancelled()) {
    if (!options.preserve_navigation_state &&
        !state_machine_.localizationSucceeded()) {
      result.message = "failed to enter localization success state";
      enterError(RuntimeState::LOCALIZATION_ERROR, result.message);
      return result;
    }
    relocalization_succeeded_.store(true);
    result.status = RelocalizationStatus::kSucceeded;
    result.message = "relocalization succeeded, map_id=" + options.map_id;
    RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
    publishMode();
    return result;
  }

  if (was_cancelled || cancelled()) {
    result.status = RelocalizationStatus::kCancelled;
    result.message = options.source + " relocalization cancelled";
  } else if (result_received) {
    result.message = result_message.empty()
                         ? "relocalization failed without a result message"
                         : result_message;
  } else {
    result.status = RelocalizationStatus::kTimedOut;
    result.message = "relocalization timed out after " +
                     std::to_string(relocalization_timeout_sec_) +
                     " seconds, map_id=" + options.map_id;
  }

  const auto cleanup_result = awaitWorkerCommand(
      "localization stop", localization_stop_timeout_sec_,
      options.operation_token ? options.operation_token
                              : std::make_shared<WorkerOperationToken>(),
      [this, localization_session_id](WorkerCompletion completion) {
        localization_controller_->stop(localization_session_id,
                                       std::move(completion));
      },
      [this, generation = options.generation,
       token = options.operation_token]() {
        return isLatestCommand(generation) && (!token || token->active());
      });
  if (superseded()) {
    result.status = RelocalizationStatus::kSuperseded;
    result.message = "relocalization superseded by a newer start request";
    return result;
  }
  if (cleanup_result.success()) {
    uint64_t expected_session_id = localization_session_id;
    active_localization_session_id_.compare_exchange_strong(expected_session_id,
                                                            0U);
    if (!options.preserve_navigation_state) {
      state_machine_.localizationFailed();
      publishMode();
    }
  } else {
    enterError(RuntimeState::LOCALIZATION_ERROR, cleanup_result.message);
  }
  result.message += "; " + cleanup_result.message;
  RCLCPP_WARN(get_logger(), "%s", result.message.c_str());
  return result;
}

bool SlamManagerNode::relocalizeForMapConnection(
    const slam_manager_interfaces::msg::NavigationNode &node,
    const std::function<bool()> &is_canceling, std::string &message) {
  if (selected_map_id_.empty()) {
    message = "no map selected for map_connection relocalization";
    return false;
  }
  if (!finitePoseFields(node.x, node.y, node.z, node.theta)) {
    message = "map_connection node contains a non-finite pose";
    return false;
  }

  const std::string map_id = selected_map_id_;
  const auto selected_map_path = map_storage_->mapPath(map_id);
  std::error_code exists_error;
  if (!std::filesystem::exists(selected_map_path / "globalmap.pcd",
                               exists_error) ||
      exists_error) {
    message = exists_error
                  ? "failed to inspect selected map: " + exists_error.message()
                  : "selected map no longer exists: " + map_id;
    return false;
  }

  const uint64_t generation = command_generation_.load();
  // 导航线程不能永久阻塞在生命周期锁上：新APP命令会先cancel导航，随后
  // 持锁调用stopAndWait；这里必须观察取消并退出，避免双方互相等待。
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_,
                                              std::defer_lock);
  while (!lifecycle_lock.try_lock()) {
    if ((is_canceling && is_canceling()) || !isLatestCommand(generation)) {
      message =
          "map_connection relocalization cancelled while waiting for "
          "lifecycle access";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!isLatestCommand(generation)) {
    message = "map_connection relocalization superseded by a newer request";
    return false;
  }

  RelocalizationOptions options;
  options.source = "map_connection";
  options.map_id = map_id;
  options.map_path = selected_map_path;
  options.use_initial_pose = true;
  options.algorithm_profile = "map_connection";
  options.initial_pose =
      pose_utils::fromXyzTheta(node.x, node.y, node.z, node.theta);
  options.generation = generation;
  options.stop_existing_worker = true;
  options.preserve_navigation_state = true;
  options.is_canceling = is_canceling;

  const auto result =
      runRelocalizationAttempt(options, std::move(lifecycle_lock));
  message = result.message;
  return result.success();
}

void SlamManagerNode::relocalizationResultCallback(
    const std_msgs::msg::Bool::SharedPtr message) {
  // 协议2.5：只有true表示最终定位成功；false表示算法仍在搜索。
  // false表示算法仍在尝试，不作为最终失败；最终失败由超时统一判定。
  if (!message->data) return;

  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    if (!localization_attempt_active_ || !localization_result_armed_ ||
        localization_result_ready_) {
      return;
    }
    localization_result_ready_ = true;
    localization_result_success_ = true;
    localization_result_message_ = "relocalization succeeded";
  }
  localization_result_cv_.notify_all();
}

void SlamManagerNode::keyframeCloudCallback(
    const MapKeyframe::SharedPtr message) {
  // 协议2.12：PGO负责关键帧选择、ID和map坐标变换，Runtime只做转发。
  // PGO仅在新增关键帧时发布一次；回环后不重发旧关键帧。
  // APP实时拼接结果用于建图预览，最终地图以保存后的globalmap.pcd为准。
  keyframe_cloud_pub_->publish(*message);
}

void SlamManagerNode::deleteMapCallback(
    const DeleteMap::Request::SharedPtr request,
    DeleteMap::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=delete_map map_id='%s' mode=%s "
              "session=%llu selected_map='%s' reloc_success=%s",
              request->map_id.c_str(),
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_),
              selected_map_id_.c_str(),
              relocalization_succeeded_.load() ? "true" : "false");

  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);

  const std::string &map_id = request->map_id;
  if (!validMapId(map_id, response->message)) {
    response->success = false;
    return;
  }

  // 非当前使用的地图可在任意运行状态下删除。当前选中的地图仅允许在
  // 空闲状态删除，避免重定位或导航算法访问已删除文件。
  const bool deleting_selected_map = map_id == selected_map_id_;
  if (deleting_selected_map && state_machine_.mode() != SlamMode::kIdle) {
    response->success = false;
    response->message = "delete map rejected, selected map is in use in " +
                        std::string(toString(state_machine_.mode())) +
                        " state: " + map_id;
    return;
  }
  if (!active_map_id_.empty() && map_id == active_map_id_) {
    response->success = false;
    response->message =
        "delete map rejected, map is currently being created: " + map_id;
    return;
  }

  response->success = map_storage_->deleteMap(map_id, response->message);
  if (response->success) {
    if (deleting_selected_map) {
      selected_map_id_.clear();
      navigation_task_executor_->configureMap({}, {});
      relocalization_succeeded_.store(false);
      std::lock_guard<std::mutex> lock(current_pose_mutex_);
      current_pose_ = geometry_msgs::msg::PoseStamped();
      has_current_pose_ = false;
    }
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Delete map rejected: %s",
                response->message.c_str());
  }
}

rclcpp_action::GoalResponse SlamManagerNode::handleNavigationGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const StartNavigation::Goal> goal) {
  RCLCPP_INFO(
      get_logger(),
      "[APP][ACTION_GOAL] command=start_navigation task_id='%s' "
      "loop=%s nodes=%zu mode=%s session=%llu selected_map='%s' "
      "reloc_success=%s",
      goal->task_id.c_str(), goal->loop ? "true" : "false", goal->nodes.size(),
      std::string(toString(state_machine_.mode())).c_str(),
      static_cast<unsigned long long>(session_id_), selected_map_id_.c_str(),
      relocalization_succeeded_.load() ? "true" : "false");

  for (std::size_t i = 0; i < goal->nodes.size(); ++i) {
    const auto &node = goal->nodes[i];

    RCLCPP_INFO(get_logger(),
                "[APP][ACTION_GOAL] command=start_navigation node[%zu] "
                "node_id=%u pose=(%.6f,%.6f,%.6f,%.6f) stop_sec=%.3f",
                i, node.node_id, node.x, node.y, node.z, node.theta,
                node.stop_duration_sec);
  }
  // 协议2.6：导航要求状态空闲并且本轮重定位已经成功。
  std::string message;
  if (!navigation_task_executor_->validateGoal(*goal, message)) {
    RCLCPP_WARN(get_logger(), "Reject navigation goal: %s", message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state_machine_.mode() != SlamMode::kIdle ||
      !relocalization_succeeded_.load()) {
    const std::string current_state(toString(state_machine_.mode()));
    RCLCPP_WARN(get_logger(), "Reject navigation goal: current state=%s",
                current_state.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!navigation_task_executor_->reserveOperation(message)) {
    RCLCPP_WARN(get_logger(), "Reject navigation goal: %s", message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SlamManagerNode::handleNavigationCancel(
    const std::shared_ptr<StartNavigationGoalHandle>) {
  RCLCPP_INFO(get_logger(),
              "[APP][ACTION_CANCEL] command=start_navigation mode=%s "
              "session=%llu",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_));
  navigation_task_executor_->cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SlamManagerNode::handleNavigationAccepted(
    const std::shared_ptr<StartNavigationGoalHandle> goal_handle) {
  ++session_id_;
  RCLCPP_INFO(get_logger(),
              "[APP][ACTION_ACCEPTED] command=start_navigation session=%llu",
              static_cast<unsigned long long>(session_id_));
  navigation_task_executor_->startPatrolAction(goal_handle, session_id_);
}

rclcpp_action::GoalResponse SlamManagerNode::handleNavigateToPoseGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal) {
  RCLCPP_INFO(get_logger(),
              "[APP][ACTION_GOAL] command=navigate_to_pose "
              "pose=(%.6f,%.6f,%.6f,%.6f) mode=%s session=%llu "
              "selected_map='%s' reloc_success=%s",
              goal->x, goal->y, goal->z, goal->theta,
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_),
              selected_map_id_.c_str(),
              relocalization_succeeded_.load() ? "true" : "false");

  // 协议2.7：校验目标、空闲状态、重定位结果和当前map位姿。
  if (state_machine_.mode() != SlamMode::kIdle ||
      !relocalization_succeeded_.load()) {
    const std::string current_state(toString(state_machine_.mode()));
    RCLCPP_WARN(get_logger(),
                "Reject single-point goal: relocalization has not succeeded, "
                "current state=%s",
                current_state.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::string message;
  if (!navigation_task_executor_->validateGoal(*goal, message)) {
    RCLCPP_WARN(get_logger(), "Reject single-point goal: %s", message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    if (!has_current_pose_) {
      RCLCPP_WARN(get_logger(),
                  "Reject single-point goal: current map pose is unavailable");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (!navigation_task_executor_->reserveOperation(message)) {
    RCLCPP_WARN(get_logger(), "Reject single-point goal: %s", message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SlamManagerNode::handleNavigateToPoseCancel(
    const std::shared_ptr<NavigateToPoseGoalHandle>) {
  RCLCPP_INFO(get_logger(),
              "[APP][ACTION_CANCEL] command=navigate_to_pose mode=%s "
              "session=%llu",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_));
  navigation_task_executor_->cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SlamManagerNode::handleNavigateToPoseAccepted(
    const std::shared_ptr<NavigateToPoseGoalHandle> goal_handle) {
  geometry_msgs::msg::PoseStamped current_pose;
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    current_pose = current_pose_;
  }
  ++session_id_;
  RCLCPP_INFO(get_logger(),
              "[APP][ACTION_ACCEPTED] command=navigate_to_pose session=%llu",
              static_cast<unsigned long long>(session_id_));
  navigation_task_executor_->startNavigateToPoseAction(
      goal_handle, current_pose, session_id_);
}

bool SlamManagerNode::robotStateAllowsNavigation(std::string &message) {
  const auto timeout = std::chrono::duration<double>(kRobotStateTimeoutSec);
  if (!get_robot_state_client_->wait_for_service(timeout)) {
    message = std::string("robot state service '") + kRobotStateServiceName +
              "' is unavailable";
    RCLCPP_WARN(get_logger(), "[NAVIGATION_GATE] %s", message.c_str());
    return false;
  }

  try {
    auto request = std::make_shared<GetState::Request>();
    auto future = get_robot_state_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      get_robot_state_client_->remove_pending_request(future);
      message = std::string("robot state service '") + kRobotStateServiceName +
                "' timed out after " + std::to_string(kRobotStateTimeoutSec) +
                " seconds";
      RCLCPP_WARN(get_logger(), "[NAVIGATION_GATE] %s", message.c_str());
      return false;
    }

    const auto response = future.get();
    const bool allowed =
        response->code == 0 && response->state == kNavigationRequiredRobotState;
    RCLCPP_INFO(get_logger(),
                "[NAVIGATION_GATE] GetState response: state='%s' code=%d "
                "required_state='%s' allowed=%s",
                response->state.c_str(), response->code,
                kNavigationRequiredRobotState, allowed ? "true" : "false");
    if (!allowed) {
      message = "navigation rejected: robot state='" + response->state +
                "', code=" + std::to_string(response->code) +
                "; required state='" + kNavigationRequiredRobotState +
                "' with code=0";
      RCLCPP_WARN(get_logger(), "[NAVIGATION_GATE] %s", message.c_str());
      return false;
    }

    message = "robot state='" + response->state + "', code=0";
    return true;
  } catch (const std::exception &error) {
    message = std::string("failed to query robot state from '") +
              kRobotStateServiceName + "': " + error.what();
    RCLCPP_ERROR(get_logger(), "[NAVIGATION_GATE] %s", message.c_str());
    return false;
  }
}

bool SlamManagerNode::setRobotMode(int32_t mode, std::string &message) {
  if (!enable_set_mode_) {
    message = "SetMode disabled by navigation.enable_set_mode";
    RCLCPP_INFO(get_logger(), "[NAVIGATION_MODE] skipped mode=%d: %s", mode,
                message.c_str());
    return true;
  }

  const auto timeout = std::chrono::duration<double>(kRobotModeTimeoutSec);
  if (!set_robot_mode_client_->wait_for_service(timeout)) {
    message = std::string("robot mode service '") + kRobotModeServiceName +
              "' is unavailable";
    RCLCPP_WARN(get_logger(), "[NAVIGATION_MODE] %s", message.c_str());
    return false;
  }

  try {
    auto request = std::make_shared<SetMode::Request>();
    request->mode = mode;
    auto future = set_robot_mode_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      set_robot_mode_client_->remove_pending_request(future);
      message = std::string("robot mode service '") + kRobotModeServiceName +
                "' timed out while setting mode=" + std::to_string(mode);
      RCLCPP_WARN(get_logger(), "[NAVIGATION_MODE] %s", message.c_str());
      return false;
    }

    const auto response = future.get();
    const bool success = response->code == 0;
    message = "SetMode mode=" + std::to_string(mode) +
              ", code=" + std::to_string(response->code);
    RCLCPP_INFO(get_logger(),
                "[NAVIGATION_MODE] SetMode response: mode=%d code=%d "
                "success=%s",
                mode, response->code, success ? "true" : "false");
    return success;
  } catch (const std::exception &error) {
    message = std::string("failed to set robot mode through '") +
              kRobotModeServiceName + "': " + error.what();
    RCLCPP_ERROR(get_logger(), "[NAVIGATION_MODE] %s", message.c_str());
    return false;
  }
}

void SlamManagerNode::navigationStateCallback(
    const std_msgs::msg::Bool::SharedPtr message) {
  // Pure Pursuit发布false表示开始跟踪，随后true表示当前目标到达。
  navigation_task_executor_->onNavigationState(message->data);
}

void SlamManagerNode::currentPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message) {
  // 建图位姿没有定位匹配分数，按协议发布0。
  publishCurrentPose(*message, 0.0F);
}

void SlamManagerNode::localizationPoseCallback(
    const LocalizationPose::SharedPtr message) {
  publishCurrentPose(message->pose,
                     std::clamp(message->match_score, 0.0F, 1.0F));
}

void SlamManagerNode::publishCurrentPose(
    const geometry_msgs::msg::PoseStamped &pose, float match_score) {
  // 协议2.10：内部保留完整Pose供导航，向APP发布平面位姿和匹配分数。
  {
    std::lock_guard<std::mutex> lock(current_pose_mutex_);
    current_pose_ = pose;
    has_current_pose_ = true;
  }
  RobotPose robot_pose;
  robot_pose.map_id =
      !active_map_id_.empty() ? active_map_id_ : selected_map_id_;
  robot_pose.x = pose.pose.position.x;
  robot_pose.y = pose.pose.position.y;
  robot_pose.z = pose.pose.position.z;
  robot_pose.theta = tf2::getYaw(pose.pose.orientation);
  robot_pose.match_score = match_score;
  current_pose_pub_->publish(robot_pose);
}

void SlamManagerNode::bodyCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message) {
  // 协议2.11：按APP配置频率转发FAST-LIO的降采样机体点云。
  const double message_time = rclcpp::Time(message->header.stamp).seconds();
  const double publish_period = 1.0 / body_cloud_publish_hz_;
  const bool timestamp_reset = last_body_cloud_publish_time_ > message_time;
  if (last_body_cloud_publish_time_ >= 0.0 && !timestamp_reset &&
      message_time - last_body_cloud_publish_time_ + 1e-6 < publish_period) {
    return;
  }
  last_body_cloud_publish_time_ = message_time;
  body_cloud_pub_->publish(*message);
}

void SlamManagerNode::pauseNavigationCallback(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=pause_navigation mode=%s session=%llu",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_));

  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);

  // 协议2.8：仅暂停Pure Pursuit控制，不销毁规划和感知Worker。
  if (state_machine_.mode() != SlamMode::kNavigating) {
    response->success = false;
    response->message = "pause navigation rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  std::string message;
  if (!navigation_task_executor_->setPaused(true, message)) {
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
  std::string mode_message;
  if (!setRobotMode(1, mode_message)) {
    response->success = false;
    response->message =
        "navigation paused, but failed to set robot mode=1: " + mode_message;
    return;
  }
  response->success = true;
  response->message = message + "; " + mode_message;
}

void SlamManagerNode::resumeNavigationCallback(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "[APP][REQUEST] command=resume_navigation mode=%s session=%llu",
              std::string(toString(state_machine_.mode())).c_str(),
              static_cast<unsigned long long>(session_id_));

  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);

  // 协议2.9：恢复Pure Pursuit控制，继续执行原导航目标。
  if (state_machine_.mode() != SlamMode::kNavigationPaused) {
    response->success = false;
    response->message = "resume navigation rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  std::string mode_message;
  if (!setRobotMode(1, mode_message)) {
    response->success = false;
    response->message =
        "resume navigation failed to set robot mode=1: " + mode_message;
    return;
  }

  std::string message;
  if (!navigation_task_executor_->setPaused(false, message)) {
    std::string mode_message_after_failure;
    setRobotMode(1, mode_message_after_failure);
    response->success = false;
    response->message = "resume navigation failed: " + message +
                        "; robot mode kept at 1: " + mode_message_after_failure;
    return;
  }
  if (!state_machine_.resumeNavigation()) {
    std::string pause_message;
    navigation_task_executor_->setPaused(true, pause_message);
    std::string mode_message_after_failure;
    setRobotMode(1, mode_message_after_failure);
    response->success = false;
    response->message =
        "resume navigation state transition failed; robot mode kept at 1: " +
        mode_message_after_failure;
    return;
  }

  publishMode();
  response->success = true;
  response->message = message + "; " + mode_message;
}

void SlamManagerNode::publishMode() {
  // 协议2.13：将更细的内部过渡状态压缩成APP定义的uint8状态。
  uint8_t protocol_state = RuntimeState::IDLE;
  switch (state_machine_.mode()) {
    case SlamMode::kLoadingMap:
      protocol_state = RuntimeState::LOADING_MAP;
      break;
    case SlamMode::kStartingMapping:
    case SlamMode::kMapping:
      protocol_state = RuntimeState::MAPPING;
      break;
    case SlamMode::kStoppingMapping:
      protocol_state = RuntimeState::SAVING_MAP;
      break;
    case SlamMode::kStartingLocalization:
    case SlamMode::kLocalizing:
      protocol_state = RuntimeState::RELOCALIZING;
      break;
    case SlamMode::kStartingNavigation:
    case SlamMode::kNavigating:
    case SlamMode::kStoppingNavigation:
      protocol_state = RuntimeState::NAVIGATING;
      break;
    case SlamMode::kNavigationPaused:
      protocol_state = RuntimeState::NAVIGATION_PAUSED;
      break;
    case SlamMode::kError: {
      std::lock_guard<std::mutex> lock(runtime_state_mutex_);
      publishRuntimeState(RuntimeState::ERROR, error_code_, error_message_);
      return;
    }
    case SlamMode::kBoot:
    case SlamMode::kIdle:
      protocol_state = RuntimeState::IDLE;
      break;
  }
  {
    std::lock_guard<std::mutex> lock(runtime_state_mutex_);
    error_code_ = RuntimeState::NO_ERROR;
    error_message_.clear();
  }
  publishRuntimeState(protocol_state);
}

void SlamManagerNode::publishRuntimeState(uint8_t state, int32_t error_code,
                                          const std::string &error_message) {
  RuntimeState message;
  message.state = state;
  message.error_code = error_code;
  message.error_msg = error_message;
  runtime_state_pub_->publish(message);
}

void SlamManagerNode::enterError(int32_t error_code,
                                 const std::string &error_message) {
  relocalization_succeeded_.store(false);
  {
    std::lock_guard<std::mutex> lock(runtime_state_mutex_);
    error_code_ = error_code;
    error_message_ = error_message;
  }
  state_machine_.enterError();
  publishRuntimeState(RuntimeState::ERROR, error_code, error_message);
}

}  // namespace slam_manager
