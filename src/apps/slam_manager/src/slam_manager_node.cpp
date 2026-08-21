#include "slam_manager/slam_manager_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <future>
#include <stdexcept>
#include <string>

#include "slam_manager/pose_utils.hpp"

namespace slam_manager {

namespace {

bool finitePoseFields(double x, double y, double z, double theta) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         std::isfinite(theta);
}

bool validMapId(const std::string &map_id, std::string &message) {
  constexpr std::size_t kMaximumIdLength = 64U;
  if (map_id.size() <= 4U || map_id.size() > kMaximumIdLength ||
      map_id.rfind("map_", 0U) != 0U) {
    message = "map_id must use format map_<name> and contain at most " +
              std::to_string(kMaximumIdLength) + " characters";
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
      "localization.relocalization_timeout_sec", 180.0);
  localization_stop_timeout_sec_ =
      declare_parameter<double>("localization.stop_timeout_sec", 10.0);
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
  const auto navigation_enable_skip_first_target =
      declare_parameter<bool>("navigation.enable_skip_first_target", true);
  const auto navigation_skip_first_target_threshold =
      declare_parameter<double>("navigation.skip_first_target_threshold", 0.8);
  map_storage_ = std::make_unique<MapStorage>(
      map_root, static_cast<std::size_t>(configured_max_maps));
  bbs_voxel_generator_ = std::make_unique<BbsVoxelGenerator>(
      BbsVoxelGenerator::Config{bbs_saver_executable, bbs_min_level_resolution,
                                bbs_max_level, bbs_generation_timeout_sec});
  topology_file_writer_ = std::make_unique<TopologyFileWriter>();

  // 可重入回调组允许 Worker 的异步响应驱动下一步启停操作。
  worker_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // 开始重定位会等待算法结果，停止服务必须能并发进入以主动取消等待。
  localization_control_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  // start_localization会同步等待结果，结果订阅必须位于独立回调组。
  localization_result_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  navigation_result_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
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
          navigation_skip_first_target_threshold},
      NavigationTaskExecutor::Callbacks{
          [this]() { publishMode(); },
          [this](uint8_t state) { publishRuntimeState(state); },
          [this](int32_t code, const std::string &message) {
            enterError(code, message);
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
      "/cloud_registered_body", rclcpp::QoS(20).best_effort(),
      std::bind(&SlamManagerNode::bodyCloudCallback, this,
                std::placeholders::_1));

  initial_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 1);

  // 协议2.1～2.5、2.8～2.9：只创建协议中定义的APP Service。
  start_mapping_service_ = create_service<StartMapping>(
      "/robot/start_mapping",
      std::bind(&SlamManagerNode::startMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  finish_mapping_service_ = create_service<Trigger>(
      "/robot/finish_mapping",
      std::bind(&SlamManagerNode::finishMappingCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  load_map_service_ = create_service<LoadMap>(
      "/robot/load_map",
      std::bind(&SlamManagerNode::loadMapCallback, this, std::placeholders::_1,
                std::placeholders::_2));
  delete_map_service_ = create_service<DeleteMap>(
      "/robot/delete_map",
      std::bind(&SlamManagerNode::deleteMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  start_localization_service_ = create_service<StartLocalization>(
      "/robot/start_localization",
      std::bind(&SlamManagerNode::startLocalizationCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, localization_control_callback_group_);
  pause_navigation_service_ = create_service<Trigger>(
      "/robot/pause_navigation",
      std::bind(&SlamManagerNode::pauseNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  resume_navigation_service_ = create_service<Trigger>(
      "/robot/resume_navigation",
      std::bind(&SlamManagerNode::resumeNavigationCallback, this,
                std::placeholders::_1, std::placeholders::_2));

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

  // fastlio_localization 重定位成功后自动将运行模式切换为 ready。
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

SlamManagerNode::~SlamManagerNode() = default;

void SlamManagerNode::startMappingCallback(
    const StartMapping::Request::SharedPtr request,
    StartMapping::Response::SharedPtr response) {
  // 协议2.1：APP提供完整地图目录标识（例如map_0）。
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

  // 只有空闲状态允许进入建图启动过程。
  if (!state_machine_.beginMapping()) {
    response->success = false;
    response->message = "start mapping rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  // 同名地图复用统一删除逻辑，旧目录在新建图开始前整体移除。
  // if (map_exists) {
  //   std::string delete_message;
  //   if (!map_storage_->deleteMap(map_id, delete_message)) {
  //     response->success = false;
  //     response->message = "failed to replace existing map: " +
  //     delete_message; enterError(RuntimeState::MAP_ERROR, response->message);
  //     return;
  //   }
  //   RCLCPP_INFO(get_logger(), "%s", delete_message.c_str());
  // }

  // 开始新建图后，之前由load_map记录的地图选择失效。
  if (!selected_map_id_.empty()) {
    selected_map_id_.clear();
  }

  // 每次新任务使用新的会话编号，避免混用旧任务命令。
  ++session_id_;
  const uint64_t session_id = session_id_;
  active_map_id_ = map_id;
  active_map_path_ = map_storage_->mapPath(active_map_id_);

  // map/task目录由GUI/launch预先创建；slam_manager只检查，不创建目录。
  std::error_code directory_error;
  const auto task_path = map_storage_->taskPath(active_map_id_);
  if (!std::filesystem::is_directory(final_data_path, directory_error) ||
      directory_error) {
    response->success = false;
    response->message =
        directory_error
            ? "failed to inspect map data directory: " +
                  directory_error.message()
            : "map data directory does not exist: " + final_data_path.string();
    enterError(RuntimeState::MAP_ERROR, response->message);
    return;
  }
  if (!std::filesystem::is_directory(active_map_path_, directory_error) ||
      directory_error) {
    response->success = false;
    response->message =
        directory_error
            ? "failed to inspect map directory: " + directory_error.message()
            : "map directory does not exist: " + active_map_path_.string();
    enterError(RuntimeState::MAP_ERROR, response->message);
    return;
  }
  if (!std::filesystem::is_directory(task_path, directory_error) ||
      directory_error) {
    response->success = false;
    response->message =
        directory_error
            ? "failed to inspect task directory: " + directory_error.message()
            : "task directory does not exist: " + task_path.string();
    enterError(RuntimeState::MAP_ERROR, response->message);
    return;
  }
  publishMode();

  using StartResult = std::pair<bool, std::string>;
  auto completion = std::make_shared<std::promise<StartResult>>();
  auto future = completion->get_future();
  // PGO先启动并接收暂存目录，随后启动FAST-LIO；两者节点级Ready后才响应APP。
  mapping_controller_->start(
      session_id, active_map_id_, active_map_path_.string(),
      [completion](bool success, const std::string &message) {
        completion->set_value({success, message});
      });

  if (future.wait_for(std::chrono::duration<double>(
          start_mapping_timeout_sec_)) != std::future_status::ready) {
    response->success = false;
    response->message = "start mapping timed out, map_id=" + active_map_id_;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    return;
  }
  const auto [started, start_message] = future.get();
  if (!started) {
    response->success = false;
    response->message = start_message;
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
  // 协议2.2：停止数据生产、触发PGO保存并发布正式地图目录。
  if (!state_machine_.beginStopMapping()) {
    response->success = false;
    response->message = "finish mapping rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  publishMode();
  // Service等待Worker停止、PGO保存和目录发布全部结束后再响应APP。
  using StopResult = std::pair<bool, std::string>;
  auto completion = std::make_shared<std::promise<StopResult>>();
  auto future = completion->get_future();
  mapping_controller_->stop(
      session_id_, [completion](bool success, const std::string &message) {
        completion->set_value({success, message});
      });

  const auto timeout =
      std::chrono::duration<double>(finish_mapping_timeout_sec_);
  if (future.wait_for(timeout) != std::future_status::ready) {
    response->success = false;
    response->message = "finish mapping timed out, map_id=" + active_map_id_;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  const auto [workers_stopped, stop_message] = future.get();
  if (!workers_stopped) {
    response->success = false;
    response->message = "finish mapping failed: " + stop_message;
    enterError(RuntimeState::SLAM_ERROR, response->message);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  // PGO停止时已将globalmap.pcd保存到GUI预先创建的map目录。
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

  state_machine_.mappingStopped();
  publishMode();
  response->success = true;
  response->message = finalize_message;
  RCLCPP_INFO(get_logger(), "%s; %s", stop_message.c_str(),
              finalize_message.c_str());
}

void SlamManagerNode::loadMapCallback(const LoadMap::Request::SharedPtr request,
                                      LoadMap::Response::SharedPtr response) {
  // 协议2.4：load_map检查地图并基于当前PCD无条件重新生成BBS，
  // 但不启动定位算法。
  if (!state_machine_.beginLoadMap()) {
    response->success = false;
    response->message =
        "load map rejected, robot must be idle, current state=" +
        std::string(toString(state_machine_.mode()));
    return;
  }

  // 从进入加载状态到本回调返回期间，其他任务会被状态机拒绝。
  publishMode();
  const auto finish_loading = [this]() {
    if (!state_machine_.loadMapFinished()) {
      RCLCPP_ERROR(get_logger(), "load_map failed to leave loading_map state");
      state_machine_.enterError();
    }
    publishMode();
  };
  const std::string &map_id = request->map_id;
  if (!validMapId(map_id, response->message)) {
    response->success = false;
    finish_loading();
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
    finish_loading();
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
    finish_loading();
    return;
  }

  // 每次load_map都删除旧体素并基于当前globalmap.pcd重新生成，
  // 避免APP覆盖或拼接PCD后误用上一版BBS。
  std::string bbs_message;
  if (!bbs_voxel_generator_->generate(map_storage_->mapPath(map_id), map_id,
                                      bbs_message)) {
    response->success = false;
    response->message =
        "load map failed: 3D-BBS generation failed: " + bbs_message;
    finish_loading();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
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

void SlamManagerNode::startLocalizationCallback(
    const StartLocalization::Request::SharedPtr request,
    StartLocalization::Response::SharedPtr response) {
  // 协议2.5：使用最近一次load_map选中的地图启动重定位。
  if (selected_map_id_.empty()) {
    response->success = false;
    response->message = "no map selected; call /robot/load_map first";
    return;
  }

  const auto selected_map_path = map_storage_->mapPath(selected_map_id_);
  std::error_code exists_error;
  if (!std::filesystem::exists(selected_map_path / "globalmap.pcd",
                               exists_error) ||
      exists_error) {
    response->success = false;
    response->message =
        exists_error
            ? "failed to inspect selected map: " + exists_error.message()
            : "selected map no longer exists: " + selected_map_id_;
    return;
  }

  const bool use_initial_pose = request->use_initial_pose;
  if (use_initial_pose &&
      !finitePoseFields(request->x, request->y, request->z, request->theta)) {
    response->success = false;
    response->message = "initial pose x/y/z/theta contain non-finite values";
    return;
  }

  // 同一时间只允许一个重定位请求执行。
  std::unique_lock<std::mutex> command_lock(localization_command_mutex_);
  const bool restart_localization = state_machine_.mode() == SlamMode::kReady;
  const bool transition_started = restart_localization
                                      ? state_machine_.beginRelocalization()
                                      : state_machine_.beginLocalization();
  if (!transition_started) {
    response->success = false;
    response->message = "start localization rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  // BBS体素地图已在最近一次load_map时重新生成，定位阶段直接加载。
  publishMode();

  // Ready时旧的纯定位算法仍在运行。再次重定位必须先完整销毁旧算法，
  // 再创建携带新session_id的算法实例，确保延迟结果不会完成新请求。
  if (restart_localization) {
    using StopResult = std::pair<bool, std::string>;
    auto stop_completion = std::make_shared<std::promise<StopResult>>();
    auto stop_future = stop_completion->get_future();
    localization_controller_->stop(
        session_id_,
        [stop_completion](bool success, const std::string &message) {
          stop_completion->set_value({success, message});
        });

    const auto stop_timeout =
        std::chrono::duration<double>(localization_stop_timeout_sec_);
    if (stop_future.wait_for(stop_timeout) != std::future_status::ready) {
      enterError(RuntimeState::LOCALIZATION_ERROR,
                 "localization stop timed out before relocalization");
      response->success = false;
      response->message =
          "timed out stopping current localization before relocalization";
      return;
    }

    const auto stop_result = stop_future.get();
    if (!stop_result.first) {
      enterError(RuntimeState::LOCALIZATION_ERROR, stop_result.second);
      response->success = false;
      response->message =
          "failed to stop current localization before relocalization: " +
          stop_result.second;
      return;
    }

    // 重新搜索期间上一轮位姿不再视为有效，禁止导航和在线打点误用旧值。
    {
      std::lock_guard<std::mutex> lock(current_pose_mutex_);
      current_pose_ = geometry_msgs::msg::PoseStamped();
      has_current_pose_ = false;
    }
  }

  // 协议使用x/y/z/theta；算法内部仍使用geometry_msgs/Pose四元数。
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.orientation.w = 1.0;
  if (use_initial_pose) {
    initial_pose = pose_utils::fromXyzTheta(request->x, request->y, request->z,
                                            request->theta);
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
  // 将map_id和地图目录传给纯定位Worker，算法对象在收到START后创建。
  localization_controller_->start(
      session_id, selected_map_id_, selected_map_path.string(),
      use_initial_pose,
      [this, session_id, use_initial_pose, initial_pose](
          bool success, const std::string &message) {
        {
          std::lock_guard<std::mutex> lock(localization_result_mutex_);
          if (!localization_attempt_active_ ||
              localization_attempt_session_id_ != session_id ||
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
          if (use_initial_pose) {
            // APP要求使用初始位姿时，转换成算法使用的 /initialpose 消息。
            geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_message;
            initial_pose_message.header.stamp = now();
            initial_pose_message.header.frame_id = "map";
            initial_pose_message.pose.pose = initial_pose;
            initial_pose_pub_->publish(initial_pose_message);
          }
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
  std::string result_message;
  {
    std::unique_lock<std::mutex> lock(localization_result_mutex_);
    result_received = localization_result_cv_.wait_for(
        lock, std::chrono::duration<double>(relocalization_timeout_sec_),
        [this, session_id]() {
          return localization_attempt_session_id_ == session_id &&
                 localization_result_ready_;
        });
    if (result_received) {
      result_success = localization_result_success_;
      result_message = localization_result_message_;
    }
    if (localization_attempt_session_id_ == session_id) {
      localization_attempt_active_ = false;
    }
  }

  if (result_received && result_success) {
    response->success = true;
    response->message = result_message;
    return;
  }

  const std::string failure_message =
      result_received ? result_message
                      : "relocalization timed out after " +
                            std::to_string(relocalization_timeout_sec_) +
                            " seconds, map_id=" + selected_map_id_;

  // 启动失败或超时均发送STOP，确保可能残留的定位节点被析构。
  using StopResult = std::pair<bool, std::string>;
  auto stop_completion = std::make_shared<std::promise<StopResult>>();
  auto stop_future = stop_completion->get_future();
  localization_controller_->stop(
      session_id, [stop_completion](bool success, const std::string &message) {
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
    enterError(RuntimeState::LOCALIZATION_ERROR, stop_result.second);
  }
  publishMode();

  response->success = false;
  response->message = failure_message + "; " + stop_result.second;
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
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

  const auto selected_map_path = map_storage_->mapPath(selected_map_id_);
  std::error_code exists_error;
  if (!std::filesystem::exists(selected_map_path / "globalmap.pcd",
                               exists_error) ||
      exists_error) {
    message = exists_error
                  ? "failed to inspect selected map: " + exists_error.message()
                  : "selected map no longer exists: " + selected_map_id_;
    return false;
  }

  std::unique_lock<std::mutex> command_lock(localization_command_mutex_);
  if (!state_machine_.beginRelocalization()) {
    message = "map_connection relocalization rejected, current state=" +
              std::string(toString(state_machine_.mode()));
    return false;
  }
  publishMode();

  using WorkerResult = std::pair<bool, std::string>;
  auto stop_completion = std::make_shared<std::promise<WorkerResult>>();
  auto stop_future = stop_completion->get_future();
  localization_controller_->stop(
      session_id_,
      [stop_completion](bool success, const std::string &worker_message) {
        stop_completion->set_value({success, worker_message});
      });
  const auto stop_timeout =
      std::chrono::duration<double>(localization_stop_timeout_sec_);
  if (stop_future.wait_for(stop_timeout) != std::future_status::ready) {
    state_machine_.localizationFailed();
    publishMode();
    message = "timed out stopping localization before map_connection";
    return false;
  }
  const auto stop_result = stop_future.get();
  if (!stop_result.first) {
    enterError(RuntimeState::LOCALIZATION_ERROR, stop_result.second);
    message = "failed to stop localization before map_connection: " +
              stop_result.second;
    return false;
  }
  if (is_canceling()) {
    state_machine_.localizationFailed();
    publishMode();
    message = "map_connection relocalization cancelled";
    return false;
  }

  const geometry_msgs::msg::Pose initial_pose =
      pose_utils::fromXyzTheta(node.x, node.y, node.z, node.theta);
  ++session_id_;
  const uint64_t localization_session_id = session_id_;
  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    localization_attempt_session_id_ = localization_session_id;
    localization_attempt_active_ = true;
    localization_result_ready_ = false;
    localization_result_success_ = false;
    localization_result_message_.clear();
  }

  localization_controller_->start(
      localization_session_id, selected_map_id_, selected_map_path.string(),
      true,
      [this, localization_session_id, initial_pose](
          bool success, const std::string &worker_message) {
        {
          std::lock_guard<std::mutex> lock(localization_result_mutex_);
          if (!localization_attempt_active_ ||
              localization_attempt_session_id_ != localization_session_id ||
              localization_result_ready_) {
            return;
          }
          if (!success) {
            localization_result_ready_ = true;
            localization_result_success_ = false;
            localization_result_message_ =
                "failed to restart localization: " + worker_message;
          }
        }
        if (success) {
          state_machine_.localizationStarted();
          geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_message;
          initial_pose_message.header.stamp = now();
          initial_pose_message.header.frame_id = "map";
          initial_pose_message.pose.pose = initial_pose;
          initial_pose_pub_->publish(initial_pose_message);
          RCLCPP_INFO(get_logger(),
                      "Map connection initial pose published: "
                      "xyz=(%.3f, %.3f, %.3f), theta=%.3f",
                      initial_pose.position.x, initial_pose.position.y,
                      initial_pose.position.z,
                      pose_utils::yaw(initial_pose.orientation));
        } else {
          state_machine_.localizationFailed();
          RCLCPP_ERROR(get_logger(), "%s", worker_message.c_str());
          localization_result_cv_.notify_all();
        }
        publishMode();
      });
  command_lock.unlock();

  bool result_received = false;
  bool result_success = false;
  bool cancelled = false;
  std::string result_message;
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration<double>(relocalization_timeout_sec_);
  {
    std::unique_lock<std::mutex> lock(localization_result_mutex_);
    while (!localization_result_ready_) {
      if (is_canceling()) {
        cancelled = true;
        break;
      }
      if (localization_result_cv_.wait_for(lock,
                                           std::chrono::milliseconds(100)) ==
              std::cv_status::timeout &&
          std::chrono::steady_clock::now() >= deadline) {
        break;
      }
    }
    result_received = localization_result_ready_;
    if (result_received) {
      result_success = localization_result_success_;
      result_message = localization_result_message_;
    }
    if (localization_attempt_session_id_ == localization_session_id) {
      localization_attempt_active_ = false;
    }
  }

  if (result_received && result_success && !cancelled) {
    message = result_message;
    return true;
  }

  auto cleanup_completion = std::make_shared<std::promise<WorkerResult>>();
  auto cleanup_future = cleanup_completion->get_future();
  localization_controller_->stop(
      localization_session_id,
      [cleanup_completion](bool success, const std::string &worker_message) {
        cleanup_completion->set_value({success, worker_message});
      });
  const bool cleanup_finished =
      cleanup_future.wait_for(stop_timeout) == std::future_status::ready;
  const auto cleanup_result =
      cleanup_finished ? cleanup_future.get()
                       : WorkerResult{false, "localization stop timed out"};
  if (cleanup_result.first) {
    state_machine_.localizationFailed();
    publishMode();
  } else {
    enterError(RuntimeState::LOCALIZATION_ERROR, cleanup_result.second);
  }

  if (cancelled) {
    message =
        "map_connection relocalization cancelled; " + cleanup_result.second;
  } else if (result_received) {
    message = result_message + "; " + cleanup_result.second;
  } else {
    message = "map_connection relocalization timed out after " +
              std::to_string(relocalization_timeout_sec_) + " seconds; " +
              cleanup_result.second;
  }
  return false;
}

void SlamManagerNode::relocalizationResultCallback(
    const std_msgs::msg::Bool::SharedPtr message) {
  // 协议2.5：只有true表示最终定位成功；false表示算法仍在搜索。
  // false表示算法仍在尝试，不作为最终失败；最终失败由超时统一判定。
  if (!message->data) return;

  std::string result_message;
  {
    std::lock_guard<std::mutex> lock(localization_result_mutex_);
    if (!localization_attempt_active_ || localization_result_ready_) {
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
  // 协议2.3：仅空闲状态允许删除指定map_id的整个地图目录。
  const SlamMode current_mode = state_machine_.mode();
  if (current_mode != SlamMode::kIdle) {
    response->success = false;
    response->message =
        "delete map rejected, robot must be idle, current state=" +
        std::string(toString(current_mode));
    return;
  }

  const std::string &map_id = request->map_id;
  if (!validMapId(map_id, response->message)) {
    response->success = false;
    return;
  }
  response->success = map_storage_->deleteMap(map_id, response->message);
  if (response->success) {
    if (selected_map_id_ == map_id) selected_map_id_.clear();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "Delete map rejected: %s",
                response->message.c_str());
  }
}

rclcpp_action::GoalResponse SlamManagerNode::handleNavigationGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const StartNavigation::Goal> goal) {
  // 协议2.6：接收前校验任务ID、节点ID、Pose以及定位Ready状态。
  std::string message;
  if (!navigation_task_executor_->validateGoal(*goal, message)) {
    RCLCPP_WARN(get_logger(), "Reject navigation goal: %s", message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (state_machine_.mode() != SlamMode::kReady) {
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
  navigation_task_executor_->cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SlamManagerNode::handleNavigationAccepted(
    const std::shared_ptr<StartNavigationGoalHandle> goal_handle) {
  ++session_id_;
  navigation_task_executor_->startPatrolAction(goal_handle, session_id_);
}

rclcpp_action::GoalResponse SlamManagerNode::handleNavigateToPoseGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal) {
  // 协议2.7：校验目标、定位Ready状态和当前map位姿后再接受Goal。
  if (state_machine_.mode() != SlamMode::kReady) {
    const std::string current_state(toString(state_machine_.mode()));
    RCLCPP_WARN(get_logger(),
                "Reject single-point goal: localization is not ready, "
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
  navigation_task_executor_->startNavigateToPoseAction(
      goal_handle, current_pose, session_id_);
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
  robot_pose.theta = pose_utils::yaw(pose.pose.orientation);
  robot_pose.match_score = match_score;
  current_pose_pub_->publish(robot_pose);
}

void SlamManagerNode::bodyCloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr message) {
  // 协议2.11：转发FAST-LIO的IMU/机体坐标系实时点云。
  body_cloud_pub_->publish(*message);
}

void SlamManagerNode::pauseNavigationCallback(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
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
  response->success = true;
  response->message = message;
}

void SlamManagerNode::resumeNavigationCallback(
    const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
  // 协议2.9：恢复Pure Pursuit控制，继续执行原导航目标。
  if (state_machine_.mode() != SlamMode::kNavigationPaused) {
    response->success = false;
    response->message = "resume navigation rejected, current state=" +
                        std::string(toString(state_machine_.mode()));
    return;
  }

  std::string message;
  if (!navigation_task_executor_->setPaused(false, message)) {
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
    case SlamMode::kReady:
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
  {
    std::lock_guard<std::mutex> lock(runtime_state_mutex_);
    error_code_ = error_code;
    error_message_ = error_message;
  }
  state_machine_.enterError();
  publishRuntimeState(RuntimeState::ERROR, error_code, error_message);
}

}  // namespace slam_manager
