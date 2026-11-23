#include "robot_geometry/lidar_extrinsics.hpp"
#include "obstacle_stop/obstacle_stop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <sstream>
#include <utility>

#include "obstacle_stop/region_profile_loader.hpp"

namespace obstacle_stop {

namespace {
bool pointInPolygon(double x, double y,
                    const std::vector<std::array<double, 2>> &polygon) {
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1U;
       index < polygon.size(); previous = index++) {
    const auto &first = polygon[previous];
    const auto &second = polygon[index];
    const double cross = (second[0] - first[0]) * (y - first[1]) -
                         (second[1] - first[1]) * (x - first[0]);
    if (std::abs(cross) <= 1e-9 && x >= std::min(first[0], second[0]) - 1e-9 &&
        x <= std::max(first[0], second[0]) + 1e-9 &&
        y >= std::min(first[1], second[1]) - 1e-9 &&
        y <= std::max(first[1], second[1]) + 1e-9) {
      return true;
    }
    const bool crosses = (first[1] > y) != (second[1] > y);
    if (crosses &&
        x < (second[0] - first[0]) * (y - first[1]) / (second[1] - first[1]) +
                first[0]) {
      inside = !inside;
    }
  }
  return inside;
}
}  // namespace

double ObstacleStop::steadySeconds() {
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

ObstacleStop::ObstacleStop(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      hard_stop_clear_deadline_(0, 0, node_->get_clock()->get_clock_type()),
      slowdown_clear_deadline_(0, 0, node_->get_clock()->get_clock_type()) {}

bool ObstacleStop::initialize(std::string &message) {
  loadParameters();
  if (!initializeImuStairDetection(message)) return false;
  default_config_ = config_;
  if (!loadRegionProfiles(message)) return false;
  initializeGridLookup();

  // Each enabled input is subscribed once; zones select sources by name.
  if (front_lidar_enabled_) {
    if (lidar_type_ == 1) {
      lidar_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
          lidar_topic_, rclcpp::SensorDataQoS(),
          std::bind(&ObstacleStop::lidarCallback, this, std::placeholders::_1));
    } else {
      pointcloud2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          lidar_topic_, rclcpp::SensorDataQoS(),
          std::bind(&ObstacleStop::pointCloud2Callback, this, std::placeholders::_1));
    }
  }
  if (rear_lidar_enabled_) {
    rear_pointcloud2_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        rear_lidar_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleStop::rearPointCloud2Callback, this, std::placeholders::_1));
  }
  planned_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      planned_cmd_topic_, rclcpp::QoS(1).reliable(),
      std::bind(&ObstacleStop::plannedVelocityCallback, this,
                std::placeholders::_1));
  planner_mode_sub_ = node_->create_subscription<navigation_interfaces::msg::PlannerMotionMode>(
      "/scan_planner/motion_mode", rclcpp::QoS(1).reliable(),
      std::bind(&ObstacleStop::plannerMotionModeCallback, this, std::placeholders::_1));
  emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", 10,
      std::bind(&ObstacleStop::emergencyStopCallback, this,
                std::placeholders::_1));
  posture_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/navigation/posture_stop",
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&ObstacleStop::postureStopCallback, this,
                std::placeholders::_1));
  obstacle_stop_enabled_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/obstacle_stop/enabled", rclcpp::QoS(1).transient_local(),
      std::bind(&ObstacleStop::obstacleStopEnabledCallback, this,
                std::placeholders::_1));
  if (!region_profiles_.empty()) {
    localization_pose_sub_ =
        node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10,
            std::bind(&ObstacleStop::localizationPoseCallback, this,
                      std::placeholders::_1));
  }

  safe_cmd_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, 10);
  status_pub_ = node_->create_publisher<navigation_interfaces::msg::ObstacleStopStatus>(
      "/obstacle_stop/status", rclcpp::QoS(1).reliable());
  control_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.control_frequency)),
      std::bind(&ObstacleStop::controlCallback, this));

  initializeVisualization();

  RCLCPP_INFO(node_->get_logger(),
              "Obstacle stop initialized: lidar_topic=%s, lidar_type=%d (%s), "
              "rear_lidar=%s, planned_cmd=%s, safe_cmd=%s",
              front_lidar_enabled_ ? lidar_topic_.c_str() : "disabled", lidar_type_,
              lidar_type_ == 1 ? "Livox CustomMsg" : "PointCloud2",
              rear_lidar_enabled_ ? rear_lidar_topic_.c_str() : "disabled",
              planned_cmd_topic_.c_str(), safe_cmd_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(),
      "[VELOCITY_GUARD] owner=planner enabled=%s check_distance=%.3f "
      "velocity_limit=%.3f mode_timeout=%.3f min_points_per_cell=%zu "
      "status=/obstacle_stop/status mode=/scan_planner/motion_mode",
      config_.planner_velocity_guard_enabled ? "true" : "false",
      config_.velocity_guard.check_distance,
      config_.velocity_guard.velocity,
      config_.planner_mode_timeout_sec, config_.min_points_per_cell);
  message = "obstacle stop initialized";
  return true;
}

void ObstacleStop::loadParameters() {
  lidar_type_ = node_->declare_parameter<int>("lidar_type", 1);
  front_lidar_enabled_ = node_->declare_parameter<bool>("lidars.front.enabled", true);
  rear_lidar_enabled_ = node_->declare_parameter<bool>("lidars.rear.enabled", lidar_type_ != 1);
  lidar_topic_ = node_->declare_parameter<std::string>("lidars.front.topic",
      lidar_type_ == 1 ? "/livox/lidar" : "/rslidar_front/points");
  lidar_frame_ = node_->declare_parameter<std::string>("lidars.front.frame",
      lidar_type_ == 1 ? "livox_frame" : "rslidar_front");
  rear_lidar_topic_ = node_->declare_parameter<std::string>(
      "lidars.rear.topic", "/rslidar_rear/points");
  rear_lidar_frame_ = node_->declare_parameter<std::string>("lidars.rear.frame", "rslidar_rear");
  planned_cmd_topic_ = node_->declare_parameter<std::string>(
      "planned_cmd_topic", "/cmd_vel_planned");
  safe_cmd_topic_ =
      node_->declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel");
  global_frame_ = node_->declare_parameter<std::string>("global_frame", "map");
  pose_topic_ = node_->declare_parameter<std::string>(
      "pose_topic", "/pose_stamped_localization");
  topology_map_file_ =
      node_->declare_parameter<std::string>("topology_map_file", "");

  lidar_to_base_.matrix() = robot_geometry::lidar_extrinsics::load(*node_);
  Eigen::Matrix4d rear_default = Eigen::Matrix4d::Identity();
  rear_default.topRightCorner<3, 1>() = Eigen::Vector3d(-0.4, 0.0, 0.55);
  rear_lidar_to_base_.matrix() = robot_geometry::lidar_extrinsics::load(
      *node_, "rear_lidar_to_base", rear_default);

  config_.min_height =
      node_->declare_parameter<double>("min_height", config_.min_height);
  config_.max_height =
      node_->declare_parameter<double>("max_height", config_.max_height);
  config_.angle_min =
      node_->declare_parameter<double>("angle_min", config_.angle_min);
  config_.angle_max =
      node_->declare_parameter<double>("angle_max", config_.angle_max);
  config_.range_min =
      node_->declare_parameter<double>("range_min", config_.range_min);
  config_.range_max =
      node_->declare_parameter<double>("range_max", config_.range_max);
  config_.grid_resolution = node_->declare_parameter<double>(
      "grid_resolution", config_.grid_resolution);
  const auto minimum_points = node_->declare_parameter<int64_t>(
      "min_points_per_cell", 1);
  config_.min_points_per_cell = static_cast<std::size_t>(minimum_points);
  config_.grid_width =
      node_->declare_parameter<int>("grid_width", config_.grid_width);
  config_.grid_height =
      node_->declare_parameter<int>("grid_height", config_.grid_height);
  config_.robot_width =
      node_->declare_parameter<double>("robot_width", config_.robot_width);
  config_.robot_length =
      node_->declare_parameter<double>("robot_length", config_.robot_length);
  config_.linear_velocity_factor = node_->declare_parameter<double>(
      "linear_velocity_factor", config_.linear_velocity_factor);
  config_.angular_velocity_factor = node_->declare_parameter<double>(
      "angular_velocity_factor", config_.angular_velocity_factor);
  config_.hard_stop_clear_delay = node_->declare_parameter<double>(
      "hard_stop_clear_delay", config_.hard_stop_clear_delay);
  config_.slowdown_clear_delay = node_->declare_parameter<double>(
      "slowdown_clear_delay", config_.slowdown_clear_delay);
  config_.planner_velocity_guard_enabled = node_->declare_parameter<bool>(
      "planner_velocity_guard_enabled", config_.planner_velocity_guard_enabled);
  config_.velocity_guard.velocity = node_->declare_parameter<double>(
      "planner_velocity_limit", config_.velocity_guard.velocity);
  config_.velocity_guard.check_distance = node_->declare_parameter<double>(
      "clearance_check_distance", config_.velocity_guard.check_distance);
  config_.planner_mode_timeout_sec = node_->declare_parameter<double>(
      "planner_motion_mode_timeout_sec", config_.planner_mode_timeout_sec);
  config_.lidar_timeout_sec = node_->declare_parameter<double>(
      "lidar_timeout_sec", config_.lidar_timeout_sec);
  config_.command_timeout_sec = node_->declare_parameter<double>(
      "command_timeout_sec", config_.command_timeout_sec);
  config_.control_frequency = node_->declare_parameter<double>(
      "control_frequency", config_.control_frequency);
  config_.publish_visualization = node_->declare_parameter<bool>(
      "publish_visualization", config_.publish_visualization);
  for (std::size_t i = 0; i < config_.zones.size(); ++i) {
    auto &zone = config_.zones[i];
    const std::string prefix = std::string("zones.") + kZoneNames[i] + ".";
    zone.enabled = node_->declare_parameter<bool>(prefix + "enabled",
        i == 0U ? front_lidar_enabled_ : i == 1U ? rear_lidar_enabled_ :
        front_lidar_enabled_ || rear_lidar_enabled_);
    std::vector<std::string> defaults;
    if (i == 0U) defaults = {"front"};
    else if (i == 1U) defaults = {"rear"};
    else {
      if (front_lidar_enabled_) defaults.push_back("front");
      if (rear_lidar_enabled_) defaults.push_back("rear");
    }
    const auto sources = node_->declare_parameter<std::vector<std::string>>(prefix + "sources", defaults);
    zone.sources = 0U;
    for (const auto &source : sources) {
      zone.sources |= source == "front" ? 1U : 2U;
    }
    const double stop = i < 2U ? 0.10 : 0.15;
    zone.stop_distance = node_->declare_parameter<double>(prefix + "stop_distance", stop);
    if (i < 2U) {
      zone.slowdown_distance = node_->declare_parameter<double>(prefix + "slowdown_distance", 0.60);
    }
    const auto threshold = node_->declare_parameter<int64_t>(prefix + "obstacle_cell_threshold",
        static_cast<int64_t>(zone.obstacle_cell_threshold));
    zone.obstacle_cell_threshold = static_cast<std::size_t>(threshold);
  }
}

bool ObstacleStop::initializeImuStairDetection(std::string &message) {
  imu_stairs_enabled_ = node_->declare_parameter<bool>("imu_stairs.enabled", true);
  const auto topic = node_->declare_parameter<std::string>("imu_stairs.topic", "/imu/data");
  imu_frame_ = node_->declare_parameter<std::string>("imu_stairs.frame", "woodox_imu");
  const auto rotation = node_->declare_parameter<std::vector<double>>(
      "imu_stairs.imu_to_base_rpy", std::vector<double>{0.0, 0.0, 0.0});
  imu_stair_detector::Config config;
  config.pitch_baseline_deg = node_->declare_parameter<double>("imu_stairs.pitch_baseline_deg", 0.0);
  config.enter_angle_deg = node_->declare_parameter<double>("imu_stairs.enter_angle_deg", 14.0);
  config.exit_angle_deg = node_->declare_parameter<double>("imu_stairs.exit_angle_deg", 10.0);
  config.filter_time_constant_sec = node_->declare_parameter<double>("imu_stairs.filter_time_constant_sec", 0.15);
  config.enter_duration_sec = node_->declare_parameter<double>("imu_stairs.enter_duration_sec", 0.15);
  config.exit_duration_sec = node_->declare_parameter<double>("imu_stairs.exit_duration_sec", 0.40);
  config.timeout_sec = node_->declare_parameter<double>("imu_stairs.timeout_sec", 0.30);
  if (!imu_stairs_enabled_) return true;
  if (rotation.size() != 3U || !std::all_of(rotation.begin(), rotation.end(),
      [](double value) { return std::isfinite(value); }) || topic.empty() || imu_frame_.empty()) {
    message = "invalid imu_stairs topic, frame or imu_to_base_rpy (expected three radians)";
    return false;
  }
  config.imu_to_base = Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX());
  if (!imu_stair_detector_.configure(config)) {
    message = "invalid imu_stairs thresholds, durations or timeout";
    return false;
  }
  stairs_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/obstacle_stop/stairs_active", rclcpp::QoS(1).transient_local());
  // Only the latest orientation matters; do not accumulate an IMU backlog.
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
      topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ObstacleStop::imuCallback, this, std::placeholders::_1));
  return true;
}

void ObstacleStop::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  const double stamp = static_cast<double>(message->header.stamp.sec) +
      static_cast<double>(message->header.stamp.nanosec) * 1e-9;
  const double now = node_->now().seconds();
  if (message->header.frame_id != imu_frame_ ||
      !std::isfinite(message->orientation_covariance[0]) ||
      message->orientation_covariance[0] < 0.0 ||
      message->header.stamp.nanosec >= 1000000000U || stamp <= 0.0 || stamp > now) {
    imu_stair_detector_.reset();
    return;
  }
  const auto &q = message->orientation;
  imu_stair_detector_.update(Eigen::Quaterniond(q.w, q.x, q.y, q.z), stamp);
  // Reject stale samples immediately rather than briefly keeping a bypass alive.
  (void)imu_stair_detector_.status(now);
}

bool ObstacleStop::loadRegionProfiles(std::string &message) {
  region_profiles_.clear();
  if (topology_map_file_.empty()) return true;
  std::size_t custom_profile_count = 0U;

  std::ifstream input(topology_map_file_);
  if (!input.is_open()) {
    message = "cannot open topology JSON: " + topology_map_file_;
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    return false;
  }
  try {
    const nlohmann::json document = nlohmann::json::parse(input);
    if (!document.contains("regions")) return true;
    if (!document["regions"].is_array()) {
      throw std::runtime_error("regions must be an array");
    }
    for (const auto &json_region : document["regions"]) {
      RegionProfile region;
      region.id = json_region.at("id").get<int>();
      region.enabled = json_region.value("enabled", true);
      if (!region.enabled) {
        continue;
      }
      const auto &polygon = json_region.at("polygon");
      if (region.id < 0 || !polygon.is_array() || polygon.size() < 3U) {
        throw std::runtime_error("region has invalid id or polygon");
      }
      for (const auto &point : polygon) {
        const double x = point.at("x").get<double>();
        const double y = point.at("y").get<double>();
        if (!std::isfinite(x) || !std::isfinite(y)) {
          throw std::runtime_error("region polygon has invalid coordinate");
        }
        region.polygon.push_back({x, y});
      }
      region.config = default_config_;
      const auto profile_file =
          std::filesystem::path(topology_map_file_).parent_path() /
          "region_obstacle_stop" /
          (std::to_string(region.id) + ".yaml");
      RegionObstacleStopOverrides overrides;
      if (!loadRegionObstacleStopOverrides(
              profile_file, overrides, region.has_custom_config, message)) {
        throw std::runtime_error("region id=" + std::to_string(region.id) +
                                 ": " + message);
      }
      if (region.has_custom_config) ++custom_profile_count;
      const auto apply = [](const std::optional<double> &source,
                            double &target) {
        if (source) target = *source;
      };
      apply(overrides.linear_velocity_factor,
            region.config.linear_velocity_factor);
      apply(overrides.angular_velocity_factor,
            region.config.angular_velocity_factor);
      for (std::size_t i = 0; i < region.config.zones.size(); ++i) {
        const auto &source = overrides.zones[i];
        auto &zone = region.config.zones[i];
        if (source.stop_distance) zone.stop_distance = *source.stop_distance;
        if (i < 2U && source.slowdown_distance) zone.slowdown_distance = *source.slowdown_distance;
        if (source.obstacle_cell_threshold) zone.obstacle_cell_threshold = *source.obstacle_cell_threshold;
      }
      region_profiles_.push_back(std::move(region));
    }
  } catch (const std::exception &error) {
    message = "failed to load topology regions: " + std::string(error.what());
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    region_profiles_.clear();
    return false;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Loaded %zu obstacle-stop regions, custom YAML profiles=%zu; "
              "missing ID YAML uses startup model YAML defaults",
              region_profiles_.size(), custom_profile_count);
  message.clear();
  return true;
}

const ObstacleStop::RegionProfile *ObstacleStop::findRegion(double x,
                                                            double y) const {
  for (const auto &region : region_profiles_) {
    if (region.enabled && pointInPolygon(x, y, region.polygon)) return &region;
  }
  return nullptr;
}

void ObstacleStop::applyRegionProfile(const RegionProfile *region) {
  const std::optional<int> next_id =
      region == nullptr ? std::nullopt : std::optional<int>(region->id);
  if (next_id == active_region_id_) return;

  std::lock_guard<std::mutex> config_lock(config_mutex_);
  config_ = region == nullptr ? default_config_ : region->config;
  initializeGridLookup();
  front_height_grid_.clear();
  rear_height_grid_.clear();
  visual_height_grid_.clear();
  visual_occupied_.clear();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_lidar_ = false;
    rear_has_lidar_ = false;
  }
  active_region_id_ = next_id;
  if (region == nullptr) {
    RCLCPP_INFO(node_->get_logger(),
                "Left navigation region; restored startup model YAML defaults");
  } else if (region->has_custom_config) {
    RCLCPP_INFO(node_->get_logger(),
                "Entered region id=%d; applied region obstacle_stop profile",
                region->id);
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Entered region id=%d; no ID YAML, using startup model YAML "
                "defaults",
                region->id);
  }
}

void ObstacleStop::localizationPoseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
  if (!message->header.frame_id.empty() &&
      message->header.frame_id != global_frame_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Ignoring localization pose in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), global_frame_.c_str());
    return;
  }
  if (std::isfinite(message->pose.position.x) && std::isfinite(message->pose.position.y))
    applyRegionProfile(findRegion(message->pose.position.x, message->pose.position.y));
}

void ObstacleStop::updateZoneState() {
  const auto now = node_->now();
  const double steady_now = steadySeconds();
  std::array<bool, 2> usable;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double front_age = steady_now-front_lidar_receive_steady_;
    const double rear_age = steady_now-rear_lidar_receive_steady_;
    usable = {has_lidar_ && lidar_valid_ && front_age >= 0.0 && front_age <= config_.lidar_timeout_sec,
              rear_has_lidar_ && rear_lidar_valid_ && rear_age >= 0.0 && rear_age <= config_.lidar_timeout_sec};
  }
  // Timeouts invalidate cached occupancy even when no new packet arrives.
  refreshZoneOccupancy(usable);
  bool hard = false, slow = false, longitudinal_hard = false;
  for (std::size_t i = 0; i < config_.zones.size(); ++i) {
    const auto &zone = config_.zones[i];
    auto &runtime = zone_runtime_[i];
    if (!zone.enabled) continue;
    runtime.hard_active = runtime.hard_occupied.size() >= zone.obstacle_cell_threshold;
    if (runtime.slow) {
      runtime.slow->active = runtime.slow->occupied.size() >= zone.obstacle_cell_threshold;
      slow = slow || runtime.slow->active;
    }
    hard = hard || runtime.hard_active;
    if (i < 2U) longitudinal_hard = longitudinal_hard || runtime.hard_active;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  updateObstacleState(hard, slow, longitudinal_hard, now);
}

void ObstacleStop::plannedVelocityCallback(
    const geometry_msgs::msg::Twist::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  planned_cmd_ = *message;
  ++command_sequence_;
  command_receive_steady_ = steadySeconds();
  has_command_ = true;
}

void ObstacleStop::plannerMotionModeCallback(
    const navigation_interfaces::msg::PlannerMotionMode::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_planner_mode_ || planner_mode_.mode != message->mode)
    mode_command_barrier_ = command_sequence_;
  planner_mode_ = *message;
  has_planner_mode_ = true;
  planner_mode_receive_steady_ = steadySeconds();
}

void ObstacleStop::emergencyStopCallback(
    const std_msgs::msg::Bool::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  emergency_stop_active_ = message->data;
}

void ObstacleStop::postureStopCallback(
    const std_msgs::msg::Bool::ConstSharedPtr message) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    posture_stop_active_ = message->data;
  }
  if (message->data) publishZeroVelocity();
}

void ObstacleStop::obstacleStopEnabledCallback(
    const std_msgs::msg::Bool::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_stop_enabled_ = message->data;
}

void ObstacleStop::updateObstacleState(bool raw_hard_stop, bool raw_slowdown,
                                       bool longitudinal_hard_stop, const rclcpp::Time &now) {
  // 障碍出现立即触发；持续出现时不断向后刷新解除截止时间。
  if (raw_hard_stop) {
    hard_stop_clear_deadline_ =
        now + rclcpp::Duration::from_seconds(config_.hard_stop_clear_delay);
  }
  // 只有前后障碍可产生减速及清障过渡；侧面停止不会建立减速状态。
  if (longitudinal_hard_stop || raw_slowdown) {
    slowdown_clear_deadline_ =
        now + rclcpp::Duration::from_seconds(config_.slowdown_clear_delay);
  }

  hard_stop_active_ = raw_hard_stop || now < hard_stop_clear_deadline_;
  slowdown_active_ = raw_slowdown || now < slowdown_clear_deadline_;
}

std::array<MotionPermission, 4> ObstacleStop::directionPermissions(
    const std::array<DirectionPrecheck, 4> &prechecks, bool healthy) const {
  std::array<MotionPermission, 4> permissions;
  const bool sides_enabled = config_.zones[2].enabled && config_.zones[3].enabled;
  for (std::size_t direction = 1; direction < 4; ++direction) {
    auto &permission = permissions[direction];
    const auto &precheck = prechecks[direction];
    if (!healthy) permission.reason = "sensor or global protection unavailable";
    else if (!config_.planner_velocity_guard_enabled) permission.reason = "planner velocity guard disabled";
    else if (!sides_enabled || !config_.zones[direction].enabled ||
        (direction == 1 && !config_.zones[0].enabled))
      permission.reason = "required motion stop zones disabled";
    else if (zone_runtime_[direction].hard_active ||
        (direction == 1 && (zone_runtime_[2].hard_active || zone_runtime_[3].hard_active)))
      permission.reason = "destination or backward side stop zone triggered";
    else if (!precheck.valid) permission.reason = "destination precheck unavailable";
    else if (precheck.available_distance <= 0.0)
      permission.reason = "destination clearance insufficient";
    else {
      permission.allowed = true;
      permission.velocity_limit = config_.velocity_guard.velocity;
      permission.reason = "destination safe";
    }
  }
  return permissions;
}

ObstacleStop::ControlInputs ObstacleStop::readControlInputs() {
  using Mode = navigation_interfaces::msg::PlannerMotionMode;
  ControlInputs inputs;
  bool has_lidar, lidar_valid, rear_has_lidar, rear_lidar_valid;
  std::uint64_t command_sequence, mode_command_barrier;
  double mode_receive_steady;
  double lidar_receive_steady, rear_lidar_receive_steady, command_receive_steady;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    inputs.planned = planned_cmd_;
    has_lidar = has_lidar_; lidar_valid = lidar_valid_;
    rear_has_lidar = rear_has_lidar_; rear_lidar_valid = rear_lidar_valid_;
    inputs.has_command = has_command_;
    inputs.hard_stop = hard_stop_active_; inputs.slowdown = slowdown_active_;
    inputs.emergency_stop = emergency_stop_active_; inputs.posture_stop = posture_stop_active_;
    inputs.enabled = obstacle_stop_enabled_;
    lidar_receive_steady = front_lidar_receive_steady_;
    rear_lidar_receive_steady = rear_lidar_receive_steady_;
    command_receive_steady = command_receive_steady_;
    inputs.mode = planner_mode_; inputs.has_mode = has_planner_mode_;
    mode_receive_steady = planner_mode_receive_steady_;
    command_sequence = command_sequence_; mode_command_barrier = mode_command_barrier_;
    if (imu_stairs_enabled_) inputs.stairs = imu_stair_detector_.status(node_->now().seconds());
  }
  const auto now = node_->now();
  const double steady_now = steadySeconds();
  const double lidar_age = has_lidar ? steady_now-lidar_receive_steady : std::numeric_limits<double>::infinity();
  const double rear_lidar_age = rear_has_lidar ? steady_now-rear_lidar_receive_steady : std::numeric_limits<double>::infinity();
  const double command_age = inputs.has_command ? steady_now-command_receive_steady : std::numeric_limits<double>::infinity();
  const bool front_required = sourceRequired(1U), rear_required = sourceRequired(2U);
  const bool front_fresh = !front_required ||
      (has_lidar && lidar_age >= 0.0 && lidar_age <= config_.lidar_timeout_sec);
  const bool rear_fresh = !rear_required ||
      (rear_has_lidar && rear_lidar_age >= 0.0 && rear_lidar_age <= config_.lidar_timeout_sec);
  inputs.lidar_fresh = front_fresh && rear_fresh;
  inputs.lidar_valid = (!front_required || lidar_valid) && (!rear_required || rear_lidar_valid);
  const auto lidar_status = [](bool required, bool received, bool fresh, bool valid) {
    return !required ? "unused" : !received ? "waiting" : !fresh ? "timeout" : !valid ? "invalid" : "ok";
  };
  inputs.lidar_status = {lidar_status(front_required, has_lidar, front_fresh, lidar_valid),
                        lidar_status(rear_required, rear_has_lidar, rear_fresh, rear_lidar_valid)};
  inputs.command_fresh = inputs.has_command && command_age >= 0.0 && command_age <= config_.command_timeout_sec;
  const auto &planned = inputs.planned;
  inputs.finite_command = std::isfinite(planned.linear.x) && std::isfinite(planned.linear.y) &&
      std::isfinite(planned.linear.z) && std::isfinite(planned.angular.x) &&
      std::isfinite(planned.angular.y) && std::isfinite(planned.angular.z);
  const auto &mode = inputs.mode;
  const double mode_stamp = static_cast<double>(mode.stamp.sec) +
      static_cast<double>(mode.stamp.nanosec) * 1e-9;
  const double mode_age = now.seconds() - mode_stamp;
  const bool mode_known = mode.mode == Mode::NORMAL || mode.mode == Mode::LATERAL_LEFT ||
      mode.mode == Mode::LATERAL_RIGHT || mode.mode == Mode::BACKWARD;
  inputs.mode_fresh = inputs.has_mode && mode_known &&
      mode.stamp.nanosec < 1000000000U && mode_stamp > 0.0 &&
      mode_age >= 0.0 && mode_age <= config_.planner_mode_timeout_sec &&
      steady_now >= mode_receive_steady &&
      steady_now-mode_receive_steady <= config_.planner_mode_timeout_sec;
  inputs.command_after_mode = !inputs.has_mode || command_sequence > mode_command_barrier;
  return inputs;
}

ObstacleStop::ControlDecision ObstacleStop::evaluateVelocity(
    const ControlInputs &inputs, const std::array<MotionPermission, 4> &permissions) const {
  using Mode = navigation_interfaces::msg::PlannerMotionMode;
  const geometry_msgs::msg::Twist zero;
  if (inputs.emergency_stop) return {zero, State::kEmergencyStop, "external emergency stop"};
  if (inputs.posture_stop) return {zero, State::kPostureStop, "navigation blocked by robot posture"};
  if (!inputs.command_fresh) return {zero, State::kWaitingForCommand,
      inputs.has_command ? "planned velocity timed out" : "waiting for planned velocity"};
  if (!inputs.finite_command) return {zero, State::kSensorFault, "non-finite planned velocity"};
  // Before the first mode message, retain legacy control. A lost mode heartbeat
  // after integration must stop output rather than silently revert to NORMAL.
  if (inputs.has_mode && !inputs.mode_fresh) return {zero, State::kPlannerMotionBlocked,
      "planner mode invalid or timed out; require fresh mode heartbeat"};
  if (!inputs.command_after_mode) return {zero, State::kWaitingForCommand,
      "waiting for new planned velocity after mode change"};
  if ((inputs.enabled || inputs.directionalMotion()) && !inputs.lidar_fresh)
    return {zero, State::kWaitingForLidar, "required lidar unavailable or timed out"};
  if ((inputs.enabled || inputs.directionalMotion()) && !inputs.lidar_valid)
    return {zero, State::kSensorFault, "required lidar point cloud invalid"};
  if (inputs.directionalMotion()) {
    const auto &permission = permissions[motionDirection(inputs.mode.mode)];
    if (!permission.allowed) return {zero, State::kPlannerMotionBlocked, permission.reason};
    const auto decision = evaluatePlannerVelocityDecision(inputs.mode.mode, inputs.planned,
                                                         permission.velocity_limit);
    if (!decision.allowed) return {zero, State::kPlannerMotionBlocked, decision.reason};
    geometry_msgs::msg::Twist velocity;
    if (inputs.mode.mode == Mode::BACKWARD) velocity.linear.x = decision.velocity;
    else velocity.linear.y = decision.velocity;
    return {velocity, State::kPlannerMotion, decision.reason};
  }
  if (!inputs.enabled) return {inputs.planned, State::kClear, "obstacle stop disabled"};
  if (inputs.hard_stop) return {zero, State::kObstacleStop, "obstacle in hard stop rectangle or clearance delay"};
  auto velocity = inputs.planned;
  if (inputs.slowdown && !(inputs.stairs.valid && inputs.stairs.on_stairs)) {
    velocity.linear.x *= config_.linear_velocity_factor;
    velocity.linear.y *= config_.linear_velocity_factor;
    velocity.angular.z *= config_.angular_velocity_factor;
    return {velocity, State::kObstacleSlowdown, "obstacle in slowdown rectangle or clearance delay"};
  }
  return {velocity, State::kClear, "clear"};
}

navigation_interfaces::msg::ObstacleStopStatus ObstacleStop::publishControlStatus(
    const ControlInputs &inputs, const std::array<MotionPermission, 4> &permissions) {
  navigation_interfaces::msg::ObstacleStopStatus status;
  for (std::size_t direction = 0; direction < 4; ++direction)
    status.stop_triggered[direction] = config_.zones[direction].enabled && zone_runtime_[direction].hard_active;
  // Environmental permission; freshness and velocity validity are checked separately.
  status.move_allowed[0] = inputs.environmentReady() && !inputs.hard_stop;
  // FRONT has no distance precheck; its distance remains the default zero.
  for (std::size_t direction = 1; direction < 4; ++direction) {
    status.move_allowed[direction] = permissions[direction].allowed;
    status.available_distance[direction] = status.move_allowed[direction]
        ? static_cast<float>(direction_prechecks_[direction].available_distance) : 0.0F;
  }
  status_pub_->publish(status);
  return status;
}

void ObstacleStop::controlCallback() {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  if (stopped_) return;
  updateZoneState();
  const auto inputs = readControlInputs();
  refreshDirectionPrechecks(inputs.sensorsValid());
  const auto permissions = directionPermissions(direction_prechecks_, inputs.environmentReady());
  const auto decision = evaluateVelocity(inputs, permissions);
  safe_cmd_pub_->publish(decision.velocity);
  const auto status = publishControlStatus(inputs, permissions);

  const bool suppress_slowdown = inputs.stairs.valid && inputs.stairs.on_stairs;
  if (stairs_active_pub_) {
    std_msgs::msg::Bool active; active.data = suppress_slowdown; stairs_active_pub_->publish(active);
  }
  if (suppress_slowdown != last_stairs_active_) {
    RCLCPP_INFO(node_->get_logger(), "IMU stairs slowdown suppression=%s, valid=%s pitch=%.2f deg",
        suppress_slowdown ? "on" : "off", inputs.stairs.valid ? "true" : "false", inputs.stairs.pitch_deg);
    last_stairs_active_ = suppress_slowdown;
  }
  logControlStatus(inputs, decision, status);
}

void ObstacleStop::logControlStatus(const ControlInputs &inputs, const ControlDecision &decision,
    const navigation_interfaces::msg::ObstacleStopStatus &status) {
  using Mode = navigation_interfaces::msg::PlannerMotionMode;
  const auto mode_name = [](std::uint8_t mode) {
    switch (mode) {
      case Mode::NORMAL: return "NORMAL";
      case Mode::LATERAL_LEFT: return "LEFT";
      case Mode::LATERAL_RIGHT: return "RIGHT";
      case Mode::BACKWARD: return "BACKWARD";
      default: return "INVALID";
    }
  };
  const char *mode = inputs.has_mode ? mode_name(inputs.mode.mode) : "LEGACY";
  const auto &zones = config_.zones;
  const auto &runtime = zone_runtime_;
  if (!has_logged_state_ || decision.state != last_state_ || decision.reason != last_state_reason_ ||
      status.stop_triggered != last_stop_triggered_) {
    std::ostringstream event;
    event << "[STOP_EVENT] state=" << stateName(decision.state) << " mode=" << mode
          << '(' << static_cast<unsigned>(inputs.mode.mode) << ") reason='" << decision.reason
          << "' stop_cells(F/R/L/R)=";
    for (std::size_t direction = 0; direction < 4; ++direction) {
      if (direction != 0U) event << ',';
      event << runtime[direction].hard_occupied.size() << '/' << zones[direction].obstacle_cell_threshold;
    }
    event << " triggered=";
    for (bool triggered : status.stop_triggered) event << triggered;
    const auto message = event.str();
    if (decision.state == State::kClear || decision.state == State::kPlannerMotion ||
        decision.state == State::kObstacleSlowdown)
      RCLCPP_INFO(node_->get_logger(), "%s", message.c_str());
    else
      RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
    has_logged_state_ = true; last_state_ = decision.state; last_state_reason_ = decision.reason;
    last_stop_triggered_ = status.stop_triggered;
  }

  const bool any_occupied = std::any_of(runtime.begin(), runtime.end(), [](const ZoneRuntime &zone) {
    return !zone.hard_occupied.empty() || (zone.slow && !zone.slow->occupied.empty());
  });
  if (!any_occupied && decision.state == State::kClear && !inputs.directionalMotion()) return;
  const double now = steadySeconds();
  if (last_diagnostic_steady_ >= 0.0 && now-last_diagnostic_steady_ < 1.0) return;
  last_diagnostic_steady_ = now;
  const auto &input = inputs.planned;
  const auto &output = decision.velocity;
  RCLCPP_INFO(node_->get_logger(),
      "[STOP_STATUS] state=%s mode=%s enabled=%d command=%s mode_status=%s lidar(F/R)=%s/%s "
      "stop_cells(F/R/L/R)=%zu/%zu,%zu/%zu,%zu/%zu,%zu/%zu slow_cells(F/R)=%zu/%zu,%zu/%zu "
      "allowed(F/R/L/R)=%d%d%d%d in(vx/vy/wz)=%.3f/%.3f/%.3f out=%.3f/%.3f/%.3f",
      stateName(decision.state), mode, inputs.enabled,
      !inputs.has_command ? "waiting" : !inputs.command_fresh ? "timeout" : !inputs.finite_command ? "invalid" : "ok",
      !inputs.has_mode ? "legacy" : inputs.mode_fresh ? "ok" : "invalid_or_timeout",
      inputs.lidar_status[0], inputs.lidar_status[1],
      runtime[0].hard_occupied.size(), zones[0].obstacle_cell_threshold,
      runtime[1].hard_occupied.size(), zones[1].obstacle_cell_threshold,
      runtime[2].hard_occupied.size(), zones[2].obstacle_cell_threshold,
      runtime[3].hard_occupied.size(), zones[3].obstacle_cell_threshold,
      runtime[0].slow ? runtime[0].slow->occupied.size() : 0U, zones[0].obstacle_cell_threshold,
      runtime[1].slow ? runtime[1].slow->occupied.size() : 0U, zones[1].obstacle_cell_threshold,
      status.move_allowed[0], status.move_allowed[1], status.move_allowed[2], status.move_allowed[3],
      input.linear.x, input.linear.y, input.angular.z, output.linear.x, output.linear.y, output.angular.z);

  const auto &rear = direction_prechecks_[1];
  const auto &left = direction_prechecks_[2], &right = direction_prechecks_[3];
  RCLCPP_INFO(node_->get_logger(),
      "[STOP_CLEARANCE] unit=m order=B/L/R valid=%d%d%d cells=%zu/%zu,%zu/%zu,%zu/%zu "
      "measured=%.3f/%.3f/%.3f available=%.3f/%.3f/%.3f",
      rear.valid, left.valid, right.valid,
      rear.occupied_cells, zones[1].obstacle_cell_threshold,
      left.occupied_cells, zones[2].obstacle_cell_threshold,
      right.occupied_cells, zones[3].obstacle_cell_threshold,
      rear.available_distance, left.available_distance, right.available_distance,
      status.available_distance[1], status.available_distance[2], status.available_distance[3]);
}

void ObstacleStop::stop() {
  if (control_timer_) control_timer_->cancel();
  if (visualization_timer_) visualization_timer_->cancel();
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  stopped_ = true;
  if (status_pub_) {
    navigation_interfaces::msg::ObstacleStopStatus status;
    status_pub_->publish(status);
  }
  // Worker停止前主动发送一次零速度。
  publishZeroVelocity();
}

void ObstacleStop::publishZeroVelocity() {
  if (safe_cmd_pub_) safe_cmd_pub_->publish(geometry_msgs::msg::Twist());
}

const char *ObstacleStop::stateName(State state) {
  switch (state) {
    case State::kWaitingForCommand:
      return "waiting_for_command";
    case State::kWaitingForLidar:
      return "waiting_for_lidar";
    case State::kClear:
      return "clear";
    case State::kObstacleSlowdown:
      return "obstacle_slowdown";
    case State::kObstacleStop:
      return "obstacle_stop";
    case State::kPlannerMotion:
      return "planner_motion";
    case State::kPlannerMotionBlocked:
      return "planner_motion_blocked";
    case State::kPostureStop:
      return "posture_stop";
    case State::kEmergencyStop:
      return "emergency_stop";
    case State::kSensorFault:
      return "sensor_fault";
  }
  return "unknown";
}

}  // namespace obstacle_stop
