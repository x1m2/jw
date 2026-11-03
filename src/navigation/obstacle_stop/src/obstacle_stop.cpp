#include "obstacle_stop/obstacle_stop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
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

ObstacleStop::ObstacleStop(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      last_lidar_time_(0, 0, node_->get_clock()->get_clock_type()),
      rear_last_lidar_time_(0, 0, node_->get_clock()->get_clock_type()),
      last_command_time_(0, 0, node_->get_clock()->get_clock_type()),
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
      planned_cmd_topic_, 10,
      std::bind(&ObstacleStop::plannedVelocityCallback, this,
                std::placeholders::_1));
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
  stop_active_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/obstacle_stop/active", rclcpp::QoS(1).transient_local());
  obstacle_stopped_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/obstacle_stop/stopped", rclcpp::QoS(1).reliable().transient_local());
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

  const auto translation = node_->declare_parameter<std::vector<double>>(
      "lidar_to_base_T", std::vector<double>());
  const auto rotation = node_->declare_parameter<std::vector<double>>(
      "lidar_to_base_R", std::vector<double>());
  const Eigen::AngleAxisd roll(rotation[0], Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch(rotation[1], Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw(rotation[2], Eigen::Vector3d::UnitZ());
  lidar_to_base_ =
      Eigen::Translation3d(translation[0], translation[1], translation[2]) *
      yaw * pitch * roll;

  const auto rear_translation = node_->declare_parameter<std::vector<double>>(
      "rear_lidar_to_base_T", std::vector<double>{-0.4, 0.0, 0.55});
  const auto rear_rotation = node_->declare_parameter<std::vector<double>>(
      "rear_lidar_to_base_R",
      std::vector<double>{0.0, 0.0, 0.0});
  const Eigen::AngleAxisd rear_roll(rear_rotation[0],
                                    Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd rear_pitch(rear_rotation[1],
                                     Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rear_yaw(rear_rotation[2],
                                   Eigen::Vector3d::UnitZ());
  rear_lidar_to_base_ =
      Eigen::Translation3d(rear_translation[0], rear_translation[1],
                           rear_translation[2]) *
      rear_yaw * rear_pitch * rear_roll;

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
  config_.directional_release_enabled = node_->declare_parameter<bool>(
      "directional_release_enabled", config_.directional_release_enabled);
  config_.directional_center_half_width = std::clamp(
      node_->declare_parameter<double>("directional_center_half_width",
                                       config_.directional_center_half_width),
      0.0, config_.robot_width * 0.5);
  config_.directional_escape_velocity_threshold =
      std::max(0.0, node_->declare_parameter<double>(
                        "directional_escape_velocity_threshold",
                        config_.directional_escape_velocity_threshold));
  config_.center_lateral_escape_enabled = node_->declare_parameter<bool>(
      "center_lateral_escape_enabled", config_.center_lateral_escape_enabled);
  config_.center_escape_max_forward_velocity =
      std::max(0.0, node_->declare_parameter<double>(
                        "center_escape_max_forward_velocity",
                        config_.center_escape_max_forward_velocity));
  config_.center_escape_max_angular_velocity =
      std::max(0.0, node_->declare_parameter<double>(
                        "center_escape_max_angular_velocity",
                        config_.center_escape_max_angular_velocity));
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
  applyRegionProfile(
      findRegion(message->pose.position.x, message->pose.position.y));
}

void ObstacleStop::updateZoneState() {
  const auto now = node_->now();
  std::array<bool, 2> usable;
  geometry_msgs::msg::Twist planned;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double front_age = (now-last_lidar_time_).seconds();
    const double rear_age = (now-rear_last_lidar_time_).seconds();
    usable = {has_lidar_ && lidar_valid_ && front_age >= 0.0 && front_age <= config_.lidar_timeout_sec,
              rear_has_lidar_ && rear_lidar_valid_ && rear_age >= 0.0 && rear_age <= config_.lidar_timeout_sec};
    planned = planned_cmd_;
  }
  // Timeouts invalidate cached occupancy even when no new packet arrives.
  refreshZoneOccupancy(usable);
  bool hard = false, slow = false, released = false, longitudinal_hard = false;
  for (std::size_t i = 0; i < config_.zones.size(); ++i) {
    const auto &zone = config_.zones[i];
    auto &runtime = zone_runtime_[i];
    if (!zone.enabled) continue;
    const auto decide = [&](const std::vector<std::size_t> &cells, const ZoneRectangle &box) {
      if (i >= 2U || !config_.directional_release_enabled) return cells.size() >= zone.obstacle_cell_threshold;
      std::array<std::size_t, 3> counts{};
      const double cy = (box.y_min+box.y_max)*0.5;
      for (const auto index : cells) {
        const int my = static_cast<int>(index/config_.grid_width);
        const double y = (my-config_.grid_height*0.5+0.5)*config_.grid_resolution - cy;
        ++counts[y > config_.directional_center_half_width ? 0U :
                 y < -config_.directional_center_half_width ? 2U : 1U];
      }
      const DirectionalZoneOccupancy occupancy{counts[0]>=zone.obstacle_cell_threshold,
          counts[1]>=zone.obstacle_cell_threshold, counts[2]>=zone.obstacle_cell_threshold};
      const double x = (box.x_min+box.x_max)*0.5;
      const auto decision = evaluateDirectionalEscape(occupancy,
          {planned.linear.x, planned.linear.y, planned.angular.z*x},
          config_.directional_escape_velocity_threshold, config_.center_escape_max_forward_velocity,
          config_.center_escape_max_angular_velocity*std::abs(x), true, config_.center_lateral_escape_enabled);
      released = released || decision.released_left || decision.released_center || decision.released_right;
      return decision.active;
    };
    runtime.hard_active = decide(runtime.hard_occupied, runtime.hard_box);
    if (runtime.slow) {
      runtime.slow->active = decide(runtime.slow->occupied, runtime.slow->box);
      slow = slow || runtime.slow->active;
    }
    hard = hard || runtime.hard_active;
    if (i < 2U) longitudinal_hard = longitudinal_hard || runtime.hard_active;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  updateObstacleState(hard, slow, longitudinal_hard, now);
  directional_escape_release_active_ = !hard && released;
}

void ObstacleStop::plannedVelocityCallback(
    const geometry_msgs::msg::Twist::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  planned_cmd_ = *message;
  last_command_time_ = node_->now();
  has_command_ = true;
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

void ObstacleStop::controlCallback() {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  updateZoneState();
  geometry_msgs::msg::Twist planned;
  bool has_lidar = false;
  bool lidar_valid = false;
  bool rear_has_lidar = false;
  bool rear_lidar_valid = false;
  bool has_command = false;
  bool hard_stop = false;
  bool slowdown = false;
  bool emergency_stop = false;
  bool posture_stop = false;
  bool obstacle_stop_enabled = true;
  bool directional_escape_release = false;
  imu_stair_detector::Status stairs;
  rclcpp::Time lidar_time(0, 0, node_->get_clock()->get_clock_type());
  rclcpp::Time rear_lidar_time(0, 0, node_->get_clock()->get_clock_type());
  rclcpp::Time command_time(0, 0, node_->get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planned = planned_cmd_;
    has_lidar = has_lidar_;
    lidar_valid = lidar_valid_;
    rear_has_lidar = rear_has_lidar_;
    rear_lidar_valid = rear_lidar_valid_;
    has_command = has_command_;
    hard_stop = hard_stop_active_;
    slowdown = slowdown_active_;
    emergency_stop = emergency_stop_active_;
    posture_stop = posture_stop_active_;
    obstacle_stop_enabled = obstacle_stop_enabled_;
    directional_escape_release = directional_escape_release_active_;
    lidar_time = last_lidar_time_;
    rear_lidar_time = rear_last_lidar_time_;
    command_time = last_command_time_;
    if (imu_stairs_enabled_) stairs = imu_stair_detector_.status(node_->now().seconds());
  }
  const auto now = node_->now();
  const double lidar_age = has_lidar ? (now - lidar_time).seconds()
                                     : std::numeric_limits<double>::infinity();
  const double rear_lidar_age =
      rear_has_lidar ? (now - rear_lidar_time).seconds()
                     : std::numeric_limits<double>::infinity();
  const double command_age = has_command
                                 ? (now - command_time).seconds()
                                 : std::numeric_limits<double>::infinity();
  const bool front_lidar_fresh =
      !sourceRequired(1U) || (has_lidar && lidar_age >= 0.0 && lidar_age <= config_.lidar_timeout_sec);
  const bool rear_lidar_fresh =
      !sourceRequired(2U) ||
      (rear_has_lidar && rear_lidar_age >= 0.0 &&
       rear_lidar_age <= config_.lidar_timeout_sec);
  const bool lidar_fresh = front_lidar_fresh && rear_lidar_fresh;
  const bool all_lidar_valid =
      (!sourceRequired(1U) || lidar_valid) && (!sourceRequired(2U) || rear_lidar_valid);
  const bool command_fresh = has_command && command_age >= 0.0 &&
                             command_age <= config_.command_timeout_sec;

  const bool suppress_slowdown = stairs.valid && stairs.on_stairs;
  if (stairs_active_pub_) {
    std_msgs::msg::Bool active;
    active.data = suppress_slowdown;
    stairs_active_pub_->publish(active);
  }
  if (suppress_slowdown != last_stairs_active_) {
    RCLCPP_INFO(node_->get_logger(), "IMU stairs slowdown suppression=%s, valid=%s pitch=%.2f deg",
        suppress_slowdown ? "on" : "off", stairs.valid ? "true" : "false", stairs.pitch_deg);
    last_stairs_active_ = suppress_slowdown;
  }

  // safe_command默认是零速度，因此所有异常分支都会失效停车。
  geometry_msgs::msg::Twist safe_command;
  State state = State::kClear;
  std::string reason = "clear";
  if (emergency_stop) {
    state = State::kEmergencyStop;
    reason = "external emergency stop";
  } else if (posture_stop) {
    state = State::kPostureStop;
    reason = "navigation blocked by robot posture";
  } else if (!command_fresh) {
    state = State::kWaitingForCommand;
    reason = has_command ? "planned velocity timed out"
                         : "waiting for planned velocity";
  } else if (!obstacle_stop_enabled) {
    // 停障未启用时仅绕过雷达停车和减速，外部急停与指令超时仍然有效。
    safe_command = planned;
  } else if (!lidar_fresh) {
    state = State::kWaitingForLidar;
    if (!front_lidar_fresh) {
      reason = has_lidar ? "front lidar data timed out"
                         : "waiting for front lidar data";
    } else {
      reason = rear_has_lidar ? "rear lidar data timed out"
                              : "waiting for rear lidar data";
    }
  } else if (!all_lidar_valid) {
    state = State::kSensorFault;
    reason = sourceRequired(1U) && !lidar_valid ? "front lidar point cloud is invalid"
                          : "rear lidar point cloud is invalid";
  } else if (hard_stop && !directional_escape_release) {
    state = State::kObstacleStop;
    reason = "obstacle in hard stop rectangle";
  } else if (hard_stop && directional_escape_release) {
    safe_command = planned;
    reason = "directional escape away from side obstacle";
  } else if (slowdown && !suppress_slowdown) {
    safe_command = planned;
    // 远矩形有障碍时降低速度，近矩形仍具有更高优先级。
    safe_command.linear.x *= config_.linear_velocity_factor;
    safe_command.linear.y *= config_.linear_velocity_factor;
    safe_command.angular.z *= config_.angular_velocity_factor;
    state = State::kObstacleSlowdown;
    reason = "obstacle in slowdown rectangle";
  } else {
    safe_command = planned;
  }

  safe_cmd_pub_->publish(safe_command);
  std_msgs::msg::Bool stop_active;
  stop_active.data = state != State::kClear;
  stop_active_pub_->publish(stop_active);
  std_msgs::msg::Bool obstacle_stopped;
  obstacle_stopped.data = state == State::kObstacleStop;
  obstacle_stopped_pub_->publish(obstacle_stopped);
  logStateChange(state, reason);
  // Report observed stop-zone obstacles independently of command availability
  // or release-delay holds. Slowdown-only occupancy produces no periodic log.
  if (std::none_of(zone_runtime_.begin(), zone_runtime_.end(),
                   [](const ZoneRuntime &zone) {
                     return !zone.hard_occupied.empty();
                   })) {
    return;
  }
  const auto lidar_status = [&](unsigned source, bool received, bool valid,
                                 double age) {
    if (!sourceRequired(source)) return "unused";
    if (!received) return "waiting";
    if (age < 0.0 || age > config_.lidar_timeout_sec) return "timeout";
    return valid ? "ok" : "invalid";
  };
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
      "[STOP_STATUS] state=%s enabled=%s command=%s "
      "front_lidar=%s rear_lidar=%s "
      "front(stop/slow)=%zu/%zu rear(stop/slow)=%zu/%zu "
      "left(stop)=%zu right(stop)=%zu output(vx/vy/wz)=%.3f/%.3f/%.3f",
      stateName(state), obstacle_stop_enabled ? "true" : "false",
      command_fresh ? "ok" : has_command ? "timeout" : "waiting",
      lidar_status(1U, has_lidar, lidar_valid, lidar_age),
      lidar_status(2U, rear_has_lidar, rear_lidar_valid, rear_lidar_age),
      zone_runtime_[0].hard_occupied.size(),
      zone_runtime_[0].slow ? zone_runtime_[0].slow->occupied.size() : 0U,
      zone_runtime_[1].hard_occupied.size(),
      zone_runtime_[1].slow ? zone_runtime_[1].slow->occupied.size() : 0U,
      zone_runtime_[2].hard_occupied.size(), zone_runtime_[3].hard_occupied.size(),
      safe_command.linear.x, safe_command.linear.y, safe_command.angular.z);
}

void ObstacleStop::stop() {
  if (control_timer_) control_timer_->cancel();
  if (visualization_timer_) visualization_timer_->cancel();
  if (obstacle_stopped_pub_) {
    std_msgs::msg::Bool stopped;
    stopped.data = false;
    obstacle_stopped_pub_->publish(stopped);
  }
  // Worker停止前主动发送一次零速度。
  publishZeroVelocity();
}

void ObstacleStop::publishZeroVelocity() {
  if (safe_cmd_pub_) safe_cmd_pub_->publish(geometry_msgs::msg::Twist());
}

void ObstacleStop::logStateChange(State state, const std::string &reason) {
  if (has_logged_state_ && state == last_state_) return;
  has_logged_state_ = true;
  last_state_ = state;
  if (state == State::kClear) {
    RCLCPP_INFO(node_->get_logger(), "Obstacle stop state=%s: %s",
                stateName(state), reason.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "Obstacle stop state=%s: %s",
                stateName(state), reason.c_str());
  }
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
