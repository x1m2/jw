#include "obstacle_stop/obstacle_stop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <utility>

#include "obstacle_stop/region_profile_loader.hpp"

namespace obstacle_stop {

namespace {
constexpr uint8_t kFree = 0U;
constexpr uint8_t kOccupied = 1U;

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
      last_command_time_(0, 0, node_->get_clock()->get_clock_type()),
      hard_stop_clear_deadline_(0, 0, node_->get_clock()->get_clock_type()),
      slowdown_clear_deadline_(0, 0, node_->get_clock()->get_clock_type()) {}

bool ObstacleStop::initialize(std::string &message) {
  loadParameters();
  default_config_ = config_;
  if (!loadRegionProfiles(message)) return false;
  initializeGridLookup();

  // 默认保持MID360原始点云模式；UWB楼梯跟随使用已经在base_link下的
  // obstacle_cloud，使台阶过滤结果同时作用于绕障和最终停障。
  if (lidar_type_ == 1) {
    lidar_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleStop::lidarCallback, this, std::placeholders::_1));
  } else {
    pointcloud2_sub_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ObstacleStop::pointCloud2Callback, this,
                      std::placeholders::_1));
  }
  planned_cmd_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      planned_cmd_topic_, 10,
      std::bind(&ObstacleStop::plannedVelocityCallback, this,
                std::placeholders::_1));
  emergency_stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", 10,
      std::bind(&ObstacleStop::emergencyStopCallback, this,
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
  if (config_.publish_visualization) {
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "/obstacle_stop/detection_marker", 1);
  }

  control_timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.control_frequency)),
      std::bind(&ObstacleStop::controlCallback, this));

  RCLCPP_INFO(node_->get_logger(),
              "Obstacle stop initialized: lidar_topic=%s, lidar_type=%d (%s), "
              "planned_cmd=%s, safe_cmd=%s",
              lidar_topic_.c_str(), lidar_type_,
              lidar_type_ == 1 ? "Livox CustomMsg" : "PointCloud2",
              planned_cmd_topic_.c_str(), safe_cmd_topic_.c_str());
  message = "obstacle stop initialized";
  return true;
}

void ObstacleStop::loadParameters() {
  lidar_topic_ =
      node_->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  lidar_type_ = node_->declare_parameter<int>("lidar_type", 1);
  pointcloud2_apply_height_filter_ =
      node_->declare_parameter<bool>("pointcloud2_apply_height_filter", true);
  planned_cmd_topic_ = node_->declare_parameter<std::string>(
      "planned_cmd_topic", "/cmd_vel_planned");
  safe_cmd_topic_ =
      node_->declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel");
  base_frame_ =
      node_->declare_parameter<std::string>("base_frame", "base_link");
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
  config_.grid_width =
      node_->declare_parameter<int>("grid_width", config_.grid_width);
  config_.grid_height =
      node_->declare_parameter<int>("grid_height", config_.grid_height);
  config_.inflation_radius = node_->declare_parameter<double>(
      "inflation_radius", config_.inflation_radius);
  config_.robot_width =
      node_->declare_parameter<double>("robot_width", config_.robot_width);
  config_.robot_length =
      node_->declare_parameter<double>("robot_length", config_.robot_length);
  config_.safety_margin =
      node_->declare_parameter<double>("safety_margin", config_.safety_margin);
  config_.hard_stop_distance = node_->declare_parameter<double>(
      "hard_stop_distance", config_.hard_stop_distance);
  config_.slowdown_distance = node_->declare_parameter<double>(
      "slowdown_distance", config_.slowdown_distance);
  config_.obstacle_ratio_threshold = node_->declare_parameter<double>(
      "obstacle_ratio_threshold", config_.obstacle_ratio_threshold);
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
      0.0, config_.robot_width * 0.5 + config_.safety_margin);
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
          "region_obstacle_stop" / (std::to_string(region.id) + ".yaml");
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
      apply(overrides.hard_stop_distance, region.config.hard_stop_distance);
      apply(overrides.slowdown_distance, region.config.slowdown_distance);
      apply(overrides.obstacle_ratio_threshold,
            region.config.obstacle_ratio_threshold);
      apply(overrides.linear_velocity_factor,
            region.config.linear_velocity_factor);
      apply(overrides.angular_velocity_factor,
            region.config.angular_velocity_factor);
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
              "missing ID YAML uses obstacle_stop.yaml defaults",
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
  active_region_id_ = next_id;
  if (region == nullptr) {
    RCLCPP_INFO(node_->get_logger(),
                "Left navigation region; restored obstacle_stop.yaml defaults");
  } else if (region->has_custom_config) {
    RCLCPP_INFO(node_->get_logger(),
                "Entered region id=%d; applied region obstacle_stop profile",
                region->id);
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Entered region id=%d; no ID YAML, using obstacle_stop.yaml "
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

void ObstacleStop::initializeGridLookup() {
  // 膨胀圆内的栅格偏移只与分辨率和膨胀半径有关，启动时计算一次。
  inflation_offsets_.clear();
  const int radius_cells = static_cast<int>(
      std::ceil(config_.inflation_radius / config_.grid_resolution));
  inflation_offsets_.reserve(static_cast<std::size_t>((2 * radius_cells + 1) *
                                                      (2 * radius_cells + 1)));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
              config_.grid_resolution <=
          config_.inflation_radius) {
        inflation_offsets_.push_back(GridCell{dx, dy});
      }
    }
  }

  // 两个检测矩形内的栅格索引固定，避免每帧遍历整张栅格并重复换算坐标。
  const double front_edge = config_.robot_length * 0.5;
  const double hard_stop_end = front_edge + config_.hard_stop_distance;
  hard_stop_zone_cells_ = buildDirectionalZoneCells(front_edge, hard_stop_end);
  slowdown_zone_cells_ = buildDirectionalZoneCells(
      hard_stop_end, hard_stop_end + config_.slowdown_distance);
}

void ObstacleStop::lidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  std::vector<uint8_t> grid;
  std::vector<GridCell> occupied_cells;
  const auto grid_cell_count =
      static_cast<std::size_t>(config_.grid_width * config_.grid_height);
  occupied_cells.reserve(std::min(message->points.size(), grid_cell_count));
  buildObstacleGrid(*message, grid, occupied_cells);
  // 原始Livox空消息仍视为传感器异常。
  processObstacleGrid(grid, occupied_cells, !message->points.empty());
}

void ObstacleStop::pointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  std::vector<uint8_t> grid;
  std::vector<GridCell> occupied_cells;
  const auto grid_cell_count =
      static_cast<std::size_t>(config_.grid_width * config_.grid_height);
  const auto point_count =
      static_cast<std::size_t>(message->width) * message->height;
  occupied_cells.reserve(std::min(point_count, grid_cell_count));

  bool input_valid = true;
  if (!message->header.frame_id.empty() &&
      message->header.frame_id != base_frame_) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Semantic obstacle cloud frame '%s' differs from base frame '%s'",
        message->header.frame_id.c_str(), base_frame_.c_str());
    input_valid = false;
  } else {
    input_valid = buildObstacleGrid(*message, grid, occupied_cells);
  }

  // obstacle_cloud为空表示本帧分割成功且没有障碍，是有效的自由空间观测。
  processObstacleGrid(grid, occupied_cells, input_valid);
}

void ObstacleStop::processObstacleGrid(std::vector<uint8_t> &grid,
                                       std::vector<GridCell> &occupied_cells,
                                       bool input_valid) {
  if (grid.empty()) {
    grid.assign(
        static_cast<std::size_t>(config_.grid_width * config_.grid_height),
        kFree);
  }
  if (input_valid) {
    inflateGrid(grid, occupied_cells);
  }

  geometry_msgs::msg::Twist planned_snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planned_snapshot = planned_cmd_;
  }

  const auto hard_occupancy =
      measureDirectionalZone(grid, hard_stop_zone_cells_);
  const auto slowdown_occupancy =
      measureDirectionalZone(grid, slowdown_zone_cells_);
  const double front_edge = config_.robot_length * 0.5;
  const double hard_center_x = front_edge + config_.hard_stop_distance * 0.5;
  const double slowdown_center_x =
      front_edge + config_.hard_stop_distance + config_.slowdown_distance * 0.5;
  const double hard_lateral_velocity =
      planned_snapshot.linear.y + planned_snapshot.angular.z * hard_center_x;
  const double slowdown_lateral_velocity =
      planned_snapshot.linear.y +
      planned_snapshot.angular.z * slowdown_center_x;
  const DirectionalEscapeMotion hard_motion{
      planned_snapshot.linear.x, planned_snapshot.linear.y,
      planned_snapshot.angular.z * hard_center_x};
  const DirectionalEscapeMotion slowdown_motion{
      planned_snapshot.linear.x, planned_snapshot.linear.y,
      planned_snapshot.angular.z * slowdown_center_x};
  const auto hard_decision = evaluateDirectionalEscape(
      hard_occupancy, hard_motion,
      config_.directional_escape_velocity_threshold,
      config_.center_escape_max_forward_velocity,
      config_.center_escape_max_angular_velocity * hard_center_x,
      config_.directional_release_enabled,
      config_.center_lateral_escape_enabled);
  const auto slowdown_decision = evaluateDirectionalEscape(
      slowdown_occupancy, slowdown_motion,
      config_.directional_escape_velocity_threshold,
      config_.center_escape_max_forward_velocity,
      config_.center_escape_max_angular_velocity * slowdown_center_x,
      config_.directional_release_enabled,
      config_.center_lateral_escape_enabled);
  const bool raw_hard_stop = hard_decision.active;
  const bool raw_slowdown = slowdown_decision.active;
  const auto now = node_->now();
  bool hard_stop = false;
  bool slowdown = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_lidar_ = true;
    lidar_valid_ = input_valid;
    if (lidar_valid_) {
      updateObstacleState(raw_hard_stop, raw_slowdown, now);
    }
    hard_stop = hard_stop_active_;
    slowdown = slowdown_active_;
    directional_escape_release_active_ = hard_decision.released_left ||
                                         hard_decision.released_center ||
                                         hard_decision.released_right;
    last_lidar_time_ = now;
  }

  if (hard_decision.released_left || hard_decision.released_center ||
      hard_decision.released_right || slowdown_decision.released_left ||
      slowdown_decision.released_center || slowdown_decision.released_right) {
    RCLCPP_INFO_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "[OBSTACLE_DIRECTIONAL_RELEASE] planned=(vy=%.3f,wz=%.3f), "
        "hard_lateral=%.3f slow_lateral=%.3f, hard(L/C/R)=%d/%d/%d, "
        "slow(L/C/R)=%d/%d/%d",
        planned_snapshot.linear.y, planned_snapshot.angular.z,
        hard_lateral_velocity, slowdown_lateral_velocity, hard_occupancy.left,
        hard_occupancy.center, hard_occupancy.right, slowdown_occupancy.left,
        slowdown_occupancy.center, slowdown_occupancy.right);
  }

  publishDetectionMarkers(hard_stop, slowdown);
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

void ObstacleStop::obstacleStopEnabledCallback(
    const std_msgs::msg::Bool::ConstSharedPtr message) {
  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_stop_enabled_ = message->data;
}

void ObstacleStop::buildObstacleGrid(
    const livox_ros_driver2::msg::CustomMsg &message,
    std::vector<uint8_t> &grid, std::vector<GridCell> &occupied_cells) const {
  grid.assign(
      static_cast<std::size_t>(config_.grid_width * config_.grid_height),
      kFree);
  occupied_cells.clear();

  for (const auto &point : message.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    // 原始Livox点先应用雷达外参；语义PointCloud2由提取器直接输出在
    // base_link下，不再走这条变换路径。
    const Eigen::Vector3d point_base =
        lidar_to_base_ * Eigen::Vector3d(point.x, point.y, point.z);
    addBasePointToGrid(point_base.x(), point_base.y(), point_base.z(), grid,
                       occupied_cells);
  }
}

bool ObstacleStop::buildObstacleGrid(
    const sensor_msgs::msg::PointCloud2 &message, std::vector<uint8_t> &grid,
    std::vector<GridCell> &occupied_cells) const {
  grid.assign(
      static_cast<std::size_t>(config_.grid_width * config_.grid_height),
      kFree);
  occupied_cells.clear();

  // 合法空点云不包含字段，也应被接受为“当前没有障碍”。
  if (message.data.empty()) return true;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(message, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      addBasePointToGrid(*iter_x, *iter_y, *iter_z, grid, occupied_cells,
                         pointcloud2_apply_height_filter_);
    }
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid semantic obstacle PointCloud2: %s", error.what());
    return false;
  }
  return true;
}

void ObstacleStop::addBasePointToGrid(double x, double y, double z,
                                      std::vector<uint8_t> &grid,
                                      std::vector<GridCell> &occupied_cells,
                                      bool apply_height_filter) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return;
  }
  if (apply_height_filter &&
      (z < config_.min_height || z > config_.max_height)) {
    return;
  }

  const double range = std::hypot(x, y);
  if (range < config_.range_min || range > config_.range_max) return;
  const double angle = std::atan2(y, x);
  if (angle < config_.angle_min || angle > config_.angle_max) return;

  int mx = 0;
  int my = 0;
  if (!localToGrid(x, y, mx, my)) return;
  const auto grid_index =
      static_cast<std::size_t>(my * config_.grid_width + mx);
  if (grid[grid_index] == kFree) {
    grid[grid_index] = kOccupied;
    occupied_cells.push_back(GridCell{mx, my});
  }
}

void ObstacleStop::inflateGrid(
    std::vector<uint8_t> &grid,
    const std::vector<GridCell> &occupied_cells) const {
  // 只遍历本帧实际障碍格及预计算偏移，不再扫描整张栅格。
  for (const auto &cell : occupied_cells) {
    for (const auto &offset : inflation_offsets_) {
      const int nx = cell.x + offset.x;
      const int ny = cell.y + offset.y;
      if (nx >= 0 && nx < config_.grid_width && ny >= 0 &&
          ny < config_.grid_height) {
        grid[static_cast<std::size_t>(ny * config_.grid_width + nx)] =
            kOccupied;
      }
    }
  }
}

std::vector<std::size_t> ObstacleStop::buildRectangleCellIndices(
    double x_min, double x_max, double y_min, double y_max) const {
  std::vector<std::size_t> cells;
  for (int my = 0; my < config_.grid_height; ++my) {
    const double y =
        (static_cast<double>(my) - config_.grid_height * 0.5 + 0.5) *
        config_.grid_resolution;
    if (y < y_min || y > y_max) continue;
    for (int mx = 0; mx < config_.grid_width; ++mx) {
      const double x =
          (static_cast<double>(mx) - config_.grid_width * 0.5 + 0.5) *
          config_.grid_resolution;
      if (x < x_min || x > x_max) continue;
      cells.push_back(static_cast<std::size_t>(my * config_.grid_width + mx));
    }
  }
  return cells;
}

ObstacleStop::DirectionalZoneCells ObstacleStop::buildDirectionalZoneCells(
    double x_min, double x_max) const {
  const double half_width = config_.robot_width * 0.5 + config_.safety_margin;
  const double center_half_width = std::min(
      half_width, std::max(0.0, config_.directional_center_half_width));
  DirectionalZoneCells zones;
  zones.left =
      buildRectangleCellIndices(x_min, x_max, center_half_width, half_width);
  zones.center = buildRectangleCellIndices(x_min, x_max, -center_half_width,
                                           center_half_width);
  zones.right =
      buildRectangleCellIndices(x_min, x_max, -half_width, -center_half_width);
  return zones;
}

DirectionalZoneOccupancy ObstacleStop::measureDirectionalZone(
    const std::vector<uint8_t> &grid,
    const DirectionalZoneCells &zone_cells) const {
  DirectionalZoneOccupancy occupancy;
  occupancy.left = obstacleInCells(grid, zone_cells.left);
  occupancy.center = obstacleInCells(grid, zone_cells.center);
  occupancy.right = obstacleInCells(grid, zone_cells.right);
  return occupancy;
}

bool ObstacleStop::obstacleInCells(
    const std::vector<uint8_t> &grid,
    const std::vector<std::size_t> &cells) const {
  if (cells.empty()) return false;

  const auto occupied = static_cast<std::size_t>(std::count_if(
      cells.begin(), cells.end(),
      [&grid](std::size_t index) { return grid[index] == kOccupied; }));
  const double ratio =
      static_cast<double>(occupied) / static_cast<double>(cells.size());
  return ratio > config_.obstacle_ratio_threshold;
}

bool ObstacleStop::localToGrid(double x, double y, int &mx, int &my) const {
  mx = static_cast<int>(
      std::floor(x / config_.grid_resolution + config_.grid_width * 0.5));
  my = static_cast<int>(
      std::floor(y / config_.grid_resolution + config_.grid_height * 0.5));
  return mx >= 0 && mx < config_.grid_width && my >= 0 &&
         my < config_.grid_height;
}

void ObstacleStop::updateObstacleState(bool raw_hard_stop, bool raw_slowdown,
                                       const rclcpp::Time &now) {
  // 障碍出现立即触发；持续出现时不断向后刷新解除截止时间。
  if (raw_hard_stop) {
    hard_stop_clear_deadline_ =
        now + rclcpp::Duration::from_seconds(config_.hard_stop_clear_delay);
    // 硬停解除后先经过减速状态，避免速度从零直接恢复到规划值。
    slowdown_clear_deadline_ =
        now + rclcpp::Duration::from_seconds(config_.slowdown_clear_delay);
  } else if (raw_slowdown) {
    slowdown_clear_deadline_ =
        now + rclcpp::Duration::from_seconds(config_.slowdown_clear_delay);
  }

  hard_stop_active_ = raw_hard_stop || now < hard_stop_clear_deadline_;
  slowdown_active_ = raw_slowdown || now < slowdown_clear_deadline_;
}

void ObstacleStop::controlCallback() {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  geometry_msgs::msg::Twist planned;
  bool has_lidar = false;
  bool lidar_valid = false;
  bool has_command = false;
  bool hard_stop = false;
  bool slowdown = false;
  bool emergency_stop = false;
  bool obstacle_stop_enabled = true;
  bool directional_escape_release = false;
  rclcpp::Time lidar_time(0, 0, node_->get_clock()->get_clock_type());
  rclcpp::Time command_time(0, 0, node_->get_clock()->get_clock_type());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planned = planned_cmd_;
    has_lidar = has_lidar_;
    lidar_valid = lidar_valid_;
    has_command = has_command_;
    hard_stop = hard_stop_active_;
    slowdown = slowdown_active_;
    emergency_stop = emergency_stop_active_;
    obstacle_stop_enabled = obstacle_stop_enabled_;
    directional_escape_release = directional_escape_release_active_;
    lidar_time = last_lidar_time_;
    command_time = last_command_time_;
  }
  const auto now = node_->now();
  const double lidar_age = has_lidar ? (now - lidar_time).seconds()
                                     : std::numeric_limits<double>::infinity();
  const double command_age = has_command
                                 ? (now - command_time).seconds()
                                 : std::numeric_limits<double>::infinity();
  const bool lidar_fresh =
      has_lidar && lidar_age >= 0.0 && lidar_age <= config_.lidar_timeout_sec;
  const bool command_fresh = has_command && command_age >= 0.0 &&
                             command_age <= config_.command_timeout_sec;

  // safe_command默认是零速度，因此所有异常分支都会失效停车。
  geometry_msgs::msg::Twist safe_command;
  State state = State::kClear;
  std::string reason = "clear";
  if (emergency_stop) {
    state = State::kEmergencyStop;
    reason = "external emergency stop";
  } else if (!command_fresh) {
    state = State::kWaitingForCommand;
    reason = has_command ? "planned velocity timed out"
                         : "waiting for planned velocity";
  } else if (!obstacle_stop_enabled) {
    // 停障未启用时仅绕过雷达停车和减速，外部急停与指令超时仍然有效。
    safe_command = planned;
  } else if (!lidar_fresh) {
    state = State::kWaitingForLidar;
    reason = has_lidar ? "MID360 data timed out" : "waiting for MID360 data";
  } else if (!lidar_valid) {
    state = State::kSensorFault;
    reason = "MID360 point cloud is empty";
  } else if (hard_stop && !directional_escape_release) {
    state = State::kObstacleStop;
    reason = "obstacle in hard stop rectangle";
  } else if (hard_stop && directional_escape_release) {
    safe_command = planned;
    reason = "directional escape away from side obstacle";
  } else if (slowdown) {
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

  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[OBSTACLE_CMD] state=%s, enabled=%s, lidar_fresh=%s, "
      "command_fresh=%s, planned=(vx=%.3f, vy=%.3f, wz=%.3f), "
      "output=(vx=%.3f, vy=%.3f, wz=%.3f)",
      stateName(state), obstacle_stop_enabled ? "true" : "false",
      lidar_fresh ? "true" : "false", command_fresh ? "true" : "false",
      planned.linear.x, planned.linear.y, planned.angular.z,
      safe_command.linear.x, safe_command.linear.y, safe_command.angular.z);
  safe_cmd_pub_->publish(safe_command);
  std_msgs::msg::Bool stop_active;
  stop_active.data = state != State::kClear;
  stop_active_pub_->publish(stop_active);
  logStateChange(state, reason);
}

void ObstacleStop::publishDetectionMarkers(bool hard_stop, bool slowdown) {
  if (!marker_pub_) return;
  const auto stamp = node_->now();
  const double front_edge = config_.robot_length * 0.5;
  const double width = config_.robot_width + 2.0 * config_.safety_margin;

  auto make_marker = [&](int id, double center_x, double length, bool active,
                         float red, float green) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = stamp;
    marker.ns = "obstacle_stop";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = center_x;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.01, length);
    marker.scale.y = width;
    marker.scale.z = 0.05;
    marker.color.a = active ? 0.65F : 0.25F;
    marker.color.r = active ? red : 0.0F;
    marker.color.g = active ? green : 1.0F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    return marker;
  };

  marker_pub_->publish(
      make_marker(0, front_edge + config_.hard_stop_distance * 0.5,
                  config_.hard_stop_distance, hard_stop, 1.0F, 0.0F));
  marker_pub_->publish(make_marker(
      1,
      front_edge + config_.hard_stop_distance + config_.slowdown_distance * 0.5,
      config_.slowdown_distance, slowdown, 1.0F, 0.65F));
}

void ObstacleStop::stop() {
  if (control_timer_) control_timer_->cancel();
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
    case State::kEmergencyStop:
      return "emergency_stop";
    case State::kSensorFault:
      return "sensor_fault";
  }
  return "unknown";
}

}  // namespace obstacle_stop
