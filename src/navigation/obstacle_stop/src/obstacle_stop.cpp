#include "obstacle_stop/obstacle_stop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace obstacle_stop {

namespace {
constexpr uint8_t kFree = 0U;
constexpr uint8_t kOccupied = 1U;
}  // namespace

ObstacleStop::ObstacleStop(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)),
      last_lidar_time_(0, 0, node_->get_clock()->get_clock_type()),
      last_command_time_(0, 0, node_->get_clock()->get_clock_type()),
      hard_stop_clear_deadline_(0, 0, node_->get_clock()->get_clock_type()),
      slowdown_clear_deadline_(0, 0, node_->get_clock()->get_clock_type()) {}

bool ObstacleStop::initialize() {
  loadParameters();
  initializeGridLookup();

  // 输入为MID360原始点云、规划速度和外部急停信号。
  lidar_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lidar_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleStop::lidarCallback, this, std::placeholders::_1));
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

  safe_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      safe_cmd_topic_, 10);
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

  RCLCPP_INFO(
      node_->get_logger(),
      "Obstacle stop initialized: lidar=%s, planned_cmd=%s, safe_cmd=%s",
      lidar_topic_.c_str(), planned_cmd_topic_.c_str(),
      safe_cmd_topic_.c_str());
  return true;
}

void ObstacleStop::loadParameters() {
  lidar_topic_ =
      node_->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  planned_cmd_topic_ = node_->declare_parameter<std::string>(
      "planned_cmd_topic", "/cmd_vel_planned");
  safe_cmd_topic_ =
      node_->declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel");
  base_frame_ =
      node_->declare_parameter<std::string>("base_frame", "base_link");

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
  config_.lidar_timeout_sec = node_->declare_parameter<double>(
      "lidar_timeout_sec", config_.lidar_timeout_sec);
  config_.command_timeout_sec = node_->declare_parameter<double>(
      "command_timeout_sec", config_.command_timeout_sec);
  config_.control_frequency = node_->declare_parameter<double>(
      "control_frequency", config_.control_frequency);
  config_.minimum_filtered_points = node_->declare_parameter<int>(
      "minimum_filtered_points", config_.minimum_filtered_points);
  config_.publish_visualization = node_->declare_parameter<bool>(
      "publish_visualization", config_.publish_visualization);
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
  const double half_width = config_.robot_width * 0.5 + config_.safety_margin;
  const double front_edge = config_.robot_length * 0.5;
  const double hard_stop_end = front_edge + config_.hard_stop_distance;
  hard_stop_cells_ = buildRectangleCellIndices(front_edge, hard_stop_end,
                                               -half_width, half_width);
  slowdown_cells_ = buildRectangleCellIndices(
      hard_stop_end, hard_stop_end + config_.slowdown_distance, -half_width,
      half_width);
}

void ObstacleStop::lidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
  // 点云经过外参和范围过滤后，直接构建并膨胀临时障碍栅格。
  std::vector<uint8_t> grid;
  std::vector<GridCell> occupied_cells;
  const auto grid_cell_count =
      static_cast<std::size_t>(config_.grid_width * config_.grid_height);
  occupied_cells.reserve(std::min(message->points.size(), grid_cell_count));
  const std::size_t filtered_points =
      buildObstacleGrid(*message, grid, occupied_cells);
  inflateGrid(grid, occupied_cells);

  // 近矩形强制停车，紧接其后的远矩形执行减速。
  const bool raw_hard_stop = obstacleInCells(grid, hard_stop_cells_);
  const bool raw_slowdown = obstacleInCells(grid, slowdown_cells_);
  const auto now = node_->now();
  bool hard_stop = false;
  bool slowdown = false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_lidar_ = true;
    lidar_valid_ = filtered_points >=
                   static_cast<std::size_t>(config_.minimum_filtered_points);
    if (lidar_valid_) {
      updateObstacleState(raw_hard_stop, raw_slowdown, now);
    }
    hard_stop = hard_stop_active_;
    slowdown = slowdown_active_;
    last_lidar_time_ = now;
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

std::size_t ObstacleStop::buildObstacleGrid(
    const livox_ros_driver2::msg::CustomMsg &message,
    std::vector<uint8_t> &grid, std::vector<GridCell> &occupied_cells) const {
  grid.assign(
      static_cast<std::size_t>(config_.grid_width * config_.grid_height),
      kFree);
  occupied_cells.clear();

  std::size_t filtered_points = 0U;
  for (const auto &point : message.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    // 先应用雷达外参，再按base_link中的高度、距离和角度过滤。
    const Eigen::Vector3d point_base =
        lidar_to_base_ * Eigen::Vector3d(point.x, point.y, point.z);
    if (point_base.z() < config_.min_height ||
        point_base.z() > config_.max_height) {
      continue;
    }

    const double range = std::hypot(point_base.x(), point_base.y());
    if (range < config_.range_min || range > config_.range_max) continue;
    const double angle = std::atan2(point_base.y(), point_base.x());
    if (angle < config_.angle_min || angle > config_.angle_max) continue;
    ++filtered_points;

    int mx = 0;
    int my = 0;
    if (localToGrid(point_base.x(), point_base.y(), mx, my)) {
      const auto grid_index =
          static_cast<std::size_t>(my * config_.grid_width + mx);
      // 同一栅格可能落入多个点云点，只记录一次障碍格。
      if (grid[grid_index] == kFree) {
        grid[grid_index] = kOccupied;
        occupied_cells.push_back(GridCell{mx, my});
      }
    }
  }
  return filtered_points;
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
  geometry_msgs::msg::Twist planned;
  bool has_lidar = false;
  bool lidar_valid = false;
  bool has_command = false;
  bool hard_stop = false;
  bool slowdown = false;
  bool emergency_stop = false;
  bool obstacle_stop_enabled = true;
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
    reason = "MID360 filtered point count is too low";
  } else if (hard_stop) {
    state = State::kObstacleStop;
    reason = "obstacle in hard stop rectangle";
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
