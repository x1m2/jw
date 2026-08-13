#include "local_planner/costmap_2d.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

namespace local_planner {

CostMap2D::CostMap2D(const rclcpp::Node::SharedPtr& node)
    : node_(node),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_),
      pitch_exceeded_state_(false),
      last_normal_pitch_time_(node_->now()) {
  loadParameters();

  RCLCPP_INFO(node_->get_logger(), "CostMap2D initialized with parameters:");
  RCLCPP_INFO(node_->get_logger(), "  Resolution: %.2f m/pixel", resolution_);
  RCLCPP_INFO(node_->get_logger(), "  Size: %d x %d pixels", width_, height_);
  RCLCPP_INFO(node_->get_logger(), "  Max obstacle range: %.2f m",
              max_obstacle_range_);
  RCLCPP_INFO(node_->get_logger(), "  Inflation radius: %.2f m",
              inflation_radius_);

  if (enable_pitch_detection_) {
    RCLCPP_INFO(node_->get_logger(),
                "  Pitch detection: ENABLED (max angle: %.1f degrees, recovery "
                "time: %.1f seconds)",
                max_pitch_angle_, pitch_recovery_time_);
  } else {
    RCLCPP_INFO(node_->get_logger(), "  Pitch detection: DISABLED");
  }
}

bool CostMap2D::initialize() {
  if (projection_config_.angle_increment <= 0.0 ||
      projection_config_.angle_max <= projection_config_.angle_min ||
      projection_config_.range_max <= projection_config_.range_min ||
      projection_config_.max_height < projection_config_.min_height) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid Livox projection parameters: angle/range/height "
                 "limits must be ordered and angle_increment must be > 0");
    return false;
  }

  if (lidar_type_ == 1) {
    livox_custom_sub_ =
        node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CostMap2D::livoxCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Point cloud input: Livox CustomMsg on %s",
                livox_topic_.c_str());
  } else {
    pointcloud2_sub_ =
        node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            livox_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CostMap2D::pointCloud2Callback, this,
                      std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Point cloud input: PointCloud2 on %s",
                livox_topic_.c_str());
  }

  costmap_pub_ =
      node_->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 1);

  initializeLookupTables();
  initializeCostMap();
  RCLCPP_INFO(node_->get_logger(), "Subscribed to Livox CustomMsg: %s",
              livox_topic_.c_str());

  return true;
}

void CostMap2D::loadParameters() {
  node_->declare_parameter<std::string>("global_frame", "map");
  node_->declare_parameter<std::string>("robot_base_frame", "base_link");
  node_->declare_parameter<std::string>("livox_topic", "/livox/lidar");
  node_->declare_parameter<std::string>("lidar_frame", "livox_frame");
  node_->declare_parameter<int>("lidar_type", 1);

  node_->declare_parameter<std::vector<double>>("lidar_to_base_T",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("lidar_to_base_R",
                                                std::vector<double>());

  node_->declare_parameter<double>("min_height", 0.2);
  node_->declare_parameter<double>("max_height", 1.3);
  node_->declare_parameter<double>("angle_min", -M_PI);
  node_->declare_parameter<double>("angle_max", M_PI);
  node_->declare_parameter<double>("angle_increment", 0.0087);
  node_->declare_parameter<double>("range_min", 0.4);
  node_->declare_parameter<double>("range_max", 8.0);

  node_->declare_parameter<double>("resolution", 0.05);
  node_->declare_parameter<int>("width", 400);
  node_->declare_parameter<int>("height", 400);

  node_->declare_parameter<double>("max_obstacle_range", 10.0);
  node_->declare_parameter<double>("min_obstacle_range", 0.1);
  node_->declare_parameter<double>("inflation_radius", 0.5);

  node_->declare_parameter<double>("obstacle_max_age", 1.0);
  node_->declare_parameter<double>("decay_rate", 0.80);
  node_->declare_parameter<bool>("use_time_decay", false);

  node_->declare_parameter<bool>("enable_pitch_detection", true);
  node_->declare_parameter<double>("max_pitch_angle", 20.0);
  node_->declare_parameter<double>("pitch_recovery_time", 2.0);

  node_->get_parameter("global_frame", global_frame_);
  node_->get_parameter("robot_base_frame", robot_base_frame_);
  node_->get_parameter("livox_topic", livox_topic_);
  node_->get_parameter("lidar_frame", lidar_frame_);
  node_->get_parameter("lidar_type", lidar_type_);

  std::vector<double> lidar_to_base_T;
  std::vector<double> lidar_to_base_R;
  node_->get_parameter("lidar_to_base_T", lidar_to_base_T);
  node_->get_parameter("lidar_to_base_R", lidar_to_base_R);
  node_->get_parameter("min_height", projection_config_.min_height);
  node_->get_parameter("max_height", projection_config_.max_height);
  node_->get_parameter("angle_min", projection_config_.angle_min);
  node_->get_parameter("angle_max", projection_config_.angle_max);
  node_->get_parameter("angle_increment", projection_config_.angle_increment);
  node_->get_parameter("range_min", projection_config_.range_min);
  node_->get_parameter("range_max", projection_config_.range_max);

  node_->get_parameter("resolution", resolution_);
  node_->get_parameter("width", width_);
  node_->get_parameter("height", height_);

  node_->get_parameter("max_obstacle_range", max_obstacle_range_);
  node_->get_parameter("min_obstacle_range", min_obstacle_range_);
  node_->get_parameter("inflation_radius", inflation_radius_);

  node_->get_parameter("obstacle_max_age", obstacle_max_age_);
  node_->get_parameter("decay_rate", decay_rate_);
  node_->get_parameter("use_time_decay", use_time_decay_);

  node_->get_parameter("enable_pitch_detection", enable_pitch_detection_);
  node_->get_parameter("max_pitch_angle", max_pitch_angle_);
  node_->get_parameter("pitch_recovery_time", pitch_recovery_time_);

  const Eigen::AngleAxisd roll_angle(lidar_to_base_R[0],
                                     Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(lidar_to_base_R[1],
                                      Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(lidar_to_base_R[2],
                                    Eigen::Vector3d::UnitZ());
  lidar_to_base_transform_ =
      Eigen::Translation3d(lidar_to_base_T[0], lidar_to_base_T[1],
                           lidar_to_base_T[2]) *
      yaw_angle * pitch_angle * roll_angle;
}

void CostMap2D::initializeCostMap() {
  costmap_.resize(width_ * height_);
  obstacle_timestamps_.resize(width_ * height_);
  costmap_scratch_.resize(width_ * height_);
  timestamp_scratch_.resize(width_ * height_);

  std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);

  rclcpp::Time current_time = node_->now();
  std::fill(obstacle_timestamps_.begin(), obstacle_timestamps_.end(),
            current_time);

  costmap_msg_.header.frame_id = global_frame_;
  costmap_msg_.info.resolution = static_cast<float>(resolution_);
  costmap_msg_.info.width = static_cast<std::uint32_t>(width_);
  costmap_msg_.info.height = static_cast<std::uint32_t>(height_);
  costmap_msg_.data.resize(width_ * height_);

  origin_x_ = -(width_ * resolution_) / 2.0;
  origin_y_ = -(height_ * resolution_) / 2.0;
  origin_z_ = 0.0;

  costmap_msg_.info.origin.position.x = origin_x_;
  costmap_msg_.info.origin.position.y = origin_y_;
  costmap_msg_.info.origin.position.z = origin_z_;
  costmap_msg_.info.origin.orientation.x = 0.0;
  costmap_msg_.info.origin.orientation.y = 0.0;
  costmap_msg_.info.origin.orientation.z = 0.0;
  costmap_msg_.info.origin.orientation.w = 1.0;

  RCLCPP_INFO(node_->get_logger(),
              "Costmap initialized: %dx%d @ %.2fm resolution", width_, height_,
              resolution_);
}

void CostMap2D::initializeLookupTables() {
  const auto ray_count = static_cast<std::size_t>(
      std::ceil((projection_config_.angle_max - projection_config_.angle_min) /
                projection_config_.angle_increment));
  ray_cosines_.resize(ray_count);
  ray_sines_.resize(ray_count);
  for (std::size_t i = 0; i < ray_count; ++i) {
    const double angle =
        projection_config_.angle_min +
        static_cast<double>(i) * projection_config_.angle_increment;
    ray_cosines_[i] = static_cast<float>(std::cos(angle));
    ray_sines_[i] = static_cast<float>(std::sin(angle));
  }

  inflation_offsets_.clear();
  if (inflation_radius_ <= 0.0) {
    return;
  }

  const int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  inflation_offsets_.reserve(static_cast<std::size_t>(
      (2 * inflation_cells + 1) * (2 * inflation_cells + 1)));
  for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      const double dist =
          std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
          resolution_;
      if (dist > inflation_radius_) {
        continue;
      }

      const double normalized_dist = dist / inflation_radius_;
      unsigned char cost = static_cast<unsigned char>(
          LETHAL_OBSTACLE - normalized_dist * (LETHAL_OBSTACLE - FREE_SPACE));
      cost = std::max(static_cast<unsigned char>(FREE_SPACE + 1), cost);
      cost = std::min(static_cast<unsigned char>(LETHAL_OBSTACLE - 1), cost);
      inflation_offsets_.push_back(InflationOffset{dx, dy, cost});
    }
  }
}

void CostMap2D::livoxCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr livox_msg) {
  if (livox_msg->points.empty()) {
    return;
  }

  if (!livox_msg->header.frame_id.empty() &&
      livox_msg->header.frame_id != lidar_frame_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Livox message frame '%s' differs from configured lidar_frame '%s'; "
        "using configured fixed extrinsics",
        livox_msg->header.frame_id.c_str(), lidar_frame_.c_str());
  }

  processLaserScan(*livoxToLaserScan(*livox_msg));
}

void CostMap2D::pointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg) {
  if (pointcloud_msg->data.empty()) {
    return;
  }

  if (!pointcloud_msg->header.frame_id.empty() &&
      pointcloud_msg->header.frame_id != lidar_frame_) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "PointCloud2 frame '%s' differs from configured lidar_frame '%s'; "
        "using configured fixed extrinsics",
        pointcloud_msg->header.frame_id.c_str(), lidar_frame_.c_str());
  }

  processLaserScan(*pointCloud2ToLaserScan(*pointcloud_msg));
}

sensor_msgs::msg::LaserScan::SharedPtr CostMap2D::makeEmptyLaserScan(
    const builtin_interfaces::msg::Time& stamp) const {
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.stamp = stamp;
  scan->header.frame_id = robot_base_frame_;
  scan->angle_min = static_cast<float>(projection_config_.angle_min);
  scan->angle_max = static_cast<float>(projection_config_.angle_max);
  scan->angle_increment =
      static_cast<float>(projection_config_.angle_increment);
  scan->range_min = static_cast<float>(projection_config_.range_min);
  scan->range_max = static_cast<float>(projection_config_.range_max);
  scan->ranges.assign(ray_cosines_.size(),
                      std::numeric_limits<float>::infinity());
  return scan;
}

void CostMap2D::projectLidarPoint(double x, double y, double z,
                                  sensor_msgs::msg::LaserScan& scan) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return;
  }

  const Eigen::Vector3d point_base =
      lidar_to_base_transform_ * Eigen::Vector3d(x, y, z);
  if (point_base.z() < projection_config_.min_height ||
      point_base.z() > projection_config_.max_height) {
    return;
  }

  const double range = std::hypot(point_base.x(), point_base.y());
  if (range < projection_config_.range_min ||
      range > projection_config_.range_max) {
    return;
  }

  const double angle = std::atan2(point_base.y(), point_base.x());
  if (angle < projection_config_.angle_min ||
      angle > projection_config_.angle_max) {
    return;
  }

  const auto index =
      static_cast<std::size_t>((angle - projection_config_.angle_min) /
                               projection_config_.angle_increment);
  if (index < scan.ranges.size() && range < scan.ranges[index]) {
    scan.ranges[index] = static_cast<float>(range);
  }
}

sensor_msgs::msg::LaserScan::SharedPtr CostMap2D::livoxToLaserScan(
    const livox_ros_driver2::msg::CustomMsg& livox_msg) const {
  auto scan = makeEmptyLaserScan(livox_msg.header.stamp);

  for (const auto& livox_point : livox_msg.points) {
    projectLidarPoint(livox_point.x, livox_point.y, livox_point.z, *scan);
  }

  return scan;
}

sensor_msgs::msg::LaserScan::SharedPtr CostMap2D::pointCloud2ToLaserScan(
    const sensor_msgs::msg::PointCloud2& pointcloud_msg) const {
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  pcl::fromROSMsg(pointcloud_msg, pointcloud);

  auto scan = makeEmptyLaserScan(pointcloud_msg.header.stamp);
  for (const auto& point : pointcloud.points) {
    projectLidarPoint(point.x, point.y, point.z, *scan);
  }
  return scan;
}

void CostMap2D::processLaserScan(const sensor_msgs::msg::LaserScan& scan_msg) {
  geometry_msgs::msg::TransformStamped robot_transform;
  try {
    // 同一帧统一使用同一个最新位姿，避免重复查询TF以及各处理阶段位姿不一致。
    robot_transform = tf_buffer_.lookupTransform(
        global_frame_, robot_base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to get robot transform: %s", ex.what());
    return;
  }

  updateMapOrigin(robot_transform);
  const bool pitch_exceeded = isPitchAngleExceeded(robot_transform);
  tf2::Transform map_from_base;
  tf2::fromMsg(robot_transform.transform, map_from_base);

  if (pitch_exceeded) {
    clearCostMap();
    RCLCPP_DEBUG(node_->get_logger(), "Pitch angle exceeded, costmap cleared");
  } else {
    raytraceFreespace(scan_msg, map_from_base);
    markObstacles(scan_msg, map_from_base);
    inflateObstacles();

    if (use_time_decay_) {
      decayObstacles();
    }
  }

  publishCostMap();
}

void CostMap2D::clearCostMap() {
  std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);
}

void CostMap2D::markObstacles(const sensor_msgs::msg::LaserScan& scan,
                              const tf2::Transform& map_from_base) {
  const rclcpp::Time current_time = node_->now();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (range < scan.range_min || range > scan.range_max ||
        range < min_obstacle_range_ || range > max_obstacle_range_) {
      continue;
    }

    double local_x;
    double local_y;
    if (i < ray_cosines_.size()) {
      local_x = range * ray_cosines_[i];
      local_y = range * ray_sines_[i];
    } else {
      const double angle =
          scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      local_x = range * std::cos(angle);
      local_y = range * std::sin(angle);
    }
    const tf2::Vector3 point_map =
        map_from_base * tf2::Vector3(local_x, local_y, 0.0);

    int mx;
    int my;
    if (worldToMap(point_map.x(), point_map.y(), mx, my)) {
      const auto index = static_cast<std::size_t>(my * width_ + mx);
      costmap_[index] = LETHAL_OBSTACLE;
      obstacle_timestamps_[index] = current_time;
    }
  }
}

void CostMap2D::inflateObstacles() {
  if (inflation_offsets_.empty()) {
    return;
  }

  // 先收集致命障碍，后续原地写入膨胀代价，不需要复制整张代价地图。
  std::vector<std::pair<int, int>> lethal_cells;
  for (int my = 0; my < height_; ++my) {
    for (int mx = 0; mx < width_; ++mx) {
      if (costmap_[my * width_ + mx] == LETHAL_OBSTACLE) {
        lethal_cells.emplace_back(mx, my);
      }
    }
  }

  for (const auto& lethal_cell : lethal_cells) {
    for (const auto& offset : inflation_offsets_) {
      const int nx = lethal_cell.first + offset.dx;
      const int ny = lethal_cell.second + offset.dy;
      if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
        continue;
      }

      const auto index = static_cast<std::size_t>(ny * width_ + nx);
      if (costmap_[index] != LETHAL_OBSTACLE && offset.cost > costmap_[index]) {
        costmap_[index] = offset.cost;
      }
    }
  }
}

void CostMap2D::updateMapOrigin(
    const geometry_msgs::msg::TransformStamped& robot_transform) {
  const double robot_x = robot_transform.transform.translation.x;
  const double robot_y = robot_transform.transform.translation.y;
  const double robot_z = robot_transform.transform.translation.z;

  double new_origin_x = robot_x - (width_ * resolution_) / 2.0;
  double new_origin_y = robot_y - (height_ * resolution_) / 2.0;
  double new_origin_z = robot_z;

  double dx = new_origin_x - origin_x_;
  double dy = new_origin_y - origin_y_;

  if (std::abs(dx) >= resolution_ || std::abs(dy) >= resolution_) {
    int offset_x = static_cast<int>(std::round(dx / resolution_));
    int offset_y = static_cast<int>(std::round(dy / resolution_));

    rollMap(offset_x, offset_y);

    // 地图内容只移动了整数个栅格，原点必须移动完全相同的距离，避免
    // 连续坐标的小数部分导致历史地图与OccupancyGrid原点逐渐错位。
    origin_x_ += offset_x * resolution_;
    origin_y_ += offset_y * resolution_;

    RCLCPP_DEBUG(node_->get_logger(),
                 "Map rolled by (%d, %d) pixels to origin (%.2f, %.2f, %.2f)",
                 offset_x, offset_y, origin_x_, origin_y_, origin_z_);
  }

  origin_z_ = new_origin_z;

  costmap_msg_.info.origin.position.x = origin_x_;
  costmap_msg_.info.origin.position.y = origin_y_;
  costmap_msg_.info.origin.position.z = origin_z_;
}

bool CostMap2D::worldToMap(double wx, double wy, int& mx, int& my) {
  // 必须向下取整。直接static_cast<int>会向零截断，使原点左侧/下侧不足
  // 一个栅格的越界点错误落入第0格。
  mx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  my = static_cast<int>(std::floor((wy - origin_y_) / resolution_));

  return mx >= 0 && mx < width_ && my >= 0 && my < height_;
}

bool CostMap2D::isPitchAngleExceeded(
    const geometry_msgs::msg::TransformStamped& robot_transform) {
  if (!enable_pitch_detection_) {
    return false;
  }

  bool current_pitch_exceeded = false;

  try {
    tf2::Quaternion q;
    tf2::fromMsg(robot_transform.transform.rotation, q);

    tf2::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);

    double pitch_degrees = pitch * 180.0 / M_PI;
    current_pitch_exceeded = std::abs(pitch_degrees) > max_pitch_angle_;

    rclcpp::Time current_time = node_->now();

    if (current_pitch_exceeded) {
      if (!pitch_exceeded_state_) {
        pitch_exceeded_state_ = true;

        RCLCPP_WARN(node_->get_logger(),
                    "Robot pitch angle (%.1f deg) exceeds threshold (%.1f "
                    "deg), activating costmap clearing mode",
                    pitch_degrees, max_pitch_angle_);
      } else {
        RCLCPP_DEBUG(node_->get_logger(),
                     "Robot pitch angle (%.1f deg) still exceeds threshold "
                     "(%.1f deg), maintaining clearing mode",
                     pitch_degrees, max_pitch_angle_);
      }
    } else {
      if (pitch_exceeded_state_) {
        if (last_normal_pitch_time_.nanoseconds() == 0) {
          last_normal_pitch_time_ = current_time;

          RCLCPP_INFO(node_->get_logger(),
                      "Robot pitch angle (%.1f deg) returned to normal, "
                      "starting recovery timer (%.1f seconds)",
                      pitch_degrees, pitch_recovery_time_);
        } else {
          double normal_duration =
              (current_time - last_normal_pitch_time_).seconds();

          if (normal_duration >= pitch_recovery_time_) {
            pitch_exceeded_state_ = false;
            last_normal_pitch_time_ =
                rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

            RCLCPP_INFO(node_->get_logger(),
                        "Robot pitch angle stable for %.1f seconds, "
                        "deactivating costmap clearing mode",
                        normal_duration);
          } else {
            RCLCPP_DEBUG(node_->get_logger(),
                         "Robot pitch angle (%.1f deg) normal for %.1f/%.1f "
                         "seconds, waiting for recovery",
                         pitch_degrees, normal_duration, pitch_recovery_time_);
          }
        }
      } else {
        RCLCPP_DEBUG(node_->get_logger(),
                     "Robot pitch angle: %.1f deg (normal)", pitch_degrees);
      }
    }

    if (current_pitch_exceeded && pitch_exceeded_state_ &&
        last_normal_pitch_time_.nanoseconds() != 0) {
      last_normal_pitch_time_ =
          rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

      RCLCPP_DEBUG(node_->get_logger(),
                   "Pitch angle exceeded again during recovery, resetting "
                   "recovery timer");
    }

    return pitch_exceeded_state_;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Failed to get robot pose for pitch detection: %s",
                         ex.what());

    return pitch_exceeded_state_;
  }
}

void CostMap2D::decayObstacles() {
  rclcpp::Time current_time = node_->now();

  for (int i = 0; i < width_ * height_; ++i) {
    if (costmap_[i] > FREE_SPACE && costmap_[i] <= LETHAL_OBSTACLE) {
      double age = (current_time - obstacle_timestamps_[i]).seconds();

      if (age > obstacle_max_age_) {
        costmap_[i] = FREE_SPACE;
      } else if (age > 0.0) {
        double decay_factor = std::exp(-decay_rate_ * age);

        if (costmap_[i] == LETHAL_OBSTACLE) {
          if (decay_factor < 0.8) {
            unsigned char new_cost = static_cast<unsigned char>(
                FREE_SPACE +
                (INSCRIBED_INFLATED_OBSTACLE - FREE_SPACE) * decay_factor);

            costmap_[i] =
                std::max(static_cast<unsigned char>(FREE_SPACE), new_cost);
          }
        } else {
          unsigned char new_cost =
              static_cast<unsigned char>(costmap_[i] * decay_factor);

          costmap_[i] =
              std::max(static_cast<unsigned char>(FREE_SPACE), new_cost);
        }
      }
    }
  }
}

void CostMap2D::raytraceFreespace(const sensor_msgs::msg::LaserScan& scan,
                                  const tf2::Transform& map_from_base) {
  const rclcpp::Time frame_time = node_->now();
  int robot_mx;
  int robot_my;

  const tf2::Vector3& robot_origin = map_from_base.getOrigin();
  if (!worldToMap(robot_origin.x(), robot_origin.y(), robot_mx, robot_my)) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Robot position outside costmap bounds for raytracing");
    return;
  }

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];

    bool valid_range = range >= scan.range_min && range <= scan.range_max;
    bool inf_range = std::isinf(range) || range > scan.range_max;

    if (!valid_range && !inf_range) {
      continue;
    }

    float effective_range;

    if (valid_range) {
      effective_range =
          std::min(range, static_cast<float>(max_obstacle_range_));
    } else {
      effective_range = static_cast<float>(max_obstacle_range_);
    }

    double local_x;
    double local_y;
    if (i < ray_cosines_.size()) {
      local_x = effective_range * ray_cosines_[i];
      local_y = effective_range * ray_sines_[i];
    } else {
      const double angle =
          scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      local_x = effective_range * std::cos(angle);
      local_y = effective_range * std::sin(angle);
    }
    const tf2::Vector3 ray_end_map =
        map_from_base * tf2::Vector3(local_x, local_y, 0.0);

    int end_mx;
    int end_my;

    if (worldToMap(ray_end_map.x(), ray_end_map.y(), end_mx, end_my)) {
      bool obstacle_hit = valid_range && range <= max_obstacle_range_ &&
                          range >= min_obstacle_range_;

      clearRayPath(robot_mx, robot_my, end_mx, end_my, obstacle_hit,
                   frame_time);
    }
  }
}

void CostMap2D::clearRayPath(int x0, int y0, int x1, int y1,
                             bool obstacle_detected,
                             const rclcpp::Time& frame_time) {
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;
  bool first_cell = true;

  while (true) {
    const bool last_cell = x == x1 && y == y1;
    if (!first_cell && !(last_cell && obstacle_detected) && x >= 0 &&
        x < width_ && y >= 0 && y < height_) {
      const auto index = static_cast<std::size_t>(y * width_ + x);
      costmap_[index] = FREE_SPACE;
      obstacle_timestamps_[index] = frame_time;
    }
    first_cell = false;

    if (last_cell) {
      break;
    }

    int e2 = 2 * err;

    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }

    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

void CostMap2D::publishCostMap() {
  costmap_msg_.header.stamp = node_->now();
  costmap_msg_.header.frame_id = global_frame_;

  for (size_t i = 0; i < costmap_.size(); ++i) {
    if (costmap_[i] == LETHAL_OBSTACLE) {
      costmap_msg_.data[i] = 100;
    } else if (costmap_[i] == FREE_SPACE) {
      costmap_msg_.data[i] = 0;
    } else {
      int occupancy_value =
          static_cast<int>((static_cast<double>(costmap_[i]) * 99.0) / 253.0);

      occupancy_value = std::max(1, std::min(99, occupancy_value));

      costmap_msg_.data[i] = static_cast<int8_t>(occupancy_value);
    }
  }

  costmap_pub_->publish(costmap_msg_);
}

void CostMap2D::rollMap(int offset_x, int offset_y) {
  const rclcpp::Time now = node_->now();
  if (std::abs(offset_x) >= width_ || std::abs(offset_y) >= height_) {
    // 新旧地图已经没有重叠区域，直接清空比逐格搬运更明确也更高效。
    std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);
    std::fill(obstacle_timestamps_.begin(), obstacle_timestamps_.end(), now);
    return;
  }

  std::fill(costmap_scratch_.begin(), costmap_scratch_.end(), FREE_SPACE);
  std::fill(timestamp_scratch_.begin(), timestamp_scratch_.end(), now);

  // 只遍历新旧地图的重叠区域，边界外区域已在scratch中初始化为空闲。
  const int old_x_begin = std::max(0, offset_x);
  const int old_x_end = std::min(width_, width_ + offset_x);
  const int old_y_begin = std::max(0, offset_y);
  const int old_y_end = std::min(height_, height_ + offset_y);
  for (int old_y = old_y_begin; old_y < old_y_end; ++old_y) {
    for (int old_x = old_x_begin; old_x < old_x_end; ++old_x) {
      const int new_x = old_x - offset_x;
      const int new_y = old_y - offset_y;
      const int old_index = old_y * width_ + old_x;
      const int new_index = new_y * width_ + new_x;

      costmap_scratch_[new_index] = costmap_[old_index];
      timestamp_scratch_[new_index] = obstacle_timestamps_[old_index];
    }
  }

  costmap_.swap(costmap_scratch_);
  obstacle_timestamps_.swap(timestamp_scratch_);
}

}  // namespace local_planner
