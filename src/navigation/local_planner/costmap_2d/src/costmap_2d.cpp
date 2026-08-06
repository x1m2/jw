#include "local_planner/costmap_2d.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
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
  if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_ ||
      range_max_ <= range_min_ || max_height_ < min_height_) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Invalid Livox projection parameters: angle/range/height "
                 "limits must be ordered and angle_increment must be > 0");
    return false;
  }

  livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      livox_topic_, rclcpp::SensorDataQoS(),
      std::bind(&CostMap2D::livoxCallback, this, std::placeholders::_1));

  costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/costmap", rclcpp::QoS(1).reliable());

  publishLidarStaticTransform();
  initializeLookupTables();
  initializeCostMap();

  RCLCPP_INFO(node_->get_logger(), "CostMap2D node initialized successfully");
  RCLCPP_INFO(node_->get_logger(), "Subscribed to Livox CustomMsg: %s",
              livox_topic_.c_str());
  RCLCPP_INFO(node_->get_logger(), "Publishing costmap to: costmap");

  return true;
}

void CostMap2D::loadParameters() {
  node_->declare_parameter<std::string>("global_frame", "map");
  node_->declare_parameter<std::string>("robot_base_frame", "base_link");
  node_->declare_parameter<std::string>("livox_topic", "/livox/lidar");
  node_->declare_parameter<std::string>("lidar_frame", "livox_frame");

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

  std::vector<double> lidar_to_base_T;
  std::vector<double> lidar_to_base_R;
  node_->get_parameter("lidar_to_base_T", lidar_to_base_T);
  node_->get_parameter("lidar_to_base_R", lidar_to_base_R);
  lidar_x_ = lidar_to_base_T[0];
  lidar_y_ = lidar_to_base_T[1];
  lidar_z_ = lidar_to_base_T[2];
  lidar_roll_ = lidar_to_base_R[0];
  lidar_pitch_ = lidar_to_base_R[1];
  lidar_yaw_ = lidar_to_base_R[2];

  node_->get_parameter("min_height", min_height_);
  node_->get_parameter("max_height", max_height_);
  node_->get_parameter("angle_min", angle_min_);
  node_->get_parameter("angle_max", angle_max_);
  node_->get_parameter("angle_increment", angle_increment_);
  node_->get_parameter("range_min", range_min_);
  node_->get_parameter("range_max", range_max_);

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

  const Eigen::AngleAxisd roll_angle(lidar_roll_, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(lidar_pitch_, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(lidar_yaw_, Eigen::Vector3d::UnitZ());
  lidar_to_base_transform_ =
      Eigen::Translation3d(lidar_x_, lidar_y_, lidar_z_) * yaw_angle *
      pitch_angle * roll_angle;
}

void CostMap2D::publishLidarStaticTransform() {
  static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(node_);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = robot_base_frame_;
  transform.child_frame_id = lidar_frame_;
  transform.transform.translation.x = lidar_x_;
  transform.transform.translation.y = lidar_y_;
  transform.transform.translation.z = lidar_z_;

  tf2::Quaternion rotation;
  rotation.setRPY(lidar_roll_, lidar_pitch_, lidar_yaw_);
  transform.transform.rotation = tf2::toMsg(rotation);
  static_tf_broadcaster_->sendTransform(transform);
}

void CostMap2D::initializeCostMap() {
  costmap_.resize(width_ * height_);
  obstacle_timestamps_.resize(width_ * height_);

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
      std::ceil((angle_max_ - angle_min_) / angle_increment_));
  ray_cosines_.resize(ray_count);
  ray_sines_.resize(ray_count);
  for (std::size_t i = 0; i < ray_count; ++i) {
    const double angle = angle_min_ + static_cast<double>(i) * angle_increment_;
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
      const double dist = std::hypot(static_cast<double>(dx),
                                     static_cast<double>(dy)) *
                          resolution_;
      if (dist > inflation_radius_) {
        continue;
      }

      const double normalized_dist = dist / inflation_radius_;
      unsigned char cost = static_cast<unsigned char>(
          LETHAL_OBSTACLE -
          normalized_dist * (LETHAL_OBSTACLE - FREE_SPACE));
      cost = std::max(static_cast<unsigned char>(FREE_SPACE + 1), cost);
      cost =
          std::min(static_cast<unsigned char>(LETHAL_OBSTACLE - 1), cost);
      inflation_offsets_.push_back(InflationOffset{dx, dy, cost});
    }
  }
}

void CostMap2D::livoxCallback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg) {
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

sensor_msgs::msg::LaserScan::SharedPtr CostMap2D::livoxToLaserScan(
    const livox_ros_driver2::msg::CustomMsg& livox_msg) const {
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.stamp = livox_msg.header.stamp;
  scan->header.frame_id = robot_base_frame_;
  scan->angle_min = static_cast<float>(angle_min_);
  scan->angle_max = static_cast<float>(angle_max_);
  scan->angle_increment = static_cast<float>(angle_increment_);
  scan->range_min = static_cast<float>(range_min_);
  scan->range_max = static_cast<float>(range_max_);

  scan->ranges.assign(ray_cosines_.size(),
                      std::numeric_limits<float>::infinity());

  for (const auto& livox_point : livox_msg.points) {
    if (!std::isfinite(livox_point.x) || !std::isfinite(livox_point.y) ||
        !std::isfinite(livox_point.z)) {
      continue;
    }

    const Eigen::Vector3d point_base =
        lidar_to_base_transform_ *
        Eigen::Vector3d(livox_point.x, livox_point.y, livox_point.z);

    if (point_base.z() < min_height_ || point_base.z() > max_height_) {
      continue;
    }

    const double range = std::hypot(point_base.x(), point_base.y());
    if (range < range_min_ || range > range_max_) {
      continue;
    }

    const double angle = std::atan2(point_base.y(), point_base.x());
    if (angle < angle_min_ || angle > angle_max_) {
      continue;
    }

    const auto index =
        static_cast<std::size_t>((angle - angle_min_) / angle_increment_);
    if (index < scan->ranges.size() && range < scan->ranges[index]) {
      scan->ranges[index] = static_cast<float>(range);
    }
  }

  return scan;
}

void CostMap2D::processLaserScan(const sensor_msgs::msg::LaserScan& scan_msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);

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

  if (pitch_exceeded) {
    clearCostMap();
    RCLCPP_DEBUG(node_->get_logger(), "Pitch angle exceeded, costmap cleared");
  } else {
    raytraceFreespace(scan_msg, robot_transform);

    pcl::PointCloud<pcl::PointXYZ> scan_cloud;
    scanToPointCloud(scan_msg, scan_cloud);

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    if (transformPointCloud(scan_cloud, global_cloud, global_frame_,
                            robot_transform)) {
      markObstacles(global_cloud);
      inflateObstacles();

      if (use_time_decay_) {
        decayObstacles();
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Failed to transform scan to global frame");
    }
  }

  publishCostMap();
}

void CostMap2D::scanToPointCloud(const sensor_msgs::msg::LaserScan& scan,
                                 pcl::PointCloud<pcl::PointXYZ>& cloud) {
  cloud.clear();
  cloud.header.frame_id = scan.header.frame_id;
  pcl_conversions::toPCL(scan.header.stamp, cloud.header.stamp);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float range = scan.ranges[i];

    if (range < scan.range_min || range > scan.range_max ||
        range < min_obstacle_range_ || range > max_obstacle_range_) {
      continue;
    }

    pcl::PointXYZ point;
    if (i < ray_cosines_.size()) {
      point.x = range * ray_cosines_[i];
      point.y = range * ray_sines_[i];
    } else {
      const float angle =
          scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
    }
    point.z = 0.0f;

    cloud.points.push_back(point);
  }

  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;
  cloud.is_dense = true;
}

bool CostMap2D::transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud_in,
    pcl::PointCloud<pcl::PointXYZ>& cloud_out,
    const std::string& target_frame,
    const geometry_msgs::msg::TransformStamped& transform) {
  try {
    cloud_out.clear();
    cloud_out.header.frame_id = target_frame;
    cloud_out.header.stamp = cloud_in.header.stamp;

    for (const auto& point : cloud_in.points) {
      geometry_msgs::msg::PointStamped point_in;
      geometry_msgs::msg::PointStamped point_out;

      point_in.header.frame_id = cloud_in.header.frame_id;
      point_in.header.stamp = transform.header.stamp;
      point_in.point.x = point.x;
      point_in.point.y = point.y;
      point_in.point.z = point.z;

      tf2::doTransform(point_in, point_out, transform);

      pcl::PointXYZ transformed_point;
      transformed_point.x = static_cast<float>(point_out.point.x);
      transformed_point.y = static_cast<float>(point_out.point.y);
      transformed_point.z = static_cast<float>(point_out.point.z);

      cloud_out.points.push_back(transformed_point);
    }

    cloud_out.width = static_cast<std::uint32_t>(cloud_out.points.size());
    cloud_out.height = 1;
    cloud_out.is_dense = true;

    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "Transform failed: %s", ex.what());
    return false;
  }
}

void CostMap2D::clearCostMap() {
  std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);
}

void CostMap2D::markObstacles(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  rclcpp::Time current_time = node_->now();

  for (const auto& point : cloud.points) {
    int mx;
    int my;

    if (worldToMap(point.x, point.y, mx, my)) {
      int index = my * width_ + mx;
      setCost(mx, my, LETHAL_OBSTACLE);

      if (index >= 0 && index < static_cast<int>(obstacle_timestamps_.size())) {
        obstacle_timestamps_[index] = current_time;
      }
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
      if (costmap_[index] != LETHAL_OBSTACLE &&
          offset.cost > costmap_[index]) {
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

    origin_x_ = new_origin_x;
    origin_y_ = new_origin_y;

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
  mx = static_cast<int>((wx - origin_x_) / resolution_);
  my = static_cast<int>((wy - origin_y_) / resolution_);

  return mx >= 0 && mx < width_ && my >= 0 && my < height_;
}

void CostMap2D::setCost(int mx, int my, unsigned char cost) {
  if (mx >= 0 && mx < width_ && my >= 0 && my < height_) {
    costmap_[my * width_ + mx] = cost;
  }
}

unsigned char CostMap2D::getCost(int mx, int my) {
  if (mx >= 0 && mx < width_ && my >= 0 && my < height_) {
    return costmap_[my * width_ + mx];
  }

  return NO_INFORMATION;
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
                                  const geometry_msgs::msg::TransformStamped&
                                      robot_transform) {
  const rclcpp::Time frame_time = node_->now();
  int robot_mx;
  int robot_my;

  if (!worldToMap(robot_transform.transform.translation.x,
                  robot_transform.transform.translation.y, robot_mx,
                  robot_my)) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "Robot position outside costmap bounds for raytracing");
    return;
  }

  geometry_msgs::msg::PointStamped sensor_origin_local;
  geometry_msgs::msg::PointStamped sensor_origin_global;

  sensor_origin_local.header.frame_id = scan.header.frame_id;
  sensor_origin_local.header.stamp = robot_transform.header.stamp;
  sensor_origin_local.point.x = 0.0;
  sensor_origin_local.point.y = 0.0;
  sensor_origin_local.point.z = 0.0;

  tf2::doTransform(sensor_origin_local, sensor_origin_global,
                   robot_transform);

  int sensor_mx;
  int sensor_my;

  if (!worldToMap(sensor_origin_global.point.x, sensor_origin_global.point.y,
                  sensor_mx, sensor_my)) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Sensor position outside costmap bounds");
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

    geometry_msgs::msg::PointStamped ray_end_local;
    geometry_msgs::msg::PointStamped ray_end_global;

    ray_end_local.header.frame_id = scan.header.frame_id;
    ray_end_local.header.stamp = robot_transform.header.stamp;
    if (i < ray_cosines_.size()) {
      ray_end_local.point.x = effective_range * ray_cosines_[i];
      ray_end_local.point.y = effective_range * ray_sines_[i];
    } else {
      const float angle =
          scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      ray_end_local.point.x = effective_range * std::cos(angle);
      ray_end_local.point.y = effective_range * std::sin(angle);
    }
    ray_end_local.point.z = 0.0;

    tf2::doTransform(ray_end_local, ray_end_global, robot_transform);

    int end_mx;
    int end_my;

    if (worldToMap(ray_end_global.point.x, ray_end_global.point.y, end_mx,
                   end_my)) {
      bool obstacle_hit = valid_range && range <= max_obstacle_range_ &&
                          range >= min_obstacle_range_;

      clearRayPath(sensor_mx, sensor_my, end_mx, end_my, obstacle_hit,
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
    if (costmap_[i] == NO_INFORMATION) {
      costmap_msg_.data[i] = -1;
    } else if (costmap_[i] == LETHAL_OBSTACLE) {
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
  std::vector<unsigned char> new_costmap(width_ * height_, FREE_SPACE);
  std::vector<rclcpp::Time> new_timestamps(width_ * height_, node_->now());

  for (int old_y = 0; old_y < height_; ++old_y) {
    for (int old_x = 0; old_x < width_; ++old_x) {
      int new_x = old_x - offset_x;
      int new_y = old_y - offset_y;

      if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_) {
        int old_index = old_y * width_ + old_x;
        int new_index = new_y * width_ + new_x;

        new_costmap[new_index] = costmap_[old_index];
        new_timestamps[new_index] = obstacle_timestamps_[old_index];
      }
    }
  }

  costmap_ = std::move(new_costmap);
  obstacle_timestamps_ = std::move(new_timestamps);
}

}  // namespace local_planner
