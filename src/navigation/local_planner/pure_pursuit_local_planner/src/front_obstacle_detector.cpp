#include "pure_pursuit_local_planner/front_obstacle_detector.hpp"

#include <algorithm>
#include <cmath>

#include "pure_pursuit_local_planner/param_utils.hpp"

namespace pure_pursuit_local_planner {

FrontObstacleDetector::FrontObstacleDetector()
    : has_costmap_(false),
      detection_width_(0.3)  // 0.3m 宽度
      ,
      detection_length_(0.5)  // 0.5m 长度
      ,
      sector_radius_(0.8)  // 0.8m 扇形半径
      ,
      sector_angle_(60.0)  // 60度扇形角度（正负30度）
      ,
      obstacle_threshold_(50)  // 代价值阈值
      ,
      linear_velocity_factor_(0.4)  // 线速度乘以0.4
      ,
      angular_velocity_factor_(2.0)  // 角速度乘以2
      ,
      detection_shape_("rectangle")  // 默认使用矩形
      ,
      debug_mode_(false),
      enable_visualization_(true) {}

FrontObstacleDetector::~FrontObstacleDetector() {}

bool FrontObstacleDetector::initialize(
    rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  node_ = node;
  tf_buffer_ = tf_buffer;

  // 加载参数
  detection_width_ = declareOrGetParameter(
      node_, "front_obstacle_detector.detection_width", 0.3);
  detection_length_ = declareOrGetParameter(
      node_, "front_obstacle_detector.detection_length", 0.5);
  sector_radius_ = declareOrGetParameter(
      node_, "front_obstacle_detector.sector_radius", 0.8);
  sector_angle_ = declareOrGetParameter(
      node_, "front_obstacle_detector.sector_angle", 60.0);
  obstacle_threshold_ = declareOrGetParameter(
      node_, "front_obstacle_detector.obstacle_threshold", 50);
  linear_velocity_factor_ = declareOrGetParameter(
      node_, "front_obstacle_detector.linear_velocity_factor", 0.4);
  angular_velocity_factor_ = declareOrGetParameter(
      node_, "front_obstacle_detector.angular_velocity_factor", 2.0);
  detection_shape_ =
      declareOrGetParameter(node_, "front_obstacle_detector.detection_shape",
                            std::string("rectangle"));
  debug_mode_ =
      declareOrGetParameter(node_, "front_obstacle_detector.debug_mode", false);
  enable_visualization_ = declareOrGetParameter(
      node_, "front_obstacle_detector.enable_visualization", true);

  // 初始化发布器
  if (enable_visualization_) {
    detection_area_pub_ =
        node_->create_publisher<visualization_msgs::msg::Marker>(
            "front_detection_area", 1);
  }

  RCLCPP_INFO(node_->get_logger(),
              "Front Obstacle Detector initialized with parameters:");
  if (detection_shape_ == "rectangle") {
    RCLCPP_INFO(node_->get_logger(),
                "  - Detection shape: Rectangle (%.2fm x %.2fm)",
                detection_width_, detection_length_);
  } else if (detection_shape_ == "sector") {
    RCLCPP_INFO(node_->get_logger(),
                "  - Detection shape: Sector (radius: %.2fm, angle: %.1f°)",
                sector_radius_, sector_angle_);
  }
  RCLCPP_INFO(node_->get_logger(), "  - Obstacle threshold: %d",
              obstacle_threshold_);
  RCLCPP_INFO(node_->get_logger(), "  - Linear velocity factor: %.2f",
              linear_velocity_factor_);
  RCLCPP_INFO(node_->get_logger(), "  - Angular velocity factor: %.2f",
              angular_velocity_factor_);
  RCLCPP_INFO(node_->get_logger(), "  - Visualization enabled: %s",
              enable_visualization_ ? "true" : "false");

  return true;
}

void FrontObstacleDetector::updateCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap) {
  costmap_ = costmap;
  has_costmap_ = true;
}

geometry_msgs::msg::Twist FrontObstacleDetector::adjustVelocityForObstacles(
    const geometry_msgs::msg::Twist& cmd_vel,
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  geometry_msgs::msg::Twist adjusted_cmd_vel = cmd_vel;

  // 如果没有代价地图，直接返回原始速度
  if (!has_costmap_) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                           "No costmap available for obstacle detection");
    }
    return adjusted_cmd_vel;
  }

  // 检测前方区域是否有障碍物
  bool obstacle_detected = false;
  if (detection_shape_ == "sector") {
    obstacle_detected = detectObstacleInSectorArea(robot_pose);
  } else  // 默认使用矩形
  {
    obstacle_detected = detectObstacleInFrontArea(robot_pose);
  }

  // 发布可视化
  if (enable_visualization_) {
    publishVisualization(robot_pose, obstacle_detected);
  }

  if (obstacle_detected) {
    // 调整速度：线速度乘以0.4，角速度乘以2
    adjusted_cmd_vel.linear.x *= linear_velocity_factor_;
    adjusted_cmd_vel.linear.y *= linear_velocity_factor_;
    adjusted_cmd_vel.angular.z *= angular_velocity_factor_;

    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 500,
          "Obstacle detected in front area! Adjusting velocity:");
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "  Original: linear_x=%.3f, angular_z=%.3f",
                           cmd_vel.linear.x, cmd_vel.angular.z);
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                           "  Adjusted: linear_x=%.3f, angular_z=%.3f",
                           adjusted_cmd_vel.linear.x,
                           adjusted_cmd_vel.angular.z);
    }
  } else if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "No obstacles detected in front area");
  }

  return adjusted_cmd_vel;
}

bool FrontObstacleDetector::detectObstacleInFrontArea(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  // 生成检测区域的四个角点（机器人前方矩形区域）
  std::vector<std::pair<double, double>> corners;
  generateDetectionAreaCorners(robot_pose, corners);

  // 确定检测区域的边界
  double min_x = std::min(
      {corners[0].first, corners[1].first, corners[2].first, corners[3].first});
  double max_x = std::max(
      {corners[0].first, corners[1].first, corners[2].first, corners[3].first});
  double min_y = std::min({corners[0].second, corners[1].second,
                           corners[2].second, corners[3].second});
  double max_y = std::max({corners[0].second, corners[1].second,
                           corners[2].second, corners[3].second});

  // 转换为地图坐标
  int min_map_x, min_map_y, max_map_x, max_map_y;
  if (!worldToMap(min_x, min_y, min_map_x, min_map_y) ||
      !worldToMap(max_x, max_y, max_map_x, max_map_y)) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Detection area outside of costmap bounds");
    }
    return false;
  }

  // 确保地图坐标在有效范围内
  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

  // 检查区域内的代价值
  int obstacle_cells = 0;
  int total_cells = 0;

  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      int cost = getCostAtMapCoordinate(map_x, map_y);
      if (cost >= 0)  // 有效的代价值
      {
        total_cells++;
        if (cost >= obstacle_threshold_) {
          obstacle_cells++;
        }
      }
    }
  }

  // 如果障碍物占比超过阈值，认为检测到障碍物
  double obstacle_ratio =
      (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells
                        : 0.0;
  bool obstacle_detected = obstacle_ratio > 0.1;  // 10%的区域有障碍物就触发

  if (debug_mode_ && obstacle_detected) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "Obstacle detection: %d/%d cells above threshold (%.1f%%)",
        obstacle_cells, total_cells, obstacle_ratio * 100.0);
  }

  return obstacle_detected;
}

bool FrontObstacleDetector::worldToMap(double world_x, double world_y,
                                       int& map_x, int& map_y) {
  if (!has_costmap_) return false;

  // 转换世界坐标到地图坐标
  map_x = static_cast<int>((world_x - costmap_.info.origin.position.x) /
                           costmap_.info.resolution);
  map_y = static_cast<int>((world_y - costmap_.info.origin.position.y) /
                           costmap_.info.resolution);

  // 检查边界
  return (map_x >= 0 && map_x < static_cast<int>(costmap_.info.width) &&
          map_y >= 0 && map_y < static_cast<int>(costmap_.info.height));
}

int FrontObstacleDetector::getCostAtMapCoordinate(int map_x, int map_y) {
  if (!has_costmap_ || map_x < 0 ||
      map_x >= static_cast<int>(costmap_.info.width) || map_y < 0 ||
      map_y >= static_cast<int>(costmap_.info.height)) {
    return -1;  // 越界
  }

  int index = map_y * costmap_.info.width + map_x;
  if (index >= 0 && index < static_cast<int>(costmap_.data.size())) {
    return costmap_.data[index];
  }

  return -1;
}

void FrontObstacleDetector::generateDetectionAreaCorners(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    std::vector<std::pair<double, double>>& corners) {
  corners.clear();

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 计算机器人前方矩形区域的四个角点
  // 矩形：宽度0.3m，长度0.5m，位于机器人前方

  double half_width = detection_width_ / 2.0;  // 0.15m
  double length = detection_length_;           // 0.5m

  // 机器人坐标系下的四个角点（机器人前方的矩形）
  std::vector<std::pair<double, double>> local_corners = {
      {0.0, -half_width},    // 左后角 (机器人位置)
      {0.0, half_width},     // 右后角 (机器人位置)
      {length, half_width},  // 右前角
      {length, -half_width}  // 左前角
  };

  // 转换到世界坐标系
  for (const auto& local_corner : local_corners) {
    double local_x = local_corner.first;
    double local_y = local_corner.second;

    // 旋转变换
    double world_x =
        robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
    double world_y =
        robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

    corners.push_back({world_x, world_y});
  }

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Detection area corners in world frame:");
    for (size_t i = 0; i < corners.size(); ++i) {
      RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                            "  Corner %zu: (%.3f, %.3f)", i, corners[i].first,
                            corners[i].second);
    }
  }
}

void FrontObstacleDetector::publishVisualization(
    const geometry_msgs::msg::PoseStamped& robot_pose, bool obstacle_detected) {
  if (!enable_visualization_) return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = robot_pose.header.frame_id;
  marker.header.stamp = node_->now();
  marker.ns = "front_obstacle_detection";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  if (detection_shape_ == "sector") {
    // 扇形可视化 - 使用 LINE_STRIP
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.scale.x = 0.02;  // 线条宽度

    // 生成扇形边界点
    std::vector<std::pair<double, double>> sector_points;
    generateSectorAreaPoints(robot_pose, sector_points);

    // 添加边界点到marker
    for (const auto& point : sector_points) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.y = point.second;
      p.z = robot_pose.pose.position.z + 0.1;
      marker.points.push_back(p);
    }

    // 闭合扇形（连接最后一点到第一点）
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points[0]);
    }
  } else {
    // 矩形可视化 - 使用 CUBE
    marker.type = visualization_msgs::msg::Marker::CUBE;

    // 计算检测区域中心点位置（机器人前方 detection_length_/2 距离处）
    double center_offset = detection_length_ / 2.0;
    marker.pose.position.x = robot_x + center_offset * std::cos(robot_yaw);
    marker.pose.position.y = robot_y + center_offset * std::sin(robot_yaw);
    marker.pose.position.z =
        robot_pose.pose.position.z + 0.1;  // 稍微抬高以便可视化

    // 设置朝向（与机器人朝向一致）
    marker.pose.orientation = robot_pose.pose.orientation;

    // 设置尺寸
    marker.scale.x = detection_length_;  // 长度
    marker.scale.y = detection_width_;   // 宽度
    marker.scale.z = 0.05;               // 高度（薄片状）
  }

  // 设置颜色：有障碍物时红色，无障碍物时绿色
  marker.color.a = 0.5;  // 半透明
  if (obstacle_detected) {
    marker.color.r = 1.0;  // 红色
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  } else {
    marker.color.r = 0.0;  // 绿色
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }

  // 设置生存时间
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 200000000;  // 0.2s = 200ms

  detection_area_pub_->publish(marker);

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Published %s detection area visualization: %s",
        detection_shape_.c_str(),
        obstacle_detected ? "RED (obstacle)" : "GREEN (clear)");
  }
}

bool FrontObstacleDetector::detectObstacleInSectorArea(
    const geometry_msgs::msg::PoseStamped& robot_pose) {
  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 计算扇形边界框
  double min_x = robot_x - sector_radius_;
  double max_x = robot_x + sector_radius_;
  double min_y = robot_y - sector_radius_;
  double max_y = robot_y + sector_radius_;

  // 转换为地图坐标
  int min_map_x, min_map_y, max_map_x, max_map_y;
  if (!worldToMap(min_x, min_y, min_map_x, min_map_y) ||
      !worldToMap(max_x, max_y, max_map_x, max_map_y)) {
    if (debug_mode_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Sector detection area outside of costmap bounds");
    }
    return false;
  }

  // 确保地图坐标在有效范围内
  min_map_x = std::max(0, min_map_x);
  min_map_y = std::max(0, min_map_y);
  max_map_x = std::min(static_cast<int>(costmap_.info.width) - 1, max_map_x);
  max_map_y = std::min(static_cast<int>(costmap_.info.height) - 1, max_map_y);

  // 检查扇形区域内的代价值
  int obstacle_cells = 0;
  int total_cells = 0;

  for (int map_x = min_map_x; map_x <= max_map_x; ++map_x) {
    for (int map_y = min_map_y; map_y <= max_map_y; ++map_y) {
      // 转换地图坐标回世界坐标
      double world_x =
          costmap_.info.origin.position.x + map_x * costmap_.info.resolution;
      double world_y =
          costmap_.info.origin.position.y + map_y * costmap_.info.resolution;

      // 检查点是否在扇形内
      if (isPointInSector(world_x, world_y, robot_x, robot_y, robot_yaw)) {
        int cost = getCostAtMapCoordinate(map_x, map_y);
        if (cost >= 0)  // 有效的代价值
        {
          total_cells++;
          if (cost >= obstacle_threshold_) {
            obstacle_cells++;
          }
        }
      }
    }
  }

  // 如果障碍物占比超过阈值，认为检测到障碍物
  double obstacle_ratio =
      (total_cells > 0) ? static_cast<double>(obstacle_cells) / total_cells
                        : 0.0;
  bool obstacle_detected = obstacle_ratio > 0.1;  // 10%的区域有障碍物就触发

  if (debug_mode_ && obstacle_detected) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 500,
        "Sector obstacle detection: %d/%d cells above threshold (%.1f%%)",
        obstacle_cells, total_cells, obstacle_ratio * 100.0);
  }

  return obstacle_detected;
}

void FrontObstacleDetector::generateSectorAreaPoints(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    std::vector<std::pair<double, double>>& sector_points) {
  sector_points.clear();

  double robot_x = robot_pose.pose.position.x;
  double robot_y = robot_pose.pose.position.y;
  double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

  // 扇形参数
  double half_angle = (sector_angle_ * M_PI / 180.0) / 2.0;  // 转换为弧度的半角
  double start_angle = robot_yaw - half_angle;
  double end_angle = robot_yaw + half_angle;

  // 添加扇形中心点（机器人位置）
  sector_points.push_back({robot_x, robot_y});

  // 生成扇形弧线上的点
  int num_arc_points = 20;  // 弧线分段数
  for (int i = 0; i <= num_arc_points; ++i) {
    double angle = start_angle + (end_angle - start_angle) * i / num_arc_points;
    double arc_x = robot_x + sector_radius_ * std::cos(angle);
    double arc_y = robot_y + sector_radius_ * std::sin(angle);
    sector_points.push_back({arc_x, arc_y});
  }

  if (debug_mode_) {
    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "Generated sector with %zu points, radius: %.2fm, angle: %.1f°",
        sector_points.size(), sector_radius_, sector_angle_);
  }
}

bool FrontObstacleDetector::isPointInSector(double point_x, double point_y,
                                            double robot_x, double robot_y,
                                            double robot_yaw) {
  // 计算点到机器人的距离
  double dx = point_x - robot_x;
  double dy = point_y - robot_y;
  double distance = std::sqrt(dx * dx + dy * dy);

  // 检查距离是否在扇形半径内
  if (distance > sector_radius_) return false;

  // 计算点相对于机器人的角度
  double point_angle = std::atan2(dy, dx);
  double angle_diff = point_angle - robot_yaw;

  // 标准化角度差到 [-π, π]
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

  // 检查角度是否在扇形范围内
  double half_angle = (sector_angle_ * M_PI / 180.0) / 2.0;  // 转换为弧度的半角
  return std::abs(angle_diff) <= half_angle;
}

}  // namespace pure_pursuit_local_planner
