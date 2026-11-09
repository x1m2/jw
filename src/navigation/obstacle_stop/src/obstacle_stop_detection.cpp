#include "obstacle_stop/obstacle_stop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace obstacle_stop {

bool ObstacleStop::sourceRequired(unsigned source) const {
  return std::any_of(config_.zones.begin(), config_.zones.end(),
      [source](const DetectionZoneConfig &zone) { return zone.enabled && (zone.sources & source); });
}

void ObstacleStop::initializeGridLookup() {
  const auto size = static_cast<std::size_t>(config_.grid_width * config_.grid_height);
  for (auto &mask : detection_masks_) mask.assign(size, kFree);
  zone_runtime_ = {};
  occupancy_dirty_ = true;
  for (std::size_t i = 0; i < config_.zones.size(); ++i) {
    const auto &zone = config_.zones[i];
    if (!zone.enabled) continue;
    auto &runtime = zone_runtime_[i];
    const double ox = i == 0U ? lidar_to_base_.translation().x() :
                      i == 1U ? rear_lidar_to_base_.translation().x() : 0.0;
    const double oy = i == 0U ? lidar_to_base_.translation().y() :
                      i == 1U ? rear_lidar_to_base_.translation().y() :
                      (i == 2U ? 1.0 : -1.0) * config_.robot_width * 0.5;
    const double span = i < 2U ? config_.robot_width : config_.robot_length;
    runtime.hard_box = zoneRectangle(i, ox, oy, 0.0, zone.stop_distance, span);
    if (i < 2U) {
      runtime.slow.emplace();
      runtime.slow->box = zoneRectangle(i, ox, oy, zone.stop_distance,
                                         *zone.slowdown_distance, span);
    }
    runtime.display_box = zoneRectangle(i, ox, oy, 0.0,
                                         zone.stop_distance + zone.slowdown_distance.value_or(0.0), span);
    for (int my = 0; my < config_.grid_height; ++my) {
      const double y = (my - config_.grid_height * 0.5 + 0.5) * config_.grid_resolution;
      for (int mx = 0; mx < config_.grid_width; ++mx) {
        const double x = (mx - config_.grid_width * 0.5 + 0.5) * config_.grid_resolution;
        const auto index = static_cast<std::size_t>(my * config_.grid_width + mx);
        // Shared boundary belongs to the slowdown region; never count it twice.
        const bool slow = runtime.slow && *zone.slowdown_distance > 0.0 && runtime.slow->box.contains(x,y);
        const bool hard = !slow && zone.stop_distance > 0.0 && runtime.hard_box.contains(x,y);
        if (hard) runtime.hard_indices.push_back(index);
        if (slow) runtime.slow->indices.push_back(index);
        if (hard || slow) {
          for (std::size_t source = 0; source < 2U; ++source) {
            if (zone.sources & (1U << source)) detection_masks_[source][index] = kOccupied;
          }
        }
      }
    }
  }
}

void ObstacleStop::lidarCallback(
    const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  std::vector<HeightCell> grid;
  std::vector<Eigen::Vector3f> base_points;
  const bool publish_crop = front_base_cloud_pub_->get_subscription_count() > 0U;
  const bool publish_base = publish_crop ||
      unfiltered_base_cloud_pubs_[0]->get_subscription_count() > 0U;
  if (publish_base) base_points.reserve(message->points.size());
  buildObstacleGrid(*message, grid, publish_base ? &base_points : nullptr);
  if (publish_crop) publishBaseCloud(base_points, message->header, false);
  if (publish_base) publishUnfilteredBaseCloud(base_points, message->header, false);
  // 原始Livox空消息仍视为传感器异常。
  processObstacleGrid(grid, !message->points.empty());
}

void ObstacleStop::pointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
  processPointCloud2(message, false);
}

void ObstacleStop::rearPointCloud2Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
  processPointCloud2(message, true);
}

void ObstacleStop::processPointCloud2(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message,
    bool rear_lidar) {
  std::lock_guard<std::mutex> config_lock(config_mutex_);
  std::vector<HeightCell> grid;
  std::vector<Eigen::Vector3f> base_points;
  const auto &base_pub =
      rear_lidar ? rear_base_cloud_pub_ : front_base_cloud_pub_;
  const bool publish_crop = base_pub && base_pub->get_subscription_count() > 0U;
  const auto &unfiltered_pub = unfiltered_base_cloud_pubs_[rear_lidar ? 1U : 0U];
  const bool publish_base = publish_crop ||
      (unfiltered_pub && unfiltered_pub->get_subscription_count() > 0U);
  if (publish_base) {
    base_points.reserve(static_cast<std::size_t>(message->width) *
                        message->height);
  }

  bool input_valid = true;
  const std::string &expected_frame =
      rear_lidar ? rear_lidar_frame_ : lidar_frame_;
  if (!message->header.frame_id.empty() &&
      message->header.frame_id != expected_frame) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "%s lidar PointCloud2 frame '%s' differs from configured frame '%s'",
        rear_lidar ? "Rear" : "Front", message->header.frame_id.c_str(),
        expected_frame.c_str());
    input_valid = false;
  } else {
    input_valid = buildObstacleGrid(
        *message, grid,
        rear_lidar ? rear_lidar_to_base_ : lidar_to_base_,
        publish_base ? &base_points : nullptr);
  }

  if (publish_base && input_valid) {
    if (publish_crop) publishBaseCloud(base_points, message->header, rear_lidar);
    publishUnfilteredBaseCloud(base_points, message->header, rear_lidar);
  }
  processObstacleGrid(grid, input_valid, rear_lidar);
}

void ObstacleStop::processObstacleGrid(std::vector<HeightCell> &grid,
                                       bool input_valid, bool rear_lidar) {
  if (!input_valid) grid.clear();
  (rear_lidar ? rear_height_grid_ : front_height_grid_) = std::move(grid);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (rear_lidar) {
      rear_has_lidar_ = true;
      rear_lidar_valid_ = input_valid;
      rear_last_lidar_time_ = node_->now();
    } else {
      has_lidar_ = true;
      lidar_valid_ = input_valid;
      last_lidar_time_ = node_->now();
    }
  }
  occupancy_dirty_ = true;
  // Observe every input frame so a short-lived obstacle still starts its
  // release delay. The control timer reuses these occupancy statistics.
  updateZoneState();
}

void ObstacleStop::refreshZoneOccupancy(const std::array<bool, 2> &usable) {
  if (!occupancy_dirty_ && usable == cached_usable_) return;
  const auto size = static_cast<std::size_t>(config_.grid_width * config_.grid_height);
  visual_height_grid_.assign(size, HeightCell{});
  visual_occupied_.clear();
  const unsigned available = (usable[0] ? 1U : 0U) | (usable[1] ? 2U : 0U);
  bool fused_built = false;
  for (std::size_t i = 0; i < config_.zones.size(); ++i) {
    const auto &zone = config_.zones[i];
    auto &runtime = zone_runtime_[i];
    runtime.hard_occupied.clear();
    if (runtime.slow) runtime.slow->occupied.clear();
    if (!zone.enabled) continue;
    const unsigned sources = zone.sources & available;
    if (sources == 0U) continue;
    // Single-source zones read the source grid directly. All dual-source zones
    // share one fusion, including left/right; reuse the buffer between updates.
    const std::vector<HeightCell> *grid = nullptr;
    if (sources == 3U) {
      if (!fused_built) {
        fused_height_grid_.assign(size, HeightCell{});
        for (std::size_t index = 0; index < size; ++index) {
          mergeOccupiedCell(fused_height_grid_[index], front_height_grid_[index], config_.min_points_per_cell);
          mergeOccupiedCell(fused_height_grid_[index], rear_height_grid_[index], config_.min_points_per_cell);
        }
        fused_built = true;
      }
      grid = &fused_height_grid_;
    } else {
      grid = sources == 1U ? &front_height_grid_ : &rear_height_grid_;
    }
    const auto collect = [&](const std::vector<std::size_t> &indices,
                             std::vector<std::size_t> &occupied) {
      for (const auto index : indices) {
        const auto &cell = (*grid)[index];
        if (!cell.occupied(config_.min_points_per_cell)) continue;
        occupied.push_back(index);
        mergeOccupiedCell(visual_height_grid_[index], cell, config_.min_points_per_cell);
      }
    };
    collect(runtime.hard_indices, runtime.hard_occupied);
    if (runtime.slow) collect(runtime.slow->indices, runtime.slow->occupied);
  }
  for (std::size_t index = 0; index < size; ++index) {
    if (visual_height_grid_[index].occupied(config_.min_points_per_cell)) visual_occupied_.push_back(index);
  }
  occupancy_dirty_ = false;
  cached_usable_ = usable;
}

void ObstacleStop::buildObstacleGrid(
    const livox_ros_driver2::msg::CustomMsg &message,
    std::vector<HeightCell> &grid,
    std::vector<Eigen::Vector3f> *base_points) const {
  grid.assign(
      static_cast<std::size_t>(config_.grid_width * config_.grid_height),
      HeightCell{});
  for (const auto &point : message.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    // 原始Livox点先应用机器人参数文件中的雷达外参。
    const Eigen::Vector3d point_base =
        lidar_to_base_ * Eigen::Vector3d(point.x, point.y, point.z);
    if (base_points && point_base.allFinite()) {
      base_points->push_back(point_base.cast<float>());
    }
    addBasePointToGrid(point_base.x(), point_base.y(), point_base.z(), grid);
  }
}

bool ObstacleStop::buildObstacleGrid(
    const sensor_msgs::msg::PointCloud2 &message, std::vector<HeightCell> &grid,
    const Eigen::Isometry3d &lidar_to_base,
    std::vector<Eigen::Vector3f> *base_points) const {
  grid.assign(
      static_cast<std::size_t>(config_.grid_width * config_.grid_height),
      HeightCell{});
  // 原始雷达空帧视为传感器异常。
  if (message.data.empty()) return false;

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(message, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      Eigen::Vector3d point_base(*iter_x, *iter_y, *iter_z);
      point_base = lidar_to_base * point_base;
      if (base_points && point_base.allFinite()) {
        base_points->push_back(point_base.cast<float>());
      }
      addBasePointToGrid(point_base.x(), point_base.y(), point_base.z(), grid);
    }
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid lidar PointCloud2: %s",
                 error.what());
    return false;
  }
  return true;
}

bool ObstacleStop::passesDetectionFilters(double x, double y, double z) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
  if (z < config_.min_height || z > config_.max_height) return false;
  const double range = std::hypot(x, y);
  if (range < config_.range_min || range > config_.range_max) return false;
  const double angle = std::atan2(y, x);
  return angle >= config_.angle_min && angle <= config_.angle_max;
}

void ObstacleStop::addBasePointToGrid(double x, double y, double z,
                                      std::vector<HeightCell> &grid) const {
  if (!passesDetectionFilters(x, y, z)) return;
  int mx = 0;
  int my = 0;
  if (!localToGrid(x, y, mx, my)) return;
  const auto grid_index =
      static_cast<std::size_t>(my * config_.grid_width + mx);
  grid[grid_index].add(z);
}

bool ObstacleStop::localToGrid(double x, double y, int &mx, int &my) const {
  mx = static_cast<int>(
      std::floor(x / config_.grid_resolution + config_.grid_width * 0.5));
  my = static_cast<int>(
      std::floor(y / config_.grid_resolution + config_.grid_height * 0.5));
  return mx >= 0 && mx < config_.grid_width && my >= 0 &&
         my < config_.grid_height;
}

}  // namespace obstacle_stop
