#include "obstacle_stop/obstacle_stop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace obstacle_stop {

void ObstacleStop::initializeVisualization() {
  front_base_cloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/obstacle_stop/front_cloud_base", rclcpp::SensorDataQoS());
  if (rear_lidar_enabled_) {
    rear_base_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/obstacle_stop/rear_cloud_base", rclcpp::SensorDataQoS());
  }
  if (front_lidar_enabled_) {
    unfiltered_base_cloud_pubs_[0] = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/obstacle_stop/front_cloud_base_unfiltered", rclcpp::SensorDataQoS());
  }
  if (rear_lidar_enabled_) {
    unfiltered_base_cloud_pubs_[1] = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/obstacle_stop/rear_cloud_base_unfiltered", rclcpp::SensorDataQoS());
  }
  if (config_.publish_visualization) {
    // 每次发布完整快照；新RViz订阅者也能立即拿到全部区域。
    marker_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacle_stop/detection_marker",
            rclcpp::QoS(1).reliable().transient_local());
    occupied_columns_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/obstacle_stop/occupied_columns",
            rclcpp::QoS(1).reliable().transient_local());
    occupied_cells_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/obstacle_stop/occupied_cells",
            rclcpp::QoS(5).reliable().durability_volatile());
  }

  if (config_.publish_visualization) {
    visualization_callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    visualization_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ObstacleStop::visualizationCallback, this),
        visualization_callback_group_);
  }
}

void ObstacleStop::publishUnfilteredBaseCloud(
    const std::vector<Eigen::Vector3f> &points,
    const std_msgs::msg::Header &header, bool rear_lidar) {
  const auto &publisher = unfiltered_base_cloud_pubs_[rear_lidar ? 1U : 0U];
  if (!publisher || publisher->get_subscription_count() == 0U) return;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = kBaseFrame;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  for (const auto &point : points) {
    *x = point.x(); *y = point.y(); *z = point.z();
    ++x; ++y; ++z;
  }
  publisher->publish(cloud);
}

void ObstacleStop::publishBaseCloud(
    const std::vector<Eigen::Vector3f> &points,
    const std_msgs::msg::Header &header, bool rear_lidar) {
  const unsigned source = rear_lidar ? 2U : 1U;
  const auto &detection_cells = detection_masks_[rear_lidar ? 1U : 0U];
  const double center_z = (config_.min_height + config_.max_height) * 0.5;
  const double half_z = (config_.max_height - config_.min_height) * 0.75;
  const auto in_display = [&](double x, double y) {
    for (std::size_t i = 0; i < config_.zones.size(); ++i) {
      const auto &zone = config_.zones[i];
      if (zone.enabled && (zone.sources & source) && zone_runtime_[i].display_box.contains(x,y,1.5)) return true;
    }
    return false;
  };

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.header.frame_id = kBaseFrame;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, "b");
  std::size_t count = 0U;
  for (const auto &point : points) {
    const double x = point.x(), y = point.y(), z = point.z();
    if (!point.allFinite() || !in_display(x,y) ||
        std::abs(z - center_z) > half_z) {
      continue;
    }
    int mx = 0, my = 0;
    const bool inside = passesDetectionFilters(x, y, z) &&
        localToGrid(x, y, mx, my) &&
        detection_cells[static_cast<std::size_t>(my * config_.grid_width + mx)] ==
            kOccupied;
    *iter_x = point.x();
    *iter_y = point.y();
    *iter_z = point.z();
    // 亮紫色：参与对应方向检测的点；蓝色：1.5倍包围盒内的周边点。
    *iter_r = inside ? 255U : 80U;
    *iter_g = inside ? 80U : 180U;
    *iter_b = 255U;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
    ++count;
  }
  modifier.resize(count);
  const auto &publisher =
      rear_lidar ? rear_base_cloud_pub_ : front_base_cloud_pub_;
  publisher->publish(cloud);
}

void ObstacleStop::visualizationCallback() {
  VisualizationSnapshot snapshot;
  {
    // Only copy the small display snapshot while locked. Message construction
    // and DDS publication run outside the control lock and callback group.
    std::lock_guard<std::mutex> config_lock(config_mutex_);
    snapshot.config = config_;
    for (std::size_t i = 0; i < zone_runtime_.size(); ++i) {
      const auto &runtime = zone_runtime_[i];
      auto &zone = snapshot.zones[i];
      zone.hard_box = runtime.hard_box;
      zone.hard_count = runtime.hard_occupied.size();
      zone.hard_active = runtime.hard_active;
      if (runtime.slow) {
        zone.slow_box = runtime.slow->box;
        zone.slow_count = runtime.slow->occupied.size();
        zone.slow_active = runtime.slow->active;
      }
    }
    snapshot.cells.reserve(visual_occupied_.size());
    for (const auto index : visual_occupied_) {
      snapshot.cells.emplace_back(index, visual_height_grid_[index]);
    }
  }
  publishDetectionMarkers(snapshot);
  publishOccupiedColumns(snapshot);
  publishOccupiedCells(snapshot);
}

void ObstacleStop::publishDetectionMarkers(const VisualizationSnapshot &snapshot) {
  if (!marker_pub_) return;
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = kBaseFrame;
  clear.header.stamp = node_->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  std::string counts;
  double left_edge = snapshot.config.robot_width * 0.5;
  double min_x = 0.0, max_x = 0.0;
  for (std::size_t i = 0; i < snapshot.config.zones.size(); ++i) {
    if (!snapshot.config.zones[i].enabled) continue;
    const auto &runtime = snapshot.zones[i];
    for (int kind = 0; kind < (runtime.slow_box ? 2 : 1); ++kind) {
      const auto &box = kind == 0 ? runtime.hard_box : *runtime.slow_box;
      const auto count_value = kind == 0 ? runtime.hard_count : runtime.slow_count;
      const bool active = kind == 0 ? runtime.hard_active : runtime.slow_active;
      if ((kind == 0 ? snapshot.config.zones[i].stop_distance : *snapshot.config.zones[i].slowdown_distance) <= 0.0) continue;
      left_edge = std::max(left_edge, box.y_max);
      min_x = std::min(min_x, box.x_min);
      max_x = std::max(max_x, box.x_max);
      if (!counts.empty()) counts += "\n";
      counts += std::string(kZoneNames[i]) + (kind == 0 ? ":Stop=" : ":Slow=") +
          std::to_string(count_value) + "/" +
          std::to_string(snapshot.config.zones[i].obstacle_cell_threshold);
      visualization_msgs::msg::Marker area;
      area.header = clear.header;
      area.ns = std::string("obstacle_stop_") + kZoneNames[i];
      area.id = kind;
      area.action = visualization_msgs::msg::Marker::ADD;
      area.type = visualization_msgs::msg::Marker::CUBE;
      area.pose.orientation.w = 1.0;
      area.pose.position.x = (box.x_min+box.x_max)*0.5;
      area.pose.position.y = (box.y_min+box.y_max)*0.5;
      area.pose.position.z = (snapshot.config.min_height+snapshot.config.max_height)*0.5;
      area.scale.x = box.x_max-box.x_min;
      area.scale.y = box.y_max-box.y_min;
      area.scale.z = snapshot.config.max_height-snapshot.config.min_height;
      area.color.a = active ? 0.18F : 0.10F;
      if (active) {
        area.color.r = 1.0F;
        area.color.g = kind == 0 ? 0.0F : 0.65F;
      } else if (count_value != 0U) {
        area.color.r = 1.0F; area.color.g = 1.0F;
      } else {
        area.color.r = kind == 0 ? 0.0F : 0.20F;
        area.color.g = kind == 0 ? 0.55F : 1.0F;
        area.color.b = kind == 0 ? 0.05F : 0.40F;
      }
      markers.markers.push_back(area);
      visualization_msgs::msg::Marker outline;
      outline.header = area.header;
      outline.ns = area.ns;
      outline.id = kind+2;
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.type = visualization_msgs::msg::Marker::LINE_LIST;
      outline.pose.orientation.w = 1.0;
      outline.scale.x = 0.015;
      outline.color = area.color; outline.color.a = 0.95F;
      std::array<geometry_msgs::msg::Point, 8> corners;
      for (std::size_t corner = 0; corner < corners.size(); ++corner) {
        corners[corner].x = (corner&1U) ? box.x_max : box.x_min;
        corners[corner].y = (corner&2U) ? box.y_max : box.y_min;
        corners[corner].z = (corner&4U) ? snapshot.config.max_height : snapshot.config.min_height;
      }
      for (std::size_t corner = 0; corner < corners.size(); ++corner) {
        for (const std::size_t bit : {1U,2U,4U}) {
          if (!(corner&bit)) {
            outline.points.push_back(corners[corner]);
            outline.points.push_back(corners[corner|bit]);
          }
        }
      }
      markers.markers.push_back(std::move(outline));
    }
  }
  if (!counts.empty()) {
    visualization_msgs::msg::Marker count;
    count.header = clear.header;
    count.ns = "obstacle_stop_counts";
    count.id = 0;
    count.action = visualization_msgs::msg::Marker::ADD;
    count.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    count.pose.orientation.w = 1.0;
    // +Y is screen-left in the default Orbit view. Keep a single text block
    // away from all boxes; compact labels avoid large inter-word spacing.
    count.pose.position.x = (min_x + max_x) * 0.5;
    count.pose.position.y = left_edge + 1.0;
    count.pose.position.z = snapshot.config.max_height + 0.35;
    count.scale.z = 0.07;
    count.color.r = count.color.g = count.color.b = count.color.a = 1.0F;
    count.text = std::move(counts);
    markers.markers.push_back(std::move(count));
  }
  marker_pub_->publish(markers);
}

void ObstacleStop::publishOccupiedColumns(const VisualizationSnapshot &snapshot) {
  if (!occupied_columns_pub_ || occupied_columns_pub_->get_subscription_count() == 0U) return;
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = kBaseFrame;
  clear.header.stamp = node_->now();
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  for (const auto &[index, cell] : snapshot.cells) {
    visualization_msgs::msg::Marker column;
    column.header = clear.header;
    column.ns = "height_cells";
    column.id = static_cast<int>(index);
    column.action = visualization_msgs::msg::Marker::ADD;
    column.type = visualization_msgs::msg::Marker::CUBE;
    column.pose.orientation.w = 1.0;
    const int mx = static_cast<int>(index%snapshot.config.grid_width);
    const int my = static_cast<int>(index/snapshot.config.grid_width);
    column.pose.position.x = (mx-snapshot.config.grid_width*0.5+0.5)*snapshot.config.grid_resolution;
    column.pose.position.y = (my-snapshot.config.grid_height*0.5+0.5)*snapshot.config.grid_resolution;
    column.pose.position.z = (cell.z_min+cell.z_max)*0.5;
    column.scale.x = column.scale.y = snapshot.config.grid_resolution;
    column.scale.z = std::max(0.02, cell.z_max-cell.z_min);
    column.color.r = column.color.g = 1.0F; column.color.a = 0.45F;
    markers.markers.push_back(std::move(column));
  }
  occupied_columns_pub_->publish(markers);
}

void ObstacleStop::publishOccupiedCells(const VisualizationSnapshot &snapshot) {
  if (!occupied_cells_pub_ || occupied_cells_pub_->get_subscription_count() == 0U) return;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = kBaseFrame;
  cloud.header.stamp = node_->now();
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  using Field = sensor_msgs::msg::PointField;
  modifier.setPointCloud2Fields(6,
      "x",1,Field::FLOAT32, "y",1,Field::FLOAT32, "z",1,Field::FLOAT32,
      "z_min",1,Field::FLOAT32, "z_max",1,Field::FLOAT32, "point_count",1,Field::UINT32);
  modifier.resize(snapshot.cells.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z");
  sensor_msgs::PointCloud2Iterator<float> z_min(cloud,"z_min"), z_max(cloud,"z_max");
  sensor_msgs::PointCloud2Iterator<uint32_t> count(cloud,"point_count");
  for (const auto &[index, cell] : snapshot.cells) {
    const int mx = static_cast<int>(index%snapshot.config.grid_width);
    const int my = static_cast<int>(index/snapshot.config.grid_width);
    *x = static_cast<float>((mx-snapshot.config.grid_width*0.5+0.5)*snapshot.config.grid_resolution);
    *y = static_cast<float>((my-snapshot.config.grid_height*0.5+0.5)*snapshot.config.grid_resolution);
    *z = static_cast<float>((cell.z_min+cell.z_max)*0.5);
    *z_min = static_cast<float>(cell.z_min); *z_max = static_cast<float>(cell.z_max);
    *count = cell.point_count;
    ++x; ++y; ++z; ++z_min; ++z_max; ++count;
  }
  occupied_cells_pub_->publish(cloud);
}

}  // namespace obstacle_stop
