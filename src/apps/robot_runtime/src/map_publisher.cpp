#include "robot_runtime/map_publisher.hpp"

#include <exception>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace robot_runtime {

MapPublisher::MapPublisher(rclcpp::Node *node, const std::string &topic,
                           const std::string &frame_id)
    : node_(node), frame_id_(frame_id) {
  // 静态地图只发布一次；后启动的APP仍可收到最近一张地图。
  publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::QoS(1).reliable().transient_local());
}

bool MapPublisher::loadAndPublish(const std::filesystem::path &pcd_path,
                                  std::string &message) {
  try {
    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(pcd_path.string(), pcl_cloud) != 0) {
      message = "failed to read map PCD: " + pcd_path.string();
      return false;
    }

    const auto point_count =
        static_cast<std::size_t>(pcl_cloud.width) * pcl_cloud.height;
    if (point_count == 0U || pcl_cloud.data.empty()) {
      message = "map PCD is empty: " + pcd_path.string();
      return false;
    }

    sensor_msgs::msg::PointCloud2 cloud_message;
    pcl_conversions::fromPCL(pcl_cloud, cloud_message);
    cloud_message.header.stamp = node_->now();
    cloud_message.header.frame_id = frame_id_;
    publisher_->publish(cloud_message);

    message = "map PCD published, points=" + std::to_string(point_count);
    return true;
  } catch (const std::exception &error) {
    message = "failed to load map PCD: " + std::string(error.what());
    return false;
  }
}

void MapPublisher::clear() {
  sensor_msgs::msg::PointCloud2 empty_map;
  empty_map.header.stamp = node_->now();
  empty_map.header.frame_id = frame_id_;
  publisher_->publish(empty_map);
}

}  // namespace robot_runtime
