#pragma once

#include <filesystem>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robot_runtime {

// 从本地读取静态地图，并通过APP标准Topic发布。
class MapPublisher {
 public:
  MapPublisher(rclcpp::Node *node, const std::string &topic,
               const std::string &frame_id);

  // PCD成功读取并提交给ROS Publisher后返回true。
  bool loadAndPublish(const std::filesystem::path &pcd_path,
                      std::string &message);

  // 发布空点云，清除transient_local中保存的旧地图。
  void clear();

 private:
  rclcpp::Node *node_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace robot_runtime
