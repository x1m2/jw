#include "topology_map_generator/topology_map_generator.hpp"

int main(int argc, char **argv) {
  // 支持中文日志和“1号、2号”等中文标签正常显示。
  setlocale(LC_ALL, "");

  rclcpp::init(argc, argv);

  // 创建手动拓扑打点节点。
  auto node = std::make_shared<TopologyMapGenerator>();

  RCLCPP_INFO(node->get_logger(), "拓扑地图生成器正在运行");

  // 进入 ROS2 回调循环：等待 Path 和手动触发话题。
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
