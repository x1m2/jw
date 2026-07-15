#include "dual_radar_traversable_extractor/dual_radar_traversable_extractor.hpp"

#include <clocale>
#include <exception>

int main(int argc, char ** argv)
{
  setlocale(LC_ALL, "");
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("dual_radar_traversable_extractor_node");

  try {
    auto extractor = std::make_shared<DualRadarTraversableExtractor>(node);
    RCLCPP_INFO(node->get_logger(), "双雷达可行区域提取节点启动");
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "节点运行异常: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(node->get_logger(), "双雷达可行区域提取节点结束");
  rclcpp::shutdown();
  return 0;
}
