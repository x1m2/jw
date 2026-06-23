#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

using std::string;

class PCDPublisher : public rclcpp::Node {
 public:
  PCDPublisher() : Node("pcd_publisher") {
    // 声明 ROS2 参数
    this->declare_parameter<std::string>("pcd_file_path",
                                         "/root/slam_data/map/result.pcd");
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("topic_name", "/pointcloud");
    this->declare_parameter<double>("publish_rate", 1.0);

    // 读取 ROS2 参数
    this->get_parameter("pcd_file_path", pcd_file_path_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("topic_name", topic_name_);
    this->get_parameter("publish_rate", publish_rate_);

    RCLCPP_INFO(this->get_logger(), "PCD file path: %s",
                pcd_file_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "topic_name: %s", topic_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "publish_rate: %.2f", publish_rate_);

    // 创建点云发布器
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_name_, rclcpp::QoS(10));

    // 加载 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, *cloud_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s",
                   pcd_file_path_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded PCD file successfully, points: %zu",
                cloud_->points.size());

    double period = 1.0;
    if (publish_rate_ > 1e-6) {
      period = 1.0 / publish_rate_;
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period)),
        std::bind(&PCDPublisher::publishPointCloud, this));
  }

 private:
  void publishPointCloud() {
    if (cloud_->empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Point cloud is empty, skip publishing");
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_, output);

    output.header.frame_id = frame_id_;
    output.header.stamp = this->now();

    publisher_->publish(output);

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    //                      "Published PointCloud2, points: %zu",
    //                      cloud_->points.size());
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string pcd_file_path_;
  std::string frame_id_;
  std::string topic_name_;
  double publish_rate_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{
      new pcl::PointCloud<pcl::PointXYZ>};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PCDPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}