#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/quaternion.hpp>
#include <locale>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/**
 * @brief 目标点结构体
 */
struct TargetPoint {
  int id;
  double x, y, z;
  double qx, qy, qz, qw;
  std::string label;
};

/**
 * @brief 导航目标点可视化节点类
 */
class TargetPointsVisualizer : public rclcpp::Node {
 public:
  /**
   * @brief 构造函数
   */
  TargetPointsVisualizer() : Node("target_points_visualizer_node") {
    // 获取参数
    this->declare_parameter<std::string>(
        "target_points_file", "/topology_map/generated_map_target_points.txt");

    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("arrow_scale", 1.0);
    this->declare_parameter<double>("text_scale", 0.5);
    this->declare_parameter<double>("text_height_offset", 1.0);
    this->declare_parameter<double>("publish_rate", 1.0);

    this->get_parameter("target_points_file", target_points_file_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("arrow_scale", arrow_scale_);
    this->get_parameter("text_scale", text_scale_);
    this->get_parameter("text_height_offset", text_height_offset_);
    this->get_parameter("publish_rate", publish_rate_);
    // 创建发布器，使用 transient_local 模拟 ROS1 latched publisher
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/target_points_markers", rclcpp::QoS(10));

    RCLCPP_INFO(this->get_logger(), "目标点可视化节点启动");
    RCLCPP_INFO(this->get_logger(), "目标点文件: %s",
                target_points_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "参考坐标系: %s", frame_id_.c_str());
  }

  /**
   * @brief 初始化函数
   */
  bool initialize() {
    // 加载目标点文件
    if (!loadTargetPoints()) {
      RCLCPP_ERROR(this->get_logger(), "加载目标点文件失败");
      return false;
    }

    // 创建可视化标记
    createVisualizationMarkers();

    RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个目标点",
                target_points_.size());
    return true;
  }

  /**
   * @brief 启动周期发布
   */
  void start() {
    double period = 1.0;
    if (publish_rate_ > 1e-6) {
      period = 1.0 / publish_rate_;
    }

    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period)),
        std::bind(&TargetPointsVisualizer::publishMarkers, this));
  }

 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string target_points_file_;
  std::string frame_id_;
  double arrow_scale_;
  double text_scale_;
  double text_height_offset_;
  double publish_rate_;

  std::vector<TargetPoint> target_points_;
  visualization_msgs::msg::MarkerArray marker_array_;

  /**
   * @brief 加载目标点文件
   */
  bool loadTargetPoints() {
    std::ifstream file(target_points_file_);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开目标点文件: %s",
                   target_points_file_.c_str());
      return false;
    }

    std::string line;
    int line_number = 0;

    while (std::getline(file, line)) {
      line_number++;

      // 跳过注释行和空行
      if (line.empty() || line[0] == '#') {
        continue;
      }

      // 解析目标点数据
      TargetPoint point;
      if (parseTargetPoint(line, point)) {
        target_points_.push_back(point);
        RCLCPP_INFO(this->get_logger(),
                    "加载目标点 ID=%d, 位置=(%.3f, %.3f, %.3f), 标签=%s",
                    point.id, point.x, point.y, point.z, point.label.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "第 %d 行格式错误: %s", line_number,
                    line.c_str());
      }
    }

    file.close();
    return !target_points_.empty();
  }

  /**
   * @brief 解析目标点数据
   */
  bool parseTargetPoint(const std::string& line, TargetPoint& point) {
    std::istringstream iss(line);
    std::string label_part;

    // 读取前8个数值字段
    if (!(iss >> point.id >> point.x >> point.y >> point.z >> point.qx >>
          point.qy >> point.qz >> point.qw)) {
      return false;
    }

    // 读取标签（可能包含空格）
    if (std::getline(iss, label_part)) {
      size_t start = label_part.find_first_not_of(" \t");
      if (start != std::string::npos) {
        point.label = label_part.substr(start);
      } else {
        point.label = "目标点" + std::to_string(point.id);
      }
    } else {
      point.label = "目标点" + std::to_string(point.id);
    }

    return true;
  }

  /**
   * @brief 创建可视化标记
   */
  void createVisualizationMarkers() {
    marker_array_.markers.clear();

    for (size_t i = 0; i < target_points_.size(); ++i) {
      const TargetPoint& point = target_points_[i];

      // 创建箭头标记
      visualization_msgs::msg::Marker arrow_marker;
      createArrowMarker(point, static_cast<int>(i * 2), arrow_marker);
      marker_array_.markers.push_back(arrow_marker);

      // 创建文本标记
      visualization_msgs::msg::Marker text_marker;
      createTextMarker(point, static_cast<int>(i * 2 + 1), text_marker);
      marker_array_.markers.push_back(text_marker);
    }
  }

  /**
   * @brief 创建箭头标记
   */
  void createArrowMarker(const TargetPoint& point, int marker_id,
                         visualization_msgs::msg::Marker& marker) {
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "target_arrows";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置位置
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;

    // 设置姿态
    marker.pose.orientation.x = point.qx;
    marker.pose.orientation.y = point.qy;
    marker.pose.orientation.z = point.qz;
    marker.pose.orientation.w = point.qw;

    // 设置尺寸
    marker.scale.x = arrow_scale_;
    marker.scale.y = arrow_scale_ * 0.1;
    marker.scale.z = arrow_scale_ * 0.1;

    // 设置颜色 - 根据ID使用不同颜色
    setMarkerColor(point.id, marker);

    // 设置生命周期，0表示永久显示
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  }

  /**
   * @brief 创建文本标记
   */
  void createTextMarker(const TargetPoint& point, int marker_id,
                        visualization_msgs::msg::Marker& marker) {
    marker.header.frame_id = frame_id_;
    marker.header.stamp = this->now();
    marker.ns = "target_labels";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // 设置位置（在箭头上方）
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z + text_height_offset_;

    // 文本标记不需要旋转
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // 设置文本内容
    marker.text = point.label + " (" + std::to_string(point.id) + ")";

    // 设置尺寸
    marker.scale.z = text_scale_;

    // 设置颜色 - 白色文本
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // 设置生命周期，0表示永久显示
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  }

  /**
   * @brief 根据ID设置标记颜色
   */
  void setMarkerColor(int id, visualization_msgs::msg::Marker& marker) {
    // 使用HSV颜色空间创建不同颜色
    double hue = (id * 60) % 360;
    double saturation = 1.0;
    double value = 1.0;

    // HSV到RGB转换
    double c = value * saturation;
    double x = c * (1 - std::abs(std::fmod(hue / 60.0, 2) - 1));
    double m = value - c;

    double r;
    double g;
    double b;
    if (hue >= 0 && hue < 60) {
      r = c;
      g = x;
      b = 0;
    } else if (hue >= 60 && hue < 120) {
      r = x;
      g = c;
      b = 0;
    } else if (hue >= 120 && hue < 180) {
      r = 0;
      g = c;
      b = x;
    } else if (hue >= 180 && hue < 240) {
      r = 0;
      g = x;
      b = c;
    } else if (hue >= 240 && hue < 300) {
      r = x;
      g = 0;
      b = c;
    } else {
      r = c;
      g = 0;
      b = x;
    }

    marker.color.r = r + m;
    marker.color.g = g + m;
    marker.color.b = b + m;
    marker.color.a = 1.0;
  }

  /**
   * @brief 发布标记
   */
  void publishMarkers() {
    if (!marker_array_.markers.empty()) {
      // 更新时间戳
      for (auto& marker : marker_array_.markers) {
        marker.header.stamp = this->now();
      }

      marker_pub_->publish(marker_array_);
    } else {
      RCLCPP_WARN(this->get_logger(), "没有可发布的目标点标记");
    }
  }
};

/**
 * @brief 主函数
 */
int main(int argc, char** argv) {
  // 设置中文编码支持
  setlocale(LC_ALL, "");

  rclcpp::init(argc, argv);

  try {
    auto visualizer = std::make_shared<TargetPointsVisualizer>();

    if (!visualizer->initialize()) {
      RCLCPP_ERROR(visualizer->get_logger(), "目标点可视化节点初始化失败");
      rclcpp::shutdown();
      return -1;
    }

    visualizer->start();
    RCLCPP_INFO(visualizer->get_logger(), "目标点可视化节点开始运行");

    rclcpp::spin(visualizer);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("target_points_visualizer_node"),
                 "目标点可视化节点运行异常: %s", e.what());
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("target_points_visualizer_node"),
              "目标点可视化节点正常退出");
  rclcpp::shutdown();
  return 0;
}
