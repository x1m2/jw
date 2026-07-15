#include <filesystem>
#include <iomanip>
#include <stdexcept>

#include "topology_map_generator/topology_map_generator.hpp"

namespace {
// 根据拓扑点文件路径自动生成目标点文件路径。
// 例如 generated_map.txt -> generated_map_target_points.txt。
std::string makeTargetPointsPath(const std::string &output_file_path) {
  const auto last_dot = output_file_path.find_last_of('.');
  if (last_dot == std::string::npos) {
    return output_file_path + "_target_points.txt";
  }
  return output_file_path.substr(0, last_dot) + "_target_points.txt";
}

// 保存文件前确保父目录存在，避免因为目录不存在导致写文件失败。
void createParentDirectoryIfNeeded(const std::string &file_path) {
  const std::filesystem::path path(file_path);
  const auto parent = path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent);
  }
}
}  // namespace

TopologyMapGenerator::TopologyMapGenerator() : Node("topology_map_generator_node") {
  // 只保留手动打点所需的三个参数：
  // output_file：拓扑点输出文件；
  // pose_topic：机器人当前路径/位姿来源；
  // trigger_topic：手动打点触发话题。
  this->declare_parameter<std::string>(
      "output_file", "/root/slam_data/topology_map/generated_map.txt");
  this->declare_parameter<std::string>("pose_topic", "/pgo/pose_stamped");
  this->declare_parameter<std::string>("trigger_topic", "/topology_node_set");

  this->get_parameter("output_file", output_file_path_);
  this->get_parameter("pose_topic", pose_topic_);
  this->get_parameter("trigger_topic", trigger_topic_);

  // 目标点文件不单独配置，直接由 output_file 自动推导出来。
  target_points_file_path_ = makeTargetPointsPath(output_file_path_);

  // 订阅 Path，只缓存最后一个 pose。
  // 这里不做自动打点、不做距离判断、不生成边，只用于获取“当前最新位姿”。
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      std::bind(&TopologyMapGenerator::poseCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "pose_topic: %s", pose_topic_.c_str());

  // 订阅手动打点触发话题。
  // 外部发布 std_msgs/Bool，且 data=true 时，才会记录一个拓扑点。
  topology_node_set_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      trigger_topic_, rclcpp::QoS(10),
      std::bind(&TopologyMapGenerator::topologyNodeSetCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Manual topology map generator started");
  RCLCPP_INFO(this->get_logger(), "pose_topic: %s", pose_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "trigger_topic: %s", trigger_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_file: %s", output_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "target_points_file: %s",
              target_points_file_path_.c_str());
}

TopologyMapGenerator::~TopologyMapGenerator() {
  // 节点退出时再保存一次，防止最后一次数据没有落盘。
  saveFiles();
}

void TopologyMapGenerator::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) {
    return;
  }

  latest_pose_ = *msg;
  has_latest_pose_ = true;
}

void TopologyMapGenerator::topologyNodeSetCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  // 只接受 Bool=true 的消息；false 或空消息直接忽略。
  if (!msg || !msg->data) {
    return;
  }

  // 简单防抖：避免按键、遥控器或脚本短时间连续发布 true，导致重复打点。
  // 这不是自动打点逻辑，只是防止一次操作产生多个重复点。
  const auto now = std::chrono::steady_clock::now();
  const auto interval_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_manual_trigger_time_)
          .count();

  if (interval_ms < kManualTriggerDelayMs) {
    RCLCPP_WARN(this->get_logger(), "Trigger ignored: too fast");
    return;
  }

  // 如果还没收到任何路径位姿，就不能打点。
  if (!has_latest_pose_) {
    RCLCPP_WARN(this->get_logger(),
                "Trigger ignored: no path pose received yet");
    return;
  }

  // 手动打点核心逻辑：收到 true 后，保存当前最新位姿。
  addManualPoint(latest_pose_);
  last_manual_trigger_time_ = now;
}

void TopologyMapGenerator::addManualPoint(
    const geometry_msgs::msg::PoseStamped &pose) {
  // 自动生成中文标签，例如：1号、2号、3号。
  const std::string label = std::to_string(next_node_id_ + 1) + "号";

  // 保存拓扑点信息：id + xyz + label。
  nodes_.emplace_back(next_node_id_, pose.pose.position.x, pose.pose.position.y,
                      pose.pose.position.z, label);

  // 同步保存目标点信息：id + xyz + 四元数姿态 + label。
  target_points_.emplace_back(next_target_id_, pose, label);

  RCLCPP_INFO(this->get_logger(),
              "Add manual point: id=%d label=%s x=%.3f y=%.3f z=%.3f",
              next_node_id_, label.c_str(), pose.pose.position.x,
              pose.pose.position.y, pose.pose.position.z);

  ++next_node_id_;
  ++next_target_id_;

  // 每打一个点立即保存，避免程序异常退出后丢点。
  saveFiles();
}

void TopologyMapGenerator::saveFiles() {
  try {
    saveTopologyMap();
    saveTargetPoints();
    RCLCPP_INFO(this->get_logger(), "Saved %zu manual topology points",
                nodes_.size());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to save topology files: %s",
                 e.what());
  }
}

void TopologyMapGenerator::saveTopologyMap() const {
  createParentDirectoryIfNeeded(output_file_path_);
  std::ofstream file(output_file_path_, std::ios::trunc);

  if (!file.is_open()) {
    throw std::runtime_error("cannot open " + output_file_path_);
  }

  // ============================================================
  // 只保存手动打出来的拓扑点，不生成边。
  //
  // 输出文件格式：
  // # 拓扑地图文件 (UTF-8编码)
  // # 格式: 节点ID X Y Z 标签
  // # Topology Nodes
  // 0 x y z 1号
  // 1 x y z 2号
  // ...
  //
  // 注意：
  // 这里不输出 # Topology Edges。
  // 连边由 topology_connect.py 后处理生成。
  // ============================================================

  file << "# 拓扑地图文件 (UTF-8编码)\n";
  file << "# 格式: 节点ID X Y Z 标签\n";
  file << "# Topology Nodes\n";

  for (const auto &node : nodes_) {
    file << node.id << " " << std::fixed << std::setprecision(6) << node.x
         << " " << node.y << " " << node.z << " " << node.label << '\n';
  }
}

void TopologyMapGenerator::saveTargetPoints() const {
  createParentDirectoryIfNeeded(target_points_file_path_);
  std::ofstream file(target_points_file_path_, std::ios::trunc);

  if (!file.is_open()) {
    throw std::runtime_error("cannot open " + target_points_file_path_);
  }

  // 输出格式：target_id x y z qx qy qz qw label
  // 该文件可用于后续导航目标点读取，保留了姿态四元数。
  for (const auto &target : target_points_) {
    file << target.id << " " << std::fixed << std::setprecision(6) << target.x
         << " " << target.y << " " << target.z << " " << target.qx << " "
         << target.qy << " " << target.qz << " " << target.qw << " "
         << target.label << '\n';
  }
}
