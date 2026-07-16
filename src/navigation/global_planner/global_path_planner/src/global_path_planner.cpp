#include "global_path_planner/global_path_planner.h"

#include <tf2/utils.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>

namespace global_path_planner {

GlobalPathPlanner::GlobalPathPlanner()
    : rclcpp::Node("global_path_planner_node"),
      has_current_pose_(false),
      has_goal_(false),
      has_free_space_cloud_(false) {
  // 声明并读取 ROS2 参数
  this->declare_parameter<std::string>(
      "topology_map_file", "/root/slam_data/topology_map/generated_map.txt");
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("robot_frame", "base_link");
  this->declare_parameter<std::string>("pose_topic",
                                       "/pose_stamped_localization");
  this->declare_parameter<double>("connection_distance_threshold", 5.0);
  this->declare_parameter<int>("interpolation_points", 20);

  // 可行性检查相关参数
  this->declare_parameter<double>("free_space_radius", 1.0);
  this->declare_parameter<double>("edge_proximity_threshold", 0.8);
  this->declare_parameter<double>("line_of_sight_resolution", 0.2);
  this->declare_parameter<bool>("enable_feasibility_check", true);
  this->declare_parameter<bool>("enable_direct_connection", true);
  this->declare_parameter<double>("direct_connection_threshold", 3.0);
  this->declare_parameter<double>("direct_connection_cost_ratio", 0.9);
  // 边合法性检查相关参数
  this->declare_parameter<double>("robot_height", 0.35);
  this->declare_parameter<int>("edge_sample_count", 100);
  this->declare_parameter<double>("safety_sphere_radius", 0.3);
  this->declare_parameter<double>("valid_point_threshold", 0.9);
  this->declare_parameter<std::string>(
      "free_space_pcd_file", "/root/slam_data/map/result_free_space.pcd");
  this->declare_parameter<bool>("enable_temp_connection_feasibility_check",
                                false);

  this->get_parameter("topology_map_file", map_file_path_);
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("pose_topic", pose_topic_);
  this->get_parameter("connection_distance_threshold",
                      connection_distance_threshold_);
  this->get_parameter("interpolation_points", interpolation_points_);
  this->get_parameter("free_space_radius", free_space_radius_);
  this->get_parameter("edge_proximity_threshold", edge_proximity_threshold_);
  this->get_parameter("line_of_sight_resolution", line_of_sight_resolution_);
  this->get_parameter("enable_feasibility_check", enable_feasibility_check_);
  this->get_parameter("enable_direct_connection", enable_direct_connection_);
  this->get_parameter("direct_connection_threshold",
                      direct_connection_threshold_);
  this->get_parameter("direct_connection_cost_ratio",
                      direct_connection_cost_ratio_);
  this->get_parameter("robot_height", robot_height_);
  this->get_parameter("edge_sample_count", edge_sample_count_);
  this->get_parameter("safety_sphere_radius", safety_sphere_radius_);
  this->get_parameter("valid_point_threshold", valid_point_threshold_);
  this->get_parameter("free_space_pcd_file", free_space_pcd_file_);
  this->get_parameter("enable_temp_connection_feasibility_check",
                      enable_temp_connection_feasibility_check_);

  // 初始化 TF2 和 PCL 相关对象
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  free_space_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZ>);

  RCLCPP_INFO(this->get_logger(), "全局路径规划器构造完成");
  RCLCPP_INFO(this->get_logger(), "地图文件: %s", map_file_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "全局坐标系: %s", global_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "机器人坐标系: %s", robot_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "实时位姿话题: %s", pose_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "机器狗高度: %.2f m", robot_height_);
  RCLCPP_INFO(this->get_logger(), "安全球半径: %.2f m", safety_sphere_radius_);
  RCLCPP_INFO(this->get_logger(), "有效点阈值: %.1f%%",
              valid_point_threshold_ * 100);
  RCLCPP_INFO(this->get_logger(), "可行域 PCD 文件: %s",
              free_space_pcd_file_.c_str());
  RCLCPP_INFO(this->get_logger(), "临时连接可行域检查: %s",
              enable_temp_connection_feasibility_check_ ? "启用" : "禁用");
}

GlobalPathPlanner::~GlobalPathPlanner() {
  RCLCPP_INFO(this->get_logger(), "全局路径规划器已清理并关闭");
}

bool GlobalPathPlanner::initialize() {
  RCLCPP_INFO(this->get_logger(), "初始化全局路径规划器...");

  // 创建发布器：transient_local 等价于 ROS1
  // latch，便于后启动的订阅者拿到最近一次路径
  global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/global_path", rclcpp::QoS(1).transient_local().reliable());

  // 注释掉可视化发布器以避免RViz崩溃
  // visualization_pub_ =
  // this->create_publisher<visualization_msgs::msg::MarkerArray>("topology_visualization",
  // 1);

  // 创建订阅器，使用 lambda 避免 Humble 下 std::bind 回调类型匹配问题
  // 实时定位位姿：/pose_stamped_localization，类型
  // geometry_msgs/msg/PoseStamped
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        this->poseCallback(msg);
      });

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", rclcpp::QoS(10),
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        this->goalCallback(msg);
      });

  target_points_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/target_points", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Pose::ConstSharedPtr msg) {
        this->targetPointsCallback(msg);
      });

  if (enable_temp_connection_feasibility_check_) {
    if (!loadFreeSpaceCloudFromPcd()) {
      RCLCPP_WARN(this->get_logger(),
                  "可行域 PCD 未加载，临时连接可行域检查将不可用。");
    }
  } else {
    RCLCPP_INFO(this->get_logger(),
                "临时连接可行域检查未启用，跳过可行域 PCD 加载。");
  }

  // 加载拓扑地图
  if (!loadTopologyMap()) {
    RCLCPP_ERROR(this->get_logger(), "加载拓扑地图失败");
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
              "全局路径规划器初始化完成，加载了 %lu 个节点和 %lu 条边",
              nodes_.size(), edges_.size());
  return true;
}

void GlobalPathPlanner::spin() { rclcpp::spin(shared_from_this()); }

bool GlobalPathPlanner::loadTopologyMap() {
  RCLCPP_INFO(this->get_logger(), "开始加载拓扑地图文件: %s",
              map_file_path_.c_str());

  std::ifstream file(map_file_path_.c_str());
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开拓扑地图文件: %s",
                 map_file_path_.c_str());
    return false;
  }

  std::string line;
  bool reading_edges = false;

  while (std::getline(file, line)) {
    // 移除前后空格
    line.erase(0, line.find_first_not_of(" \t\r\n"));
    line.erase(line.find_last_not_of(" \t\r\n") + 1);

    // 跳过空行和注释
    if (line.empty() || line[0] == '#') {
      if (line.find("Topology Edges") != std::string::npos) {
        reading_edges = true;
        RCLCPP_INFO(this->get_logger(), "开始读取边数据");
      }
      continue;
    }

    std::istringstream iss(line);

    if (!reading_edges) {
      // 读取节点: ID X Y Z 标签
      TopologyNode node;
      if (iss >> node.id >> node.x >> node.y >> node.z) {
        // 读取剩余部分作为标签
        std::string remaining;
        std::getline(iss, remaining);
        if (!remaining.empty() && remaining[0] == ' ') {
          remaining = remaining.substr(1);
        }
        node.label =
            remaining.empty() ? "node_" + std::to_string(node.id) : remaining;

        nodes_.push_back(node);
        node_map_[node.id] = node;

        RCLCPP_DEBUG(this->get_logger(),
                     "读取节点: ID=%d, 位置=(%.2f,%.2f,%.2f)", node.id, node.x,
                     node.y, node.z);
      }
    } else {
      // 读取边: ID 起始ID 结束ID 权重 类型 双向
      TopologyEdge edge;
      std::string type;
      int bidirectional_int;

      if (iss >> edge.id >> edge.from_id >> edge.to_id >> edge.weight >> type >>
          bidirectional_int) {
        edge.bidirectional = (bidirectional_int == 1);
        edges_.push_back(edge);

        RCLCPP_DEBUG(this->get_logger(),
                     "读取边: ID=%d, %d->%d, 权重=%.2f, 双向=%s", edge.id,
                     edge.from_id, edge.to_id, edge.weight,
                     edge.bidirectional ? "是" : "否");
      }
    }
  }

  file.close();

  RCLCPP_INFO(this->get_logger(), "拓扑地图加载完成: %lu 个节点, %lu 条边",
              nodes_.size(), edges_.size());
  return true;
}

void GlobalPathPlanner::poseCallback(
    geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
  // 接收实时定位位姿，并统一转换到 global_frame_（默认 map）下使用。
  // 如果 /pose_stamped_localization 的 header.frame_id 已经是
  // map，这里相当于直接使用该位姿。
  geometry_msgs::msg::PoseStamped pose_out;

  try {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "收到的实时位姿 header.frame_id 为空，无法转换到 %s",
                           global_frame_.c_str());
      return;
    }

    if (msg->header.frame_id == global_frame_) {
      // 已经是全局坐标系，避免不必要的 TF 查询。
      pose_out = *msg;
    } else {
      tf_buffer_->transform(*msg, pose_out, global_frame_,
                            tf2::durationFromSec(0.2));
    }

    current_position_.x = pose_out.pose.position.x;
    current_position_.y = pose_out.pose.position.y;
    current_position_.z = pose_out.pose.position.z;
    has_current_pose_ = true;

    const double yaw = tf2::getYaw(pose_out.pose.orientation);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "SLAM  [%s]: x=%.3f, y=%.3f, z=%.3f, yaw=%.2f deg",
                         pose_out.header.frame_id.c_str(), current_position_.x,
                         current_position_.y, current_position_.z,
                         yaw * 180.0 / M_PI);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "实时位姿坐标变换失败: %s", ex.what());
  }
}

void GlobalPathPlanner::goalCallback(
    geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "收到点击目标点");

  // 转换目标点到全局坐标系
  geometry_msgs::msg::PointStamped goal_in_global;
  try {
    tf_buffer_->transform(*msg, goal_in_global, global_frame_,
                          tf2::durationFromSec(0.2));
    goal_position_.x = goal_in_global.point.x;
    goal_position_.y = goal_in_global.point.y;
    goal_position_.z = goal_in_global.point.z;
    has_goal_ = true;

    RCLCPP_INFO(this->get_logger(), "目标点位置: (%.2f, %.2f, %.2f)",
                goal_position_.x, goal_position_.y, goal_position_.z);

    // 如果有当前位置，立即规划路径
    if (has_current_pose_) {
      nav_msgs::msg::Path global_path = planGlobalPath();
      if (!global_path.poses.empty()) {
        global_path_pub_->publish(global_path);
        RCLCPP_INFO(this->get_logger(), "发布全局路径，包含 %lu 个路径点",
                    global_path.poses.size());
      } else {
        RCLCPP_WARN(this->get_logger(), "无法规划路径");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "尚未接收到当前位置信息");
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "目标点坐标变换失败: %s", ex.what());
  }
}

void GlobalPathPlanner::targetPointsCallback(
    geometry_msgs::msg::Pose::ConstSharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "收到目标点位姿");

  // 直接使用位姿的位置信息
  goal_position_.x = msg->position.x;
  goal_position_.y = msg->position.y;
  goal_position_.z = msg->position.z;
  has_goal_ = true;

  RCLCPP_INFO(this->get_logger(), "目标点位置: (%.2f, %.2f, %.2f)",
              goal_position_.x, goal_position_.y, goal_position_.z);
  RCLCPP_INFO(this->get_logger(),
              "目标点朝向: (x=%.3f, y=%.3f, z=%.3f, w=%.3f)",
              msg->orientation.x, msg->orientation.y, msg->orientation.z,
              msg->orientation.w);

  // 如果有当前位置，立即规划路径
  if (has_current_pose_) {
    nav_msgs::msg::Path global_path = planGlobalPath();
    if (!global_path.poses.empty()) {
      global_path_pub_->publish(global_path);
      RCLCPP_INFO(this->get_logger(), "发布全局路径，包含 %lu 个路径点",
                  global_path.poses.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "无法规划路径");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "尚未接收到当前位置信息");
  }
}

nav_msgs::msg::Path GlobalPathPlanner::planGlobalPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;

  if (!has_current_pose_ || !has_goal_) {
    RCLCPP_WARN(this->get_logger(), "缺少起点或终点信息");
    return path;
  }

  RCLCPP_INFO(this->get_logger(),
              "开始规划从 (%.2f, %.2f, %.2f) 到 (%.2f, %.2f, %.2f) 的路径",
              current_position_.x, current_position_.y, current_position_.z,
              goal_position_.x, goal_position_.y, goal_position_.z);

  try {
    // 创建临时拓扑图，包含起点和终点作为临时节点
    std::vector<TopologyNode> temp_nodes;
    std::vector<TopologyEdge> temp_edges;

    // 预分配内存以避免频繁重新分配
    temp_nodes.reserve(nodes_.size() + 2);
    temp_edges.reserve(edges_.size() + 10);  // 额外空间用于临时连接

    // 复制原始节点和边
    temp_nodes = nodes_;
    temp_edges = edges_;

    // 添加起点作为临时节点
    int start_node_id = temp_nodes.size();
    TopologyNode start_node;
    start_node.id = start_node_id;
    start_node.x = current_position_.x;
    start_node.y = current_position_.y;
    start_node.z = current_position_.z;
    start_node.label = "start_temp";
    temp_nodes.push_back(start_node);

    // 添加终点作为临时节点
    int goal_node_id = temp_nodes.size();
    TopologyNode goal_node;
    goal_node.id = goal_node_id;
    goal_node.x = goal_position_.x;
    goal_node.y = goal_position_.y;
    goal_node.z = goal_position_.z;
    goal_node.label = "goal_temp";
    temp_nodes.push_back(goal_node);

    // 起点是机器人当前实时位置，可以保留原来的“最近节点 +
    // 第二层邻居”逻辑，避免刚好在边中间时折返
    addTemporaryConnections(temp_nodes, temp_edges, start_node_id,
                            current_position_);

    // 终点是真实目标点，不允许第二层邻居把目标点带到错误拓扑点
    addGoalTemporaryConnection(temp_nodes, temp_edges, goal_node_id,
                               goal_position_);

    // 检查起点和终点距离，如果很近且可行，直接连接
    double direct_distance =
        calculateDistance(current_position_, goal_position_);
    RCLCPP_INFO(this->get_logger(), "起点到终点直线距离: %.2f 米",
                direct_distance);

    // 创建临时节点用于边合法性检查
    TopologyNode start_temp_node, goal_temp_node;
    start_temp_node.id = start_node_id;
    start_temp_node.x = current_position_.x;
    start_temp_node.y = current_position_.y;
    start_temp_node.z = current_position_.z;
    start_temp_node.label = "start_temp";

    goal_temp_node.id = goal_node_id;
    goal_temp_node.x = goal_position_.x;
    goal_temp_node.y = goal_position_.y;
    goal_temp_node.z = goal_position_.z;
    goal_temp_node.label = "goal_temp";

    // 检查直接连接的可行性
    bool direct_connection_feasible = true;
    if (enable_temp_connection_feasibility_check_) {
      direct_connection_feasible = isEdgeLegal(start_temp_node, goal_temp_node);
    }

    if (enable_direct_connection_ &&
        direct_distance <= direct_connection_threshold_ &&
        direct_connection_feasible) {
      // 创建起点到终点的直接连接
      TopologyEdge direct_edge;
      direct_edge.id = temp_edges.size();
      direct_edge.from_id = start_node_id;
      direct_edge.to_id = goal_node_id;
      direct_edge.weight =
          direct_distance *
          direct_connection_cost_ratio_;  // 给直接连接一个小的奖励
      direct_edge.bidirectional = true;
      temp_edges.push_back(direct_edge);

      RCLCPP_INFO(this->get_logger(), "添加起点到终点的直接连接，距离: %.2f 米",
                  direct_distance);
    } else {
      if (!enable_direct_connection_) {
        RCLCPP_DEBUG(this->get_logger(), "直接连接功能已禁用");
      } else if (direct_distance > direct_connection_threshold_) {
        RCLCPP_DEBUG(this->get_logger(),
                     "起点到终点距离 %.2f 米超过阈值 %.2f 米", direct_distance,
                     direct_connection_threshold_);
      } else {
        if (enable_temp_connection_feasibility_check_) {
          RCLCPP_DEBUG(this->get_logger(),
                       "起点到终点直接连接不可行（合法性检查失败）");
        } else {
          RCLCPP_DEBUG(this->get_logger(),
                       "起点到终点直接连接被跳过（未启用合法性检查）");
        }
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "临时拓扑图创建完成：%lu 个节点，%lu 条边",
                 temp_nodes.size(), temp_edges.size());

    // 在临时拓扑图中进行Dijkstra搜索
    std::vector<int> topology_path = dijkstraSearchInTempGraph(
        temp_nodes, temp_edges, start_node_id, goal_node_id);
    {
      RCLCPP_WARN(this->get_logger(),
                  "========== TOPOLOGY PATH RESULT ==========");

      for (size_t i = 0; i < topology_path.size(); ++i) {
        int id = topology_path[i];

        auto it =
            std::find_if(temp_nodes.begin(), temp_nodes.end(),
                         [id](const TopologyNode &n) { return n.id == id; });

        if (it != temp_nodes.end()) {
          RCLCPP_WARN(
              this->get_logger(),
              "[TOPO_PATH] order=%zu id=%d label=%s x=%.3f y=%.3f z=%.3f", i,
              it->id, it->label.c_str(), it->x, it->y, it->z);
        }
      }
    }
    if (topology_path.empty()) {
      RCLCPP_WARN(this->get_logger(), "未找到可行路径");
      return path;
    }

    // 构建完整路径
    path = buildCompletePathFromTempGraph(temp_nodes, topology_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "路径规划过程中发生异常: %s", e.what());
    return nav_msgs::msg::Path();  // 返回空路径
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "路径规划过程中发生未知异常");
    return nav_msgs::msg::Path();  // 返回空路径
  }

  return path;
}

void GlobalPathPlanner::addTemporaryConnections(
    std::vector<TopologyNode> &temp_nodes,
    std::vector<TopologyEdge> &temp_edges, int temp_node_id,
    const geometry_msgs::msg::Point &position) {
  // 创建临时节点用于边合法性检查
  TopologyNode temp_node;
  temp_node.id = temp_node_id;
  temp_node.x = position.x;
  temp_node.y = position.y;
  temp_node.z = position.z;
  temp_node.label = "temp";

  RCLCPP_INFO(this->get_logger(), "开始为临时节点 %d 查找连接", temp_node_id);

  // 第一步：找到距离最近的第一个合法节点
  int nearest_legal_node_id = -1;
  double min_distance = std::numeric_limits<double>::max();

  // 按距离排序所有节点
  std::vector<std::pair<double, int>> sorted_nodes;
  for (size_t i = 0; i < nodes_.size(); ++i) {
    geometry_msgs::msg::Point node_pos;
    node_pos.x = nodes_[i].x;
    node_pos.y = nodes_[i].y;
    node_pos.z = nodes_[i].z;

    double dist = calculateDistance(position, node_pos);
    sorted_nodes.push_back(std::make_pair(dist, nodes_[i].id));
  }

  // 按距离排序
  std::sort(sorted_nodes.begin(), sorted_nodes.end());

  // 找到第一个合法节点
  for (const auto &node_pair : sorted_nodes) {
    int node_id = node_pair.second;

    // 找到对应的节点
    auto it = std::find_if(
        nodes_.begin(), nodes_.end(),
        [node_id](const TopologyNode &n) { return n.id == node_id; });

    if (it != nodes_.end()) {
      // 根据参数控制是否进行边合法性检查
      bool is_legal = true;
      if (enable_temp_connection_feasibility_check_) {
        is_legal = isEdgeLegal(temp_node, *it);
      }

      if (is_legal) {
        nearest_legal_node_id = node_id;
        min_distance = node_pair.first;
        RCLCPP_INFO(this->get_logger(), "找到第一个合法节点: %d，距离: %.2f",
                    node_id, min_distance);
        break;
      } else {
        RCLCPP_DEBUG(this->get_logger(), "节点 %d 连接非法，继续查找", node_id);
      }
    }
  }

  if (nearest_legal_node_id == -1) {
    RCLCPP_ERROR(this->get_logger(), "临时节点 %d 无法找到任何合法连接节点",
                 temp_node_id);
    return;
  }

  // 第二步：收集第一层和第二层候选节点
  std::set<int> candidate_nodes;
  candidate_nodes.insert(nearest_legal_node_id);  // 第一层：最近的合法节点

  // 查找第二层：与第一个合法节点相邻的所有节点
  for (const auto &edge : edges_) {
    if (edge.from_id == nearest_legal_node_id) {
      candidate_nodes.insert(edge.to_id);
      RCLCPP_DEBUG(this->get_logger(), "添加第二层候选节点: %d (通过边 %d->%d)",
                   edge.to_id, edge.from_id, edge.to_id);
    } else if (edge.bidirectional && edge.to_id == nearest_legal_node_id) {
      candidate_nodes.insert(edge.from_id);
      RCLCPP_DEBUG(this->get_logger(),
                   "添加第二层候选节点: %d (通过双向边 %d<->%d)", edge.from_id,
                   edge.from_id, edge.to_id);
    }
  }

  RCLCPP_INFO(this->get_logger(), "第一个合法节点: %d, 第二层候选节点总数: %lu",
              nearest_legal_node_id, candidate_nodes.size() - 1);

  // 第三步：对所有候选节点进行合法性检查并创建连接
  int actual_connections = 0;
  for (int candidate_id : candidate_nodes) {
    // 找到候选节点
    auto it = std::find_if(
        nodes_.begin(), nodes_.end(),
        [candidate_id](const TopologyNode &n) { return n.id == candidate_id; });

    if (it != nodes_.end()) {
      // 根据参数控制是否进行边合法性检查
      bool is_legal = true;
      if (enable_temp_connection_feasibility_check_) {
        is_legal = isEdgeLegal(temp_node, *it);
      }

      if (is_legal) {
        // 计算连接代价
        geometry_msgs::msg::Point node_pos;
        node_pos.x = it->x;
        node_pos.y = it->y;
        node_pos.z = it->z;

        double cost = calculateConnectionCost(position, node_pos);

        // 创建双向连接
        TopologyEdge edge;
        edge.id = temp_edges.size();
        edge.from_id = temp_node_id;
        edge.to_id = candidate_id;
        edge.weight = cost;
        edge.bidirectional = true;
        temp_edges.push_back(edge);

        actual_connections++;
        RCLCPP_INFO(this->get_logger(),
                    "创建合法连接: 临时节点%d -> 节点%d, 代价: %.2f",
                    temp_node_id, candidate_id, cost);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "候选节点 %d 连接非法，跳过",
                     candidate_id);
      }
    }
  }

  if (actual_connections == 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "临时节点 %d 无法创建任何合法连接，尝试强制连接到最近合法节点",
                 temp_node_id);

    // 强制连接到第一个合法节点作为备选
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
                           [nearest_legal_node_id](const TopologyNode &n) {
                             return n.id == nearest_legal_node_id;
                           });

    if (it != nodes_.end()) {
      geometry_msgs::msg::Point node_pos;
      node_pos.x = it->x;
      node_pos.y = it->y;
      node_pos.z = it->z;

      double cost = calculateDistance(position, node_pos);

      TopologyEdge edge;
      edge.id = temp_edges.size();
      edge.from_id = temp_node_id;
      edge.to_id = nearest_legal_node_id;
      edge.weight = cost;
      edge.bidirectional = true;
      temp_edges.push_back(edge);

      actual_connections = 1;
      RCLCPP_WARN(this->get_logger(), "强制创建备选连接: 临时节点%d -> 节点%d",
                  temp_node_id, nearest_legal_node_id);
    }
  }

  RCLCPP_INFO(this->get_logger(), "临时节点 %d 成功连接到 %d 个节点",
              temp_node_id, actual_connections);
}

void GlobalPathPlanner::addGoalTemporaryConnection(
    std::vector<TopologyNode> &temp_nodes,
    std::vector<TopologyEdge> &temp_edges, int goal_node_id,
    const geometry_msgs::msg::Point &goal_position) {
  RCLCPP_INFO(this->get_logger(),
              "[GOAL_TEMP_CONNECT] start connect real goal_temp id=%d, "
              "goal=(%.3f, %.3f, %.3f)",
              goal_node_id, goal_position.x, goal_position.y, goal_position.z);

  int nearest_node_id = -1;
  double nearest_dist = std::numeric_limits<double>::max();

  for (const auto &node : nodes_) {
    geometry_msgs::msg::Point node_pos;
    node_pos.x = node.x;
    node_pos.y = node.y;
    node_pos.z = node.z;

    double dist = calculateDistance(goal_position, node_pos);

    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_node_id = node.id;
    }
  }

  if (nearest_node_id < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "[GOAL_TEMP_CONNECT_FAIL] no nearest topology node found");
    return;
  }

  auto it = std::find_if(nodes_.begin(), nodes_.end(),
                         [nearest_node_id](const TopologyNode &n) {
                           return n.id == nearest_node_id;
                         });

  if (it == nodes_.end()) {
    RCLCPP_ERROR(this->get_logger(),
                 "[GOAL_TEMP_CONNECT_FAIL] nearest node id=%d not found",
                 nearest_node_id);
    return;
  }

  geometry_msgs::msg::Point node_pos;
  node_pos.x = it->x;
  node_pos.y = it->y;
  node_pos.z = it->z;

  /*
   * 这里不要直接 return。
   * 因为有些目标点本来就不完全贴在拓扑节点上。
   * 如果太远，只打印 WARN，方便你检查目标点文件和拓扑图是否一致。
   */
  if (nearest_dist > connection_distance_threshold_) {
    RCLCPP_WARN(this->get_logger(),
                "[GOAL_TEMP_CONNECT_FAR] goal_temp -> node%d too far, "
                "dist=%.3f > threshold=%.3f. Still connect nearest node. "
                "Please check target point and topology map.",
                nearest_node_id, nearest_dist, connection_distance_threshold_);
  }

  TopologyEdge edge;
  edge.id = temp_edges.size();
  edge.from_id = goal_node_id;
  edge.to_id = nearest_node_id;
  edge.weight = calculateConnectionCost(goal_position, node_pos);
  edge.bidirectional = true;
  temp_edges.push_back(edge);

  RCLCPP_WARN(this->get_logger(),
              "[GOAL_TEMP_CONNECT_OK] goal_temp -> node%d, "
              "goal=(%.3f, %.3f), node=(%.3f, %.3f), dist=%.3f, weight=%.3f",
              nearest_node_id, goal_position.x, goal_position.y, it->x, it->y,
              nearest_dist, edge.weight);
}

std::vector<int> GlobalPathPlanner::findTopologyPath(int start_node_id,
                                                     int goal_node_id) {
  if (start_node_id == goal_node_id) {
    return std::vector<int>{start_node_id};
  }

  return dijkstraSearch(start_node_id, goal_node_id);
}

std::vector<int> GlobalPathPlanner::dijkstraSearch(int start_id, int goal_id) {
  std::map<int, double> distances;
  std::map<int, int> previous;
  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      pq;

  // 初始化距离
  for (const auto &node : nodes_) {
    distances[node.id] = std::numeric_limits<double>::infinity();
  }
  distances[start_id] = 0.0;
  pq.push(std::make_pair(0.0, start_id));

  while (!pq.empty()) {
    int current_id = pq.top().second;
    double current_dist = pq.top().first;
    pq.pop();

    if (current_id == goal_id) {
      break;
    }

    if (current_dist > distances[current_id]) {
      continue;
    }

    // 检查所有相邻节点
    for (const auto &edge : edges_) {
      int neighbor_id = -1;
      if (edge.from_id == current_id) {
        neighbor_id = edge.to_id;
      } else if (edge.bidirectional && edge.to_id == current_id) {
        neighbor_id = edge.from_id;
      }

      if (neighbor_id != -1 && node_map_.find(neighbor_id) != node_map_.end()) {
        double edge_weight = edge.weight;
        double new_distance = distances[current_id] + edge_weight;

        if (new_distance < distances[neighbor_id]) {
          distances[neighbor_id] = new_distance;
          previous[neighbor_id] = current_id;
          pq.push(std::make_pair(new_distance, neighbor_id));
        }
      }
    }
  }

  // 重构路径
  std::vector<int> path;
  if (distances[goal_id] == std::numeric_limits<double>::infinity()) {
    return path;
  }

  int current = goal_id;
  while (current != start_id) {
    path.push_back(current);
    current = previous[current];
  }
  path.push_back(start_id);

  std::reverse(path.begin(), path.end());

  return path;
}

nav_msgs::msg::Path GlobalPathPlanner::buildCompletePath(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &goal,
    const std::vector<int> &topology_path) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;

  std::vector<geometry_msgs::msg::Point> waypoints;

  // 1. 添加起点
  waypoints.push_back(start);

  // 2. 添加拓扑路径点
  for (int node_id : topology_path) {
    if (node_map_.find(node_id) != node_map_.end()) {
      const TopologyNode &node = node_map_[node_id];
      geometry_msgs::msg::Point point;
      point.x = node.x;
      point.y = node.y;
      point.z = node.z;
      waypoints.push_back(point);
    }
  }

  // 3. 添加终点
  waypoints.push_back(goal);

  // 4. 插值生成完整路径
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    std::vector<geometry_msgs::msg::Point> segment =
        interpolatePath(waypoints[i], waypoints[i + 1], interpolation_points_);

    for (const auto &point : segment) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point;

      // 计算朝向
      if (i < waypoints.size() - 2 || &point != &segment.back()) {
        geometry_msgs::msg::Point next_point;
        if (&point == &segment.back()) {
          next_point = waypoints[i + 2];
        } else {
          auto it = std::find_if(segment.begin(), segment.end(),
                                 [&point](const geometry_msgs::msg::Point &p) {
                                   return p.x == point.x && p.y == point.y &&
                                          p.z == point.z;
                                 });
          if (it != segment.end() && std::next(it) != segment.end()) {
            next_point = *std::next(it);
          } else {
            next_point = waypoints[i + 1];
          }
        }
        pose.pose.orientation = calculateOrientation(point, next_point);
      } else {
        pose.pose.orientation.w = 1.0;
      }

      path.poses.push_back(pose);
    }
  }

  return path;
}

int GlobalPathPlanner::findNearestNode(const geometry_msgs::msg::Point &point) {
  int nearest_id = -1;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto &node : nodes_) {
    geometry_msgs::msg::Point node_pos;
    node_pos.x = node.x;
    node_pos.y = node.y;
    node_pos.z = node.z;

    double dist = calculateDistance(point, node_pos);
    if (dist < min_distance) {
      min_distance = dist;
      nearest_id = node.id;
    }
  }

  return nearest_id;
}

TopologyEdge GlobalPathPlanner::findNearestEdge(
    const geometry_msgs::msg::Point &point) {
  TopologyEdge nearest_edge;
  double min_distance = std::numeric_limits<double>::max();
  bool found = false;

  for (const auto &edge : edges_) {
    if (node_map_.find(edge.from_id) != node_map_.end() &&
        node_map_.find(edge.to_id) != node_map_.end()) {
      const TopologyNode &from_node = node_map_[edge.from_id];
      const TopologyNode &to_node = node_map_[edge.to_id];

      geometry_msgs::msg::Point from_point;
      from_point.x = from_node.x;
      from_point.y = from_node.y;
      from_point.z = from_node.z;

      geometry_msgs::msg::Point to_point;
      to_point.x = to_node.x;
      to_point.y = to_node.y;
      to_point.z = to_node.z;

      double dist = pointToSegmentDistance(point, from_point, to_point);
      if (dist < min_distance) {
        min_distance = dist;
        nearest_edge = edge;
        found = true;
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "未找到最近的边");
  }

  return nearest_edge;
}

geometry_msgs::msg::Point GlobalPathPlanner::findNearestPointOnEdge(
    const geometry_msgs::msg::Point &point, const TopologyEdge &edge) {
  geometry_msgs::msg::Point result = point;

  if (node_map_.find(edge.from_id) != node_map_.end() &&
      node_map_.find(edge.to_id) != node_map_.end()) {
    const TopologyNode &from_node = node_map_[edge.from_id];
    const TopologyNode &to_node = node_map_[edge.to_id];

    geometry_msgs::msg::Point from_point;
    from_point.x = from_node.x;
    from_point.y = from_node.y;
    from_point.z = from_node.z;

    geometry_msgs::msg::Point to_point;
    to_point.x = to_node.x;
    to_point.y = to_node.y;
    to_point.z = to_node.z;

    // 计算点在线段上的投影
    double dx = to_point.x - from_point.x;
    double dy = to_point.y - from_point.y;
    double dz = to_point.z - from_point.z;

    double segment_length_squared = dx * dx + dy * dy + dz * dz;

    if (segment_length_squared > 1e-6) {
      double px = point.x - from_point.x;
      double py = point.y - from_point.y;
      double pz = point.z - from_point.z;

      double t = (px * dx + py * dy + pz * dz) / segment_length_squared;
      t = std::max(0.0, std::min(1.0, t));  // 限制在线段范围内

      result.x = from_point.x + t * dx;
      result.y = from_point.y + t * dy;
      result.z = from_point.z + t * dz;
    } else {
      result = from_point;
    }
  }

  return result;
}

double GlobalPathPlanner::pointToSegmentDistance(
    const geometry_msgs::msg::Point &point,
    const geometry_msgs::msg::Point &seg_start,
    const geometry_msgs::msg::Point &seg_end) {
  // 计算线段向量
  double dx = seg_end.x - seg_start.x;
  double dy = seg_end.y - seg_start.y;
  double dz = seg_end.z - seg_start.z;

  double segment_length_squared = dx * dx + dy * dy + dz * dz;

  if (segment_length_squared < 1e-6) {
    // 线段退化为点
    return calculateDistance(point, seg_start);
  }

  // 计算点到起点的向量
  double px = point.x - seg_start.x;
  double py = point.y - seg_start.y;
  double pz = point.z - seg_start.z;

  // 计算投影参数 t
  double t = (px * dx + py * dy + pz * dz) / segment_length_squared;
  t = std::max(0.0, std::min(1.0, t));  // 限制在 [0, 1] 范围内

  // 计算投影点
  geometry_msgs::msg::Point projection;
  projection.x = seg_start.x + t * dx;
  projection.y = seg_start.y + t * dy;
  projection.z = seg_start.z + t * dz;

  return calculateDistance(point, projection);
}

std::vector<geometry_msgs::msg::Point> GlobalPathPlanner::interpolatePath(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &end, int num_points) {
  std::vector<geometry_msgs::msg::Point> points;

  if (num_points <= 0) {
    return points;
  }

  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;

    geometry_msgs::msg::Point point;
    point.x = start.x + t * (end.x - start.x);
    point.y = start.y + t * (end.y - start.y);
    point.z = start.z + t * (end.z - start.z);

    points.push_back(point);
  }

  return points;
}

double GlobalPathPlanner::calculateDistance(
    const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) {
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

geometry_msgs::msg::Quaternion GlobalPathPlanner::calculateOrientation(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to) {
  geometry_msgs::msg::Quaternion quat;

  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double yaw = std::atan2(dy, dx);

  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = std::sin(yaw / 2.0);
  quat.w = std::cos(yaw / 2.0);

  return quat;
}

std::vector<int> GlobalPathPlanner::dijkstraSearchInTempGraph(
    const std::vector<TopologyNode> &temp_nodes,
    const std::vector<TopologyEdge> &temp_edges, int start_id, int goal_id) {
  std::map<int, double> distances;
  std::map<int, int> previous;
  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      pq;

  // 初始化距离
  for (const auto &node : temp_nodes) {
    distances[node.id] = std::numeric_limits<double>::infinity();
  }
  distances[start_id] = 0.0;
  pq.push(std::make_pair(0.0, start_id));

  while (!pq.empty()) {
    int current_id = pq.top().second;
    double current_dist = pq.top().first;
    pq.pop();

    if (current_id == goal_id) {
      break;
    }

    if (current_dist > distances[current_id]) {
      continue;
    }

    // 检查所有相邻节点
    for (const auto &edge : temp_edges) {
      int neighbor_id = -1;
      if (edge.from_id == current_id) {
        neighbor_id = edge.to_id;
      } else if (edge.bidirectional && edge.to_id == current_id) {
        neighbor_id = edge.from_id;
      }

      if (neighbor_id != -1) {
        double edge_weight = edge.weight;
        double new_distance = distances[current_id] + edge_weight;

        if (new_distance < distances[neighbor_id]) {
          distances[neighbor_id] = new_distance;
          previous[neighbor_id] = current_id;
          pq.push(std::make_pair(new_distance, neighbor_id));
        }
      }
    }
  }

  // 重构路径
  std::vector<int> path;
  if (distances[goal_id] == std::numeric_limits<double>::infinity()) {
    RCLCPP_WARN(this->get_logger(), "在临时拓扑图中未找到从 %d 到 %d 的路径",
                start_id, goal_id);
    return path;
  }

  int current = goal_id;
  while (current != start_id) {
    path.push_back(current);
    if (previous.find(current) == previous.end()) {
      RCLCPP_ERROR(this->get_logger(), "路径重构失败：节点 %d 没有前驱节点",
                   current);
      return std::vector<int>();
    }
    current = previous[current];
  }
  path.push_back(start_id);

  std::reverse(path.begin(), path.end());

  RCLCPP_INFO(this->get_logger(), "在临时拓扑图中找到路径，包含 %lu 个节点",
              path.size());
  return path;
}

nav_msgs::msg::Path GlobalPathPlanner::buildCompletePathFromTempGraph(
    const std::vector<TopologyNode> &temp_nodes,
    const std::vector<int> &topology_path) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;

  if (topology_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "拓扑路径为空，无法构建完整路径");
    return path;
  }

  // 创建节点ID到节点的映射
  std::map<int, TopologyNode> temp_node_map;
  for (const auto &node : temp_nodes) {
    temp_node_map[node.id] = node;
  }

  std::vector<geometry_msgs::msg::Point> waypoints;

  // 将拓扑路径中的所有节点转换为路径点
  for (int node_id : topology_path) {
    if (temp_node_map.find(node_id) != temp_node_map.end()) {
      const TopologyNode &node = temp_node_map[node_id];
      geometry_msgs::msg::Point point;
      point.x = node.x;
      point.y = node.y;
      point.z = node.z;
      waypoints.push_back(point);
    } else {
      RCLCPP_WARN(this->get_logger(), "节点 %d 在临时节点映射中不存在",
                  node_id);
    }
  }

  if (waypoints.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "路径点不足，无法构建完整路径");
    return path;
  }

  // 插值生成完整路径
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    std::vector<geometry_msgs::msg::Point> segment =
        interpolatePath(waypoints[i], waypoints[i + 1], interpolation_points_);

    for (size_t j = 0; j < segment.size(); ++j) {
      // 避免重复添加相同的点（除了第一个段）
      if (i == 0 || j != 0) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = segment[j];

        // 计算朝向
        if (j < segment.size() - 1) {
          pose.pose.orientation =
              calculateOrientation(segment[j], segment[j + 1]);
        } else if (i < waypoints.size() - 2) {
          pose.pose.orientation =
              calculateOrientation(segment[j], waypoints[i + 2]);
        } else {
          // 最后一个点，保持前一个点的朝向
          if (!path.poses.empty()) {
            pose.pose.orientation = path.poses.back().pose.orientation;
          } else {
            pose.pose.orientation.w = 1.0;
          }
        }

        path.poses.push_back(pose);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "构建完整路径完成，包含 %lu 个路径点",
              path.poses.size());
  return path;
}

bool GlobalPathPlanner::loadFreeSpaceCloudFromPcd() {
  if (free_space_pcd_file_.empty()) {
    RCLCPP_WARN(this->get_logger(), "free_space_pcd_file 为空");
    has_free_space_cloud_ = false;
    return false;
  }

  free_space_cloud_->clear();
  try {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(free_space_pcd_file_,
                                            *free_space_cloud_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "加载可行域 PCD 失败: %s",
                   free_space_pcd_file_.c_str());
      has_free_space_cloud_ = false;
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "加载可行域 PCD 异常: %s, file=%s",
                 e.what(), free_space_pcd_file_.c_str());
    has_free_space_cloud_ = false;
    return false;
  }

  if (free_space_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "可行域 PCD 为空: %s",
                free_space_pcd_file_.c_str());
    has_free_space_cloud_ = false;
    return false;
  }

  kdtree_->setInputCloud(free_space_cloud_);
  has_free_space_cloud_ = true;

  RCLCPP_INFO(this->get_logger(), "已加载可行域 PCD: %s, points=%lu",
              free_space_pcd_file_.c_str(), free_space_cloud_->size());
  return true;
}

bool GlobalPathPlanner::isEdgeLegal(const TopologyNode &node1,
                                    const TopologyNode &node2) {
  if (!has_free_space_cloud_ || free_space_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "没有可行域点云数据，无法进行边合法性检查");
    return false;
  }

  // 将节点转换为Point并减去机器狗高度
  geometry_msgs::msg::Point point1, point2;
  point1.x = node1.x;
  point1.y = node1.y;
  point1.z = node1.z - robot_height_;

  point2.x = node2.x;
  point2.y = node2.y;
  point2.z = node2.z - robot_height_;

  RCLCPP_DEBUG(this->get_logger(),
               "检查边合法性: 节点%d(%.2f,%.2f,%.2f) -> 节点%d(%.2f,%.2f,%.2f)",
               node1.id, point1.x, point1.y, point1.z, node2.id, point2.x,
               point2.y, point2.z);

  // 对边进行采样
  std::vector<geometry_msgs::msg::Point> sample_points =
      sampleEdgePoints(point1, point2, edge_sample_count_);

  if (sample_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "边采样失败");
    return false;
  }

  int valid_points = 0;
  int total_points = sample_points.size();

  // 检查每个采样点
  for (const auto &sample_point : sample_points) {
    if (isPointInFreeSpaceWithRadius(sample_point, safety_sphere_radius_)) {
      valid_points++;
    }
  }

  // 计算有效点比例
  double valid_ratio = static_cast<double>(valid_points) / total_points;

  RCLCPP_DEBUG(this->get_logger(),
               "边合法性检查结果: %d/%d 个点有效 (%.1f%%), 阈值: %.1f%%",
               valid_points, total_points, valid_ratio * 100,
               valid_point_threshold_ * 100);

  return valid_ratio >= valid_point_threshold_;
}

std::vector<geometry_msgs::msg::Point> GlobalPathPlanner::sampleEdgePoints(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &end, int sample_count) {
  std::vector<geometry_msgs::msg::Point> sample_points;

  if (sample_count <= 0) {
    RCLCPP_WARN(this->get_logger(), "采样点数量必须大于0");
    return sample_points;
  }

  sample_points.reserve(sample_count);

  // 均匀采样
  for (int i = 0; i < sample_count; ++i) {
    double t = static_cast<double>(i) / (sample_count - 1);  // 从0到1的参数

    geometry_msgs::msg::Point sample_point;
    sample_point.x = start.x + t * (end.x - start.x);
    sample_point.y = start.y + t * (end.y - start.y);
    sample_point.z = start.z + t * (end.z - start.z);

    sample_points.push_back(sample_point);
  }

  RCLCPP_DEBUG(this->get_logger(), "生成 %lu 个边采样点", sample_points.size());
  return sample_points;
}

bool GlobalPathPlanner::isPointInFreeSpaceWithRadius(
    const geometry_msgs::msg::Point &point, double radius) {
  if (!has_free_space_cloud_ || free_space_cloud_->empty() || !kdtree_) {
    return false;
  }

  try {
    // 将geometry_msgs::msg::Point转换为PCL点
    pcl::PointXYZ search_point;
    search_point.x = point.x;
    search_point.y = point.y;
    search_point.z = point.z;

    // 使用KdTree进行半径搜索
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    int found_points = kdtree_->radiusSearch(search_point, radius,
                                             point_indices, point_distances);

    if (found_points > 0) {
      RCLCPP_DEBUG(this->get_logger(), "在半径 %.2f 内找到 %d 个可行域点",
                   radius, found_points);
      return true;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "在半径 %.2f 内未找到可行域点", radius);
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "半径搜索时发生异常: %s", e.what());
    return false;
  }
}

double GlobalPathPlanner::calculateConnectionCost(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to) {
  // 基础距离代价
  double distance_cost = calculateDistance(from, to);

  // 简单的线性代价函数，距离越远代价越高
  return distance_cost;
}

bool GlobalPathPlanner::isConnectionFeasible(
    const geometry_msgs::msg::Point &from,
    const geometry_msgs::msg::Point &to) {
  // 1. 检查距离是否合理（不能太远）
  double distance = calculateDistance(from, to);
  if (distance > connection_distance_threshold_) {
    return false;
  }

  // 2. 如果禁用可行性检查，只检查距离
  if (!enable_feasibility_check_) {
    return true;
  }

  // 3. 检查两点间是否有视线（简单的直线障碍物检测）
  if (!checkLineOfSight(from, to)) {
    return false;
  }

  return true;
}

bool GlobalPathPlanner::checkLineOfSight(const geometry_msgs::msg::Point &from,
                                         const geometry_msgs::msg::Point &to) {
  // 简化的视线检查：沿直线采样多个点，检查是否都在自由空间
  int num_samples =
      static_cast<int>(calculateDistance(from, to) / line_of_sight_resolution_);
  num_samples = std::max(5, std::min(num_samples, 20));  // 限制采样点数量

  for (int i = 1; i < num_samples; ++i) {
    double t = static_cast<double>(i) / num_samples;
    geometry_msgs::msg::Point sample_point;
    sample_point.x = from.x + t * (to.x - from.x);
    sample_point.y = from.y + t * (to.y - from.y);
    sample_point.z = from.z + t * (to.z - from.z);

    if (!isPointInFreeSpace(sample_point)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "视线检查失败：采样点 (%.2f, %.2f, %.2f) 不在自由空间",
                   sample_point.x, sample_point.y, sample_point.z);
      return false;
    }
  }

  return true;
}

bool GlobalPathPlanner::isPointInFreeSpace(
    const geometry_msgs::msg::Point &point) {
  // 简化的自由空间检查
  // 这里可以集成点云地图或栅格地图进行更精确的检查

  // 检查点是否距离现有拓扑节点足够近（假设拓扑节点都在自由空间）
  for (const auto &node : nodes_) {
    geometry_msgs::msg::Point node_pos;
    node_pos.x = node.x;
    node_pos.y = node.y;
    node_pos.z = node.z;

    double dist = calculateDistance(point, node_pos);
    if (dist < free_space_radius_)  // 使用配置的自由空间半径
    {
      return true;
    }
  }

  // 也可以检查点是否距离任何拓扑边足够近
  for (const auto &edge : edges_) {
    if (node_map_.find(edge.from_id) != node_map_.end() &&
        node_map_.find(edge.to_id) != node_map_.end()) {
      const TopologyNode &from_node = node_map_[edge.from_id];
      const TopologyNode &to_node = node_map_[edge.to_id];

      geometry_msgs::msg::Point from_point, to_point;
      from_point.x = from_node.x;
      from_point.y = from_node.y;
      from_point.z = from_node.z;

      to_point.x = to_node.x;
      to_point.y = to_node.y;
      to_point.z = to_node.z;

      double dist_to_edge = pointToSegmentDistance(point, from_point, to_point);
      if (dist_to_edge < edge_proximity_threshold_)  // 使用配置的边接近阈值
      {
        return true;
      }
    }
  }

  // 默认：如果距离所有拓扑元素都太远，认为可能不在自由空间
  RCLCPP_DEBUG(this->get_logger(), "点 (%.2f, %.2f, %.2f) 可能不在自由空间",
               point.x, point.y, point.z);
  return false;
}
}  // namespace global_path_planner
