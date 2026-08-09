#include "global_path_planner/global_path_planner.h"

#include <tf2/utils.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <sstream>

namespace global_path_planner {

GlobalPathPlanner::GlobalPathPlanner(const rclcpp::NodeOptions& options)
    : rclcpp::Node("global_path_planner_node", options),
      has_free_space_cloud_(false),
      has_current_pose_(false),
      has_goal_(false) {
  // 声明并读取 ROS2 参数
  this->declare_parameter<std::string>(
      "topology_map_file",
      "/robotfs/data/slam_data/map_0/topology_map/connected_topology_map.txt");
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("robot_frame", "base_link");
  this->declare_parameter<std::string>("pose_topic",
                                       "/pose_stamped_localization");
  this->declare_parameter<double>("connection_distance_threshold", 5.0);
  this->declare_parameter<int>("interpolation_points", 20);

  this->declare_parameter<bool>("enable_direct_connection", true);
  this->declare_parameter<double>("direct_connection_threshold", 3.0);
  this->declare_parameter<double>("direct_connection_cost_ratio", 0.9);
  // 边合法性检查相关参数
  this->declare_parameter<double>("robot_height", 0.35);
  this->declare_parameter<int>("edge_sample_count", 100);
  this->declare_parameter<double>("safety_sphere_radius", 0.3);
  this->declare_parameter<double>("valid_point_threshold", 0.9);
  this->declare_parameter<std::string>(
      "free_space_pcd_file",
      "/robotfs/data/slam_data/map_0/map/result_free_space.pcd");
  this->declare_parameter<bool>("enable_temp_connection_feasibility_check",
                                false);

  this->get_parameter("topology_map_file", map_file_path_);
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("robot_frame", robot_frame_);
  this->get_parameter("pose_topic", pose_topic_);
  this->get_parameter("connection_distance_threshold",
                      connection_distance_threshold_);
  this->get_parameter("interpolation_points", interpolation_points_);
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
  global_path_with_policy_pub_ =
      this->create_publisher<navigation_interfaces::msg::PathWithPolicy>(
          "/global_path_with_policy",
          rclcpp::QoS(1).transient_local().reliable());

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
        RCLCPP_DEBUG(this->get_logger(),
                     "读取节点: ID=%d, 位置=(%.2f,%.2f,%.2f)", node.id, node.x,
                     node.y, node.z);
      }
    } else {
      // 读取边: ID 起始ID 结束ID 权重 类型 双向 [停障策略] [绕障策略]
      TopologyEdge edge;
      std::string type;
      int bidirectional_int;
      int obstacle_stop_enabled_int = 1;
      int obstacle_avoidance_enabled_int = 0;

      if (iss >> edge.id >> edge.from_id >> edge.to_id >> edge.weight >> type >>
          bidirectional_int) {
        edge.bidirectional = (bidirectional_int == 1);
        // 可选字段缺省时保持停障启用、主动绕障关闭。
        if (iss >> obstacle_stop_enabled_int) {
          edge.obstacle_stop_enabled = obstacle_stop_enabled_int == 1;
          if (iss >> obstacle_avoidance_enabled_int) {
            edge.obstacle_avoidance_enabled =
                obstacle_avoidance_enabled_int == 1;
          }
        }
        edges_.push_back(edge);

        RCLCPP_DEBUG(this->get_logger(),
                     "读取边: ID=%d, %d->%d, 权重=%.2f, 双向=%s, "
                     "启用停障=%s, 启用绕障=%s",
                     edge.id,
                     edge.from_id, edge.to_id, edge.weight,
                     edge.bidirectional ? "是" : "否",
                     edge.obstacle_stop_enabled ? "是" : "否",
                     edge.obstacle_avoidance_enabled ? "是" : "否");
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
        navigation_interfaces::msg::PathWithPolicy path_with_policy;
        path_with_policy.path = global_path;
        path_with_policy.obstacle_stop_enabled =
            last_path_obstacle_stop_policy_;
        path_with_policy.obstacle_avoidance_enabled =
            last_path_obstacle_avoidance_policy_;
        global_path_with_policy_pub_->publish(path_with_policy);
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
      navigation_interfaces::msg::PathWithPolicy path_with_policy;
      path_with_policy.path = global_path;
      path_with_policy.obstacle_stop_enabled =
          last_path_obstacle_stop_policy_;
      path_with_policy.obstacle_avoidance_enabled =
          last_path_obstacle_avoidance_policy_;
      global_path_with_policy_pub_->publish(path_with_policy);
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

    // 直连只有在起点和终点依附同一条真实拓扑边时才能准确表达边策略。
    // 跨越多条真实边的直连无法用一组布尔属性描述，因此不创建这种直连。
    const TopologyEdge *start_attached_edge =
        findNearestTopologyEdge(current_position_);
    const TopologyEdge *goal_attached_edge =
        findNearestTopologyEdge(goal_position_);
    const bool direct_policy_unambiguous =
        start_attached_edge != nullptr &&
        start_attached_edge == goal_attached_edge;

    if (enable_direct_connection_ &&
        direct_distance <= direct_connection_threshold_ &&
        direct_connection_feasible && direct_policy_unambiguous) {
      // 创建起点到终点的直接连接
      TopologyEdge direct_edge;
      direct_edge.id = temp_edges.size();
      direct_edge.from_id = start_node_id;
      direct_edge.to_id = goal_node_id;
      direct_edge.weight =
          direct_distance *
          direct_connection_cost_ratio_;  // 给直接连接一个小的奖励
      direct_edge.bidirectional = true;
      inheritEdgePolicy(start_attached_edge, direct_edge);
      temp_edges.push_back(direct_edge);

      RCLCPP_INFO(this->get_logger(),
                  "添加起点到终点的直接连接，距离: %.2f 米，继承边%d策略",
                  direct_distance, start_attached_edge->id);
    } else {
      if (!enable_direct_connection_) {
        RCLCPP_DEBUG(this->get_logger(), "直接连接功能已禁用");
      } else if (direct_distance > direct_connection_threshold_) {
        RCLCPP_DEBUG(this->get_logger(),
                     "起点到终点距离 %.2f 米超过阈值 %.2f 米", direct_distance,
                     direct_connection_threshold_);
      } else if (!direct_policy_unambiguous) {
        RCLCPP_INFO(this->get_logger(),
                    "起点和终点不属于同一条真实拓扑边，跳过直连以保留分段策略");
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
    path =
        buildCompletePathFromTempGraph(temp_nodes, temp_edges, topology_path);
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
    std::vector<TopologyNode> & /*temp_nodes*/,
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

  const TopologyEdge *nearest_position_edge =
      findNearestTopologyEdge(position);

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
        // 连接第二层节点时，优先继承最近节点到该节点的真实边；
        // 连接最近节点时，继承临时点所在的最近真实边。
        const TopologyEdge *policy_edge = nullptr;
        if (candidate_id != nearest_legal_node_id) {
          policy_edge =
              findTopologyEdge(nearest_legal_node_id, candidate_id);
        }
        if (policy_edge == nullptr) {
          policy_edge = nearest_position_edge;
        }
        inheritEdgePolicy(policy_edge, edge);
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
      inheritEdgePolicy(nearest_position_edge, edge);
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
    std::vector<TopologyNode> & /*temp_nodes*/,
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
  inheritEdgePolicy(findNearestTopologyEdge(goal_position), edge);
  temp_edges.push_back(edge);

  RCLCPP_WARN(this->get_logger(),
              "[GOAL_TEMP_CONNECT_OK] goal_temp -> node%d, "
              "goal=(%.3f, %.3f), node=(%.3f, %.3f), dist=%.3f, weight=%.3f",
              nearest_node_id, goal_position.x, goal_position.y, it->x, it->y,
              nearest_dist, edge.weight);
}

const TopologyEdge *GlobalPathPlanner::findTopologyEdge(
    int first_node_id, int second_node_id) const {
  const TopologyEdge *selected = nullptr;
  for (const auto &edge : edges_) {
    const bool forward =
        edge.from_id == first_node_id && edge.to_id == second_node_id;
    const bool reverse = edge.bidirectional &&
                         edge.from_id == second_node_id &&
                         edge.to_id == first_node_id;
    if ((forward || reverse) &&
        (selected == nullptr || edge.weight < selected->weight)) {
      selected = &edge;
    }
  }
  return selected;
}

const TopologyEdge *GlobalPathPlanner::findNearestTopologyEdge(
    const geometry_msgs::msg::Point &position) const {
  const TopologyEdge *nearest_edge = nullptr;
  double nearest_distance_squared = std::numeric_limits<double>::infinity();

  for (const auto &edge : edges_) {
    const auto from_it = std::find_if(
        nodes_.begin(), nodes_.end(),
        [&edge](const TopologyNode &node) { return node.id == edge.from_id; });
    const auto to_it = std::find_if(
        nodes_.begin(), nodes_.end(),
        [&edge](const TopologyNode &node) { return node.id == edge.to_id; });
    if (from_it == nodes_.end() || to_it == nodes_.end()) {
      continue;
    }

    const double dx = to_it->x - from_it->x;
    const double dy = to_it->y - from_it->y;
    const double dz = to_it->z - from_it->z;
    const double length_squared = dx * dx + dy * dy + dz * dz;
    double interpolation = 0.0;
    if (length_squared > 1e-12) {
      interpolation =
          ((position.x - from_it->x) * dx +
           (position.y - from_it->y) * dy +
           (position.z - from_it->z) * dz) /
          length_squared;
      interpolation = std::clamp(interpolation, 0.0, 1.0);
    }

    const double nearest_x = from_it->x + interpolation * dx;
    const double nearest_y = from_it->y + interpolation * dy;
    const double nearest_z = from_it->z + interpolation * dz;
    const double error_x = position.x - nearest_x;
    const double error_y = position.y - nearest_y;
    const double error_z = position.z - nearest_z;
    const double distance_squared =
        error_x * error_x + error_y * error_y + error_z * error_z;
    if (distance_squared < nearest_distance_squared) {
      nearest_distance_squared = distance_squared;
      nearest_edge = &edge;
    }
  }
  return nearest_edge;
}

void GlobalPathPlanner::inheritEdgePolicy(const TopologyEdge *source,
                                          TopologyEdge &destination) {
  if (source == nullptr) {
    return;
  }
  destination.obstacle_stop_enabled = source->obstacle_stop_enabled;
  destination.obstacle_avoidance_enabled =
      source->obstacle_avoidance_enabled;
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
    const std::vector<TopologyEdge> &temp_edges,
    const std::vector<int> &topology_path) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;

  if (topology_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "拓扑路径为空，无法构建完整路径");
    return path;
  }
  last_path_obstacle_stop_policy_.clear();
  last_path_obstacle_avoidance_policy_.clear();

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

  // 每个拓扑段继承所选边的停障策略；临时连接边默认启用停障。
  std::vector<bool> segment_obstacle_stop_enabled;
  std::vector<bool> segment_obstacle_avoidance_enabled;
  segment_obstacle_stop_enabled.reserve(topology_path.size() - 1U);
  segment_obstacle_avoidance_enabled.reserve(topology_path.size() - 1U);
  for (std::size_t i = 0; i + 1U < topology_path.size(); ++i) {
    const int from_id = topology_path[i];
    const int to_id = topology_path[i + 1U];
    bool obstacle_stop_enabled = true;
    bool obstacle_avoidance_enabled = false;
    double selected_weight = std::numeric_limits<double>::infinity();
    for (const auto &candidate : temp_edges) {
      const bool matches =
          (candidate.from_id == from_id && candidate.to_id == to_id) ||
          (candidate.bidirectional && candidate.from_id == to_id &&
           candidate.to_id == from_id);
      if (matches && candidate.weight < selected_weight) {
        selected_weight = candidate.weight;
        obstacle_stop_enabled = candidate.obstacle_stop_enabled;
        obstacle_avoidance_enabled = candidate.obstacle_avoidance_enabled;
      }
    }
    segment_obstacle_stop_enabled.push_back(obstacle_stop_enabled);
    segment_obstacle_avoidance_enabled.push_back(obstacle_avoidance_enabled);
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
        // 拓扑节点边界采用下一段策略，确保进入下一段前完成模式切换。
        const bool at_segment_end = j + 1U == segment.size();
        const std::size_t policy_segment =
            at_segment_end && i + 1U < segment_obstacle_stop_enabled.size()
                ? i + 1U
                : i;
        last_path_obstacle_stop_policy_.push_back(
            segment_obstacle_stop_enabled[policy_segment]);
        last_path_obstacle_avoidance_policy_.push_back(
            segment_obstacle_avoidance_enabled[policy_segment]);
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

}  // namespace global_path_planner
