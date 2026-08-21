#include "global_path_planner/global_path_planner.h"

#include <tf2/utils.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <limits>
#include <nlohmann/json.hpp>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace global_path_planner {

GlobalPathPlanner::GlobalPathPlanner(const rclcpp::NodeOptions &options)
    : rclcpp::Node("global_path_planner_node", options),
      has_free_space_cloud_(false),
      has_current_pose_(false),
      has_goal_(false) {
  // 声明并读取 ROS2 参数
  this->declare_parameter<std::string>(
      "topology_map_file",
      "/robotfs/data/slam_data/map_0/task/topology_map.json");
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
  RCLCPP_INFO(this->get_logger(), "开始加载拓扑JSON: %s",
              map_file_path_.c_str());

  std::ifstream file(map_file_path_.c_str());
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开拓扑JSON: %s",
                 map_file_path_.c_str());
    return false;
  }

  nodes_.clear();
  edges_.clear();

  try {
    const nlohmann::json document = nlohmann::json::parse(file);
    if (!document.is_object() || document.value("schema_version", 0) != 1 ||
        document.value("format", "") != "navigation_topology" ||
        !document.contains("nodes") || !document["nodes"].is_array() ||
        !document.contains("edges") || !document["edges"].is_array()) {
      throw std::runtime_error(
          "expected schema_version=1, format=navigation_topology, nodes[] and "
          "edges[]");
    }
    const std::string topology_frame = document.value("frame_id", "map");
    if (topology_frame != global_frame_) {
      throw std::runtime_error("topology frame_id does not match global_frame");
    }

    std::unordered_set<int> node_ids;
    nodes_.reserve(document["nodes"].size());
    for (const auto &json_node : document["nodes"]) {
      TopologyNode node;
      node.id = json_node.at("id").get<int>();
      node.x = json_node.at("x").get<double>();
      node.y = json_node.at("y").get<double>();
      node.z = json_node.value("z", 0.0);
      node.label = json_node.value("label", "node_" + std::to_string(node.id));
      if (node.id < 0 || node.id > std::numeric_limits<int>::max() - 2 ||
          !node_ids.insert(node.id).second || !std::isfinite(node.x) ||
          !std::isfinite(node.y) || !std::isfinite(node.z)) {
        throw std::runtime_error(
            "node contains a negative/duplicate id or invalid coordinate");
      }
      nodes_.push_back(std::move(node));
    }

    std::unordered_set<int> edge_ids;
    edges_.reserve(document["edges"].size());
    for (const auto &json_edge : document["edges"]) {
      TopologyEdge edge;
      edge.id = json_edge.at("id").get<int>();
      edge.from_id = json_edge.at("from_id").get<int>();
      edge.to_id = json_edge.at("to_id").get<int>();
      edge.edge_type = json_edge.value("edge_type", "straight");
      edge.weight = edge.edge_type == "map_connection"
                        ? 0.0
                        : json_edge.at("weight").get<double>();
      edge.bidirectional = false;
      edge.policy.velocity_limit_enabled = true;
      edge.policy.linear_velocity = json_edge.value("linear_velocity", 0.5);
      edge.policy.angular_velocity = json_edge.value("angular_velocity", 0.5);
      edge.policy.motion_direction = json_edge.value("motion_direction", 1);
      edge.policy.obstacle_stop_enabled =
          json_edge.value("obstacle_stop_enabled", true);
      edge.policy.obstacle_avoidance_enabled =
          json_edge.value("obstacle_avoidance_enabled", false);
      edge.policy.obstacle_avoidance_distance_m =
          json_edge.value("obstacle_avoidance_distance_m", 0.5);
      if (edge.id < 0 || !edge_ids.insert(edge.id).second) {
        throw std::runtime_error("edge contains a negative or duplicate id");
      }
      if (edge.edge_type == "bezier") {
        const auto &points = json_edge.at("bezier_control_points");
        if (!points.is_array() || points.size() != 2U) {
          throw std::runtime_error(
              "bezier edge requires exactly two control points");
        }
        for (std::size_t index = 0; index < 2U; ++index) {
          edge.bezier_control_points[index].x =
              points[index].at("x").get<double>();
          edge.bezier_control_points[index].y =
              points[index].at("y").get<double>();
          edge.bezier_control_points[index].z = points[index].value("z", 0.0);
        }
        edge.has_bezier_control_points = true;
      }
      edges_.push_back(std::move(edge));
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(), "拓扑JSON解析失败: %s", error.what());
    nodes_.clear();
    edges_.clear();
    return false;
  }

  if (!rebuildGraphIndexes()) {
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "拓扑JSON加载完成: %lu 个节点, %lu 条边",
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

    planAndPublish();
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

  planAndPublish();
}

void GlobalPathPlanner::planAndPublish() {
  if (!has_current_pose_) {
    RCLCPP_WARN(this->get_logger(), "尚未接收到当前位置信息");
    return;
  }

  nav_msgs::msg::Path global_path = planGlobalPath();
  if (global_path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "无法规划路径");
    return;
  }

  global_path_pub_->publish(global_path);
  navigation_interfaces::msg::PathWithPolicy path_with_policy;
  path_with_policy.path = global_path;
  path_with_policy.edge_policies = last_path_edge_policies_;
  path_with_policy.dynamic_path = false;
  global_path_with_policy_pub_->publish(path_with_policy);
  RCLCPP_INFO(this->get_logger(), "发布全局路径，包含 %lu 个路径点",
              global_path.poses.size());
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
    // 基础图已经建立索引；每次规划只创建起终点及其临时连接。
    std::vector<TopologyNode> temporary_nodes;
    std::vector<TopologyEdge> temporary_edges;
    temporary_nodes.reserve(2);
    temporary_edges.reserve(10);

    // 添加起点作为临时节点
    const int start_node_id = max_node_id_ + 1;
    TopologyNode start_node;
    start_node.id = start_node_id;
    start_node.x = current_position_.x;
    start_node.y = current_position_.y;
    start_node.z = current_position_.z;
    start_node.label = "start_temp";
    temporary_nodes.push_back(start_node);

    // 添加终点作为临时节点
    const int goal_node_id = start_node_id + 1;
    TopologyNode goal_node;
    goal_node.id = goal_node_id;
    goal_node.x = goal_position_.x;
    goal_node.y = goal_position_.y;
    goal_node.z = goal_position_.z;
    goal_node.label = "goal_temp";
    temporary_nodes.push_back(goal_node);

    // 起点是机器人当前实时位置，可以保留原来的“最近节点 +
    // 第二层邻居”逻辑，避免刚好在边中间时折返
    addTemporaryConnections(temporary_edges, start_node_id, current_position_);

    // 终点是真实目标点，不允许第二层邻居把目标点带到错误拓扑点
    addGoalTemporaryConnection(temporary_edges, goal_node_id, goal_position_);

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
      direct_edge.id = edges_.size() + temporary_edges.size();
      direct_edge.from_id = start_node_id;
      direct_edge.to_id = goal_node_id;
      direct_edge.weight =
          direct_distance *
          direct_connection_cost_ratio_;  // 给直接连接一个小的奖励
      direct_edge.bidirectional = true;
      inheritEdgePolicy(start_attached_edge, direct_edge);
      temporary_edges.push_back(direct_edge);

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

    const EdgeAdjacency temporary_adjacency = buildAdjacency(temporary_edges);
    RCLCPP_DEBUG(this->get_logger(), "临时拓扑图创建完成：%lu 个节点，%lu 条边",
                 nodes_.size() + temporary_nodes.size(),
                 edges_.size() + temporary_edges.size());

    // 在临时拓扑图中进行Dijkstra搜索
    TopologySearchResult topology_path = dijkstraSearchInTempGraph(
        temporary_nodes, temporary_adjacency, start_node_id, goal_node_id);
    {
      RCLCPP_WARN(this->get_logger(),
                  "========== TOPOLOGY PATH RESULT ==========");

      for (size_t i = 0; i < topology_path.node_ids.size(); ++i) {
        const int id = topology_path.node_ids[i];
        const TopologyNode *node = findNodeById(id, &temporary_nodes);
        if (node != nullptr) {
          RCLCPP_WARN(
              this->get_logger(),
              "[TOPO_PATH] order=%zu id=%d label=%s x=%.3f y=%.3f z=%.3f", i,
              node->id, node->label.c_str(), node->x, node->y, node->z);
        }
      }
    }
    if (topology_path.node_ids.empty()) {
      RCLCPP_WARN(this->get_logger(), "未找到可行路径");
      return path;
    }

    // 构建完整路径
    path = buildCompletePathFromTempGraph(temporary_nodes, topology_path);
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

    const TopologyNode *node = findNodeById(node_id);
    if (node != nullptr) {
      // 根据参数控制是否进行边合法性检查
      bool is_legal = true;
      if (enable_temp_connection_feasibility_check_) {
        is_legal = isEdgeLegal(temp_node, *node);
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

  const TopologyEdge *nearest_position_edge = findNearestTopologyEdge(position);

  // 第三步：对所有候选节点进行合法性检查并创建连接
  int actual_connections = 0;
  for (int candidate_id : candidate_nodes) {
    const TopologyNode *candidate = findNodeById(candidate_id);
    if (candidate != nullptr) {
      // 根据参数控制是否进行边合法性检查
      bool is_legal = true;
      if (enable_temp_connection_feasibility_check_ &&
          candidate_id != nearest_legal_node_id) {
        is_legal = isEdgeLegal(temp_node, *candidate);
      }

      if (is_legal) {
        // 计算连接代价
        geometry_msgs::msg::Point node_pos;
        node_pos.x = candidate->x;
        node_pos.y = candidate->y;
        node_pos.z = candidate->z;

        const double cost = calculateDistance(position, node_pos);

        // 创建双向连接
        TopologyEdge edge;
        edge.id = edges_.size() + temp_edges.size();
        edge.from_id = temp_node_id;
        edge.to_id = candidate_id;
        edge.weight = cost;
        edge.bidirectional = true;
        // 连接第二层节点时，优先继承最近节点到该节点的真实边；
        // 连接最近节点时，继承临时点所在的最近真实边。
        const TopologyEdge *policy_edge = nullptr;
        if (candidate_id != nearest_legal_node_id) {
          policy_edge = findTopologyEdge(nearest_legal_node_id, candidate_id);
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

  RCLCPP_INFO(this->get_logger(), "临时节点 %d 成功连接到 %d 个节点",
              temp_node_id, actual_connections);
}

void GlobalPathPlanner::addGoalTemporaryConnection(
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

  const TopologyNode *nearest_node = findNodeById(nearest_node_id);
  if (nearest_node == nullptr) {
    RCLCPP_ERROR(this->get_logger(),
                 "[GOAL_TEMP_CONNECT_FAIL] nearest node id=%d not found",
                 nearest_node_id);
    return;
  }

  geometry_msgs::msg::Point node_pos;
  node_pos.x = nearest_node->x;
  node_pos.y = nearest_node->y;
  node_pos.z = nearest_node->z;

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
  edge.id = edges_.size() + temp_edges.size();
  edge.from_id = goal_node_id;
  edge.to_id = nearest_node_id;
  edge.weight = calculateDistance(goal_position, node_pos);
  edge.bidirectional = true;
  inheritEdgePolicy(findNearestTopologyEdge(goal_position), edge);
  temp_edges.push_back(edge);

  RCLCPP_WARN(this->get_logger(),
              "[GOAL_TEMP_CONNECT_OK] goal_temp -> node%d, "
              "goal=(%.3f, %.3f), node=(%.3f, %.3f), dist=%.3f, weight=%.3f",
              nearest_node_id, goal_position.x, goal_position.y,
              nearest_node->x, nearest_node->y, nearest_dist, edge.weight);
}

bool GlobalPathPlanner::rebuildGraphIndexes() {
  node_index_.clear();
  base_adjacency_.clear();
  max_node_id_ = -1;

  node_index_.reserve(nodes_.size());
  for (std::size_t index = 0; index < nodes_.size(); ++index) {
    const int node_id = nodes_[index].id;
    if (!node_index_.emplace(node_id, index).second) {
      RCLCPP_ERROR(this->get_logger(), "拓扑地图包含重复节点ID: %d", node_id);
      return false;
    }
    max_node_id_ = std::max(max_node_id_, node_id);
  }

  if (nodes_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "拓扑地图不包含任何节点");
    return false;
  }
  if (max_node_id_ > std::numeric_limits<int>::max() - 2) {
    RCLCPP_ERROR(this->get_logger(), "拓扑节点ID过大，无法分配临时节点ID");
    return false;
  }

  for (const auto &edge : edges_) {
    if (node_index_.find(edge.from_id) == node_index_.end() ||
        node_index_.find(edge.to_id) == node_index_.end()) {
      RCLCPP_ERROR(this->get_logger(), "拓扑边%d引用不存在的节点: %d -> %d",
                   edge.id, edge.from_id, edge.to_id);
      return false;
    }
    if (!std::isfinite(edge.weight) || edge.weight < 0.0) {
      RCLCPP_ERROR(this->get_logger(), "拓扑边%d包含非法权重: %.3f", edge.id,
                   edge.weight);
      return false;
    }
    if (edge.edge_type != "straight" && edge.edge_type != "map_connection" &&
        edge.edge_type != "bezier") {
      RCLCPP_ERROR(this->get_logger(), "拓扑边%d包含未支持的类型: %s", edge.id,
                   edge.edge_type.c_str());
      return false;
    }
    if (!std::isfinite(edge.policy.linear_velocity) ||
        edge.policy.linear_velocity < 0.0 ||
        !std::isfinite(edge.policy.angular_velocity) ||
        edge.policy.angular_velocity < 0.0 ||
        !std::isfinite(edge.policy.obstacle_avoidance_distance_m) ||
        edge.policy.obstacle_avoidance_distance_m < 0.0 ||
        (edge.policy.motion_direction != 1 &&
         edge.policy.motion_direction != -1)) {
      RCLCPP_ERROR(this->get_logger(), "拓扑边%d包含非法边策略参数", edge.id);
      return false;
    }
    if (edge.edge_type == "bezier" && !edge.has_bezier_control_points) {
      RCLCPP_ERROR(this->get_logger(), "贝塞尔边%d缺少两个控制点", edge.id);
      return false;
    }
    if (edge.has_bezier_control_points) {
      for (const auto &point : edge.bezier_control_points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          RCLCPP_ERROR(this->get_logger(), "贝塞尔边%d包含非法控制点", edge.id);
          return false;
        }
      }
    }
  }

  base_adjacency_ = buildAdjacency(edges_);
  return true;
}

GlobalPathPlanner::EdgeAdjacency GlobalPathPlanner::buildAdjacency(
    const std::vector<TopologyEdge> &graph_edges) const {
  EdgeAdjacency adjacency;
  adjacency.reserve(nodes_.size() + 2U);
  for (const auto &edge : graph_edges) {
    // map_connection由slam_manager执行重定位，不属于可行驶的几何边。
    if (edge.edge_type == "map_connection") continue;
    adjacency[edge.from_id].push_back(&edge);
    if (edge.bidirectional && edge.to_id != edge.from_id) {
      adjacency[edge.to_id].push_back(&edge);
    }
  }
  return adjacency;
}

const TopologyNode *GlobalPathPlanner::findNodeById(
    int node_id, const std::vector<TopologyNode> *temporary_nodes) const {
  const auto index_it = node_index_.find(node_id);
  if (index_it != node_index_.end()) {
    return &nodes_[index_it->second];
  }
  if (temporary_nodes != nullptr) {
    for (const auto &node : *temporary_nodes) {
      if (node.id == node_id) {
        return &node;
      }
    }
  }
  return nullptr;
}

const TopologyEdge *GlobalPathPlanner::findTopologyEdge(
    int first_node_id, int second_node_id) const {
  const TopologyEdge *selected = nullptr;
  const auto adjacency_it = base_adjacency_.find(first_node_id);
  if (adjacency_it == base_adjacency_.end()) {
    return nullptr;
  }
  for (const TopologyEdge *edge_ptr : adjacency_it->second) {
    const auto &edge = *edge_ptr;
    const bool forward =
        edge.from_id == first_node_id && edge.to_id == second_node_id;
    const bool reverse = edge.bidirectional && edge.from_id == second_node_id &&
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
    if (edge.edge_type == "map_connection") continue;
    const TopologyNode *from_node = findNodeById(edge.from_id);
    const TopologyNode *to_node = findNodeById(edge.to_id);
    if (from_node == nullptr || to_node == nullptr) {
      continue;
    }

    const double dx = to_node->x - from_node->x;
    const double dy = to_node->y - from_node->y;
    const double dz = to_node->z - from_node->z;
    const double length_squared = dx * dx + dy * dy + dz * dz;
    double interpolation = 0.0;
    if (length_squared > 1e-12) {
      interpolation =
          ((position.x - from_node->x) * dx + (position.y - from_node->y) * dy +
           (position.z - from_node->z) * dz) /
          length_squared;
      interpolation = std::clamp(interpolation, 0.0, 1.0);
    }

    const double nearest_x = from_node->x + interpolation * dx;
    const double nearest_y = from_node->y + interpolation * dy;
    const double nearest_z = from_node->z + interpolation * dz;
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
  destination.policy = source->policy;
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

GlobalPathPlanner::TopologySearchResult
GlobalPathPlanner::dijkstraSearchInTempGraph(
    const std::vector<TopologyNode> &temporary_nodes,
    const EdgeAdjacency &temporary_adjacency, int start_id, int goal_id) {
  TopologySearchResult result;
  std::unordered_map<int, double> distances;
  std::unordered_map<int, int> previous;
  std::unordered_map<int, const TopologyEdge *> previous_edge;
  std::priority_queue<std::pair<double, int>,
                      std::vector<std::pair<double, int>>,
                      std::greater<std::pair<double, int>>>
      pq;

  distances.reserve(nodes_.size() + temporary_nodes.size());
  previous.reserve(nodes_.size() + temporary_nodes.size());
  previous_edge.reserve(nodes_.size() + temporary_nodes.size());
  for (const auto &node : nodes_) {
    distances.emplace(node.id, std::numeric_limits<double>::infinity());
  }
  for (const auto &node : temporary_nodes) {
    distances.emplace(node.id, std::numeric_limits<double>::infinity());
  }
  auto start_it = distances.find(start_id);
  const auto goal_it = distances.find(goal_id);
  if (start_it == distances.end() || goal_it == distances.end()) {
    RCLCPP_ERROR(this->get_logger(), "Dijkstra起点或终点不存在: %d -> %d",
                 start_id, goal_id);
    return result;
  }
  start_it->second = 0.0;
  pq.push(std::make_pair(0.0, start_id));

  while (!pq.empty()) {
    int current_id = pq.top().second;
    double current_dist = pq.top().first;
    pq.pop();

    if (current_id == goal_id) {
      break;
    }

    const auto current_distance_it = distances.find(current_id);
    if (current_distance_it == distances.end() ||
        current_dist > current_distance_it->second) {
      continue;
    }

    const auto relax_edges = [&](const EdgeAdjacency &adjacency) {
      const auto adjacency_it = adjacency.find(current_id);
      if (adjacency_it == adjacency.end()) {
        return;
      }
      for (const TopologyEdge *edge : adjacency_it->second) {
        const int neighbor_id =
            edge->from_id == current_id ? edge->to_id : edge->from_id;
        auto neighbor_distance_it = distances.find(neighbor_id);
        if (neighbor_distance_it == distances.end()) {
          continue;
        }
        const double new_distance = current_dist + edge->weight;
        if (new_distance < neighbor_distance_it->second) {
          neighbor_distance_it->second = new_distance;
          previous[neighbor_id] = current_id;
          previous_edge[neighbor_id] = edge;
          pq.push(std::make_pair(new_distance, neighbor_id));
        }
      }
    };
    relax_edges(base_adjacency_);
    relax_edges(temporary_adjacency);
  }

  if (!std::isfinite(goal_it->second)) {
    RCLCPP_WARN(this->get_logger(), "在临时拓扑图中未找到从 %d 到 %d 的路径",
                start_id, goal_id);
    return result;
  }

  std::vector<int> reversed_nodes;
  std::vector<const TopologyEdge *> reversed_edges;
  int current = goal_id;
  while (current != start_id) {
    reversed_nodes.push_back(current);
    const auto previous_it = previous.find(current);
    const auto edge_it = previous_edge.find(current);
    if (previous_it == previous.end() || edge_it == previous_edge.end()) {
      RCLCPP_ERROR(this->get_logger(), "路径重构失败：节点 %d 没有前驱节点",
                   current);
      return TopologySearchResult{};
    }
    reversed_edges.push_back(edge_it->second);
    current = previous_it->second;
  }
  reversed_nodes.push_back(start_id);

  result.node_ids.assign(reversed_nodes.rbegin(), reversed_nodes.rend());
  result.selected_edges.assign(reversed_edges.rbegin(), reversed_edges.rend());

  RCLCPP_INFO(this->get_logger(), "在临时拓扑图中找到路径，包含 %lu 个节点",
              result.node_ids.size());
  return result;
}

nav_msgs::msg::Path GlobalPathPlanner::buildCompletePathFromTempGraph(
    const std::vector<TopologyNode> &temporary_nodes,
    const TopologySearchResult &topology_path) {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = global_frame_;

  if (topology_path.node_ids.empty()) {
    RCLCPP_WARN(this->get_logger(), "拓扑路径为空，无法构建完整路径");
    return path;
  }
  last_path_edge_policies_.clear();

  std::vector<geometry_msgs::msg::Point> waypoints;
  waypoints.reserve(topology_path.node_ids.size());

  // 将拓扑路径中的所有节点转换为路径点
  for (int node_id : topology_path.node_ids) {
    const TopologyNode *node = findNodeById(node_id, &temporary_nodes);
    if (node != nullptr) {
      geometry_msgs::msg::Point point;
      point.x = node->x;
      point.y = node->y;
      point.z = node->z;
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
  if (topology_path.selected_edges.size() + 1U != waypoints.size()) {
    RCLCPP_ERROR(this->get_logger(),
                 "路径节点和边数量不匹配: nodes=%zu, edges=%zu",
                 waypoints.size(), topology_path.selected_edges.size());
    return path;
  }

  // 每个拓扑段使用一个完整EdgePolicy；临时连接会继承邻近真实边策略。
  std::vector<navigation_interfaces::msg::EdgePolicy> segment_policies;
  segment_policies.reserve(topology_path.selected_edges.size());
  for (const TopologyEdge *edge : topology_path.selected_edges) {
    segment_policies.push_back(edge->policy);
  }

  // 插值生成完整路径
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    if (topology_path.selected_edges[i]->edge_type == "map_connection") {
      RCLCPP_INFO(this->get_logger(), "跳过map_connection虚拟路径段: %d -> %d",
                  topology_path.node_ids[i], topology_path.node_ids[i + 1U]);
      continue;
    }
    std::size_t next_motion_segment = i + 1U;
    while (next_motion_segment < topology_path.selected_edges.size() &&
           topology_path.selected_edges[next_motion_segment]->edge_type ==
               "map_connection") {
      ++next_motion_segment;
    }
    // 除map_connection外的edge_type暂不参与路径几何，统一按直线插值。
    std::vector<geometry_msgs::msg::Point> segment =
        interpolatePath(waypoints[i], waypoints[i + 1U], interpolation_points_);

    for (size_t j = 0; j < segment.size(); ++j) {
      // 避免重复添加相同的点（除了第一个段）
      if (path.poses.empty() || j != 0) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = segment[j];

        // 计算朝向
        if (j < segment.size() - 1) {
          pose.pose.orientation =
              calculateOrientation(segment[j], segment[j + 1]);
        } else if (next_motion_segment < topology_path.selected_edges.size()) {
          pose.pose.orientation = calculateOrientation(
              segment[j], waypoints[next_motion_segment + 1U]);
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
            at_segment_end && next_motion_segment < segment_policies.size()
                ? next_motion_segment
                : i;
        last_path_edge_policies_.push_back(segment_policies[policy_segment]);
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

}  // namespace global_path_planner
