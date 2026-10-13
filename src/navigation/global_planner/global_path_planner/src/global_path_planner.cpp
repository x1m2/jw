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
namespace {

bool pointInPolygon(double x, double y,
                    const std::vector<geometry_msgs::msg::Point> &polygon) {
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1U;
       index < polygon.size(); previous = index++) {
    const auto &first = polygon[previous];
    const auto &second = polygon[index];
    const double cross = (second.x - first.x) * (y - first.y) -
                         (second.y - first.y) * (x - first.x);
    if (std::abs(cross) <= 1e-9 && x >= std::min(first.x, second.x) - 1e-9 &&
        x <= std::max(first.x, second.x) + 1e-9 &&
        y >= std::min(first.y, second.y) - 1e-9 &&
        y <= std::max(first.y, second.y) + 1e-9) {
      return true;
    }
    const bool crosses = (first.y > y) != (second.y > y);
    if (crosses &&
        x < (second.x - first.x) * (y - first.y) / (second.y - first.y) +
                first.x) {
      inside = !inside;
    }
  }
  return inside;
}

geometry_msgs::msg::Point bezierPoint(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &control_point_1,
    const geometry_msgs::msg::Point &control_point_2,
    const geometry_msgs::msg::Point &end, double t) {
  const double u = 1.0 - t;
  const double w0 = u * u * u;
  const double w1 = 3.0 * u * u * t;
  const double w2 = 3.0 * u * t * t;
  const double w3 = t * t * t;
  geometry_msgs::msg::Point point;
  point.x = w0 * start.x + w1 * control_point_1.x + w2 * control_point_2.x +
            w3 * end.x;
  point.y = w0 * start.y + w1 * control_point_1.y + w2 * control_point_2.y +
            w3 * end.y;
  point.z = w0 * start.z + w1 * control_point_1.z + w2 * control_point_2.z +
            w3 * end.z;
  return point;
}

}  // namespace

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
  this->declare_parameter<double>("interpolation_spacing_m", 0.2);

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
  this->get_parameter("interpolation_spacing_m", interpolation_spacing_m_);
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

  // 到点任务偏离重规划参数。
  deviation_replan_enabled_ = declare_parameter("deviation_replan_enabled", true);
  deviation_distance_ = declare_parameter("deviation_replan_distance", 0.5);
  deviation_confirm_sec_ = declare_parameter("deviation_replan_confirm_sec", 0.5);
  replan_interval_sec_ = declare_parameter("deviation_replan_interval_sec", 1.0);
  replan_goal_tolerance_ = declare_parameter("deviation_replan_goal_tolerance", 0.2);
  replan_pose_timeout_sec_ = declare_parameter("deviation_replan_pose_timeout_sec", 0.5);
  resume_replan_distance_ = declare_parameter("resume_replan_distance", 0.25);
  if (!std::isfinite(deviation_distance_) || deviation_distance_ <= 0.0 ||
      !std::isfinite(deviation_confirm_sec_) || deviation_confirm_sec_ < 0.0 ||
      !std::isfinite(replan_interval_sec_) || replan_interval_sec_ <= 0.0 ||
      !std::isfinite(replan_goal_tolerance_) || replan_goal_tolerance_ < 0.0 ||
      !std::isfinite(replan_pose_timeout_sec_) || replan_pose_timeout_sec_ <= 0.0 ||
      !std::isfinite(resume_replan_distance_) || resume_replan_distance_ < 0.0) {
    throw std::invalid_argument("Invalid deviation replan parameters");
  }
  // 初始化 TF2 和 PCL 相关对象。
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

  // 时间戳是控制器的路径接收下限，等待对应新路径通过平滑并被接收后才恢复。
  replan_request_pub_ = create_publisher<std_msgs::msg::Header>(
      "/navigation/global_replan_request", rclcpp::QoS(10).reliable());
  paused_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/pure_pursuit/paused", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        const bool was_paused = navigation_paused_;
        navigation_paused_ = msg->data;
        deviation_since_.reset();
        if (msg->data) {
          pause_position_valid_ = has_current_pose_;
          if (pause_position_valid_) pause_position_ = current_position_;
          resume_replan_check_pending_ = false;
          resume_replan_check_deadline_.reset();
        } else if (was_paused && point_task_active_ && has_goal_ &&
                   pause_position_valid_) {
          // 位姿话题和暂停服务异步到达；留出短暂窗口接收人工挪动后的定位。
          resume_replan_check_pending_ = true;
          resume_replan_check_deadline_ = now() + rclcpp::Duration::from_seconds(1.0);
          checkResumeReplan();
        }
      });
  avoidance_state_sub_ = create_subscription<navigation_interfaces::msg::AvoidanceState>(
      "/local_avoidance/state", rclcpp::QoS(1).reliable().transient_local(),
      [this](navigation_interfaces::msg::AvoidanceState::ConstSharedPtr msg) {
        avoidance_state_ = msg->state;
      });
  nav_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/nav_state", 10, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        // 避免上一目标延迟到达的完成通知终止当前到点任务。
        if (msg->data && has_goal_ && has_current_pose_ &&
            std::hypot(current_position_.x - goal_position_.x,
                       current_position_.y - goal_position_.y) <= replan_goal_tolerance_) {
          resetDeviationTracking(false);
          has_goal_ = false;
          planning_pending_ = false;
        }
      });
  path_accepted_sub_ = create_subscription<std_msgs::msg::Header>(
      "/navigation/global_path_accepted", 10,
      [this](std_msgs::msg::Header::ConstSharedPtr msg) {
        if (msg->frame_id == global_frame_ &&
            rclcpp::Time(msg->stamp).nanoseconds() == last_published_path_stamp_ &&
            last_published_path_stamp_ >= replan_request_stamp_) {
          replan_pending_ = false;
          deviation_since_.reset();
        }
      });
  smoothed_path_sub_ = create_subscription<navigation_interfaces::msg::PathWithPolicy>(
      "/smoothed_global_path_with_policy", 10,
      [this](navigation_interfaces::msg::PathWithPolicy::ConstSharedPtr msg) {
        if (!msg->dynamic_path && !msg->path.poses.empty() &&
            msg->path.header.frame_id == global_frame_ &&
            rclcpp::Time(msg->path.header.stamp).nanoseconds() == last_published_path_stamp_) {
          // 检测实际跟踪的平滑路径，避免把正常的弯道平滑误判为偏离。
          active_path_ = msg->path;
        }
      });

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

  task_target_sub_ =
      create_subscription<slam_manager_interfaces::msg::TaskTarget>(
          "/navigation/task_target", 10,
          [this](slam_manager_interfaces::msg::TaskTarget::ConstSharedPtr msg) {
            resetDeviationTracking(!msg->use_topology);
            planning_pending_ = false;
            has_goal_ = false;
            task_target_.reset();
            if (msg->use_topology) {
              const auto *goal =
                  msg->target.node_id <=
                          static_cast<uint32_t>(std::numeric_limits<int>::max())
                      ? findNodeById(static_cast<int>(msg->target.node_id))
                      : nullptr;
              if (!goal) {
                RCLCPP_ERROR(get_logger(), "Unknown patrol target ID: %u",
                             msg->target.node_id);
                return;
              }
              task_target_ = *msg;
              goal_position_.x = goal->x;
              goal_position_.y = goal->y;
              goal_position_.z = goal->z;
            } else {
              goal_position_.x = msg->target.x;
              goal_position_.y = msg->target.y;
              goal_position_.z = msg->target.z;
            }
            has_goal_ = true;
            planAndPublish();
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
  regions_.clear();

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
      edge.bidirectional = json_edge.value("bidirectional", false);
      edge.policy.velocity_limit_enabled = true;
      edge.policy.linear_velocity = json_edge.value("linear_velocity", 0.5);
      edge.policy.angular_velocity = json_edge.value("angular_velocity", 0.5);
      edge.policy.motion_direction = json_edge.value("motion_direction", 1);
      edge.policy.obstacle_stop_enabled = true;
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

    if (document.contains("regions")) {
      if (!document["regions"].is_array()) {
        throw std::runtime_error("regions must be an array");
      }
      std::unordered_set<int> region_ids;
      regions_.reserve(document["regions"].size());
      for (const auto &json_region : document["regions"]) {
        TopologyRegion region;
        region.id = json_region.at("id").get<int>();
        region.region_type = json_region.at("region_type").get<std::string>();
        region.enabled = json_region.value("enabled", true);
        if (region.id < 0 || !region_ids.insert(region.id).second ||
            (region.region_type != "obstacle_avoidance" &&
             region.region_type != "speed_limit" &&
             region.region_type != "stop")) {
          throw std::runtime_error(
              "region has an invalid/duplicate id or unsupported type");
        }
        const auto &polygon = json_region.at("polygon");
        if (!polygon.is_array() || polygon.size() < 3U) {
          throw std::runtime_error("region polygon requires at least 3 points");
        }
        for (const auto &json_point : polygon) {
          geometry_msgs::msg::Point point;
          point.x = json_point.at("x").get<double>();
          point.y = json_point.at("y").get<double>();
          if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            throw std::runtime_error("region polygon contains invalid point");
          }
          region.polygon.push_back(point);
        }
        if (json_region.contains("param")) {
          const auto &parameters = json_region.at("param");
          if (!parameters.is_object()) {
            throw std::runtime_error("region param must be an object");
          }
          if (parameters.contains("linear_velocity")) {
            region.linear_velocity =
                parameters.at("linear_velocity").get<double>();
          }
          if (parameters.contains("angular_velocity")) {
            region.angular_velocity =
                parameters.at("angular_velocity").get<double>();
          }
          if (parameters.contains("obstacle_avoidance_distance_m")) {
            region.obstacle_avoidance_distance_m =
                parameters.at("obstacle_avoidance_distance_m").get<double>();
          }
        }
        const auto valid = [](const std::optional<double> &value) {
          return !value || (std::isfinite(*value) && *value >= 0.0);
        };
        if (!valid(region.linear_velocity) || !valid(region.angular_velocity) ||
            !valid(region.obstacle_avoidance_distance_m)) {
          throw std::runtime_error("region contains an invalid param");
        }
        regions_.push_back(std::move(region));
      }
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(this->get_logger(), "拓扑JSON解析失败: %s", error.what());
    nodes_.clear();
    edges_.clear();
    regions_.clear();
    return false;
  }

  if (!rebuildGraphIndexes()) {
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
              "拓扑JSON加载完成: %lu 个节点, %lu 条边, %lu 个区域",
              nodes_.size(), edges_.size(), regions_.size());
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

    const auto &position = pose_out.pose.position;
    const rclcpp::Time pose_stamp(pose_out.header.stamp);
    const double pose_age = (now() - pose_stamp).seconds();
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || pose_stamp.nanoseconds() == 0 ||
        pose_age > replan_pose_timeout_sec_ || pose_age < -replan_pose_timeout_sec_) {
      deviation_since_.reset();
      has_current_pose_ = false;
      return;
    }
    if (last_pose_stamp_ &&
        ((pose_stamp - *last_pose_stamp_).seconds() > replan_pose_timeout_sec_ ||
         pose_stamp <= *last_pose_stamp_)) {
      deviation_since_.reset();
    }
    current_position_.x = pose_out.pose.position.x;
    current_position_.y = pose_out.pose.position.y;
    current_position_.z = pose_out.pose.position.z;
    has_current_pose_ = true;
    last_pose_stamp_ = pose_stamp;
    checkResumeReplan();

    const double yaw = tf2::getYaw(pose_out.pose.orientation);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "SLAM  [%s]: x=%.3f, y=%.3f, z=%.3f, yaw=%.2f deg",
                         pose_out.header.frame_id.c_str(), current_position_.x,
                         current_position_.y, current_position_.z,
                         yaw * 180.0 / M_PI);
    if (planning_pending_) {
      RCLCPP_INFO(this->get_logger(), "已收到有效定位，执行待处理的目标规划");
      planAndPublish();
    } else {
      checkPathDeviation();
    }
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "实时位姿坐标变换失败: %s", ex.what());
  }
}

void GlobalPathPlanner::goalCallback(
    geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
  task_target_.reset();
  RCLCPP_INFO(this->get_logger(), "收到点击目标点");
  resetDeviationTracking(true);
  has_goal_ = false;

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

void GlobalPathPlanner::resetDeviationTracking(bool point_task) {
  point_task_active_ = point_task;
  replan_pending_ = false;
  replan_request_stamp_ = 0;
  deviation_since_.reset();
  last_replan_attempt_.reset();
  active_path_.poses.clear();
  pause_position_valid_ = false;
  resume_replan_check_pending_ = false;
  resume_replan_check_deadline_.reset();
}

double GlobalPathPlanner::distanceToActivePath() const {
  double best = std::numeric_limits<double>::infinity();
  // XY 点到线段距离；不因点间插值稀疏或定位高度轻微波动触发重规划。
  for (std::size_t i = 0; i < active_path_.poses.size(); ++i) {
    const auto &a = active_path_.poses[i].pose.position;
    const auto &b = active_path_.poses[std::min(i + 1, active_path_.poses.size() - 1)].pose.position;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double t = length_sq > 1e-12
        ? std::clamp(((current_position_.x - a.x) * dx +
                      (current_position_.y - a.y) * dy) / length_sq, 0.0, 1.0)
        : 0.0;
    best = std::min(best, std::hypot(current_position_.x - a.x - t * dx,
                                   current_position_.y - a.y - t * dy));
  }
  return best;
}

void GlobalPathPlanner::checkPathDeviation() {
  if (!deviation_replan_enabled_ || !point_task_active_ || !has_goal_ ||
      navigation_paused_ || active_path_.poses.empty()) {
    deviation_since_.reset();
    return;
  }
  const auto time = now();
  // ROS 时钟回退时重建确认窗口，避免冷却条件永远无法满足。
  if (last_replan_attempt_ && time < *last_replan_attempt_) last_replan_attempt_.reset();
  if (deviation_since_ && time < *deviation_since_) deviation_since_.reset();
  const double distance = distanceToActivePath();
  if (!replan_pending_) {
    if (avoidance_state_ != navigation_interfaces::msg::AvoidanceState::IDLE ||
        std::hypot(current_position_.x - goal_position_.x,
                   current_position_.y - goal_position_.y) <= replan_goal_tolerance_ ||
        distance <= deviation_distance_) {
      deviation_since_.reset();
      return;
    }
    if (!deviation_since_) deviation_since_ = time;
    if ((time - *deviation_since_).seconds() < deviation_confirm_sec_) return;
  }
  if (last_replan_attempt_ &&
      (time - *last_replan_attempt_).seconds() < replan_interval_sec_) return;

  requestGlobalReplan(distance, "deviation");
}

void GlobalPathPlanner::checkResumeReplan() {
  if (!resume_replan_check_pending_ || navigation_paused_ ||
      !pause_position_valid_ || !has_current_pose_) return;
  const auto time = now();
  const double distance = std::hypot(current_position_.x - pause_position_.x,
                                     current_position_.y - pause_position_.y);
  if (distance > resume_replan_distance_) {
    resume_replan_check_pending_ = false;
    resume_replan_check_deadline_.reset();
    RCLCPP_WARN(get_logger(),
                "[RESUME_REPLAN] manual displacement=%.3f m exceeds %.3f m",
                distance, resume_replan_distance_);
    requestGlobalReplan(distance, "resume");
    return;
  }
  if (resume_replan_check_deadline_ && time >= *resume_replan_check_deadline_) {
    resume_replan_check_pending_ = false;
    resume_replan_check_deadline_.reset();
    pause_position_valid_ = false;
  }
}

void GlobalPathPlanner::requestGlobalReplan(double distance,
                                            const char *reason) {
  const auto time = now();
  replan_pending_ = true;
  last_replan_attempt_ = time;
  std_msgs::msg::Header request;
  request.frame_id = global_frame_;
  request.stamp = rclcpp::Time(
      std::max(time.nanoseconds(), last_published_path_stamp_ + 1),
      get_clock()->get_clock_type());
  replan_request_stamp_ = rclcpp::Time(request.stamp).nanoseconds();
  replan_request_pub_->publish(request);
  RCLCPP_WARN(get_logger(),
              "[GLOBAL_REPLAN] reason=%s distance=%.3f, start=(%.3f, %.3f), goal=(%.3f, %.3f)",
              reason, distance, current_position_.x, current_position_.y,
              goal_position_.x, goal_position_.y);
  // 请求先锁停旧路径；规划失败时保留等待状态，后续定位周期按间隔重试。
  planAndPublish();
}

void GlobalPathPlanner::planAndPublish() {
  if (!has_goal_) return;
  if (!has_current_pose_ || !last_pose_stamp_ ||
      (now() - *last_pose_stamp_).seconds() > replan_pose_timeout_sec_) {
    planning_pending_ = true;
    RCLCPP_WARN(this->get_logger(),
                "尚未接收到当前位置信息，已保留目标，等待有效定位后规划");
    return;
  }

  // 每个请求只补执行一次，规划失败也不随定位消息反复重试。
  planning_pending_ = false;
  nav_msgs::msg::Path global_path = planGlobalPath();
  if (global_path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "无法规划路径");
    return;
  }

  // 成功版本时间戳严格递增，供平滑器/控制器识别旧路径。
  const auto stamp = rclcpp::Time(
      std::max(now().nanoseconds(), last_published_path_stamp_ + 1),
      get_clock()->get_clock_type());
  global_path.header.stamp = stamp;
  last_published_path_stamp_ = stamp.nanoseconds();
  active_path_ = global_path;
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

  // 巡检段的起终点由 manager 指定，复用已有边插值，不重新选择附近节点。
  if (task_target_ && task_target_->has_from_node) {
    const auto from = task_target_->from_node_id;
    const auto to = task_target_->target.node_id;
    if (from > static_cast<uint32_t>(std::numeric_limits<int>::max()))
      return path;
    const auto *edge =
        findTopologyEdge(static_cast<int>(from), static_cast<int>(to));
    if (!edge || edge->edge_type == "map_connection") {
      RCLCPP_ERROR(get_logger(), "Missing drivable patrol edge: %u -> %u", from,
                   to);
      return path;
    }
    TopologySearchResult route;
    route.node_ids = {static_cast<int>(from), static_cast<int>(to)};
    route.selected_edges = {edge};
    return buildCompletePathFromTempGraph({}, route);
  }

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

    if (!task_target_ && enable_direct_connection_ &&
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

        // 正常巡检按相邻节点逐段下发目标。起点临时节点会连接最近节点的
        // 下一层邻居，因此可能直接替代一条真实拓扑边。若被替代的是
        // 贝塞尔边，需要同时继承它的几何，不能退化成临时直线。
        if (candidate_id != nearest_legal_node_id && policy_edge != nullptr &&
            policy_edge->edge_type == "bezier" &&
            policy_edge->has_bezier_control_points) {
          const bool follows_edge_forward =
              policy_edge->from_id == nearest_legal_node_id &&
              policy_edge->to_id == candidate_id;
          const bool follows_edge_reverse =
              policy_edge->bidirectional &&
              policy_edge->to_id == nearest_legal_node_id &&
              policy_edge->from_id == candidate_id;
          if (follows_edge_forward || follows_edge_reverse) {
            edge.edge_type = "bezier";
            edge.has_bezier_control_points = true;
            edge.bezier_control_points[0] =
                follows_edge_forward ? policy_edge->bezier_control_points[0]
                                     : policy_edge->bezier_control_points[1];
            edge.bezier_control_points[1] =
                follows_edge_forward ? policy_edge->bezier_control_points[1]
                                     : policy_edge->bezier_control_points[0];
          }
        }
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

  if (task_target_) {
    nearest_node_id = static_cast<int>(task_target_->target.node_id);
    nearest_dist = 0.0;
  } else {
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

const TopologyRegion *GlobalPathPlanner::findRegion(
    const geometry_msgs::msg::Point &point) const {
  for (const auto &region : regions_) {
    if (region.enabled && pointInPolygon(point.x, point.y, region.polygon)) {
      return &region;
    }
  }
  return nullptr;
}

navigation_interfaces::msg::EdgePolicy GlobalPathPlanner::applyRegionPolicy(
    const navigation_interfaces::msg::EdgePolicy &edge_policy,
    const geometry_msgs::msg::Point &point) const {
  auto policy = edge_policy;
  const TopologyRegion *region = findRegion(point);
  if (region == nullptr) return policy;

  if (region->linear_velocity) {
    policy.velocity_limit_enabled = true;
    policy.linear_velocity = *region->linear_velocity;
  }
  if (region->angular_velocity) {
    policy.velocity_limit_enabled = true;
    policy.angular_velocity = *region->angular_velocity;
  }
  if (region->obstacle_avoidance_distance_m) {
    policy.obstacle_avoidance_distance_m =
        *region->obstacle_avoidance_distance_m;
  }
  if (region->region_type == "obstacle_avoidance") {
    policy.obstacle_avoidance_enabled = true;
  } else if (region->region_type == "stop") {
    policy.obstacle_stop_enabled = true;
    policy.obstacle_avoidance_enabled = false;
  }
  return policy;
}

std::vector<geometry_msgs::msg::Point> GlobalPathPlanner::interpolatePath(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &end) {
  std::vector<geometry_msgs::msg::Point> points;
  const std::size_t intervals = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(calculateDistance(start, end) /
                                             interpolation_spacing_m_)));
  points.reserve(intervals + 1U);
  for (std::size_t i = 0; i <= intervals; ++i) {
    const double t = static_cast<double>(i) / intervals;

    geometry_msgs::msg::Point point;
    point.x = start.x + t * (end.x - start.x);
    point.y = start.y + t * (end.y - start.y);
    point.z = start.z + t * (end.z - start.z);

    points.push_back(point);
  }

  return points;
}

std::vector<geometry_msgs::msg::Point> GlobalPathPlanner::interpolateBezierPath(
    const geometry_msgs::msg::Point &start,
    const geometry_msgs::msg::Point &control_point_1,
    const geometry_msgs::msg::Point &control_point_2,
    const geometry_msgs::msg::Point &end) {
  std::vector<geometry_msgs::msg::Point> points;
  // 先用较密的折线估算弧长，再反查参数 t；最终点仍落在贝塞尔曲线上。
  const double control_length =
      calculateDistance(start, control_point_1) +
      calculateDistance(control_point_1, control_point_2) +
      calculateDistance(control_point_2, end);
  const std::size_t estimate_intervals = std::max<std::size_t>(
      32U, static_cast<std::size_t>(
               std::ceil(4.0 * control_length / interpolation_spacing_m_)));
  std::vector<double> cumulative_length(estimate_intervals + 1U, 0.0);
  auto previous = start;
  for (std::size_t i = 1; i <= estimate_intervals; ++i) {
    const auto current =
        bezierPoint(start, control_point_1, control_point_2, end,
                    static_cast<double>(i) / estimate_intervals);
    cumulative_length[i] =
        cumulative_length[i - 1U] + calculateDistance(previous, current);
    previous = current;
  }
  const double length = cumulative_length.back();
  const std::size_t intervals = std::max<std::size_t>(
      1U,
      static_cast<std::size_t>(std::ceil(length / interpolation_spacing_m_)));
  points.reserve(intervals + 1U);
  points.push_back(start);
  for (std::size_t i = 1; i < intervals; ++i) {
    const double target = length * static_cast<double>(i) / intervals;
    const auto upper = std::lower_bound(cumulative_length.begin(),
                                        cumulative_length.end(), target);
    const std::size_t upper_index =
        static_cast<std::size_t>(upper - cumulative_length.begin());
    const double lower_length = cumulative_length[upper_index - 1U];
    const double span = *upper - lower_length;
    const double fraction = span > 0.0 ? (target - lower_length) / span : 0.0;
    const double t =
        (static_cast<double>(upper_index - 1U) + fraction) / estimate_intervals;
    points.push_back(
        bezierPoint(start, control_point_1, control_point_2, end, t));
  }
  points.push_back(end);
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
    const TopologyEdge *selected_edge = topology_path.selected_edges[i];
    std::vector<geometry_msgs::msg::Point> segment;
    if (selected_edge->edge_type == "bezier" &&
        selected_edge->has_bezier_control_points) {
      const bool traversing_forward =
          selected_edge->from_id == topology_path.node_ids[i] &&
          selected_edge->to_id == topology_path.node_ids[i + 1U];
      const bool traversing_reverse =
          selected_edge->bidirectional &&
          selected_edge->to_id == topology_path.node_ids[i] &&
          selected_edge->from_id == topology_path.node_ids[i + 1U];
      if (!traversing_forward && !traversing_reverse) {
        RCLCPP_ERROR(this->get_logger(),
                     "贝塞尔边%d与路径节点方向不匹配: %d -> %d",
                     selected_edge->id, topology_path.node_ids[i],
                     topology_path.node_ids[i + 1U]);
        return nav_msgs::msg::Path();
      }

      // 反向遍历同一条三次贝塞尔边时必须交换两个控制点，才能保持
      // 与正向完全相同的几何曲线。
      const auto &first_control = traversing_forward
                                      ? selected_edge->bezier_control_points[0]
                                      : selected_edge->bezier_control_points[1];
      const auto &second_control =
          traversing_forward ? selected_edge->bezier_control_points[1]
                             : selected_edge->bezier_control_points[0];
      segment = interpolateBezierPath(waypoints[i], first_control,
                                      second_control, waypoints[i + 1U]);
      RCLCPP_INFO(this->get_logger(),
                  "生成贝塞尔路径段: edge=%d, direction=%s, points=%zu",
                  selected_edge->id, traversing_forward ? "forward" : "reverse",
                  segment.size());
    } else {
      segment = interpolatePath(waypoints[i], waypoints[i + 1U]);
    }

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
        last_path_edge_policies_.push_back(applyRegionPolicy(
            segment_policies[policy_segment], pose.pose.position));
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
