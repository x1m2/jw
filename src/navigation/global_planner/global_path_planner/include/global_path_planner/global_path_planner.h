#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/edge_policy_utils.hpp>
#include <navigation_interfaces/msg/edge_policy.hpp>
#include <navigation_interfaces/msg/path_with_policy.hpp>
#include <navigation_interfaces/msg/avoidance_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <slam_manager_interfaces/msg/task_target.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <vector>

namespace global_path_planner {

struct TopologyNode {
  int id;
  double x, y, z;
  std::string label;

  TopologyNode() : id(0), x(0.0), y(0.0), z(0.0), label("") {}
  TopologyNode(int _id, double _x, double _y, double _z,
               const std::string &_label)
      : id(_id), x(_x), y(_y), z(_z), label(_label) {}
};

struct TopologyEdge {
  int id;
  int from_id;
  int to_id;
  double weight;
  std::string edge_type;
  bool bidirectional;
  navigation_interfaces::msg::EdgePolicy policy;
  bool has_bezier_control_points;
  std::array<geometry_msgs::msg::Point, 2> bezier_control_points;

  TopologyEdge()
      : id(0),
        from_id(0),
        to_id(0),
        weight(0.0),
        edge_type("straight"),
        bidirectional(false),
        policy(navigation_interfaces::makeDefaultEdgePolicy()),
        has_bezier_control_points(false) {}
  TopologyEdge(int _id, int _from, int _to, double _weight, bool _bi)
      : id(_id),
        from_id(_from),
        to_id(_to),
        weight(_weight),
        edge_type("straight"),
        bidirectional(_bi),
        policy(navigation_interfaces::makeDefaultEdgePolicy()),
        has_bezier_control_points(false) {}
};

struct TopologyRegion {
  int id{0};
  std::string region_type;
  bool enabled{true};
  std::vector<geometry_msgs::msg::Point> polygon;
  std::optional<double> linear_velocity;
  std::optional<double> angular_velocity;
  std::optional<double> obstacle_avoidance_distance_m;
};

class GlobalPathPlanner : public rclcpp::Node {
 public:
  explicit GlobalPathPlanner(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~GlobalPathPlanner();

  bool initialize();

 private:
  using EdgeAdjacency =
      std::unordered_map<int, std::vector<const TopologyEdge *>>;

  struct TopologySearchResult {
    std::vector<int> node_ids;
    std::vector<const TopologyEdge *> selected_edges;
  };

  // ROS2组件
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<navigation_interfaces::msg::PathWithPolicy>::SharedPtr
      global_path_with_policy_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 参数
  std::string map_file_path_;
  std::string global_frame_;
  std::string robot_frame_;
  std::string
      pose_topic_;  // 实时定位位姿话题名，类型 geometry_msgs/msg/PoseStamped
  double connection_distance_threshold_;
  double interpolation_spacing_m_;
  double path_z_offset_;
  bool path_interpolation_enable_;
  double path_interpolation_z_threshold_;
  int path_interpolation_insert_count_;
  double path_interpolation_start_offset_;
  double path_interpolation_end_offset_;

  bool enable_direct_connection_;
  double direct_connection_threshold_;
  double direct_connection_cost_ratio_;
  // 边合法性检查参数
  double robot_height_;              // 机器狗高度
  int edge_sample_count_;            // 边采样点数量
  double safety_sphere_radius_;      // 安全球半径
  double valid_point_threshold_;     // 有效点阈值百分比
  std::string free_space_pcd_file_;  // 可行域点云 PCD 文件
  bool
      enable_temp_connection_feasibility_check_;  // 是否对临时连接启用可行域检查

  // 可行域点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_space_cloud_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_;
  bool has_free_space_cloud_;

  // 数据
  std::vector<TopologyNode> nodes_;
  std::vector<TopologyEdge> edges_;
  std::vector<TopologyRegion> regions_;
  std::unordered_map<int, std::size_t> node_index_;
  EdgeAdjacency base_adjacency_;
  int max_node_id_{-1};
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point goal_position_;
  bool has_current_pose_;
  bool has_goal_;
  std::optional<slam_manager_interfaces::msg::TaskTarget> task_target_;
  rclcpp::Subscription<slam_manager_interfaces::msg::TaskTarget>::SharedPtr
      task_target_sub_;
  bool planning_pending_{false};  // 目标先到时，等待有效定位后补规划一次。
  std::vector<navigation_interfaces::msg::EdgePolicy> last_path_edge_policies_;

  // 到点任务偏离重规划；巡检指定边不参与。确认与重试使用节点时钟。
  bool deviation_replan_enabled_{true};
  double deviation_distance_{0.5};
  double deviation_confirm_sec_{0.5};
  double replan_interval_sec_{1.0};
  double replan_goal_tolerance_{0.2};
  double replan_pose_timeout_sec_{0.5};
  double resume_replan_distance_{0.25};
  bool point_task_active_{false};
  bool navigation_paused_{false};
  bool pause_position_valid_{false};
  bool resume_replan_check_pending_{false};
  geometry_msgs::msg::Point pause_position_;
  std::optional<rclcpp::Time> resume_replan_check_deadline_;
  bool replan_pending_{false};
  uint8_t avoidance_state_{navigation_interfaces::msg::AvoidanceState::IDLE};
  std::optional<rclcpp::Time> deviation_since_;
  std::optional<rclcpp::Time> last_replan_attempt_;
  std::optional<rclcpp::Time> last_pose_stamp_;
  nav_msgs::msg::Path active_path_;
  int64_t last_published_path_stamp_{0};
  int64_t replan_request_stamp_{0};
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr replan_request_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr paused_sub_;
  rclcpp::Subscription<navigation_interfaces::msg::AvoidanceState>::SharedPtr
      avoidance_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr path_accepted_sub_;
  rclcpp::Subscription<navigation_interfaces::msg::PathWithPolicy>::SharedPtr
      smoothed_path_sub_;

  void resetDeviationTracking(bool point_task);
  void checkResumeReplan();
  void checkPathDeviation();
  void requestGlobalReplan(double distance, const char *reason);
  double distanceToActivePath() const;

  // 方法
  bool loadTopologyMap();
  void poseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
  void goalCallback(geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
  void planAndPublish();

  // 路径规划核心方法
  nav_msgs::msg::Path planGlobalPath();
  // 路径构建方法
  std::vector<double> interpolationRatios(
      const geometry_msgs::msg::Point &start,
      const geometry_msgs::msg::Point &end, std::size_t intervals,
      bool insert_waypoints = true);
  std::vector<geometry_msgs::msg::Point> interpolatePath(
      const geometry_msgs::msg::Point &start,
      const geometry_msgs::msg::Point &end);
  // 贝塞尔曲线插值方法
  std::vector<geometry_msgs::msg::Point> interpolateBezierPath(
      const geometry_msgs::msg::Point &start,
      const geometry_msgs::msg::Point &control_point_1,
      const geometry_msgs::msg::Point &control_point_2,
      const geometry_msgs::msg::Point &end);
  // 工具方法
  double calculateDistance(const geometry_msgs::msg::Point &p1,
                           const geometry_msgs::msg::Point &p2);
  geometry_msgs::msg::Quaternion calculateOrientation(
      const geometry_msgs::msg::Point &from,
      const geometry_msgs::msg::Point &to);
  bool loadFreeSpaceCloudFromPcd();

  // 临时拓扑图相关方法
  void addTemporaryConnections(std::vector<TopologyEdge> &temp_edges,
                               int temp_node_id,
                               const geometry_msgs::msg::Point &position);
  void addGoalTemporaryConnection(
      std::vector<TopologyEdge> &temp_edges, int goal_node_id,
      const geometry_msgs::msg::Point &goal_position);
  TopologySearchResult dijkstraSearchInTempGraph(
      const std::vector<TopologyNode> &temporary_nodes,
      const EdgeAdjacency &temporary_adjacency, int start_id, int goal_id);
  nav_msgs::msg::Path buildCompletePathFromTempGraph(
      const std::vector<TopologyNode> &temporary_nodes,
      const TopologySearchResult &topology_path);
  bool rebuildGraphIndexes();
  EdgeAdjacency buildAdjacency(
      const std::vector<TopologyEdge> &graph_edges) const;
  const TopologyNode *findNodeById(
      int node_id,
      const std::vector<TopologyNode> *temporary_nodes = nullptr) const;
  const TopologyEdge *findTopologyEdge(int first_node_id,
                                       int second_node_id) const;
  const TopologyEdge *findNearestTopologyEdge(
      const geometry_msgs::msg::Point &position) const;
  static void inheritEdgePolicy(const TopologyEdge *source,
                                TopologyEdge &destination);
  const TopologyRegion *findRegion(
      const geometry_msgs::msg::Point &point) const;
  navigation_interfaces::msg::EdgePolicy applyRegionPolicy(
      const navigation_interfaces::msg::EdgePolicy &edge_policy,
      const geometry_msgs::msg::Point &point) const;

  // 新的边合法性检查方法
  bool isEdgeLegal(const TopologyNode &node1, const TopologyNode &node2);
  bool isPointInFreeSpaceWithRadius(const geometry_msgs::msg::Point &point,
                                    double radius);
  std::vector<geometry_msgs::msg::Point> sampleEdgePoints(
      const geometry_msgs::msg::Point &start,
      const geometry_msgs::msg::Point &end, int sample_count);
};

}  // namespace global_path_planner

#endif  // GLOBAL_PATH_PLANNER_H
