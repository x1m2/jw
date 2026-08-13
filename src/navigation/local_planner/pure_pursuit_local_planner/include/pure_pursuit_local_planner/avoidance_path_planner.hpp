#ifndef AVOIDANCE_PATH_PLANNER_HPP
#define AVOIDANCE_PATH_PLANNER_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pure_pursuit_local_planner/astar_planner.hpp"
#include "pure_pursuit_local_planner/avoidance_types.hpp"
#include "pure_pursuit_local_planner/costmap_types.hpp"

namespace pure_pursuit_local_planner {

struct AvoidancePathPlannerConfig {
  double lateral_clearance{0.8}; // 绕障点相对路径的横向净空
  double return_goal_clearance{0.8}; // 回归点周围的机器人安全半径
  double return_obstacle_clearance{2.0}; // 回归点到本次障碍中心的最小距离
  double obstacle_threshold{50.0}; // Costmap障碍阈值
};

// 同步绕障路径计算层：负责目标点几何计算和A*调用。
// 本类不维护绕障阶段，也不创建规划线程；这些运行状态由Manager统一管理。
class AvoidancePathPlanner {
public:
  bool initialize(rclcpp::Node::SharedPtr node,
                  const AvoidancePathPlannerConfig &config);

  void setGlobalPath(const nav_msgs::msg::Path &global_path);
  void updateCostmap(CostmapConstPtr costmap);
  bool prepareLatestCostmap();
  bool hasPreparedCostmap() const;

  // 生成一个侧向临时点，并通过沿路径前移限制机器人切入角。
  bool buildSideGoal(const geometry_msgs::msg::PoseStamped &robot_pose,
                     const geometry_msgs::msg::Point &obstacle_center,
                     AvoidanceSide side,
                     const nav_msgs::msg::Path &global_path, int closest_index,
                     geometry_msgs::msg::Point &side_goal,
                     double &side_goal_yaw, double lateral_clearance,
                     double max_entry_angle);

  // 沿全局路径加密查找最近安全回归点。直线安全时直接返回插值直线，
  // 否则使用A*作为同一候选点的兜底路径。
  bool buildReturnPath(const geometry_msgs::msg::Point &start_point,
                       int closest_index,
                       const geometry_msgs::msg::Point &obstacle_center,
                       const nav_msgs::msg::Path &global_path,
                       geometry_msgs::msg::Point &return_goal,
                       nav_msgs::msg::Path &return_path,
                       double max_return_angle,
                       double required_obstacle_clearance,
                       bool &used_direct_path);

  // 以下接口是对底层AStarPlanner的语义封装，避免Manager直接依赖A*实现。
  bool planPath(const geometry_msgs::msg::Point &start,
                const geometry_msgs::msg::Point &goal,
                nav_msgs::msg::Path &path,
                double goal_yaw = std::numeric_limits<double>::quiet_NaN());
  bool isPointInObstacle(const geometry_msgs::msg::Point &point);
  bool isPointInObstacleWithDiagnostics(
      const geometry_msgs::msg::Point &point, const char *point_name,
      double yaw = 0.0);
  bool isPathCollisionFree(const nav_msgs::msg::Path &path) const;
  // 候选走廊不能进入控制阶段的近障停车半径。返回整条折线路径到
  // 本次障碍中心的最小距离，便于候选淘汰日志说明原因。
  bool isPathOutsideObstacleRadius(
      const nav_msgs::msg::Path &path,
      const geometry_msgs::msg::Point &obstacle_center,
      double required_clearance, double *minimum_clearance = nullptr) const;
  void publishVisualization(const geometry_msgs::msg::Point &start,
                            const geometry_msgs::msg::Point &goal,
                            const nav_msgs::msg::Path &path);

private:
  bool getPathYawAtIndex(const nav_msgs::msg::Path &path, int index,
                         double &yaw) const;
  bool isPointSafeForRobot(const geometry_msgs::msg::Point &point,
                           double clearance_radius) const;
  static double calculateDistance(const geometry_msgs::msg::Point &p1,
                                  const geometry_msgs::msg::Point &p2);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<AStarPlanner> astar_planner_;
  AvoidancePathPlannerConfig config_;

};

} // namespace pure_pursuit_local_planner

#endif // AVOIDANCE_PATH_PLANNER_HPP
