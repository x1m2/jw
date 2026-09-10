#ifndef AVOIDANCE_PATH_PLANNER_HPP
#define AVOIDANCE_PATH_PLANNER_HPP

#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "pure_pursuit/astar_planner.hpp"
#include "pure_pursuit/avoidance_types.hpp"
#include "pure_pursuit/costmap_types.hpp"

namespace pure_pursuit_local_planner {

struct AvoidancePathPlannerConfig {
  double lateral_clearance{0.8}; // 绕障点相对路径的横向净空
  double return_goal_clearance{0.8}; // 回归点周围的机器人安全半径
  double return_obstacle_clearance{2.0}; // 回归点到本次障碍中心的最小距离
  double temporary_goal_search_distance{6.0}; // 绕障临时点最大搜索弧长
  double temporary_goal_clearance{0.8}; // 绕障临时点及后续路径的净空半径
  double temporary_goal_safe_path_length{1.0}; // 临时点后连续安全路径长度
  double min_temporary_goal_distance_from_obstacle{0.5}; // 离开最后不安全位置后的距离
  double temporary_goal_sample_spacing{0.05}; // 全局路径碰撞检查采样间隔
  double side_goal_boundary_search_radius{1.5};
  double side_goal_boundary_longitudinal_band{0.5};
  double side_goal_boundary_lateral_band{0.5};
  double side_goal_min_boundary_clearance{0.65};
  double side_goal_lateral_sample_step{0.15};
  int side_goal_lateral_sample_count{3};
  double side_goal_min_forward_clearance{0.35};
  double side_goal_forward_sample_step{0.30};
  int side_goal_forward_sample_count{4};
  int side_goal_expansion_rounds{3};
  double side_goal_expansion_lateral_step{0.20};
  double side_goal_expansion_forward_step{0.30};
  double obstacle_threshold{50.0}; // Costmap障碍阈值
  std::string smoothing_method{"bezier"}; // bezier、bspline或none
  bool bezier_smoothing_enabled{true}; // 是否平滑局部A*折角
  double bezier_corner_distance{0.8}; // 拐点前后切线段的最大长度
  double bezier_sample_spacing{0.05}; // 贝塞尔曲线最大采样间隔
  double bezier_simplify_tolerance{0.10}; // 平滑前消除栅格阶梯的最大偏差
  double bezier_target_turn_radius{1.15}; // 根据局部速度/角速度期望的转弯半径
  double bezier_min_turn_radius{0.30}; // 低于该半径不冒充平滑曲线
  double bezier_max_deviation{0.25}; // 曲线偏离原始折线的上限
  bool bezier_visualization_enabled{true};
  bool bspline_smoothing_enabled{true};
  int bspline_degree{3};
  double bspline_sample_spacing{0.05};
  double bspline_simplify_tolerance{0.10};
  double bspline_control_point_spacing{0.70};
  double bspline_min_control_point_spacing{0.15};
  int bspline_adaptive_attempts{4};
  double bspline_max_deviation{0.25};
  double bspline_min_turn_radius{0.30};
  // 默认0保持独立库调用兼容；ROS节点通过YAML显式启用起始趋势保护。
  double smoothing_start_trend_distance{0.0};
  double bspline_start_control_blend{0.0};
};

// 侧方临时点只是候选。Manager必须对候选运行完整A*、平滑、footprint、
// 初始旋转和净空检查后，才能锁定对应目标与路径。
struct SideGoalCandidate {
  geometry_msgs::msg::Point point;
  double yaw{0.0};
  AvoidanceSide side{AvoidanceSide::NONE};
  double boundary_lateral{0.0};
  double boundary_forward{0.0};
  double boundary_clearance{0.0};
  double forward_clearance{0.0};
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
  std::uint64_t preparedCostmapVersion() const;

  // 用start_index限定活动路径段，从机器人在该段附近的投影位置开始，
  // 沿原始全局路径按弧长搜索障碍后的绕障临时点。候选可取线段内部的
  // 插值位置，不要求恰好落在原始pose上；朝向按所在路径线段切线设置。
  // minimum_goal_index只淘汰此前已失败的候选，不改变障碍扫描起点。
  TemporaryGoalSearchResult findTemporaryGoal(const nav_msgs::msg::Path &global_path,
                                      const geometry_msgs::msg::Point &robot_position,
                                      std::size_t start_index,
                                      std::size_t minimum_goal_index = 0U);
  // 只验证已锁定的障碍后绕障临时点及其后续安全窗口，不重新选点。
  // following_path_index是插值目标之后的首个原始路径点。
  bool isTemporaryGoalValid(
      const nav_msgs::msg::Path &global_path,
      const geometry_msgs::msg::PoseStamped &temporary_goal,
      std::size_t following_path_index);
  // 保留原始路径点调用语义，主要用于兼容已有调用和测试。
  bool isTemporaryGoalValid(const nav_msgs::msg::Path &global_path,
                            std::size_t temporary_goal_index);

  // 生成一个侧向临时点，并通过沿路径前移限制机器人切入角。
  bool buildSideGoal(const geometry_msgs::msg::PoseStamped &robot_pose,
                     const geometry_msgs::msg::Point &obstacle_center,
                     AvoidanceSide side,
                     const nav_msgs::msg::Path &global_path, int closest_index,
                     geometry_msgs::msg::Point &side_goal,
                     double &side_goal_yaw, double lateral_clearance,
                     double max_entry_angle,
                     double minimum_forward_offset = 0.0);

  // 根据Costmap中的障碍物边界生成一组侧方候选。横向位置相对障碍物
  // 左/右边界，纵向位置至少越过障碍物前向边界，并满足最大切入角。
  std::vector<SideGoalCandidate> buildSideGoalCandidates(
      const geometry_msgs::msg::PoseStamped &robot_pose,
      const geometry_msgs::msg::Point &obstacle_center, AvoidanceSide side,
      const nav_msgs::msg::Path &global_path, int closest_index,
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
  // planPath保留平移安全的原始折线供贝塞尔修复；最终采用原始路径前，
  // 调用方必须再通过isPathMotionCollisionFree验证折点和终点旋转扫掠。
  bool planPath(const geometry_msgs::msg::Point &start,
                const geometry_msgs::msg::Point &goal,
                nav_msgs::msg::Path &path,
                double goal_yaw = std::numeric_limits<double>::quiet_NaN(),
                const AStarAvoidanceHint &avoidance_hint = {});
  // 先在偏差和footprint碰撞约束内消除A*栅格阶梯，再对剩余折角做分段
  // 三次贝塞尔圆弧近似。曲线同时受可用直线长度、目标/最小转弯半径、
  // 偏离量和footprint碰撞约束。
  bool smoothPathWithBezier(const nav_msgs::msg::Path &raw_path,
                            nav_msgs::msg::Path &smoothed_path);
  // 使用夹持均匀B样条整体平滑简化后的A*控制折线，并执行偏差、曲率及
  // footprint运动安全检查。失败时保持raw_path，供上层安全回退。
  bool smoothPathWithBSpline(const nav_msgs::msg::Path &raw_path,
                             nav_msgs::msg::Path &smoothed_path);
  // 按smoothing_method选择实际交给局部PP的平滑算法。
  bool smoothPath(const nav_msgs::msg::Path &raw_path,
                  nav_msgs::msg::Path &smoothed_path);
  bool isPointInObstacle(const geometry_msgs::msg::Point &point,
                         double yaw = 0.0);
  bool isPointInObstacleWithDiagnostics(
      const geometry_msgs::msg::Point &point, const char *point_name,
      double yaw = 0.0);
  bool isPathCollisionFree(const nav_msgs::msg::Path &path,
                           bool allow_initial_escape = false) const;
  // 除线段平移外，再检查每个折点的入射->出射旋转，以及最后入射方向
  // ->终点姿态的 footprint 扫掠。
  bool isPathMotionCollisionFree(
      const nav_msgs::msg::Path &path,
      bool allow_initial_escape = false) const;
  bool isRotationCollisionFree(const geometry_msgs::msg::Point &center,
                               double start_yaw, double end_yaw) const;
  bool isInitialEscapePathCollisionFree(
      const nav_msgs::msg::Path &path,
      const geometry_msgs::msg::Point &obstacle_center,
      double required_clearance) const;
  // 候选走廊不能进入控制阶段的近障停车半径。若机器人触发规划时已经
  // 位于该半径内，allow_initial_escape可允许一条不再靠近障碍、并最终
  // 退出净空圈的脱离路径。返回整条折线路径到障碍中心的最小距离。
  bool isPathOutsideObstacleRadius(
      const nav_msgs::msg::Path &path,
      const geometry_msgs::msg::Point &obstacle_center,
      double required_clearance, double *minimum_clearance = nullptr,
      bool allow_initial_escape = false) const;
  void publishVisualization(const geometry_msgs::msg::Point &start,
                            const geometry_msgs::msg::Point &goal,
                            const nav_msgs::msg::Path &path);

private:
  bool getPathYawAtIndex(const nav_msgs::msg::Path &path, int index,
                         double &yaw) const;
  bool isPointSafeForRobot(const geometry_msgs::msg::Point &point,
                           double clearance_radius) const;
  bool isPointSafeForTemporaryGoal(const geometry_msgs::msg::Point &point,
                              double clearance_radius,
                              const CostmapConstPtr &costmap) const;
  static double calculateDistance(const geometry_msgs::msg::Point &p1,
                                  const geometry_msgs::msg::Point &p2);
  void publishSmoothingPaths(const nav_msgs::msg::Path &raw_path,
                             const nav_msgs::msg::Path &smoothed_path,
                             bool bspline = false);
  void publishRawPath(const nav_msgs::msg::Path &raw_path);
  void publishPrunedPath(const nav_msgs::msg::Path &pruned_path);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<AStarPlanner> astar_planner_;
  AvoidancePathPlannerConfig config_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bspline_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pruned_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      selected_smoothed_path_pub_;

};

} // namespace pure_pursuit_local_planner

#endif // AVOIDANCE_PATH_PLANNER_HPP
