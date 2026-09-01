#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pure_pursuit/costmap_types.hpp"

namespace pure_pursuit_local_planner {

// 队列项保存入队时的代价快照，避免节点降价后破坏priority_queue堆序。
struct AStarQueueEntry {
  size_t cell_index;
  double g_cost;
  double f_cost;
};

struct AStarQueueEntryCompare {
  bool operator()(const AStarQueueEntry &a, const AStarQueueEntry &b) const {
    if (a.f_cost == b.f_cost) {
      return a.g_cost < b.g_cost;
    }
    return a.f_cost > b.f_cost;
  }
};

class AStarPlanner {
 public:
  AStarPlanner();

  // 初始化
  bool initialize(rclcpp::Node::SharedPtr node);

  // 设置代价地图
  // 运行时接口只提交最新输入，footprint掩码由规划线程准备。
  void setCostmap(CostmapConstPtr costmap);
  // 保留值类型接口，便于单元测试和独立调用；运行时使用上面的共享快照接口。
  void setCostmap(const nav_msgs::msg::OccupancyGrid &costmap);
  // 在调用线程构建最新地图对应的掩码，并原子发布完整只读快照。
  // 返回true表示至少存在一份可用的完整快照。
  bool prepareLatestCostmap();
  bool hasPreparedCostmap() const;
  CostmapConstPtr getPreparedCostmap() const;

  // 设置全局路径（用于计算路径偏离代价）
  void setGlobalPath(const nav_msgs::msg::Path &global_path);

  // A*路径规划
  bool planPath(const geometry_msgs::msg::Point &start,
                const geometry_msgs::msg::Point &goal,
                nav_msgs::msg::Path &path,
                double goal_yaw = std::numeric_limits<double>::quiet_NaN());

  // 检查点是否在障碍物中
  bool isObstacle(const geometry_msgs::msg::Point &point);
  bool isObstacle(const geometry_msgs::msg::Point &point, double yaw);
  // 使用同一份已准备的Costmap快照检查点，并以INFO输出完整判定依据。
  bool isObstacleWithDiagnostics(const geometry_msgs::msg::Point &point,
                                 const char *point_name,
                                 double yaw = 0.0);
  bool isPathCollisionFree(const nav_msgs::msg::Path &path,
                           bool allow_initial_escape = false);
  // 在固定位置采样起止航向之间的最短旋转，检查长方形车体角点的扫掠区。
  bool isRotationCollisionFree(const geometry_msgs::msg::Point &center,
                               double start_yaw, double end_yaw) const;

  // 可视化
  void publishVisualization(const geometry_msgs::msg::Point &start,
                            const geometry_msgs::msg::Point &goal,
                            const nav_msgs::msg::Path &path);

 private:
  // ROS相关
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  struct PreparedCostmapSnapshot {
    CostmapConstPtr costmap;
    std::shared_ptr<const std::vector<uint8_t>> footprint_mask;
    uint64_t version{0};

    bool valid() const {
      return costmap && footprint_mask &&
             footprint_mask->size() == costmap->data.size();
    }
  };

  // Costmap回调只更新pending输入；规划线程完成预处理后一次替换完整快照。
  CostmapConstPtr pending_costmap_;
  uint64_t pending_costmap_version_{0};
  uint64_t last_preparation_attempt_version_{0};
  PreparedCostmapSnapshot prepared_costmap_;

  // 全局路径
  nav_msgs::msg::Path global_path_;
  bool global_path_received_;
  mutable std::mutex data_mutex_;

  // A*搜索工作区按地图栅格连续存储。地图尺寸不变时复用容量，每次规划仅
  // 使用std::fill重置，避免搜索过程中频繁new/delete和哈希表分配。
  std::vector<double> search_g_costs_;
  std::vector<int32_t> search_parent_indices_;
  std::vector<uint8_t> search_closed_;
  std::mutex search_mutex_;

  // A*算法参数
  int obstacle_threshold_;            // 检测器与A*共用的障碍阻塞阈值
  int safety_margin_;                 // 安全边距（栅格数）
  bool allow_diagonal_;               // 是否允许对角线移动
  double diagonal_cost_;              // 对角线移动代价
  double straight_cost_;              // 直线移动代价
  double path_deviation_weight_;      // 路径偏离权重
  double obstacle_avoidance_weight_;  // 障碍物避障权重
  double footprint_radius_;           // 矩形footprint外接圆，仅用于限定检查范围
  double footprint_half_length_;      // 车体长度一半加安全边距
  double footprint_half_width_;       // 车体宽度一半加安全边距
  int start_obstacle_threshold_;      // 起点允许的最大Costmap代价值
  int footprint_obstacle_threshold_;  // footprint膨胀使用的硬障碍阈值
  static constexpr double kPlanningTimeoutSec = 0.15;
  static constexpr size_t kMaxSearchNodes = 50000;

  // 可视化参数
  std::string global_frame_;

  // A*算法内部函数
  double calculateHeuristic(int x, int y, int goal_x, int goal_y) const;
  double calculateObstacleCost(
      int x, int y, const nav_msgs::msg::OccupancyGrid &costmap,
      const std::vector<uint8_t> &footprint_mask) const;
  double calculatePathDeviationCost(
      int x, int y, const nav_msgs::msg::OccupancyGrid &costmap,
      const nav_msgs::msg::Path &global_path) const;
  bool isValidCell(int x, int y,
                   const nav_msgs::msg::OccupancyGrid &costmap) const;
  bool isObstacle(int grid_x, int grid_y,
                  const nav_msgs::msg::OccupancyGrid &costmap,
                  const std::vector<uint8_t> &footprint_mask) const;
  bool isRectangleFootprintBlocked(
      int center_x, int center_y, double yaw,
      const nav_msgs::msg::OccupancyGrid &costmap,
      const std::vector<uint8_t> &hard_obstacle_mask) const;
  bool isStartPositionValid(int grid_x, int grid_y,
                            const nav_msgs::msg::OccupancyGrid &costmap,
                            const std::vector<uint8_t> &footprint_mask) const;
  void worldToGrid(const geometry_msgs::msg::Point &world_point,
                   const nav_msgs::msg::OccupancyGrid &costmap, int &grid_x,
                   int &grid_y) const;
  void gridToWorld(int grid_x, int grid_y,
                   const nav_msgs::msg::OccupancyGrid &costmap,
                   geometry_msgs::msg::Point &world_point) const;
  std::vector<uint8_t> buildFootprintMask(
      const nav_msgs::msg::OccupancyGrid &costmap) const;
  nav_msgs::msg::Path reconstructPath(
      size_t goal_index, const std::vector<int32_t> &parent_indices,
      const nav_msgs::msg::OccupancyGrid &costmap) const;

  // 可视化函数
  visualization_msgs::msg::Marker createPointMarker(
      const geometry_msgs::msg::Point &point, int id, double r, double g,
      double b);
};

}  // namespace pure_pursuit_local_planner

#endif  // ASTAR_PLANNER_H
