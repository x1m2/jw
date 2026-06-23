#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

namespace pure_pursuit_local_planner
{

struct DijkstraNode
{
    int x, y;           // 栅格坐标
    double cost;        // 从起点到当前节点的实际代价
    DijkstraNode* parent;  // 父节点指针
    
    DijkstraNode(int x_, int y_) : x(x_), y(y_), cost(std::numeric_limits<double>::max()), parent(nullptr) {}
};

// 用于priority_queue的比较器（指针版本）
struct DijkstraNodeCompare
{
    bool operator()(const DijkstraNode* a, const DijkstraNode* b) const {
        return a->cost > b->cost;  // 最小堆
    }
};

// 用于哈希表的节点键
struct NodeKey
{
    int x, y;
    NodeKey(int x_, int y_) : x(x_), y(y_) {}
    
    bool operator==(const NodeKey& other) const {
        return x == other.x && y == other.y;
    }
};

// 自定义哈希函数
struct NodeKeyHash
{
    std::size_t operator()(const NodeKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
    }
};

class AStarPlanner
{
public:
    AStarPlanner();
    ~AStarPlanner();
    
    // 初始化
    bool initialize(rclcpp::Node::SharedPtr node);
    
    // 设置代价地图
    void setCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
    
    // 设置全局路径（用于计算路径偏离代价）
    void setGlobalPath(const nav_msgs::msg::Path& global_path);
    
    // A*路径规划
    bool planPath(const geometry_msgs::msg::Point& start, 
                  const geometry_msgs::msg::Point& goal, 
                  nav_msgs::msg::Path& path);
    
    // 检查点是否在障碍物中
    bool isObstacle(const geometry_msgs::msg::Point& point);
    bool isObstacle(int grid_x, int grid_y);
    
    // 坐标转换
    void worldToGrid(const geometry_msgs::msg::Point& world_point, int& grid_x, int& grid_y);
    void gridToWorld(int grid_x, int grid_y, geometry_msgs::msg::Point& world_point);
    
    // 可视化
    void publishVisualization(const geometry_msgs::msg::Point& start, 
                             const geometry_msgs::msg::Point& goal,
                             const nav_msgs::msg::Path& path);

private:
    // ROS相关
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // 代价地图
    nav_msgs::msg::OccupancyGrid costmap_;
    bool costmap_received_;
    
    // 全局路径
    nav_msgs::msg::Path global_path_;
    bool global_path_received_;
    
    // Dijkstra算法参数
    double obstacle_threshold_;     // 障碍物阈值
    int safety_margin_;             // 安全边距（栅格数）
    bool allow_diagonal_;           // 是否允许对角线移动
    double diagonal_cost_;          // 对角线移动代价
    double straight_cost_;          // 直线移动代价
    double path_deviation_weight_;  // 路径偏离权重
    double obstacle_avoidance_weight_;  // 障碍物避障权重
    
    // 可视化参数
    std::string global_frame_;
    
    // Dijkstra算法内部函数
    double calculateMoveCost(const DijkstraNode* from, const DijkstraNode* to);
    double calculateObstacleCost(int x, int y);  // 计算障碍物代价
    double calculatePathDeviationCost(int x, int y) const;  // 计算路径偏离代价
    std::vector<DijkstraNode*> getNeighbors(DijkstraNode* current);
    bool isValidCell(int x, int y);
    bool isStartPositionValid(int grid_x, int grid_y);  // 专门用于起点检查的函数
    nav_msgs::msg::Path reconstructPath(DijkstraNode* goal_node);
    void cleanupNodes(std::unordered_map<NodeKey, DijkstraNode*, NodeKeyHash>& all_nodes);
    
    // 辅助函数
    double distanceToGlobalPath(const geometry_msgs::msg::Point& point);
    
    // 可视化函数
    visualization_msgs::msg::Marker createPointMarker(const geometry_msgs::msg::Point& point, 
                                                 int id, double r, double g, double b);
};

} // namespace pure_pursuit_local_planner

#endif // ASTAR_PLANNER_H
