#ifndef LOCAL_PATH_PLANNER_HPP
#define LOCAL_PATH_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <thread>
#include <mutex>
#include <atomic>
#include "pure_pursuit_local_planner/astar_planner.hpp"

namespace pure_pursuit_local_planner
{

class LocalPathPlanner
{
public:
    LocalPathPlanner();
    ~LocalPathPlanner();
    
    // 初始化
    bool initialize(rclcpp::Node::SharedPtr node, 
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer);
    
    // 设置全局路径
    void setGlobalPath(const nav_msgs::msg::Path& global_path);
    
    // 更新代价地图
    void updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap);
    
    // 获取局部路径
    nav_msgs::msg::Path getLocalPath() const;
    
    // 检查是否有有效的局部路径
    bool hasValidLocalPath() const;
    
    // 停止规划器
    void stop();

private:
    // ROS相关
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // 发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    
    // A*规划器
    std::unique_ptr<AStarPlanner> astar_planner_;
    
        // 线程管理
    std::thread planning_thread_;
    std::atomic<bool> running_;
    mutable std::mutex path_mutex_;
    mutable std::mutex costmap_mutex_;
    mutable std::mutex global_path_mutex_;
    
    // 数据
    nav_msgs::msg::Path global_path_;
    nav_msgs::msg::Path local_path_;
    nav_msgs::msg::OccupancyGrid costmap_;
    bool global_path_received_;
    bool costmap_received_;
    bool local_path_valid_;
    int last_goal_index_;              // 记录上一次目标点索引，防止倒退
    
    // 路径稳定性相关
    nav_msgs::msg::Path previous_local_path_;  // 上一次的局部路径
    int path_stability_counter_;          // 路径稳定性计数器
    double path_similarity_threshold_;    // 路径相似度阈值
    
    // 参数
    double planning_frequency_;       // 规划频率 (Hz)
    double lookahead_distance_;      // 前瞻距离 (m)
    double min_lookahead_distance_;  // 最小前瞻距离 (m)
    double max_lookahead_distance_;  // 最大前瞻距离 (m)
    std::string global_frame_;       // 全局坐标系
    std::string robot_frame_;        // 机器人坐标系
    double transform_tolerance_;     // TF变换容忍度
    
    // 主要函数
    void planningLoop();
    bool computeLocalPath();
    
    // 辅助函数
    bool getRobotPose(geometry_msgs::msg::PoseStamped& robot_pose);
    geometry_msgs::msg::Point findGoalPoint(const geometry_msgs::msg::PoseStamped& robot_pose);
    int findClosestPointOnGlobalPath(const geometry_msgs::msg::PoseStamped& robot_pose);
    bool isPointInObstacle(const geometry_msgs::msg::Point& point);
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2);
    
    // 新增的路径分析函数
    int findProjectionOnPath(const geometry_msgs::msg::PoseStamped& robot_pose, 
                            const nav_msgs::msg::Path& path);
    int findBestPathPoint(const geometry_msgs::msg::PoseStamped& robot_pose, 
                         const nav_msgs::msg::Path& path);
    double calculateDistance3D(const geometry_msgs::msg::Point& p1, 
                              const geometry_msgs::msg::Point& p2);
    double pointToLineDistance(const geometry_msgs::msg::Point& point,
                              const geometry_msgs::msg::Point& line_start,
                              const geometry_msgs::msg::Point& line_end);
    geometry_msgs::msg::Point projectPointOnLine(const geometry_msgs::msg::Point& point,
                                           const geometry_msgs::msg::Point& line_start,
                                           const geometry_msgs::msg::Point& line_end);
    
    // 路径稳定性函数
    double calculatePathSimilarity(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2);
    bool shouldKeepPreviousPath(const nav_msgs::msg::Path& new_path);
    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path);
    
    // 可视化
    void publishGoalMarker(const geometry_msgs::msg::Point& goal_point);
};

} // namespace pure_pursuit_local_planner

#endif // LOCAL_PATH_PLANNER_H
