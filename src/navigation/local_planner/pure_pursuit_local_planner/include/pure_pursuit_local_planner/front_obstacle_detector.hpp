#ifndef FRONT_OBSTACLE_DETECTOR_HPP
#define FRONT_OBSTACLE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <tf2/time.h>

namespace pure_pursuit_local_planner
{

class FrontObstacleDetector
{
public:
    FrontObstacleDetector();
    ~FrontObstacleDetector();

    /**
     * @brief 初始化障碍物检测器
     * @param nh ROS节点句柄
     * @param tf_buffer TF变换缓冲区
     * @return 是否初始化成功
     */
    bool initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

    /**
     * @brief 更新代价地图
     * @param costmap 代价地图
     */
    void updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap);

    /**
     * @brief 检测前方障碍物并调整速度
     * @param cmd_vel 输入速度指令
     * @param robot_pose 机器人当前位姿
     * @return 调整后的速度指令
     */
    geometry_msgs::msg::Twist adjustVelocityForObstacles(const geometry_msgs::msg::Twist& cmd_vel,
                                                    const geometry_msgs::msg::PoseStamped& robot_pose);

    /**
     * @brief 发布检测区域可视化
     * @param robot_pose 机器人当前位姿
     * @param obstacle_detected 是否检测到障碍物
     */
    void publishVisualization(const geometry_msgs::msg::PoseStamped& robot_pose, bool obstacle_detected);

private:
    /**
     * @brief 检测指定区域内是否有障碍物
     * @param robot_pose 机器人位姿
     * @return 是否检测到障碍物
     */
    bool detectObstacleInFrontArea(const geometry_msgs::msg::PoseStamped& robot_pose);

    /**
     * @brief 将世界坐标转换为代价地图坐标
     * @param world_x 世界坐标X
     * @param world_y 世界坐标Y
     * @param map_x 输出地图坐标X
     * @param map_y 输出地图坐标Y
     * @return 是否转换成功
     */
    bool worldToMap(double world_x, double world_y, int& map_x, int& map_y);

    /**
     * @brief 获取地图指定位置的代价值
     * @param map_x 地图坐标X
     * @param map_y 地图坐标Y
     * @return 代价值，-1表示越界
     */
    int getCostAtMapCoordinate(int map_x, int map_y);

    /**
     * @brief 生成检测区域的角点
     * @param robot_pose 机器人位姿
     * @param corners 输出的角点坐标
     */
    void generateDetectionAreaCorners(const geometry_msgs::msg::PoseStamped& robot_pose,
                                     std::vector<std::pair<double, double>>& corners);

    /**
     * @brief 检测扇形区域内是否有障碍物
     * @param robot_pose 机器人位姿
     * @return 是否检测到障碍物
     */
    bool detectObstacleInSectorArea(const geometry_msgs::msg::PoseStamped& robot_pose);

    /**
     * @brief 生成扇形检测区域的边界点
     * @param robot_pose 机器人位姿
     * @param sector_points 输出的扇形边界点
     */
    void generateSectorAreaPoints(const geometry_msgs::msg::PoseStamped& robot_pose,
                                 std::vector<std::pair<double, double>>& sector_points);

    /**
     * @brief 检查点是否在扇形区域内
     * @param point_x 点的X坐标
     * @param point_y 点的Y坐标
     * @param robot_x 机器人X坐标
     * @param robot_y 机器人Y坐标
     * @param robot_yaw 机器人朝向
     * @return 是否在扇形内
     */
    bool isPointInSector(double point_x, double point_y, double robot_x, double robot_y, double robot_yaw);

    // ROS相关
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_area_pub_;   // 检测区域可视化发布器

    // 代价地图
    nav_msgs::msg::OccupancyGrid costmap_;
    bool has_costmap_;

    // 参数
    double detection_width_;     // 检测区域宽度 (m)
    double detection_length_;    // 检测区域长度 (m)
    double sector_radius_;       // 扇形半径 (m)
    double sector_angle_;        // 扇形角度 (度)
    int obstacle_threshold_;     // 障碍物阈值 (代价值)
    double linear_velocity_factor_;  // 线速度衰减因子
    double angular_velocity_factor_; // 角速度增强因子
    std::string detection_shape_;    // 检测形状 ("rectangle" 或 "sector")
    
    // 调试和可视化
    bool debug_mode_;
    bool enable_visualization_;      // 是否启用可视化
};

} // namespace pure_pursuit_local_planner

#endif // FRONT_OBSTACLE_DETECTOR_H
