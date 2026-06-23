#include "pure_pursuit_local_planner/obstacle_avoidance.hpp"
#include "pure_pursuit_local_planner/param_utils.hpp"

namespace pure_pursuit_local_planner
{

ObstacleAvoidance::ObstacleAvoidance()
    : costmap_received_(false), scan_received_(false)
{
}

ObstacleAvoidance::~ObstacleAvoidance()
{
}

bool ObstacleAvoidance::initialize(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    
    // 加载参数
    loadParameters();
    
    // 初始化发布器
    force_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("potential_field/force_markers", 1);
    field_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("potential_field/field_markers", 1);
    front_detection_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("potential_field/front_detection", 1);
    
    RCLCPP_INFO(node_->get_logger(), "Potential Field Obstacle Avoidance module initialized");
    RCLCPP_INFO(node_->get_logger(), "  Attractive gain: %.2f", field_params_.attractive_gain);
    RCLCPP_INFO(node_->get_logger(), "  Repulsive gain: %.2f", field_params_.repulsive_gain);
    RCLCPP_INFO(node_->get_logger(), "  Influence radius: %.2f m", field_params_.influence_radius);
    RCLCPP_INFO(node_->get_logger(), "  Safety radius: %.2f m", field_params_.safety_radius);
    RCLCPP_INFO(node_->get_logger(), "  Y force threshold: %.2f", field_params_.y_force_threshold);
    RCLCPP_INFO(node_->get_logger(), "  Y smoothing factor: %.2f", field_params_.y_smoothing_factor);
    RCLCPP_INFO(node_->get_logger(), "  Bilateral threshold: %.2f", field_params_.bilateral_threshold);
    
    return true;
}

void ObstacleAvoidance::loadParameters()
{
    // 势场参数
    field_params_.attractive_gain = declareOrGetParameter(node_, "potential_field.attractive_gain", 1.5);
    field_params_.repulsive_gain = declareOrGetParameter(node_, "potential_field.repulsive_gain", 20.0);
    field_params_.influence_radius = declareOrGetParameter(node_, "potential_field.influence_radius", 2.5);
    field_params_.safety_radius = declareOrGetParameter(node_, "potential_field.safety_radius", 0.8);
    field_params_.force_limit = declareOrGetParameter(node_, "potential_field.force_limit", 10.0);
    field_params_.angular_gain = declareOrGetParameter(node_, "potential_field.angular_gain", 2.0);
    field_params_.speed_factor_gain = declareOrGetParameter(node_, "potential_field.speed_factor_gain", 0.5);
    
    // Y方向避障参数
    field_params_.y_force_threshold = declareOrGetParameter(node_, "potential_field.y_force_threshold", 0.5);
    field_params_.y_velocity_gain = declareOrGetParameter(node_, "potential_field.y_velocity_gain", 0.3);
    field_params_.y_smoothing_factor = declareOrGetParameter(node_, "potential_field.y_smoothing_factor", 0.3);
    field_params_.y_dead_zone = declareOrGetParameter(node_, "potential_field.y_dead_zone", 0.05);
    
    // 角速度调整参数
    field_params_.angular_smoothing_factor = declareOrGetParameter(node_, "potential_field.angular_smoothing_factor", 0.4);
    
    // 双侧障碍物检测参数
    field_params_.bilateral_threshold = declareOrGetParameter(node_, "potential_field.bilateral_threshold", 0.8);
    field_params_.bilateral_damping = declareOrGetParameter(node_, "potential_field.bilateral_damping", 0.1);
    
    // 代价地图阈值
    costmap_threshold_ = declareOrGetParameter(node_, "costmap_threshold", 50.0);
}

void ObstacleAvoidance::updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
    current_costmap_ = costmap;
    costmap_received_ = true;
}

void ObstacleAvoidance::updateLaserScan(const sensor_msgs::msg::LaserScan& scan)
{
    current_scan_ = scan;
    scan_received_ = true;
}

geometry_msgs::msg::Twist ObstacleAvoidance::computePotentialFieldAvoidance(
    const geometry_msgs::msg::Twist& original_cmd,
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::PointStamped& target_point)
{
    if (!costmap_received_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "No costmap received, using original command");
        return original_cmd;
    }
    
    // // 1. 计算吸引力（指向目标点）
    // geometry_msgs::msg::Vector3 attractive_force = computeAttractiveForce(robot_pose, target_point);
    
    // // 2. 计算排斥力（远离障碍物）
    // geometry_msgs::msg::Vector3 repulsive_force = computeRepulsiveForce(robot_pose);
    
    // // 3. 检查正前方是否有近距离障碍物
    bool front_obstacle = checkFrontObstacle(robot_pose);
    
    // // 4. 合成总力
    // geometry_msgs::msg::Vector3 total_force;
    // total_force.x = attractive_force.x + repulsive_force.x;
    // total_force.y = attractive_force.y + repulsive_force.y;
    // total_force.z = 0.0;
    
    // // 5. 限制力的大小
    // total_force = limitForce(total_force, field_params_.force_limit);
    
    // // 6. 转换为速度指令
    // geometry_msgs::msg::Twist result_cmd = convertForceToVelocity(total_force, original_cmd, robot_pose);
    geometry_msgs::msg::Twist result_cmd = original_cmd;
    
    // 7. 如果正前方有障碍物，添加负的线速度
    if (front_obstacle) {
        double backward_speed = -0.1;  // 后退速度 -0.3 m/s
        result_cmd.linear.x = std::min(result_cmd.linear.x, backward_speed);
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Front obstacle detected, applying backward motion");
    }
    
    
    // 8. 发布可视化
    // publishVisualization(robot_pose, attractive_force, repulsive_force);
    
    // 9. 发布前方检测区域可视化
    publishFrontDetectionVisualization(robot_pose);
    
    return result_cmd;
}

geometry_msgs::msg::Vector3 ObstacleAvoidance::computeAttractiveForce(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::PointStamped& target_point)
{
    geometry_msgs::msg::Vector3 force;
    
    double dx = target_point.point.x - robot_pose.pose.position.x;
    double dy = target_point.point.y - robot_pose.pose.position.y;
    double distance = std::hypot(dx, dy);
    
    if (distance > 1e-6) {
        // 归一化方向向量乘以增益
        force.x = field_params_.attractive_gain * dx / distance;
        force.y = field_params_.attractive_gain * dy / distance;
    } else {
        force.x = 0.0;
        force.y = 0.0;
    }
    force.z = 0.0;
    
    return force;
}

geometry_msgs::msg::Vector3 ObstacleAvoidance::computeRepulsiveForce(
    const geometry_msgs::msg::PoseStamped& robot_pose)
{
    geometry_msgs::msg::Vector3 total_force;
    total_force.x = 0.0;
    total_force.y = 0.0;
    total_force.z = 0.0;
    
    if (!costmap_received_) {
        return total_force;
    }
    
    // 获取机器人在代价地图中的位置
    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    
    int robot_map_x = static_cast<int>((robot_x - current_costmap_.info.origin.position.x) / 
                                       current_costmap_.info.resolution);
    int robot_map_y = static_cast<int>((robot_y - current_costmap_.info.origin.position.y) / 
                                       current_costmap_.info.resolution);
    
    // 搜索影响半径内的障碍物
    int search_radius = static_cast<int>(field_params_.influence_radius / current_costmap_.info.resolution);
    
    for (int dx = -search_radius; dx <= search_radius; ++dx) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            int map_x = robot_map_x + dx;
            int map_y = robot_map_y + dy;
            
            // 边界检查
            if (map_x < 0 || map_x >= static_cast<int>(current_costmap_.info.width) ||
                map_y < 0 || map_y >= static_cast<int>(current_costmap_.info.height))
                continue;
            
            // 获取代价值
            int index = map_y * current_costmap_.info.width + map_x;
            int8_t cost = current_costmap_.data[index];
            
            // 只处理高代价点（障碍物）
            if (cost > costmap_threshold_) {
                double obstacle_x = current_costmap_.info.origin.position.x + 
                                   map_x * current_costmap_.info.resolution;
                double obstacle_y = current_costmap_.info.origin.position.y + 
                                   map_y * current_costmap_.info.resolution;
                
                double distance = std::hypot(obstacle_x - robot_x, obstacle_y - robot_y);
                
                if (distance < field_params_.influence_radius && distance > 1e-6) {
                    // 计算排斥力 - 使用二次势场
                    double cost_factor = std::min(cost / 100.0, 1.0);  // 归一化代价值
                    double force_magnitude = field_params_.repulsive_gain * cost_factor * 
                        (1.0/distance - 1.0/field_params_.influence_radius) / (distance * distance);
                    
                    // 如果距离太近，增强排斥力
                    if (distance < field_params_.safety_radius) {
                        force_magnitude *= 3.0;  // 安全区域内增强排斥力
                    }
                    
                    // 力的方向：从障碍物指向机器人
                    double force_x = force_magnitude * (robot_x - obstacle_x) / distance;
                    double force_y = force_magnitude * (robot_y - obstacle_y) / distance;
                    
                    total_force.x += force_x;
                    total_force.y += force_y;
                }
            }
        }
    }
    
    return total_force;
}

geometry_msgs::msg::Twist ObstacleAvoidance::convertForceToVelocity(
    const geometry_msgs::msg::Vector3& total_force,
    const geometry_msgs::msg::Twist& original_cmd,
    const geometry_msgs::msg::PoseStamped& robot_pose)
{
    geometry_msgs::msg::Twist cmd_vel = original_cmd;
    
    // 获取机器人当前朝向
    double robot_yaw = getRobotYaw(robot_pose);
    
    // 将力转换到机器人坐标系
    double force_robot_x = total_force.x * std::cos(robot_yaw) + total_force.y * std::sin(robot_yaw);
    double force_robot_y = -total_force.x * std::sin(robot_yaw) + total_force.y * std::cos(robot_yaw);
    
    // 计算排斥力大小（用于判断是否需要侧向避障）
    double force_magnitude = std::hypot(total_force.x, total_force.y);
    
    // 分离排斥力分量来检测双侧障碍物
    geometry_msgs::msg::Vector3 repulsive_component;
    repulsive_component.x = total_force.x;
    repulsive_component.y = total_force.y;
    
    // 需求1：当排斥力很小时，y方向的线速度为0
    if (force_magnitude < field_params_.y_force_threshold) {
        cmd_vel.linear.y = 0.0;  // 排斥力小时不进行侧向运动
    } else {
        // 检测是否为双侧障碍物情况
        bool is_bilateral = detectBilateralObstacles(robot_pose, repulsive_component);
        
        // 静态变量用于平滑
        static double last_y_velocity = 0.0;  // 上一次的Y方向速度
        static double last_angular_adjustment = 0.0; // 上一次的角速度调整
        
        // 计算目标Y方向速度
        double target_y_velocity = force_robot_y * field_params_.y_velocity_gain;
        
        // 如果是双侧障碍物情况，应用阻尼
        if (is_bilateral) {
            target_y_velocity *= field_params_.bilateral_damping;
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Bilateral obstacles detected, applying damping");
        }
        
        // 对Y方向速度进行平滑处理
        double smooth_y_velocity = field_params_.y_smoothing_factor * target_y_velocity + 
                                  (1.0 - field_params_.y_smoothing_factor) * last_y_velocity;
        
        // 添加死区以避免小幅抖动
        if (std::abs(smooth_y_velocity) < field_params_.y_dead_zone) {
            smooth_y_velocity = 0.0;
        }
        
        cmd_vel.linear.y = smooth_y_velocity;
        last_y_velocity = smooth_y_velocity;  // 保存当前值用于下次平滑
        
        // 根据侧向力调整角速度
        double angular_adjustment = std::atan2(force_robot_y, std::abs(force_robot_x) + 0.1) * field_params_.angular_gain;
        
        // 双侧障碍物时也对角速度应用阻尼
        if (is_bilateral) {
            angular_adjustment *= field_params_.bilateral_damping;
        }
        
        // 对角速度也进行平滑处理以避免抖动
        double smooth_angular = field_params_.angular_smoothing_factor * angular_adjustment + 
                               (1.0 - field_params_.angular_smoothing_factor) * last_angular_adjustment;
        
        cmd_vel.angular.z += smooth_angular;
        last_angular_adjustment = smooth_angular;
    }
    
    // 根据总力大小调整线速度
    double speed_factor = 1.0 / (1.0 + force_magnitude * field_params_.speed_factor_gain);
    cmd_vel.linear.x *= std::max(0.1, speed_factor);  // 保持最小速度
    
    // 限制速度指令
    cmd_vel.angular.z = std::max(-2.0, std::min(2.0, cmd_vel.angular.z));  // 限制角速度
    
    // 强制禁用Y方向运动（仅前进模式）
    // cmd_vel.linear.y = 0.0;
    
    return cmd_vel;
}

double ObstacleAvoidance::getRobotYaw(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    tf2::Quaternion q;
    tf2::fromMsg(robot_pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

geometry_msgs::msg::Vector3 ObstacleAvoidance::limitForce(const geometry_msgs::msg::Vector3& force, double max_force)
{
    geometry_msgs::msg::Vector3 limited_force = force;
    double magnitude = std::hypot(force.x, force.y);
    
    if (magnitude > max_force && magnitude > 1e-6) {
        double scale = max_force / magnitude;
        limited_force.x *= scale;
        limited_force.y *= scale;
    }
    
    return limited_force;
}

bool ObstacleAvoidance::isPathSafe(const geometry_msgs::msg::PoseStamped& robot_pose,
                                   const geometry_msgs::msg::Twist& cmd_vel,
                                   double prediction_time)
{
    if (!costmap_received_) {
        return true;  // 没有代价地图信息，假设安全
    }
    
    // 预测机器人在prediction_time后的位置
    double predicted_x = robot_pose.pose.position.x + cmd_vel.linear.x * prediction_time;
    double predicted_y = robot_pose.pose.position.y + cmd_vel.linear.y * prediction_time;
    
    // 检查预测位置的代价值
    int pred_map_x = static_cast<int>((predicted_x - current_costmap_.info.origin.position.x) / 
                                      current_costmap_.info.resolution);
    int pred_map_y = static_cast<int>((predicted_y - current_costmap_.info.origin.position.y) / 
                                      current_costmap_.info.resolution);
    
    // 边界检查
    if (pred_map_x < 0 || pred_map_x >= static_cast<int>(current_costmap_.info.width) ||
        pred_map_y < 0 || pred_map_y >= static_cast<int>(current_costmap_.info.height)) {
        return false;  // 预测位置超出地图边界
    }
    
    // 检查预测位置的代价值
    int index = pred_map_y * current_costmap_.info.width + pred_map_x;
    int8_t cost = current_costmap_.data[index];
    
    return cost <= costmap_threshold_;  // 代价值低于阈值认为安全
}

void ObstacleAvoidance::publishVisualization(const geometry_msgs::msg::PoseStamped& robot_pose,
                                            const geometry_msgs::msg::Vector3& attractive_force,
                                            const geometry_msgs::msg::Vector3& repulsive_force)
{
    // 发布力向量可视化
    visualization_msgs::msg::MarkerArray force_markers = createForceMarkers(robot_pose, attractive_force, repulsive_force);
    force_markers_pub_->publish(force_markers);
    
    // 发布势场网格可视化（较低频率）
    static int counter = 0;
    if (++counter % 10 == 0) {  // 每10次发布一次势场网格
        visualization_msgs::msg::MarkerArray field_markers = createFieldMarkers(robot_pose);
        field_markers_pub_->publish(field_markers);
    }
}

visualization_msgs::msg::MarkerArray ObstacleAvoidance::createForceMarkers(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Vector3& attractive_force,
    const geometry_msgs::msg::Vector3& repulsive_force)
{
    visualization_msgs::msg::MarkerArray markers;
    
    // 吸引力箭头（绿色）
    visualization_msgs::msg::Marker attractive_marker;
    attractive_marker.header.frame_id = robot_pose.header.frame_id;
    attractive_marker.header.stamp = node_->now();
    attractive_marker.ns = "attractive_force";
    attractive_marker.id = 0;
    attractive_marker.type = visualization_msgs::msg::Marker::ARROW;
    attractive_marker.action = visualization_msgs::msg::Marker::ADD;
    
    attractive_marker.pose.position = robot_pose.pose.position;
    attractive_marker.pose.position.z = 0.5;
    
    // 计算箭头方向
    double att_angle = std::atan2(attractive_force.y, attractive_force.x);
    tf2::Quaternion att_q;
    att_q.setRPY(0, 0, att_angle);
    attractive_marker.pose.orientation = tf2::toMsg(att_q);
    
    attractive_marker.scale.x = std::min(2.0, std::hypot(attractive_force.x, attractive_force.y) * 0.3);
    attractive_marker.scale.y = 0.1;
    attractive_marker.scale.z = 0.1;
    
    attractive_marker.color.r = 0.0;
    attractive_marker.color.g = 1.0;
    attractive_marker.color.b = 0.0;
    attractive_marker.color.a = 0.8;
    
    markers.markers.push_back(attractive_marker);
    
    // 排斥力箭头（红色）
    visualization_msgs::msg::Marker repulsive_marker;
    repulsive_marker.header.frame_id = robot_pose.header.frame_id;
    repulsive_marker.header.stamp = node_->now();
    repulsive_marker.ns = "repulsive_force";
    repulsive_marker.id = 1;
    repulsive_marker.type = visualization_msgs::msg::Marker::ARROW;
    repulsive_marker.action = visualization_msgs::msg::Marker::ADD;
    
    repulsive_marker.pose.position = robot_pose.pose.position;
    repulsive_marker.pose.position.z = 0.7;
    
    // 计算箭头方向
    double rep_angle = std::atan2(repulsive_force.y, repulsive_force.x);
    tf2::Quaternion rep_q;
    rep_q.setRPY(0, 0, rep_angle);
    repulsive_marker.pose.orientation = tf2::toMsg(rep_q);
    
    repulsive_marker.scale.x = std::min(2.0, std::hypot(repulsive_force.x, repulsive_force.y) * 0.1);
    repulsive_marker.scale.y = 0.1;
    repulsive_marker.scale.z = 0.1;
    
    repulsive_marker.color.r = 1.0;
    repulsive_marker.color.g = 0.0;
    repulsive_marker.color.b = 0.0;
    repulsive_marker.color.a = 0.8;
    
    markers.markers.push_back(repulsive_marker);
    
    // 合力箭头（蓝色）
    geometry_msgs::msg::Vector3 total_force;
    total_force.x = attractive_force.x + repulsive_force.x;
    total_force.y = attractive_force.y + repulsive_force.y;
    
    visualization_msgs::msg::Marker total_marker;
    total_marker.header.frame_id = robot_pose.header.frame_id;
    total_marker.header.stamp = node_->now();
    total_marker.ns = "total_force";
    total_marker.id = 2;
    total_marker.type = visualization_msgs::msg::Marker::ARROW;
    total_marker.action = visualization_msgs::msg::Marker::ADD;
    
    total_marker.pose.position = robot_pose.pose.position;
    total_marker.pose.position.z = 0.9;
    
    double total_angle = std::atan2(total_force.y, total_force.x);
    tf2::Quaternion total_q;
    total_q.setRPY(0, 0, total_angle);
    total_marker.pose.orientation = tf2::toMsg(total_q);
    
    total_marker.scale.x = std::min(2.0, std::hypot(total_force.x, total_force.y) * 0.2);
    total_marker.scale.y = 0.15;
    total_marker.scale.z = 0.15;
    
    total_marker.color.r = 0.0;
    total_marker.color.g = 0.0;
    total_marker.color.b = 1.0;
    total_marker.color.a = 0.9;
    
    markers.markers.push_back(total_marker);
    
    return markers;
}

visualization_msgs::msg::MarkerArray ObstacleAvoidance::createFieldMarkers(
    const geometry_msgs::msg::PoseStamped& robot_pose)
{
    visualization_msgs::msg::MarkerArray markers;
    
    // 创建势场影响区域可视化（圆圈）
    visualization_msgs::msg::Marker influence_circle;
    influence_circle.header.frame_id = robot_pose.header.frame_id;
    influence_circle.header.stamp = node_->now();
    influence_circle.ns = "influence_radius";
    influence_circle.id = 0;
    influence_circle.type = visualization_msgs::msg::Marker::CYLINDER;
    influence_circle.action = visualization_msgs::msg::Marker::ADD;
    
    influence_circle.pose.position = robot_pose.pose.position;
    influence_circle.pose.position.z = 0.01;
    influence_circle.pose.orientation.w = 1.0;
    
    influence_circle.scale.x = field_params_.influence_radius * 2.0;
    influence_circle.scale.y = field_params_.influence_radius * 2.0;
    influence_circle.scale.z = 0.02;
    
    influence_circle.color.r = 1.0;
    influence_circle.color.g = 1.0;
    influence_circle.color.b = 0.0;
    influence_circle.color.a = 0.1;
    
    markers.markers.push_back(influence_circle);
    
    // 创建安全区域可视化（圆圈）
    visualization_msgs::msg::Marker safety_circle;
    safety_circle.header.frame_id = robot_pose.header.frame_id;
    safety_circle.header.stamp = node_->now();
    safety_circle.ns = "safety_radius";
    safety_circle.id = 1;
    safety_circle.type = visualization_msgs::msg::Marker::CYLINDER;
    safety_circle.action = visualization_msgs::msg::Marker::ADD;
    
    safety_circle.pose.position = robot_pose.pose.position;
    safety_circle.pose.position.z = 0.02;
    safety_circle.pose.orientation.w = 1.0;
    
    safety_circle.scale.x = field_params_.safety_radius * 2.0;
    safety_circle.scale.y = field_params_.safety_radius * 2.0;
    safety_circle.scale.z = 0.02;
    
    safety_circle.color.r = 1.0;
    safety_circle.color.g = 0.0;
    safety_circle.color.b = 0.0;
    safety_circle.color.a = 0.2;
    
    markers.markers.push_back(safety_circle);
    
    return markers;
}

bool ObstacleAvoidance::detectBilateralObstacles(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                 const geometry_msgs::msg::Vector3& repulsive_force)
{
    if (!costmap_received_) {
        return false;
    }
    
    // 获取机器人在代价地图中的位置
    int robot_map_x = static_cast<int>((robot_pose.pose.position.x - current_costmap_.info.origin.position.x) / 
                                       current_costmap_.info.resolution);
    int robot_map_y = static_cast<int>((robot_pose.pose.position.y - current_costmap_.info.origin.position.y) / 
                                       current_costmap_.info.resolution);
    
    // 边界检查
    if (robot_map_x < 0 || robot_map_x >= static_cast<int>(current_costmap_.info.width) ||
        robot_map_y < 0 || robot_map_y >= static_cast<int>(current_costmap_.info.height)) {
        return false;
    }
    
    // 获取机器人当前朝向
    double robot_yaw = getRobotYaw(robot_pose);
    
    // 计算左右两侧的检测点（距离机器人一定距离）
    double check_distance = field_params_.influence_radius * 0.7; // 检测距离为影响半径的70%
    
    // 计算左侧检测点（相对于机器人朝向向左90度）
    double left_x = robot_pose.pose.position.x + check_distance * std::cos(robot_yaw + M_PI/2);
    double left_y = robot_pose.pose.position.y + check_distance * std::sin(robot_yaw + M_PI/2);
    
    // 计算右侧检测点（相对于机器人朝向向右90度）
    double right_x = robot_pose.pose.position.x + check_distance * std::cos(robot_yaw - M_PI/2);
    double right_y = robot_pose.pose.position.y + check_distance * std::sin(robot_yaw - M_PI/2);
    
    // 转换到地图坐标
    int left_map_x = static_cast<int>((left_x - current_costmap_.info.origin.position.x) / 
                                      current_costmap_.info.resolution);
    int left_map_y = static_cast<int>((left_y - current_costmap_.info.origin.position.y) / 
                                      current_costmap_.info.resolution);
    
    int right_map_x = static_cast<int>((right_x - current_costmap_.info.origin.position.x) / 
                                       current_costmap_.info.resolution);
    int right_map_y = static_cast<int>((right_y - current_costmap_.info.origin.position.y) / 
                                       current_costmap_.info.resolution);
    
    // 检查左右两侧是否都有高代价值
    bool left_obstacle = false;
    bool right_obstacle = false;
    
    // 检查左侧区域
    if (left_map_x >= 0 && left_map_x < static_cast<int>(current_costmap_.info.width) &&
        left_map_y >= 0 && left_map_y < static_cast<int>(current_costmap_.info.height)) {
        int left_index = left_map_y * current_costmap_.info.width + left_map_x;
        int8_t left_cost = current_costmap_.data[left_index];
        left_obstacle = (left_cost > costmap_threshold_ * field_params_.bilateral_threshold);
    }
    
    // 检查右侧区域  
    if (right_map_x >= 0 && right_map_x < static_cast<int>(current_costmap_.info.width) &&
        right_map_y >= 0 && right_map_y < static_cast<int>(current_costmap_.info.height)) {
        int right_index = right_map_y * current_costmap_.info.width + right_map_x;
        int8_t right_cost = current_costmap_.data[right_index];
        right_obstacle = (right_cost > costmap_threshold_ * field_params_.bilateral_threshold);
    }
    
    // 额外检查：排斥力的Y分量是否在左右之间振荡
    static double last_repulsive_y = 0.0;
    double current_repulsive_y = repulsive_force.y;
    bool force_oscillating = (last_repulsive_y * current_repulsive_y < 0) && 
                            (std::abs(current_repulsive_y) > field_params_.y_force_threshold * 0.5);
    last_repulsive_y = current_repulsive_y;
    
    // 当两侧都有障碍物或者力在振荡时，认为是双侧障碍物情况
    return (left_obstacle && right_obstacle) || force_oscillating;
}

bool ObstacleAvoidance::checkFrontObstacle(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    if (!scan_received_) {
        return false;
    }
    
    // 矩形检测区域参数
    double rect_width = 0.4;   // 宽度40cm
    double rect_length = 0.6;  // 长度60cm
    
    // 激光扫描参数
    double angle_min = current_scan_.angle_min;
    double angle_max = current_scan_.angle_max;
    double angle_increment = current_scan_.angle_increment;
    double range_min = current_scan_.range_min;
    double range_max = current_scan_.range_max;
    
    // 计算检测角度范围 (基于矩形宽度和长度)
    double max_detection_angle = std::atan2(rect_width / 2.0, rect_length);
    
    // 遍历激光扫描数据
    for (size_t i = 0; i < current_scan_.ranges.size(); ++i) {
        double angle = angle_min + i * angle_increment;
        double range = current_scan_.ranges[i];
        
        // 跳过无效数据
        if (std::isnan(range) || std::isinf(range) || range < range_min || range > range_max) {
            continue;
        }
        
        // 只检查前方扇形区域内的点
        if (std::abs(angle) <= max_detection_angle) {
            // 计算激光点在机器人坐标系中的位置
            double point_x = range * std::cos(angle);
            double point_y = range * std::sin(angle);
            
            // 检查点是否在矩形检测区域内
            if (point_x >= 0 && point_x <= rect_length && 
                std::abs(point_y) <= rect_width / 2.0) {
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "Front obstacle detected at scan angle %.2f rad, distance %.2f m", 
                         angle, range);
                return true;
            }
        }
    }
    
    return false;
}

void ObstacleAvoidance::publishFrontDetectionVisualization(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    visualization_msgs::msg::MarkerArray markers;
    
    // 获取机器人当前朝向
    double robot_yaw = getRobotYaw(robot_pose);
    
    // 矩形检测区域参数
    double rect_width = 0.4;   // 宽度40cm
    double rect_length = 0.6;  // 长度60cm
    
    // 创建矩形框可视化
    visualization_msgs::msg::Marker rect_marker;
    rect_marker.header.frame_id = robot_pose.header.frame_id;
    rect_marker.header.stamp = node_->now();
    rect_marker.ns = "front_detection_rectangle";
    rect_marker.id = 0;
    rect_marker.type = visualization_msgs::msg::Marker::CUBE;
    rect_marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 计算矩形中心位置（机器人前方rect_length/2的位置）
    double rect_center_x = robot_pose.pose.position.x + (rect_length / 2.0) * std::cos(robot_yaw);
    double rect_center_y = robot_pose.pose.position.y + (rect_length / 2.0) * std::sin(robot_yaw);
    
    rect_marker.pose.position.x = rect_center_x;
    rect_marker.pose.position.y = rect_center_y;
    rect_marker.pose.position.z = 0.05;  // 稍微抬高一点以便可视化
    
    // 设置矩形朝向与机器人相同
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_yaw);
    rect_marker.pose.orientation = tf2::toMsg(q);
    
    // 设置矩形尺寸
    rect_marker.scale.x = rect_length;  // 长度
    rect_marker.scale.y = rect_width;   // 宽度
    rect_marker.scale.z = 0.02;         // 高度（很薄的矩形）
    
    // 设置颜色（半透明黄色）
    rect_marker.color.r = 1.0;
    rect_marker.color.g = 1.0;
    rect_marker.color.b = 0.0;
    rect_marker.color.a = 0.7;
    
    markers.markers.push_back(rect_marker);
    
    // 发布标记
    front_detection_pub_->publish(markers);
}

} // namespace pure_pursuit_local_planner
