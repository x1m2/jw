#include "pure_pursuit_local_planner/local_path_planner.hpp"
#include "pure_pursuit_local_planner/param_utils.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <limits>
#include <tf2/utils.hpp>

namespace pure_pursuit_local_planner
{

LocalPathPlanner::LocalPathPlanner()
    : running_(false), global_path_received_(false), costmap_received_(false), 
      local_path_valid_(false), planning_frequency_(1.0), lookahead_distance_(3.0),
      min_lookahead_distance_(1.0), max_lookahead_distance_(5.0), last_goal_index_(-1),
      path_stability_counter_(0), path_similarity_threshold_(0.8),
      avoidance_active_(false), avoidance_completed_(false),
      avoidance_skip_current_goal_(false),
      avoidance_stage_(AvoidanceStage::IDLE),
      avoidance_side_(AvoidanceSide::NONE),
      avoidance_lateral_clearance_(0.8),
      avoidance_obstacle_forward_offset_(0.4),
      avoidance_return_min_distance_(1.5),
      avoidance_return_max_distance_(4.0),
      avoidance_search_step_distance_(0.3),
      avoidance_goal_tolerance_(0.35),
      avoidance_return_goal_clearance_(0.8),
      avoidance_return_obstacle_clearance_(2.0),
      avoidance_obstacle_threshold_(50.0)
{
}

LocalPathPlanner::~LocalPathPlanner()
{
    stop();
}

bool LocalPathPlanner::initialize(rclcpp::Node::SharedPtr node, 
                                 std::shared_ptr<tf2_ros::Buffer> tf_buffer)
{
    node_ = node;
    tf_buffer_ = tf_buffer;
    
    // 加载参数
    planning_frequency_ = declareOrGetParameter(node_, "local_planning_frequency", 1.0);
    lookahead_distance_ = declareOrGetParameter(node_, "local_lookahead_distance", 3.0);
    min_lookahead_distance_ = declareOrGetParameter(node_, "local_min_lookahead_distance", 1.0);
    max_lookahead_distance_ = declareOrGetParameter(node_, "local_max_lookahead_distance", 5.0);
    path_similarity_threshold_ = declareOrGetParameter(node_, "path_similarity_threshold", 0.8);
    global_frame_ = declareOrGetParameter<std::string>(node_, "global_frame", "map");
    robot_frame_ = declareOrGetParameter<std::string>(node_, "robot_frame", "base_link");
    transform_tolerance_ = declareOrGetParameter(node_, "transform_tolerance", 0.1);
    avoidance_lateral_clearance_ = declareOrGetParameter(node_, "avoidance_lateral_clearance", 0.8);
    avoidance_obstacle_forward_offset_ = declareOrGetParameter(node_, "avoidance_obstacle_forward_offset", 0.4);
    avoidance_return_min_distance_ = declareOrGetParameter(node_, "avoidance_return_min_distance", 1.5);
    avoidance_return_max_distance_ = declareOrGetParameter(node_, "avoidance_return_max_distance", 4.0);
    avoidance_search_step_distance_ = declareOrGetParameter(node_, "avoidance_search_step_distance", 0.3);
    avoidance_goal_tolerance_ = declareOrGetParameter(node_, "avoidance_goal_tolerance", 0.35);
    const double robot_width =
        declareOrGetParameter(node_, "front_obstacle_detector.robot_width", 0.55);
    const double robot_length =
        declareOrGetParameter(node_, "front_obstacle_detector.robot_length", 0.75);
    const double robot_safety_margin =
        declareOrGetParameter(node_, "front_obstacle_detector.safety_margin", 0.10);
    const double default_return_goal_clearance =
        std::hypot(robot_width * 0.5, robot_length * 0.5) + robot_safety_margin;
    avoidance_return_goal_clearance_ = declareOrGetParameter(
        node_, "avoidance_return_goal_clearance",
        default_return_goal_clearance);
    avoidance_return_obstacle_clearance_ = declareOrGetParameter(
        node_, "avoidance_return_obstacle_clearance", 2.0);
    avoidance_obstacle_threshold_ = static_cast<double>(
        declareOrGetParameter(node_, "front_obstacle_detector.obstacle_threshold", 50));
    
    // 初始化A*规划器
    astar_planner_ = std::make_unique<AStarPlanner>();
    if (!astar_planner_->initialize(node_))
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to initialize A* planner");
        return false;
    }
    
    // 初始化发布器
    local_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_path_astar", 1);
    goal_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("local_goal_marker", 1);
    
    // 启动规划线程
    running_ = true;
    planning_thread_ = std::thread(&LocalPathPlanner::planningLoop, this);
    
    RCLCPP_INFO(node_->get_logger(), "Local Path Planner initialized with frequency: %.1f Hz", planning_frequency_);
    RCLCPP_INFO(node_->get_logger(),
                "Obstacle-based avoidance: clearance=%.2f, forward_offset=%.2f, return=[%.2f, %.2f], step=%.2f, tolerance=%.2f, return_goal_clearance=%.2f, return_obstacle_clearance=%.2f",
                avoidance_lateral_clearance_, avoidance_obstacle_forward_offset_,
                avoidance_return_min_distance_, avoidance_return_max_distance_,
                avoidance_search_step_distance_, avoidance_goal_tolerance_,
                avoidance_return_goal_clearance_,
                avoidance_return_obstacle_clearance_);
    return true;
}

void LocalPathPlanner::setGlobalPath(const nav_msgs::msg::Path& global_path)
{
    std::lock_guard<std::mutex> lock(global_path_mutex_);
    global_path_ = global_path;
    global_path_received_ = !global_path.poses.empty();
    
    // 重置目标点索引，因为全局路径已更新
    last_goal_index_ = -1;
    
    // 将全局路径传递给A*规划器
    if (astar_planner_ && global_path_received_)
    {
        astar_planner_->setGlobalPath(global_path);
        RCLCPP_DEBUG(node_->get_logger(), "Global path passed to A* planner with %zu poses", global_path.poses.size());
    }
    
    if (global_path_received_)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Local planner received global path with %zu poses", global_path_.poses.size());
    }
}

void LocalPathPlanner::updateCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_ = costmap;
    costmap_received_ = true;
    
    if (astar_planner_)
    {
        astar_planner_->setCostmap(costmap);
    }
}

nav_msgs::msg::Path LocalPathPlanner::getLocalPath() const
{
    std::lock_guard<std::mutex> lock(path_mutex_);
    return local_path_;
}

bool LocalPathPlanner::hasValidLocalPath() const
{
    return local_path_valid_;
}

bool LocalPathPlanner::startAvoidance(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Point& obstacle_center,
    AvoidanceSide side)
{
    if (side == AvoidanceSide::NONE)
    {
        return false;
    }

    geometry_msgs::msg::Point side_goal;
    if (!buildAvoidanceSideGoal(robot_pose, obstacle_center, side, side_goal))
    {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        avoidance_active_ = false;
        avoidance_completed_ = false;
        avoidance_skip_current_goal_ = false;
        avoidance_stage_ = AvoidanceStage::FAILED;
        avoidance_side_ = side;
        avoidance_obstacle_center_ = obstacle_center;
        local_path_valid_ = false;
        RCLCPP_WARN(node_->get_logger(),
                    "[AVOID_PLAN] Failed to build obstacle-based side goal");
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        avoidance_active_ = true;
        avoidance_completed_ = false;
        avoidance_skip_current_goal_ = false;
        avoidance_stage_ = AvoidanceStage::SIDE_GOAL;
        avoidance_side_ = side;
        avoidance_obstacle_center_ = obstacle_center;
        avoidance_side_goal_ = side_goal;
        avoidance_return_goal_ = geometry_msgs::msg::Point();
    }

    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        local_path_valid_ = false;
        local_path_.poses.clear();
        previous_local_path_.poses.clear();
    }
    path_stability_counter_ = 0;

    RCLCPP_WARN(node_->get_logger(),
                "[AVOID_PLAN] Started %s avoidance: obstacle=(%.3f, %.3f), side_goal=(%.3f, %.3f)",
                avoidanceSideToString(side), obstacle_center.x,
                obstacle_center.y, side_goal.x, side_goal.y);
    return true;
}

void LocalPathPlanner::resetAvoidance()
{
    std::lock_guard<std::mutex> avoidance_lock(avoidance_mutex_);
    avoidance_active_ = false;
    avoidance_completed_ = false;
    avoidance_skip_current_goal_ = false;
    avoidance_stage_ = AvoidanceStage::IDLE;
    avoidance_side_ = AvoidanceSide::NONE;
    avoidance_obstacle_center_ = geometry_msgs::msg::Point();

    std::lock_guard<std::mutex> path_lock(path_mutex_);
    local_path_valid_ = false;
    local_path_.poses.clear();
    previous_local_path_.poses.clear();
    path_stability_counter_ = 0;
}

bool LocalPathPlanner::isAvoidanceActive() const
{
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    return avoidance_active_;
}

bool LocalPathPlanner::isAvoidanceCompleted() const
{
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    return avoidance_completed_;
}

bool LocalPathPlanner::isAvoidanceFailed() const
{
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    return avoidance_stage_ == AvoidanceStage::FAILED;
}

bool LocalPathPlanner::shouldSkipCurrentGoal() const
{
    std::lock_guard<std::mutex> lock(avoidance_mutex_);
    return avoidance_skip_current_goal_;
}

void LocalPathPlanner::stop()
{
    running_ = false;
    if (planning_thread_.joinable())
    {
        planning_thread_.join();
    }
}

void LocalPathPlanner::planningLoop()
{
    rclcpp::Rate rate(planning_frequency_);
    
    while (running_ && rclcpp::ok())
    {
        try
        {
            computeLocalPath();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception in local path planning: %s", e.what());
        }
        
        rate.sleep();
    }
}

bool LocalPathPlanner::computeLocalPath()
{
    // 检查必要的数据是否可用
    if (!global_path_received_ || !costmap_received_)
    {
        local_path_valid_ = false;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);
        if (!avoidance_active_)
        {
            bool should_publish_empty = false;
            {
                std::lock_guard<std::mutex> path_lock(path_mutex_);
                should_publish_empty = local_path_valid_ ||
                                       !local_path_.poses.empty() ||
                                       !previous_local_path_.poses.empty();
                local_path_valid_ = false;
                local_path_.poses.clear();
                previous_local_path_.poses.clear();
            }

            if (should_publish_empty && local_path_pub_)
            {
                nav_msgs::msg::Path empty_path;
                empty_path.header.frame_id = global_frame_;
                empty_path.header.stamp = node_->now();
                local_path_pub_->publish(empty_path);
            }

            return false;
        }
    }
    
    // 获取机器人当前位姿
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose))
    {
        local_path_valid_ = false;
        return false;
    }
    
    // 复制全局路径（线程安全）
    nav_msgs::msg::Path global_path_copy;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        global_path_copy = global_path_;
    }
    
    if (global_path_copy.poses.empty())
    {
        local_path_valid_ = false;
        return false;
    }
    
    geometry_msgs::msg::Point goal_point;
    bool using_avoidance_goal = false;
    {
        std::lock_guard<std::mutex> lock(avoidance_mutex_);

        if (avoidance_active_ && avoidance_stage_ == AvoidanceStage::SIDE_GOAL)
        {
            const double side_goal_distance =
                calculateDistance(robot_pose.pose.position, avoidance_side_goal_);
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 500,
                "[AVOID_PROGRESS] stage=SIDE_GOAL, robot=(%.3f, %.3f), "
                "side_goal=(%.3f, %.3f), dist=%.3f, tolerance=%.3f",
                robot_pose.pose.position.x, robot_pose.pose.position.y,
                avoidance_side_goal_.x, avoidance_side_goal_.y,
                side_goal_distance, avoidance_goal_tolerance_);

            if (side_goal_distance <= avoidance_goal_tolerance_)
            {
                geometry_msgs::msg::Point return_goal;
                const int closest_index = findClosestPointOnGlobalPath(robot_pose);
                if (closest_index < 0 ||
                    !buildReturnGoalFromPoint(robot_pose.pose.position,
                                              closest_index, return_goal))
                {
                    avoidance_active_ = false;
                    avoidance_completed_ = true;
                    avoidance_skip_current_goal_ = true;
                    avoidance_stage_ = AvoidanceStage::COMPLETED;
                    {
                        std::lock_guard<std::mutex> path_lock(path_mutex_);
                        local_path_valid_ = false;
                        local_path_.poses.clear();
                        previous_local_path_.poses.clear();
                    }
                    RCLCPP_WARN(node_->get_logger(),
                                "[AVOID_PLAN] Side goal reached, no reachable forward path point; avoidance completed");
                    return false;
                }

                avoidance_return_goal_ = return_goal;
                avoidance_skip_current_goal_ = false;
                avoidance_stage_ = AvoidanceStage::RETURN_GOAL;
                {
                    std::lock_guard<std::mutex> path_lock(path_mutex_);
                    local_path_valid_ = false;
                    local_path_.poses.clear();
                    previous_local_path_.poses.clear();
                }
                path_stability_counter_ = 0;
                RCLCPP_WARN(node_->get_logger(),
                            "[AVOID_PLAN] Side goal reached, switching to return goal=(%.3f, %.3f)",
                            avoidance_return_goal_.x, avoidance_return_goal_.y);
            }
        }

        if (avoidance_active_ && avoidance_stage_ == AvoidanceStage::RETURN_GOAL)
        {
            const double return_goal_distance =
                calculateDistance(robot_pose.pose.position, avoidance_return_goal_);
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 500,
                "[AVOID_PROGRESS] stage=RETURN_GOAL, robot=(%.3f, %.3f), "
                "return_goal=(%.3f, %.3f), dist=%.3f, tolerance=%.3f",
                robot_pose.pose.position.x, robot_pose.pose.position.y,
                avoidance_return_goal_.x, avoidance_return_goal_.y,
                return_goal_distance, avoidance_goal_tolerance_);

            if (return_goal_distance <= avoidance_goal_tolerance_)
            {
                avoidance_active_ = false;
                avoidance_completed_ = true;
                avoidance_skip_current_goal_ = false;
                avoidance_stage_ = AvoidanceStage::COMPLETED;
                {
                    std::lock_guard<std::mutex> path_lock(path_mutex_);
                    local_path_valid_ = false;
                    local_path_.poses.clear();
                    previous_local_path_.poses.clear();
                }
                RCLCPP_WARN(node_->get_logger(),
                            "[AVOID_PLAN] Return goal reached, avoidance completed");
                return false;
            }
        }

        if (avoidance_active_ && avoidance_stage_ == AvoidanceStage::SIDE_GOAL)
        {
            goal_point = avoidance_side_goal_;
            using_avoidance_goal = true;
        }
        else if (avoidance_active_ && avoidance_stage_ == AvoidanceStage::RETURN_GOAL)
        {
            goal_point = avoidance_return_goal_;
            using_avoidance_goal = true;
        }
    }

    if (!using_avoidance_goal)
    {
        std::lock_guard<std::mutex> path_lock(path_mutex_);
        local_path_valid_ = false;
        local_path_.poses.clear();
        previous_local_path_.poses.clear();
        return false;
    }

    auto get_costmap_cost = [this](const geometry_msgs::msg::Point& point) -> int {
        if (!costmap_received_ || costmap_.info.resolution <= 0.0)
        {
            return -999;
        }

        const int grid_x = static_cast<int>(
            (point.x - costmap_.info.origin.position.x) / costmap_.info.resolution);
        const int grid_y = static_cast<int>(
            (point.y - costmap_.info.origin.position.y) / costmap_.info.resolution);

        if (grid_x < 0 || grid_y < 0 ||
            grid_x >= static_cast<int>(costmap_.info.width) ||
            grid_y >= static_cast<int>(costmap_.info.height))
        {
            return -998;
        }

        const int index =
            grid_y * static_cast<int>(costmap_.info.width) + grid_x;
        if (index < 0 || index >= static_cast<int>(costmap_.data.size()))
        {
            return -997;
        }

        return static_cast<int>(costmap_.data[index]);
    };

    const bool start_in_obstacle = isPointInObstacle(robot_pose.pose.position);
    const bool goal_in_obstacle = isPointInObstacle(goal_point);
    const int start_cost = get_costmap_cost(robot_pose.pose.position);
    const int goal_cost = get_costmap_cost(goal_point);
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[LOCAL_PLAN_DEBUG] mode=%s, robot=(%.3f, %.3f), goal=(%.3f, %.3f), "
        "start_obstacle=%d, goal_obstacle=%d, start_cost=%d, goal_cost=%d",
        using_avoidance_goal ? "AVOIDANCE" : "GLOBAL_LOOKAHEAD",
        robot_pose.pose.position.x, robot_pose.pose.position.y,
        goal_point.x, goal_point.y,
        start_in_obstacle ? 1 : 0, goal_in_obstacle ? 1 : 0,
        start_cost, goal_cost);
    
    // 检查目标点是否在障碍物中
    if (goal_in_obstacle)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Goal point is in obstacle, local path planning failed");
        local_path_valid_ = false;
        return false;
    }
    
    // 使用A*规划局部路径
    nav_msgs::msg::Path new_local_path;
    bool planning_success = astar_planner_->planPath(robot_pose.pose.position, goal_point, new_local_path);
    
    if (planning_success && !new_local_path.poses.empty())
    {
        // 路径稳定性检查：如果新路径与之前路径相似，保持之前的路径以减少抖动
        if (shouldKeepPreviousPath(new_local_path))
        {
            RCLCPP_DEBUG(node_->get_logger(), "Keeping previous path to reduce oscillation");
            // 发布当前稳定的路径
            if (local_path_valid_)
            {
                local_path_pub_->publish(local_path_);
                publishGoalMarker(goal_point);
            }
            return local_path_valid_;
        }
        
        // 绕障路径很短且贴近障碍物，简单移动平均会把折角拉偏，甚至把点拉回障碍附近。
        // 这里直接使用 A* 原始路径，保证可视化和跟踪都忠实于栅格规划结果。
        nav_msgs::msg::Path planned_path = new_local_path;
        
        // 更新局部路径
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            previous_local_path_ = local_path_;  // 保存之前的路径
            local_path_ = planned_path;
            local_path_valid_ = true;
        }
        
        // 重置稳定性计数器
        path_stability_counter_ = 0;
        
        // 发布局部路径和可视化
        local_path_pub_->publish(planned_path);
        publishGoalMarker(goal_point);
        astar_planner_->publishVisualization(robot_pose.pose.position, goal_point, planned_path);
        
        RCLCPP_DEBUG(node_->get_logger(), "Local path computed successfully with %zu poses", planned_path.poses.size());
        return true;
    }
    else
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "A* planning failed");
        local_path_valid_ = false;
        return false;
    }
}

bool LocalPathPlanner::getRobotPose(geometry_msgs::msg::PoseStamped& robot_pose)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            global_frame_, robot_frame_, tf2::TimePointZero, 
            tf2::durationFromSec(transform_tolerance_));
        
        robot_pose.header.frame_id = global_frame_;
        robot_pose.header.stamp = transform.header.stamp;
        robot_pose.pose.position.x = transform.transform.translation.x;
        robot_pose.pose.position.y = transform.transform.translation.y;
        robot_pose.pose.position.z = transform.transform.translation.z;
        robot_pose.pose.orientation = transform.transform.rotation;
        
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Failed to get robot pose: %s", ex.what());
        return false;
    }
}

geometry_msgs::msg::Point LocalPathPlanner::findGoalPoint(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    geometry_msgs::msg::Point goal_point;
    
    // 复制全局路径（线程安全）
    nav_msgs::msg::Path global_path_copy;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        global_path_copy = global_path_;
    }
    
    // 找到最近的路径点
    int closest_index = findClosestPointOnGlobalPath(robot_pose);
    
    if (closest_index < 0)
    {
        // 如果没找到合适的点，使用路径终点
        goal_point = global_path_copy.poses.back().pose.position;
        return goal_point;
    }
    
    // 从最近点开始，沿着路径向前寻找前瞻距离内的点
    double accumulated_distance = 0.0;
    int goal_index = closest_index;
    
    for (size_t i = closest_index; i < global_path_copy.poses.size() - 1; ++i)
    {
        double segment_distance = calculateDistance(
            global_path_copy.poses[i].pose.position,
            global_path_copy.poses[i + 1].pose.position);
        
        accumulated_distance += segment_distance;
        
        if (accumulated_distance >= lookahead_distance_)
        {
            goal_index = i + 1;
            break;
        }
        
        goal_index = i + 1;
    }
    
    // 确保不超出路径范围
    goal_index = std::min(goal_index, static_cast<int>(global_path_copy.poses.size() - 1));
    
    // 防止目标点倒退：确保新的目标点索引不小于上一次的索引
    if (last_goal_index_ >= 0 && goal_index < last_goal_index_)
    {
        goal_index = last_goal_index_;
        RCLCPP_DEBUG(node_->get_logger(), "Prevented goal point regression, using previous index: %d", goal_index);
    }
    
    goal_point = global_path_copy.poses[goal_index].pose.position;
    
    // 如果目标点在障碍物中，继续向前寻找（增加跳跃距离）
    int search_index = goal_index;
    bool found_obstacle = false;
    
    while (search_index < static_cast<int>(global_path_copy.poses.size()) && 
           isPointInObstacle(goal_point))
    {
        found_obstacle = true;
        search_index++;
        if (search_index < static_cast<int>(global_path_copy.poses.size()))
        {
            goal_point = global_path_copy.poses[search_index].pose.position;
        }
        else
        {
            break;
        }
    }
    
    // 如果遇到障碍物，额外向前跳跃更多距离以避开障碍物区域
    if (found_obstacle && search_index < static_cast<int>(global_path_copy.poses.size()))
    {
        // 额外向前跳跃 max_lookahead_distance_ 的一半距离
        double extra_jump_distance = max_lookahead_distance_ * 0.3;
        double accumulated_extra_distance = 0.0;
        
        for (size_t i = search_index; i < global_path_copy.poses.size() - 1; ++i)
        {
            double segment_distance = calculateDistance(
                global_path_copy.poses[i].pose.position,
                global_path_copy.poses[i + 1].pose.position);
            
            accumulated_extra_distance += segment_distance;
            
            if (accumulated_extra_distance >= extra_jump_distance)
            {
                search_index = i + 1;
                break;
            }
            
            search_index = i + 1;
        }
        
        // 确保索引在范围内，并且选择的点不在障碍物中
        search_index = std::min(search_index, static_cast<int>(global_path_copy.poses.size() - 1));
        geometry_msgs::msg::Point candidate_point = global_path_copy.poses[search_index].pose.position;
        
        if (!isPointInObstacle(candidate_point))
        {
            goal_point = candidate_point;
            goal_index = search_index;
            RCLCPP_DEBUG(node_->get_logger(), "Extra jump due to obstacle, new goal index: %d", goal_index);
        }
    }
    
    // 更新上一次的目标点索引
    last_goal_index_ = goal_index;
    
    return goal_point;
}

bool LocalPathPlanner::buildAvoidanceSideGoal(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Point& obstacle_center,
    AvoidanceSide side,
    geometry_msgs::msg::Point& side_goal)
{
    if (!astar_planner_ || !global_path_received_ || !costmap_received_)
    {
        return false;
    }

    nav_msgs::msg::Path global_path_copy;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        global_path_copy = global_path_;
    }

    if (global_path_copy.poses.size() < 2)
    {
        return false;
    }

    int closest_index = findClosestPointOnGlobalPath(robot_pose);
    if (closest_index < 0)
    {
        closest_index = 0;
    }

    double path_yaw = 0.0;
    if (!getPathYawAtIndex(global_path_copy, closest_index, path_yaw))
    {
        path_yaw = tf2::getYaw(robot_pose.pose.orientation);
    }

    const double side_sign = (side == AvoidanceSide::LEFT) ? 1.0 : -1.0;
    const double path_x = std::cos(path_yaw);
    const double path_y = std::sin(path_yaw);
    const double normal_x = -std::sin(path_yaw) * side_sign;
    const double normal_y = std::cos(path_yaw) * side_sign;

    const double base_clearance = std::max(0.2, avoidance_lateral_clearance_);
    const double base_forward =
        std::max(0.0, avoidance_obstacle_forward_offset_);

    for (int i = 0; i < 4; ++i)
    {
        const double clearance = base_clearance + 0.2 * i;
        geometry_msgs::msg::Point candidate_side;
        candidate_side.x = obstacle_center.x + path_x * base_forward +
                           normal_x * clearance;
        candidate_side.y = obstacle_center.y + path_y * base_forward +
                           normal_y * clearance;
        candidate_side.z = obstacle_center.z;

        if (isPointInObstacle(candidate_side))
        {
            continue;
        }

        nav_msgs::msg::Path side_path;
        if (!astar_planner_->planPath(robot_pose.pose.position,
                                      candidate_side, side_path) ||
            side_path.poses.empty())
        {
            continue;
        }

        side_goal = candidate_side;
        const double side_dx = side_goal.x - robot_pose.pose.position.x;
        const double side_dy = side_goal.y - robot_pose.pose.position.y;
        RCLCPP_WARN(
            node_->get_logger(),
            "[AVOID_PLAN] selected side=%s obstacle=(%.3f, %.3f), path_yaw=%.1f deg, side_yaw=%.1f deg, side_goal=(%.3f, %.3f)",
            avoidanceSideToString(side), obstacle_center.x, obstacle_center.y,
            path_yaw * 180.0 / M_PI,
            std::atan2(side_dy, side_dx) * 180.0 / M_PI,
            side_goal.x, side_goal.y);
        return true;
    }

    return false;
}

bool LocalPathPlanner::buildReturnGoalFromPoint(
    const geometry_msgs::msg::Point& start_point,
    int closest_index,
    geometry_msgs::msg::Point& return_goal)
{
    nav_msgs::msg::Path global_path_copy;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        global_path_copy = global_path_;
    }

    if (closest_index < 0 ||
        closest_index >= static_cast<int>(global_path_copy.poses.size()))
    {
        return false;
    }

    auto try_return_candidate =
        [this, &start_point, &return_goal](
            const geometry_msgs::msg::Point& candidate_return) -> bool {
            const double obstacle_distance =
                calculateDistance(candidate_return, avoidance_obstacle_center_);
            if (obstacle_distance < avoidance_return_obstacle_clearance_)
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[AVOID_PLAN] reject return goal=(%.3f, %.3f): too close to obstacle center=(%.3f, %.3f), dist=%.2fm < %.2fm",
                    candidate_return.x, candidate_return.y,
                    avoidance_obstacle_center_.x, avoidance_obstacle_center_.y,
                    obstacle_distance, avoidance_return_obstacle_clearance_);
                return false;
            }

            if (isPointInObstacle(candidate_return))
            {
                return false;
            }

            if (!isPointSafeForRobot(candidate_return,
                                     avoidance_return_goal_clearance_))
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "[AVOID_PLAN] reject return goal=(%.3f, %.3f): obstacle within robot clearance %.2fm",
                    candidate_return.x, candidate_return.y,
                    avoidance_return_goal_clearance_);
                return false;
            }

            nav_msgs::msg::Path return_path;
            if (!astar_planner_->planPath(start_point, candidate_return,
                                          return_path) ||
                return_path.poses.empty())
            {
                return false;
            }

            return_goal = candidate_return;
            return true;
        };

    for (int candidate_index = closest_index + 1;
         candidate_index < static_cast<int>(global_path_copy.poses.size());
         ++candidate_index)
    {
        geometry_msgs::msg::Point candidate_return =
            global_path_copy.poses[candidate_index].pose.position;
        if (!try_return_candidate(candidate_return))
        {
            continue;
        }

        RCLCPP_WARN(node_->get_logger(),
                    "[AVOID_PLAN] return goal idx=%d forward path point: (%.3f, %.3f)",
                    candidate_index,
                    return_goal.x, return_goal.y);
        return true;
    }

    const int target_index = static_cast<int>(global_path_copy.poses.size()) - 1;
    const geometry_msgs::msg::Point target_goal =
        global_path_copy.poses[target_index].pose.position;
    if (try_return_candidate(target_goal))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[AVOID_PLAN] return goal idx=%d target point fallback: (%.3f, %.3f)",
                    target_index, return_goal.x, return_goal.y);
        return true;
    }

    return false;
}

bool LocalPathPlanner::findGlobalPathPointAhead(
    const nav_msgs::msg::Path& path,
    int closest_index,
    double target_distance,
    int& goal_index)
{
    if (path.poses.empty() || closest_index < 0 ||
        closest_index >= static_cast<int>(path.poses.size()))
    {
        return false;
    }

    double accumulated_distance = 0.0;
    goal_index = closest_index;
    for (size_t i = static_cast<size_t>(closest_index);
         i + 1 < path.poses.size(); ++i)
    {
        accumulated_distance += calculateDistance(
            path.poses[i].pose.position, path.poses[i + 1].pose.position);
        goal_index = static_cast<int>(i + 1);
        if (accumulated_distance >= target_distance)
        {
            return true;
        }
    }

    return false;
}

bool LocalPathPlanner::getPathYawAtIndex(
    const nav_msgs::msg::Path& path,
    int index,
    double& yaw) const
{
    if (path.poses.size() < 2 || index < 0 ||
        index >= static_cast<int>(path.poses.size()))
    {
        return false;
    }

    int prev_index = std::max(0, index - 1);
    int next_index = std::min(static_cast<int>(path.poses.size()) - 1,
                              index + 1);
    if (prev_index == next_index)
    {
        return false;
    }

    const auto& p0 = path.poses[prev_index].pose.position;
    const auto& p1 = path.poses[next_index].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    if (std::hypot(dx, dy) < 1e-6)
    {
        return false;
    }

    yaw = std::atan2(dy, dx);
    return true;
}

const char* LocalPathPlanner::avoidanceSideToString(AvoidanceSide side) const
{
    switch (side)
    {
        case AvoidanceSide::LEFT:
            return "LEFT";
        case AvoidanceSide::RIGHT:
            return "RIGHT";
        default:
            return "NONE";
    }
}

const char* LocalPathPlanner::avoidanceStageToString(AvoidanceStage stage) const
{
    switch (stage)
    {
        case AvoidanceStage::SIDE_GOAL:
            return "SIDE_GOAL";
        case AvoidanceStage::RETURN_GOAL:
            return "RETURN_GOAL";
        case AvoidanceStage::COMPLETED:
            return "COMPLETED";
        case AvoidanceStage::FAILED:
            return "FAILED";
        default:
            return "IDLE";
    }
}

int LocalPathPlanner::findClosestPointOnGlobalPath(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    // 复制全局路径（线程安全）
    nav_msgs::msg::Path global_path_copy;
    {
        std::lock_guard<std::mutex> lock(global_path_mutex_);
        global_path_copy = global_path_;
    }
    
    if (global_path_copy.poses.empty())
        return -1;
    
    // 方法1：找到机器人在路径上的投影点
    int best_index = findProjectionOnPath(robot_pose, global_path_copy);
    
    // 方法2：如果投影失败，使用改进的最近点算法
    if (best_index < 0)
    {
        best_index = findBestPathPoint(robot_pose, global_path_copy);
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "Selected path point index: %d", best_index);
    return best_index;
}

bool LocalPathPlanner::isPointInObstacle(const geometry_msgs::msg::Point& point)
{
    if (!astar_planner_)
        return true;
    
    return astar_planner_->isObstacle(point);
}

bool LocalPathPlanner::isPointSafeForRobot(
    const geometry_msgs::msg::Point& point,
    double clearance_radius) const
{
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    if (!costmap_received_ || costmap_.info.resolution <= 0.0 ||
        costmap_.data.empty())
    {
        return false;
    }

    const double resolution = costmap_.info.resolution;
    const int center_x = static_cast<int>(
        (point.x - costmap_.info.origin.position.x) / resolution);
    const int center_y = static_cast<int>(
        (point.y - costmap_.info.origin.position.y) / resolution);
    const int width = static_cast<int>(costmap_.info.width);
    const int height = static_cast<int>(costmap_.info.height);
    if (center_x < 0 || center_y < 0 || center_x >= width ||
        center_y >= height)
    {
        return false;
    }

    const int radius_cells =
        std::max(1, static_cast<int>(std::ceil(clearance_radius / resolution)));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx)
        {
            const double cell_distance =
                std::hypot(static_cast<double>(dx),
                           static_cast<double>(dy)) *
                resolution;
            if (cell_distance > clearance_radius)
            {
                continue;
            }

            const int grid_x = center_x + dx;
            const int grid_y = center_y + dy;
            if (grid_x < 0 || grid_y < 0 || grid_x >= width ||
                grid_y >= height)
            {
                return false;
            }

            const int index = grid_y * width + grid_x;
            if (index < 0 || index >= static_cast<int>(costmap_.data.size()))
            {
                return false;
            }

            const int cost = costmap_.data[index];
            if (cost < 0 || cost >= avoidance_obstacle_threshold_)
            {
                return false;
            }
        }
    }

    return true;
}

double LocalPathPlanner::calculateDistance(const geometry_msgs::msg::Point& p1, 
                                          const geometry_msgs::msg::Point& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + 
                    (p1.y - p2.y) * (p1.y - p2.y));
}

int LocalPathPlanner::findProjectionOnPath(const geometry_msgs::msg::PoseStamped& robot_pose, 
                                          const nav_msgs::msg::Path& path)
{
    if (path.poses.size() < 2)
        return -1;
    
    double min_distance_to_line = std::numeric_limits<double>::max();
    int best_segment_index = -1;
    
    // 找到机器人到路径线段的最短距离
    for (size_t i = 0; i < path.poses.size() - 1; ++i)
    {
        double distance = pointToLineDistance(
            robot_pose.pose.position,
            path.poses[i].pose.position,
            path.poses[i + 1].pose.position);
        
        if (distance < min_distance_to_line)
        {
            min_distance_to_line = distance;
            best_segment_index = i;
        }
    }
    
    if (best_segment_index < 0)
        return -1;
    
    // 在最佳线段上找投影点，返回更合适的端点
    geometry_msgs::msg::Point projection = projectPointOnLine(
        robot_pose.pose.position,
        path.poses[best_segment_index].pose.position,
        path.poses[best_segment_index + 1].pose.position);
    
    // 判断投影点更接近哪个端点
    double dist_to_start = calculateDistance3D(projection, path.poses[best_segment_index].pose.position);
    double dist_to_end = calculateDistance3D(projection, path.poses[best_segment_index + 1].pose.position);
    
    // 考虑路径方向：优先选择前进方向的点
    return (dist_to_end <= dist_to_start) ? best_segment_index + 1 : best_segment_index;
}

int LocalPathPlanner::findBestPathPoint(const geometry_msgs::msg::PoseStamped& robot_pose, 
                                       const nav_msgs::msg::Path& path)
{
    double min_distance = std::numeric_limits<double>::max();
    int closest_index = 0;
    
    // 考虑路径连续性的权重最近点算法
    for (size_t i = 0; i < path.poses.size(); ++i)
    {
        // 基础距离（3D）
        double distance_3d = calculateDistance3D(
            robot_pose.pose.position,
            path.poses[i].pose.position);
        
        // 路径方向性权重：优先选择前进方向的点
        double direction_weight = 1.0;
        if (i > 0)
        {
            // 计算机器人到当前点的方向向量
            double robot_to_point_x = path.poses[i].pose.position.x - robot_pose.pose.position.x;
            double robot_to_point_y = path.poses[i].pose.position.y - robot_pose.pose.position.y;
            
            // 计算路径前进方向向量
            double path_direction_x = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
            double path_direction_y = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
            
            // 计算方向一致性
            double dot_product = robot_to_point_x * path_direction_x + robot_to_point_y * path_direction_y;
            
            if (dot_product > 0)
            {
                direction_weight = 0.8;  // 前进方向，降低权重（优先）
            }
            else
            {
                direction_weight = 1.2;  // 后退方向，增加权重（惩罚）
            }
        }
        
        double weighted_distance = distance_3d * direction_weight;
        
        if (weighted_distance < min_distance)
        {
            min_distance = weighted_distance;
            closest_index = i;
        }
    }
    
    return closest_index;
}

double LocalPathPlanner::calculateDistance3D(const geometry_msgs::msg::Point& p1, 
                                            const geometry_msgs::msg::Point& p2)
{
    return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + 
                    (p1.y - p2.y) * (p1.y - p2.y) + 
                    (p1.z - p2.z) * (p1.z - p2.z));
}

double LocalPathPlanner::pointToLineDistance(const geometry_msgs::msg::Point& point,
                                            const geometry_msgs::msg::Point& line_start,
                                            const geometry_msgs::msg::Point& line_end)
{
    // 计算点到线段的最短距离（3D）
    double A = point.x - line_start.x;
    double B = point.y - line_start.y;
    double C = point.z - line_start.z;
    
    double D = line_end.x - line_start.x;
    double E = line_end.y - line_start.y;
    double F = line_end.z - line_start.z;
    
    double dot = A * D + B * E + C * F;
    double len_sq = D * D + E * E + F * F;
    
    if (len_sq == 0)
    {
        // 线段退化为点
        return calculateDistance3D(point, line_start);
    }
    
    double param = dot / len_sq;
    
    geometry_msgs::msg::Point closest;
    if (param < 0)
    {
        closest = line_start;
    }
    else if (param > 1)
    {
        closest = line_end;
    }
    else
    {
        closest.x = line_start.x + param * D;
        closest.y = line_start.y + param * E;
        closest.z = line_start.z + param * F;
    }
    
    return calculateDistance3D(point, closest);
}

geometry_msgs::msg::Point LocalPathPlanner::projectPointOnLine(const geometry_msgs::msg::Point& point,
                                                         const geometry_msgs::msg::Point& line_start,
                                                         const geometry_msgs::msg::Point& line_end)
{
    double A = point.x - line_start.x;
    double B = point.y - line_start.y;
    double C = point.z - line_start.z;
    
    double D = line_end.x - line_start.x;
    double E = line_end.y - line_start.y;
    double F = line_end.z - line_start.z;
    
    double dot = A * D + B * E + C * F;
    double len_sq = D * D + E * E + F * F;
    
    geometry_msgs::msg::Point projection;
    if (len_sq == 0)
    {
        projection = line_start;
    }
    else
    {
        double param = std::max(0.0, std::min(1.0, dot / len_sq));
        projection.x = line_start.x + param * D;
        projection.y = line_start.y + param * E;
        projection.z = line_start.z + param * F;
    }
    
    return projection;
}

void LocalPathPlanner::publishGoalMarker(const geometry_msgs::msg::Point& goal_point)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "local_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = goal_point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;  // 蓝色表示局部目标点
    
    goal_marker_pub_->publish(marker);
}

double LocalPathPlanner::calculatePathSimilarity(const nav_msgs::msg::Path& path1, const nav_msgs::msg::Path& path2)
{
    if (path1.poses.empty() || path2.poses.empty())
    {
        return 0.0;
    }
    
    // 计算路径相似度基于关键点的距离
    size_t min_size = std::min(path1.poses.size(), path2.poses.size());
    if (min_size == 0) return 0.0;
    
    double total_distance = 0.0;
    double max_allowed_distance = 2.0;  // 最大允许距离
    
    // 采样关键点进行比较（避免计算量过大）
    size_t step = std::max(1, static_cast<int>(min_size / 10));  // 最多取10个采样点
    size_t comparison_points = 0;
    
    for (size_t i = 0; i < min_size; i += step)
    {
        double dx = path1.poses[i].pose.position.x - path2.poses[i].pose.position.x;
        double dy = path1.poses[i].pose.position.y - path2.poses[i].pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);
        
        total_distance += distance;
        comparison_points++;
        
        // 如果某点距离过大，直接返回低相似度
        if (distance > max_allowed_distance)
        {
            return 0.0;
        }
    }
    
    if (comparison_points == 0) return 0.0;
    
    double average_distance = total_distance / comparison_points;
    // 距离越小，相似度越高（归一化到0-1）
    double similarity = std::max(0.0, 1.0 - (average_distance / max_allowed_distance));
    
    return similarity;
}

bool LocalPathPlanner::shouldKeepPreviousPath(const nav_msgs::msg::Path& new_path)
{
    // 如果没有有效的前一路径，直接使用新路径
    if (previous_local_path_.poses.empty() || !local_path_valid_)
    {
        path_stability_counter_ = 0;
        return false;
    }
    
    // 计算路径相似度
    double similarity = calculatePathSimilarity(new_path, previous_local_path_);
    
    RCLCPP_DEBUG(node_->get_logger(), "Path similarity: %.3f, threshold: %.3f", similarity, path_similarity_threshold_);
    
    // 如果相似度高于阈值，增加稳定性计数器
    if (similarity > path_similarity_threshold_)
    {
        path_stability_counter_++;
        
        // 连续多次路径相似，保持之前路径以减少抖动
        if (path_stability_counter_ >= 3)  // 连续3次相似就保持稳定
        {
            path_stability_counter_ = 3;  // 避免计数器无限增长
            return true;
        }
    }
    else
    {
        // 相似度低，重置计数器
        path_stability_counter_ = 0;
    }
    
    return false;
}

nav_msgs::msg::Path LocalPathPlanner::smoothPath(const nav_msgs::msg::Path& path)
{
    if (path.poses.size() <= 2)
    {
        return path;  // 路径太短，无需平滑
    }
    
    nav_msgs::msg::Path smoothed_path = path;
    
    // 简单的移动平均平滑
    for (size_t i = 1; i < smoothed_path.poses.size() - 1; ++i)
    {
        double weight_prev = 0.25;
        double weight_curr = 0.5;
        double weight_next = 0.25;
        
        double smoothed_x = weight_prev * path.poses[i-1].pose.position.x +
                           weight_curr * path.poses[i].pose.position.x +
                           weight_next * path.poses[i+1].pose.position.x;
                           
        double smoothed_y = weight_prev * path.poses[i-1].pose.position.y +
                           weight_curr * path.poses[i].pose.position.y +
                           weight_next * path.poses[i+1].pose.position.y;
        
        smoothed_path.poses[i].pose.position.x = smoothed_x;
        smoothed_path.poses[i].pose.position.y = smoothed_y;
    }
    
    return smoothed_path;
}

} // namespace pure_pursuit_local_planner
