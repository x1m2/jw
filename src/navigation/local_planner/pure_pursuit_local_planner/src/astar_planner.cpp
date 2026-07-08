#include "pure_pursuit_local_planner/astar_planner.hpp"
#include "pure_pursuit_local_planner/param_utils.hpp"
#include <algorithm>
#include <cmath>

namespace pure_pursuit_local_planner
{

AStarPlanner::AStarPlanner()
    : costmap_received_(false), global_path_received_(false), obstacle_threshold_(50), safety_margin_(2),
      allow_diagonal_(true), diagonal_cost_(1.414), straight_cost_(1.0), 
      path_deviation_weight_(1.5), obstacle_avoidance_weight_(2.0)
{
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::initialize(rclcpp::Node::SharedPtr node)
{
    node_ = node;
    
    // 加载参数
    obstacle_threshold_ = declareOrGetParameter(node_, "dijkstra_obstacle_threshold", 50.0);
    safety_margin_ = declareOrGetParameter(node_, "dijkstra_safety_margin", 2);  // 降低安全边距
    allow_diagonal_ = declareOrGetParameter(node_, "dijkstra_allow_diagonal", true);
    diagonal_cost_ = declareOrGetParameter(node_, "dijkstra_diagonal_cost", 1.414);
    straight_cost_ = declareOrGetParameter(node_, "dijkstra_straight_cost", 1.0);
    path_deviation_weight_ = declareOrGetParameter(node_, "dijkstra_path_deviation_weight", 1.5);
    obstacle_avoidance_weight_ = declareOrGetParameter(node_, "dijkstra_obstacle_avoidance_weight", 2.0);  // 降低障碍物避让权重
    global_frame_ = declareOrGetParameter<std::string>(node_, "global_frame", "map");
    
    // 初始化发布器
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("dijkstra_path", 1);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("dijkstra_markers", 1);
    
    RCLCPP_INFO(node_->get_logger(), "Dijkstra Planner initialized with optimized inflation handling");
    RCLCPP_INFO(node_->get_logger(), "  Obstacle threshold: %.1f", obstacle_threshold_);
    RCLCPP_INFO(node_->get_logger(), "  Safety margin: %d", safety_margin_);
    RCLCPP_INFO(node_->get_logger(), "  Obstacle avoidance weight: %.1f", obstacle_avoidance_weight_);
    return true;
}

void AStarPlanner::setCostmap(const nav_msgs::msg::OccupancyGrid& costmap)
{
    costmap_ = costmap;
    costmap_received_ = true;
}

void AStarPlanner::setGlobalPath(const nav_msgs::msg::Path& global_path)
{
    global_path_ = global_path;
    global_path_received_ = !global_path.poses.empty();
}

bool AStarPlanner::planPath(const geometry_msgs::msg::Point& start, 
                           const geometry_msgs::msg::Point& goal, 
                           nav_msgs::msg::Path& path)
{
    if (!costmap_received_)
    {
        RCLCPP_WARN(node_->get_logger(), "No costmap received for A* planning");
        return false;
    }
    
    // 转换为栅格坐标
    int start_x, start_y, goal_x, goal_y;
    worldToGrid(start, start_x, start_y);
    worldToGrid(goal, goal_x, goal_y);

    auto get_cell_cost = [this](int grid_x, int grid_y) -> int {
        if (!isValidCell(grid_x, grid_y))
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

    const int start_cost = get_cell_cost(start_x, start_y);
    const int goal_cost = get_cell_cost(goal_x, goal_y);
    const bool start_valid = isValidCell(start_x, start_y);
    const bool goal_valid = isValidCell(goal_x, goal_y);

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[ASTAR_DEBUG] start=(%.3f, %.3f)->(%d, %d), goal=(%.3f, %.3f)->(%d, %d), "
        "start_valid=%d, goal_valid=%d, start_cost=%d, goal_cost=%d, "
        "threshold=%.1f, costmap=%ux%u res=%.3f origin=(%.3f, %.3f)",
        start.x, start.y, start_x, start_y, goal.x, goal.y, goal_x, goal_y,
        start_valid ? 1 : 0, goal_valid ? 1 : 0, start_cost, goal_cost,
        obstacle_threshold_, costmap_.info.width, costmap_.info.height,
        costmap_.info.resolution, costmap_.info.origin.position.x,
        costmap_.info.origin.position.y);
    
    // 检查起点和终点是否有效
    if (!start_valid || !goal_valid)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Start or goal position is invalid: start_valid=%d, goal_valid=%d",
                    start_valid ? 1 : 0, goal_valid ? 1 : 0);
        return false;
    }
    
    // 对于起点，使用宽松的检查（只检查真正的高代价障碍物）
    if (!isStartPositionValid(start_x, start_y))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Start position is in solid obstacle: grid=(%d, %d), cost=%d",
                    start_x, start_y, start_cost);
        return false;
    }
    
    // 终点必须完全安全
    if (isObstacle(goal_x, goal_y))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Goal position is in obstacle: grid=(%d, %d), cost=%d, threshold=%.1f",
                    goal_x, goal_y, goal_cost, obstacle_threshold_);
        return false;
    }

    // Check if global path is available
    if (!global_path_received_)
    {
        RCLCPP_WARN(node_->get_logger(), "Global path not set for A* planner");
        return false;
    }
    
    // Dijkstra算法实现（无启发式函数）
    std::priority_queue<DijkstraNode*, std::vector<DijkstraNode*>, DijkstraNodeCompare> open_list;
    std::unordered_map<NodeKey, DijkstraNode*, NodeKeyHash> all_nodes;
    std::unordered_map<NodeKey, bool, NodeKeyHash> visited;
    
    // 创建起始节点
    DijkstraNode* start_node = new DijkstraNode(start_x, start_y);
    start_node->cost = 0.0;
    
    open_list.push(start_node);
    all_nodes[NodeKey(start_x, start_y)] = start_node;
    
    DijkstraNode* goal_node = nullptr;
    size_t searched_nodes = 0;
    
    while (!open_list.empty())
    {
        // 获取代价最小的节点
        DijkstraNode* current = open_list.top();
        open_list.pop();
        ++searched_nodes;
        
        // 检查是否到达目标
        if (current->x == goal_x && current->y == goal_y)
        {
            goal_node = current;
            break;
        }
        
        // 将当前节点标记为已访问
        visited[NodeKey(current->x, current->y)] = true;
        
        // 遍历邻居节点
        std::vector<DijkstraNode*> neighbors = getNeighbors(current);
        for (DijkstraNode* neighbor : neighbors)
        {
            NodeKey neighbor_key(neighbor->x, neighbor->y);
            
            // 跳过已经访问过的节点
            if (visited.find(neighbor_key) != visited.end())
            {
                delete neighbor;
                continue;
            }
            
            // 跳过障碍物
            if (isObstacle(neighbor->x, neighbor->y))
            {
                delete neighbor;
                continue;
            }
            
            // 计算从当前节点到邻居节点的代价
            double move_cost = (abs(neighbor->x - current->x) + abs(neighbor->y - current->y) == 2) 
                              ? diagonal_cost_ : straight_cost_;
            
            // 添加障碍物代价，让路径倾向于远离障碍物（降低权重）
            double obstacle_cost = calculateObstacleCost(neighbor->x, neighbor->y);
            
            // 添加路径偏离代价，让路径倾向于跟随全局路径
            double deviation_cost = calculatePathDeviationCost(neighbor->x, neighbor->y);
            
            // 降低障碍物避让权重，允许在膨胀区域通过
            double obstacle_weight = std::min(obstacle_avoidance_weight_, 2.0);  // 限制最大权重为2.0
            double tentative_cost = current->cost + move_cost + 
                                   obstacle_weight * obstacle_cost + deviation_cost;
            
            // 检查是否已经在all_nodes中
            auto existing_node_it = all_nodes.find(neighbor_key);
            if (existing_node_it != all_nodes.end())
            {
                DijkstraNode* existing_node = existing_node_it->second;
                if (tentative_cost < existing_node->cost)
                {
                    // 找到更好的路径
                    existing_node->cost = tentative_cost;
                    existing_node->parent = current;
                    open_list.push(existing_node);
                }
                delete neighbor;
            }
            else
            {
                // 新节点
                neighbor->cost = tentative_cost;
                neighbor->parent = current;
                
                open_list.push(neighbor);
                all_nodes[neighbor_key] = neighbor;
            }
        }
    }
    
    bool success = false;
    if (goal_node)
    {
        path = reconstructPath(goal_node);
        success = true;
        RCLCPP_DEBUG(node_->get_logger(), "Dijkstra path planning successful, path length: %zu", path.poses.size());
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Dijkstra path planning failed: searched_nodes=%zu, visited_nodes=%zu",
                    searched_nodes, visited.size());
    }
    
    // 清理内存
    cleanupNodes(all_nodes);
    
    return success;
}

bool AStarPlanner::isObstacle(const geometry_msgs::msg::Point& point)
{
    int grid_x, grid_y;
    worldToGrid(point, grid_x, grid_y);
    return isObstacle(grid_x, grid_y);
}

bool AStarPlanner::isObstacle(int grid_x, int grid_y)
{
    if (!isValidCell(grid_x, grid_y))
        return true;
    
    // 只检查当前点是否为真实障碍物，不再使用安全边距
    int index = grid_y * costmap_.info.width + grid_x;
    if (index >= 0 && index < costmap_.data.size())
    {
        // 只有当前点本身是高代价障碍物时才认为是障碍物
        return (costmap_.data[index] >= obstacle_threshold_ || costmap_.data[index] < 0);
    }
    
    return true;  // 无效索引认为是障碍物
}

void AStarPlanner::worldToGrid(const geometry_msgs::msg::Point& world_point, int& grid_x, int& grid_y)
{
    grid_x = static_cast<int>((world_point.x - costmap_.info.origin.position.x) / costmap_.info.resolution);
    grid_y = static_cast<int>((world_point.y - costmap_.info.origin.position.y) / costmap_.info.resolution);
}

void AStarPlanner::gridToWorld(int grid_x, int grid_y, geometry_msgs::msg::Point& world_point)
{
    world_point.x = costmap_.info.origin.position.x + (grid_x + 0.5) * costmap_.info.resolution;
    world_point.y = costmap_.info.origin.position.y + (grid_y + 0.5) * costmap_.info.resolution;
    world_point.z = costmap_.info.origin.position.z;  // 直接使用costmap的z原点
}

void AStarPlanner::publishVisualization(const geometry_msgs::msg::Point& start, 
                                       const geometry_msgs::msg::Point& goal,
                                       const nav_msgs::msg::Path& path)
{
    // 发布路径
    nav_msgs::msg::Path vis_path = path;
    vis_path.header.frame_id = global_frame_;
    vis_path.header.stamp = node_->now();
    path_pub_->publish(vis_path);
    
    // 发布标记
    visualization_msgs::msg::MarkerArray marker_array;
    
    // 起点标记
    visualization_msgs::msg::Marker start_marker = createPointMarker(start, 0, 0.0, 1.0, 0.4);
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.3;
    marker_array.markers.push_back(start_marker);
    
    // 终点标记
    visualization_msgs::msg::Marker goal_marker = createPointMarker(goal, 1, 1.0, 0.0, 0.0);
    goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3;
    marker_array.markers.push_back(goal_marker);
    
    marker_pub_->publish(marker_array);
}

double AStarPlanner::calculateObstacleCost(int x, int y)
{
    if (!isValidCell(x, y))
        return 100.0;  // 高代价
    
    // 首先检查当前点的代价值
    int current_index = y * costmap_.info.width + x;
    if (current_index >= 0 && current_index < costmap_.data.size())
    {
        int8_t current_cost = costmap_.data[current_index];
        
        // 如果当前点本身就是高代价区域（但不是障碍物），给予适中的代价
        if (current_cost > 20 && current_cost < obstacle_threshold_)
        {
            // 膨胀区域，给予较小的代价，允许通过但不鼓励
            return current_cost / 20.0;  // 将costmap代价值转换为合理的路径代价
        }
    }
    
    double min_obstacle_distance = std::numeric_limits<double>::max();
    
    // 在安全边距范围内检查真实障碍物距离
    for (int dx = -safety_margin_; dx <= safety_margin_; ++dx)
    {
        for (int dy = -safety_margin_; dy <= safety_margin_; ++dy)
        {
            int check_x = x + dx;
            int check_y = y + dy;
            
            if (!isValidCell(check_x, check_y))
                continue;
                
            int index = check_y * costmap_.info.width + check_x;
            if (index >= 0 && index < costmap_.data.size())
            {
                // 只考虑真实的高代价障碍物
                if (costmap_.data[index] >= obstacle_threshold_ || costmap_.data[index] < 0)
                {
                    double distance = std::sqrt(dx * dx + dy * dy);
                    min_obstacle_distance = std::min(min_obstacle_distance, distance);
                }
            }
        }
    }
    
    // 根据到最近真实障碍物的距离计算代价
    if (min_obstacle_distance == std::numeric_limits<double>::max())
    {
        return 0.0;  // 没有附近的真实障碍物
    }
    else if (min_obstacle_distance < 2.0)  // 降低安全距离要求
    {
        // 距离真实障碍物较近，给予适中代价
        return 2.0 * (2.0 - min_obstacle_distance);
    }
    else
    {
        return 0.0;  // 距离足够远，不增加额外代价
    }
}

double AStarPlanner::calculatePathDeviationCost(int x, int y) const
{
    if (!global_path_received_ || global_path_.poses.empty())
    {
        return 0.0;  // No global path available, no deviation cost
    }
    
    // Convert grid coordinates to world coordinates
    double world_x = x * costmap_.info.resolution + costmap_.info.origin.position.x;
    double world_y = y * costmap_.info.resolution + costmap_.info.origin.position.y;
    
    // Find minimum distance to global path
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& pose : global_path_.poses)
    {
        double dx = world_x - pose.pose.position.x;
        double dy = world_y - pose.pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    
    // Apply weight to deviation cost
    return path_deviation_weight_ * min_distance;
}

std::vector<DijkstraNode*> AStarPlanner::getNeighbors(DijkstraNode* current)
{
    std::vector<DijkstraNode*> neighbors;
    
    // 8方向邻居（如果允许对角线移动）
    std::vector<std::pair<int, int>> directions;
    if (allow_diagonal_)
    {
        directions = {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}};
    }
    else
    {
        directions = {{-1,0}, {0,-1}, {0,1}, {1,0}};
    }
    
    for (const auto& dir : directions)
    {
        int new_x = current->x + dir.first;
        int new_y = current->y + dir.second;
        
        if (isValidCell(new_x, new_y))
        {
            neighbors.push_back(new DijkstraNode(new_x, new_y));
        }
    }
    
    return neighbors;
}

bool AStarPlanner::isValidCell(int x, int y)
{
    return x >= 0 && x < costmap_.info.width && y >= 0 && y < costmap_.info.height;
}

bool AStarPlanner::isStartPositionValid(int grid_x, int grid_y)
{
    if (!isValidCell(grid_x, grid_y))
        return false;
    
    // 对于起点，只检查是否为真正的高代价障碍物（threshold 80以上）
    // 允许机器人在膨胀区域（通常是20-79的代价值）内开始规划
    int index = grid_y * costmap_.info.width + grid_x;
    if (index >= 0 && index < costmap_.data.size())
    {
        int8_t cost = costmap_.data[index];
        // 只有真正的高代价障碍物才认为起点无效
        return !(cost >= 80 || cost < 0);
    }
    
    return false;  // 无效索引认为无效
}

nav_msgs::msg::Path AStarPlanner::reconstructPath(DijkstraNode* goal_node)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();
    
    std::vector<DijkstraNode*> path_nodes;
    DijkstraNode* current = goal_node;
    
    // 从目标节点回溯到起始节点
    while (current != nullptr)
    {
        path_nodes.push_back(current);
        current = current->parent;
    }
    
    // 反转路径（从起点到终点）
    std::reverse(path_nodes.begin(), path_nodes.end());
    
    // 转换为世界坐标
    for (DijkstraNode* node : path_nodes)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = node_->now();
        
        gridToWorld(node->x, node->y, pose.pose.position);
        pose.pose.orientation.w = 1.0;
        
        path.poses.push_back(pose);
    }
    
    return path;
}

void AStarPlanner::cleanupNodes(std::unordered_map<NodeKey, DijkstraNode*, NodeKeyHash>& all_nodes)
{
    for (auto& pair : all_nodes)
    {
        delete pair.second;
    }
    all_nodes.clear();
}

visualization_msgs::msg::Marker AStarPlanner::createPointMarker(const geometry_msgs::msg::Point& point, 
                                                          int id, double r, double g, double b)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "dijkstra_points";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = point;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    return marker;
}

} // namespace pure_pursuit_local_planner
