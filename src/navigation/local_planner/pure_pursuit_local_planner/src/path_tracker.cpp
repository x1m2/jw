#include "pure_pursuit_local_planner/path_tracker.hpp"
#include "pure_pursuit_local_planner/param_utils.hpp"
#include <tf2/utils.hpp>

namespace pure_pursuit_local_planner
{

    PathTracker::PathTracker()
        : path_valid_(false)
    {
    }

    PathTracker::~PathTracker()
    {
    }

    bool PathTracker::initialize(rclcpp::Node::SharedPtr node)
    {
        node_ = node;

        // 加载参数
        loadParameters();

        // 初始化发布器
        path_markers_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("path_markers", 1);
        lookahead_marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("lookahead_marker", 1);

        RCLCPP_INFO(node_->get_logger(), "Path tracker initialized");
        return true;
    }

    void PathTracker::loadParameters()
    {
        curvature_smoothing_window_ = declareOrGetParameter(node_, "curvature_smoothing_window", 3.0);
        max_curvature_ = declareOrGetParameter(node_, "max_curvature", 2.0);
        min_lookahead_distance_ = declareOrGetParameter(node_, "min_lookahead_distance", 0.5);
        max_lookahead_distance_ = declareOrGetParameter(node_, "max_lookahead_distance", 3.0);
    }

    bool PathTracker::setPath(const nav_msgs::msg::Path &path)
    {
        if (path.poses.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Received empty path");
            path_valid_ = false;
            return false;
        }

        // 清空之前的路径
        path_points_.clear();

        // 转换路径格式
        path_points_.reserve(path.poses.size());
        for (const auto &pose : path.poses)
        {
            PathPoint point;
            point.pose = pose;
            point.curvature = 0.0;
            point.target_velocity = 0.0;
            point.is_valid = true;
            path_points_.push_back(point);
        }

        // 预处理路径
        preprocessPath();

        path_valid_ = true;
        RCLCPP_INFO(node_->get_logger(), "Path set with %zu points", path_points_.size());
        return true;
    }

    void PathTracker::preprocessPath()
    {
        if (path_points_.size() < 3)
            return;

        // 计算每个点的曲率
        for (size_t i = 1; i < path_points_.size() - 1; ++i)
        {
            path_points_[i].curvature = computePathCurvature(static_cast<int>(i));
        }

        // 边界点的曲率设为相邻点的值
        if (path_points_.size() > 1)
        {
            path_points_[0].curvature = path_points_[1].curvature;
            path_points_.back().curvature = path_points_[path_points_.size() - 2].curvature;
        }

        // 预处理阶段设置默认目标速度（后续会根据实际情况动态计算）
        for (auto &point : path_points_)
        {
            point.target_velocity = 1.0; // 默认速度，实际使用时会重新计算
        }
    }

    int PathTracker::findClosestPathPoint(const geometry_msgs::msg::PoseStamped &robot_pose)
    {
        if (!path_valid_ || path_points_.empty())
            return -1;

        double min_distance = std::numeric_limits<double>::max();
        int closest_index = -1;

        for (size_t i = 0; i < path_points_.size(); ++i)
        {
            double distance = calculateDistance(robot_pose, path_points_[i].pose);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_index = static_cast<int>(i);
            }
        }

        return closest_index;
    }

    bool PathTracker::computeLookaheadPoint(const geometry_msgs::msg::PoseStamped &robot_pose,
                                            double lookahead_distance,
                                            int closest_index,
                                            geometry_msgs::msg::PointStamped &lookahead_point)
    {
        if (!path_valid_ || closest_index < 0 ||
            closest_index >= static_cast<int>(path_points_.size()))
            return false;

        // 从最近点开始搜索前瞻点
        for (size_t i = closest_index; i < path_points_.size(); ++i)
        {
            double distance = calculateDistance(robot_pose, path_points_[i].pose);

            if (distance >= lookahead_distance)
            {
                // 找到了目标距离的点
                if (i == 0)
                {
                    // 第一个点就满足条件
                    lookahead_point.header = path_points_[i].pose.header;
                    lookahead_point.point = path_points_[i].pose.pose.position;
                    return true;
                }
                else
                {
                    // 在两个点之间插值
                    lookahead_point = interpolateLookaheadPoint(path_points_[i - 1], path_points_[i],
                                                                robot_pose, lookahead_distance);
                    return true;
                }
            }
        }

        // 如果没有找到满足距离的点，返回路径终点
        if (!path_points_.empty())
        {
            const auto &last_point = path_points_.back();
            lookahead_point.header = last_point.pose.header;
            lookahead_point.point = last_point.pose.pose.position;
            return true;
        }

        return false;
    }

    geometry_msgs::msg::PointStamped PathTracker::interpolateLookaheadPoint(const PathPoint &p1,
                                                                       const PathPoint &p2,
                                                                       const geometry_msgs::msg::PoseStamped &robot_pose,
                                                                       double lookahead_distance)
    {
        geometry_msgs::msg::PointStamped lookahead_point;

        // 计算机器人到两个路径点的距离
        double dist1 = calculateDistance(robot_pose, p1.pose);
        double dist2 = calculateDistance(robot_pose, p2.pose);

        // 在两点间进行线性插值
        double total_dist = calculateDistance(p1.pose, p2.pose);
        if (total_dist < 1e-6)
        {
            lookahead_point.header = p2.pose.header;
            lookahead_point.point = p2.pose.pose.position;
            return lookahead_point;
        }

        // 计算插值比例
        double target_dist_from_p1 = std::sqrt(lookahead_distance * lookahead_distance -
                                               dist1 * dist1 +
                                               (lookahead_distance - dist1) * (lookahead_distance - dist1));
        double ratio = target_dist_from_p1 / total_dist;
        ratio = std::max(0.0, std::min(1.0, ratio));

        // 插值计算前瞻点
        lookahead_point.header = p2.pose.header;
        lookahead_point.point.x = p1.pose.pose.position.x +
                                  ratio * (p2.pose.pose.position.x - p1.pose.pose.position.x);
        lookahead_point.point.y = p1.pose.pose.position.y +
                                  ratio * (p2.pose.pose.position.y - p1.pose.pose.position.y);
        lookahead_point.point.z = p1.pose.pose.position.z +
                                  ratio * (p2.pose.pose.position.z - p1.pose.pose.position.z);

        return lookahead_point;
    }

    double PathTracker::computePathCurvature(int index)
    {
        if (!path_valid_ || index <= 0 || index >= static_cast<int>(path_points_.size()) - 1)
            return 0.0;

        // 使用三点法计算曲率
        const auto &p1 = path_points_[index - 1].pose.pose.position;
        const auto &p2 = path_points_[index].pose.pose.position;
        const auto &p3 = path_points_[index + 1].pose.pose.position;

        // 计算向量
        double dx1 = p2.x - p1.x;
        double dy1 = p2.y - p1.y;
        double dx2 = p3.x - p2.x;
        double dy2 = p3.y - p2.y;

        // 计算角度变化
        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);
        double angle_diff = angle2 - angle1;

        // 标准化角度到[-π, π]
        while (angle_diff > M_PI)
            angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI)
            angle_diff += 2.0 * M_PI;

        // 计算弧长
        double arc_length = (std::hypot(dx1, dy1) + std::hypot(dx2, dy2)) / 2.0;

        if (arc_length < 1e-6)
            return 0.0;

        // 曲率 = 角度变化 / 弧长
        double curvature = angle_diff / arc_length;

        // 限制曲率范围
        return std::max(-max_curvature_, std::min(max_curvature_, curvature));
    }

    double PathTracker::computeTargetVelocity(const geometry_msgs::msg::PoseStamped &robot_pose,
                                              const geometry_msgs::msg::PointStamped &lookahead_point,
                                              double max_velocity)
    {
        // 计算机器人方向单位向量
        double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
        double robot_dir_x = std::cos(robot_yaw);
        double robot_dir_y = std::sin(robot_yaw);

        // 计算机器人位置到前瞻点的方向向量
        double dx = lookahead_point.point.x - robot_pose.pose.position.x;
        double dy = lookahead_point.point.y - robot_pose.pose.position.y;
        double distance = std::hypot(dx, dy);

        if (distance < 1e-6)
        {
            // 如果距离过小，返回最大速度
            return max_velocity;
        }

        // 计算前瞻点方向角度
        double theta = std::atan2(dy, dx);

        // 计算方向误差
        double angle_error = theta - robot_yaw;
        // 标准化角度到[-π, π]
        while (angle_error > M_PI)
            angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI)
            angle_error += 2.0 * M_PI;

        // 当误差大于1.57弧度(90度)时，降低速度进行方向调整
        if (std::abs(angle_error) > 1.57)
        {
            // 方向误差过大，原地转向
            return 0;
        }
        else
        {
            // 计算前瞻点方向单位向量
            double lookahead_dir_x = dx / distance;
            double lookahead_dir_y = dy / distance;

            // 计算两个单位向量的点乘（余弦值）
            double dot_product = robot_dir_x * lookahead_dir_x + robot_dir_y * lookahead_dir_y;

            // 确保点乘结果在合理范围内 [0, 1]
            dot_product = std::max(0.0, std::min(1.0, dot_product));

            // 返回点乘结果乘以最大速度
            return dot_product * max_velocity;
        }
    }

    bool PathTracker::isPathComplete(const geometry_msgs::msg::PoseStamped &robot_pose, double tolerance)
    {
        if (!path_valid_ || path_points_.empty())
            return false;

        const auto &goal_pose = path_points_.back().pose;
        double distance = calculateDistance(robot_pose, goal_pose);

        return distance < tolerance;
    }

    double PathTracker::getPathLength()
    {
        if (!path_valid_ || path_points_.size() < 2)
            return 0.0;

        double total_length = 0.0;
        for (size_t i = 1; i < path_points_.size(); ++i)
        {
            total_length += calculateDistance(path_points_[i - 1].pose, path_points_[i].pose);
        }

        return total_length;
    }

    double PathTracker::getRemainingPathLength(int current_index)
    {
        if (!path_valid_ || current_index < 0 ||
            current_index >= static_cast<int>(path_points_.size()) - 1)
            return 0.0;

        double remaining_length = 0.0;
        for (size_t i = current_index + 1; i < path_points_.size(); ++i)
        {
            remaining_length += calculateDistance(path_points_[i - 1].pose, path_points_[i].pose);
        }

        return remaining_length;
    }

    void PathTracker::publishVisualization(const geometry_msgs::msg::PoseStamped &robot_pose,
                                           const geometry_msgs::msg::PointStamped &lookahead_point,
                                           int closest_index)
    {
        // 发布路径标记
        visualization_msgs::msg::Marker path_marker = createPathMarker();
        path_markers_pub_->publish(path_marker);

        // 发布前瞻点标记
        visualization_msgs::msg::Marker lookahead_marker = createLookaheadMarker(lookahead_point);
        lookahead_marker_pub_->publish(lookahead_marker);

        // 可以添加更多可视化信息，如最近点标记等
    }

    visualization_msgs::msg::Marker PathTracker::createPathMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // 或使用路径的frame_id
        marker.header.stamp = node_->now();
        marker.ns = "path";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05;
        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        for (const auto &point : path_points_)
        {
            marker.points.push_back(point.pose.pose.position);
        }

        return marker;
    }

    visualization_msgs::msg::Marker PathTracker::createLookaheadMarker(const geometry_msgs::msg::PointStamped &lookahead_point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = lookahead_point.header;
        marker.ns = "lookahead";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = lookahead_point.point;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        return marker;
    }

    visualization_msgs::msg::Marker PathTracker::createClosestPointMarker(int closest_index)
    {
        visualization_msgs::msg::Marker marker;

        if (!path_valid_ || closest_index < 0 ||
            closest_index >= static_cast<int>(path_points_.size()))
        {
            marker.action = visualization_msgs::msg::Marker::DELETE;
            return marker;
        }

        marker.header = path_points_[closest_index].pose.header;
        marker.ns = "closest_point";
        marker.id = 2;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose = path_points_[closest_index].pose.pose;

        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.3;

        marker.color.a = 0.8;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        return marker;
    }

    double PathTracker::calculateDistance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }

    double PathTracker::calculateDistance(const geometry_msgs::msg::PoseStamped &pose1,
                                          const geometry_msgs::msg::PoseStamped &pose2)
    {
        return calculateDistance(pose1.pose.position, pose2.pose.position);
    }

} // namespace pure_pursuit_local_planner
