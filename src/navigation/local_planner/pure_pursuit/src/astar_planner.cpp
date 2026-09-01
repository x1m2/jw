#include "pure_pursuit/astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>


#include "pure_pursuit/param_utils.hpp"

namespace pure_pursuit_local_planner {

AStarPlanner::AStarPlanner()
    : global_path_received_(false),
      obstacle_threshold_(50),
      safety_margin_(2),
      allow_diagonal_(true),
      diagonal_cost_(1.414),
      straight_cost_(1.0),
      path_deviation_weight_(1.5),
      obstacle_avoidance_weight_(2.0),
      footprint_radius_(0.78),
      footprint_half_length_(0.65),
      footprint_half_width_(0.50),
      start_obstacle_threshold_(80),
      footprint_obstacle_threshold_(100) {}

bool AStarPlanner::initialize(rclcpp::Node::SharedPtr node) {
  node_ = node;

  // 加载参数
  // 检测器和A*共用同一个阻塞阈值，避免检测结果为“可通行”而规划器拒绝。
  obstacle_threshold_ =
      declareOrGetParameter(node_, "obstacle_cost_threshold", 50);
  safety_margin_ = declareOrGetParameter(node_, "astar_safety_margin", 2);
  allow_diagonal_ = declareOrGetParameter(node_, "astar_allow_diagonal", true);
  diagonal_cost_ = declareOrGetParameter(node_, "astar_diagonal_cost", 1.414);
  straight_cost_ = declareOrGetParameter(node_, "astar_straight_cost", 1.0);
  path_deviation_weight_ =
      declareOrGetParameter(node_, "astar_path_deviation_weight", 0.0);
  obstacle_avoidance_weight_ =
      declareOrGetParameter(node_, "astar_obstacle_avoidance_weight", 2.0);
  const double robot_width = declareOrGetParameter(node_, "robot_width", 0.80);
  const double robot_length =
      declareOrGetParameter(node_, "robot_length", 1.10);
  const double footprint_padding =
      declareOrGetParameter(node_, "astar_footprint_padding", 0.10);
  footprint_half_length_ = robot_length * 0.5 + std::max(0.0, footprint_padding);
  footprint_half_width_ = robot_width * 0.5 + std::max(0.0, footprint_padding);
  footprint_radius_ = std::hypot(footprint_half_length_, footprint_half_width_);
  start_obstacle_threshold_ =
      declareOrGetParameter(node_, "astar_start_obstacle_threshold", 80);
  footprint_obstacle_threshold_ =
      declareOrGetParameter(node_, "astar_footprint_obstacle_threshold", 100);
  global_frame_ =
      declareOrGetParameter<std::string>(node_, "global_frame", "map");

  straight_cost_ = std::max(straight_cost_, 1e-6);
  diagonal_cost_ =
      std::clamp(diagonal_cost_, straight_cost_, 2.0 * straight_cost_);
  safety_margin_ = std::max(safety_margin_, 0);
  path_deviation_weight_ = std::max(path_deviation_weight_, 0.0);
  obstacle_avoidance_weight_ = std::max(obstacle_avoidance_weight_, 0.0);
  start_obstacle_threshold_ = std::clamp(start_obstacle_threshold_, 1, 100);
  obstacle_threshold_ = std::clamp(obstacle_threshold_, 1, 100);
  footprint_obstacle_threshold_ =
      std::clamp(footprint_obstacle_threshold_, 1, 100);

  // 初始化发布器
  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("astar_path", 1);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "astar_markers", 1);

  RCLCPP_INFO(node_->get_logger(),
              "A* initialized: obstacle=%d, rectangular footprint=%.2fx%.2fm, "
              "timeout=%.3fs, max_nodes=%zu",
              obstacle_threshold_, 2.0 * footprint_half_length_,
              2.0 * footprint_half_width_, kPlanningTimeoutSec,
              kMaxSearchNodes);
  return true;
}

void AStarPlanner::setCostmap(const nav_msgs::msg::OccupancyGrid &costmap) {
  setCostmap(std::make_shared<const nav_msgs::msg::OccupancyGrid>(costmap));
  // 值类型接口只用于测试和独立同步调用，保持设置后即可规划的原有语义。
  prepareLatestCostmap();
}

void AStarPlanner::setCostmap(CostmapConstPtr costmap) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_costmap_ = std::move(costmap);
  ++pending_costmap_version_;
  if (!pending_costmap_) {
    prepared_costmap_ = PreparedCostmapSnapshot();
  }
}

bool AStarPlanner::prepareLatestCostmap() {
  CostmapConstPtr costmap_to_prepare;
  uint64_t version_to_prepare = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!pending_costmap_) {
      return prepared_costmap_.valid();
    }
    if (prepared_costmap_.version == pending_costmap_version_) {
      return prepared_costmap_.valid();
    }
    // 无效的不可变消息不必在每个规划周期重复处理。
    if (last_preparation_attempt_version_ == pending_costmap_version_) {
      return prepared_costmap_.valid();
    }
    costmap_to_prepare = pending_costmap_;
    version_to_prepare = pending_costmap_version_;
    last_preparation_attempt_version_ = version_to_prepare;
  }

  const auto &info = costmap_to_prepare->info;
  const size_t expected_size =
      static_cast<size_t>(info.width) * static_cast<size_t>(info.height);
  if (info.resolution <= 0.0 || info.width == 0 || info.height == 0 ||
      costmap_to_prepare->data.size() != expected_size) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Ignoring invalid costmap while preparing A* snapshot: %ux%u, "
        "resolution=%.3f, data=%zu/%zu",
        info.width, info.height, info.resolution,
        costmap_to_prepare->data.size(), expected_size);
    std::lock_guard<std::mutex> lock(data_mutex_);
    return prepared_costmap_.valid();
  }

  auto footprint_mask = std::make_shared<const std::vector<uint8_t>>(
      buildFootprintMask(*costmap_to_prepare));
  if (footprint_mask->size() != costmap_to_prepare->data.size()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "Failed to prepare A* footprint mask");
    std::lock_guard<std::mutex> lock(data_mutex_);
    return prepared_costmap_.valid();
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 即使构建期间收到了更新地图，也先发布这一份自洽快照；下一规划周期
    // 会继续准备更新版本，避免输入持续更新时一直没有可用地图。
    prepared_costmap_ = {std::move(costmap_to_prepare),
                         std::move(footprint_mask), version_to_prepare};
  }
  return true;
}

bool AStarPlanner::hasPreparedCostmap() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return prepared_costmap_.valid();
}

CostmapConstPtr AStarPlanner::getPreparedCostmap() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  return prepared_costmap_.valid() ? prepared_costmap_.costmap : nullptr;
}

void AStarPlanner::setGlobalPath(const nav_msgs::msg::Path &global_path) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_path_ = global_path;
  global_path_received_ = !global_path.poses.empty();
}

bool AStarPlanner::planPath(const geometry_msgs::msg::Point &start,
                            const geometry_msgs::msg::Point &goal,
                            nav_msgs::msg::Path &path, double goal_yaw) {
  CostmapConstPtr costmap_snapshot;
  nav_msgs::msg::Path global_path_snapshot;
  std::shared_ptr<const std::vector<uint8_t>> footprint_mask_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!prepared_costmap_.valid()) {
      RCLCPP_WARN(node_->get_logger(),
                  "No prepared costmap snapshot for A* planning");
      return false;
    }
    if (!global_path_received_) {
      RCLCPP_WARN(node_->get_logger(), "Global path not set for A* planner");
      return false;
    }
    costmap_snapshot = prepared_costmap_.costmap;
    global_path_snapshot = global_path_;
    footprint_mask_snapshot = prepared_costmap_.footprint_mask;
  }

  const auto &costmap = *costmap_snapshot;
  const auto &footprint_mask = *footprint_mask_snapshot;

  if (costmap.info.resolution <= 0.0 ||
      footprint_mask.size() != costmap.data.size()) {
    RCLCPP_WARN(node_->get_logger(), "No costmap received for A* planning");
    return false;
  }

  // 转换为栅格坐标
  int start_x, start_y, goal_x, goal_y;
  worldToGrid(start, costmap, start_x, start_y);
  worldToGrid(goal, costmap, goal_x, goal_y);

  auto get_cell_cost = [this, &costmap](int grid_x, int grid_y) -> int {
    if (!isValidCell(grid_x, grid_y, costmap)) {
      return -998;
    }

    const int index =
        grid_y * static_cast<int>(costmap.info.width) + grid_x;
    if (index < 0 || index >= static_cast<int>(costmap.data.size())) {
      return -997;
    }

    return static_cast<int>(costmap.data[index]);
  };

  const int start_cost = get_cell_cost(start_x, start_y);
  const int goal_cost = get_cell_cost(goal_x, goal_y);
  const bool start_valid = isValidCell(start_x, start_y, costmap);
  const bool goal_valid = isValidCell(goal_x, goal_y, costmap);

  RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "[ASTAR_DEBUG] start=(%.3f, %.3f)->(%d, %d), goal=(%.3f, %.3f)->(%d, "
      "%d), "
      "start_valid=%d, goal_valid=%d, start_cost=%d, goal_cost=%d, "
      "threshold=%d, costmap=%ux%u res=%.3f origin=(%.3f, %.3f)",
      start.x, start.y, start_x, start_y, goal.x, goal.y, goal_x, goal_y,
      start_valid ? 1 : 0, goal_valid ? 1 : 0, start_cost, goal_cost,
      obstacle_threshold_, costmap.info.width, costmap.info.height,
      costmap.info.resolution, costmap.info.origin.position.x,
      costmap.info.origin.position.y);

  // 检查起点和终点是否有效
  if (!start_valid || !goal_valid) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Start or goal position is invalid: start_valid=%d, goal_valid=%d",
        start_valid ? 1 : 0, goal_valid ? 1 : 0);
    return false;
  }

  // 对于起点，使用宽松的检查（只检查真正的高代价障碍物）
  if (!isStartPositionValid(start_x, start_y, costmap, footprint_mask)) {
    RCLCPP_WARN(node_->get_logger(),
                "Start position is in solid obstacle: grid=(%d, %d), cost=%d",
                start_x, start_y, start_cost);
    return false;
  }

  // 终点必须完全安全
  const double final_yaw = std::isfinite(goal_yaw)
                               ? goal_yaw
                               : std::atan2(goal.y - start.y, goal.x - start.x);
  if (isRectangleFootprintBlocked(goal_x, goal_y, final_yaw, costmap,
                                  footprint_mask)) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Goal position is in obstacle: grid=(%d, %d), cost=%d, threshold=%d",
        goal_x, goal_y, goal_cost, obstacle_threshold_);
    return false;
  }

  std::priority_queue<AStarQueueEntry, std::vector<AStarQueueEntry>,
                      AStarQueueEntryCompare>
      open_list;

  // 工作区属于规划器实例，串行使用可复用已分配容量。尺寸不变时不会重新
  // 分配，只将上一轮搜索状态恢复为初始值。
  std::lock_guard<std::mutex> search_lock(search_mutex_);
  const size_t cell_count = costmap.data.size();
  search_g_costs_.resize(cell_count);
  search_parent_indices_.resize(cell_count);
  search_closed_.resize(cell_count);
  std::fill(search_g_costs_.begin(), search_g_costs_.end(),
            std::numeric_limits<double>::infinity());
  std::fill(search_parent_indices_.begin(), search_parent_indices_.end(), -1);
  std::fill(search_closed_.begin(), search_closed_.end(), 0U);

  const int width = static_cast<int>(costmap.info.width);
  const auto to_index = [width](int x, int y) {
    return static_cast<size_t>(y * width + x);
  };
  const size_t start_index = to_index(start_x, start_y);
  const size_t goal_index = to_index(goal_x, goal_y);
  search_g_costs_[start_index] = 0.0;
  const double start_h =
      calculateHeuristic(start_x, start_y, goal_x, goal_y);
  open_list.push({start_index, 0.0, start_h});

  bool goal_found = false;
  size_t searched_nodes = 0;
  bool search_limited = false;
  const auto search_start = std::chrono::steady_clock::now();

  while (!open_list.empty()) {
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - search_start)
                               .count();
    if (searched_nodes >= kMaxSearchNodes || elapsed >= kPlanningTimeoutSec) {
      search_limited = true;
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "A* search limited: nodes=%zu/%zu, elapsed=%.3f/%.3fs",
          searched_nodes, kMaxSearchNodes, elapsed, kPlanningTimeoutSec);
      break;
    }
    const AStarQueueEntry entry = open_list.top();
    open_list.pop();

    // 跳过节点被降价前留下的旧队列项。
    if (entry.g_cost > search_g_costs_[entry.cell_index] + 1e-9) {
      continue;
    }

    if (search_closed_[entry.cell_index] != 0U) {
      continue;
    }
    ++searched_nodes;
    const int current_x = static_cast<int>(entry.cell_index % width);
    const int current_y = static_cast<int>(entry.cell_index / width);

    // 检查是否到达目标
    if (entry.cell_index == goal_index) {
      goal_found = true;
      break;
    }

    // 将当前节点标记为已访问
    search_closed_[entry.cell_index] = 1U;

    static constexpr int kDirections[8][2] = {
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
        {0, 1},   {1, -1}, {1, 0},  {1, 1}};
    const int direction_count = allow_diagonal_ ? 8 : 4;
    static constexpr int kStraightDirections[4][2] = {
        {-1, 0}, {0, -1}, {0, 1}, {1, 0}};
    const int (*directions)[2] =
        allow_diagonal_ ? kDirections : kStraightDirections;

    // 栅格索引就是节点ID，邻居扩展不再创建临时节点对象。
    for (int direction = 0; direction < direction_count; ++direction) {
      const int neighbor_x = current_x + directions[direction][0];
      const int neighbor_y = current_y + directions[direction][1];
      if (!isValidCell(neighbor_x, neighbor_y, costmap)) {
        continue;
      }
      const size_t neighbor_index = to_index(neighbor_x, neighbor_y);

      // 跳过已经访问过的节点
      if (search_closed_[neighbor_index] != 0U) {
        continue;
      }

      // 用本次移动方向作为车体朝向，检查旋转后的矩形车体占用范围。
      const double motion_yaw = std::atan2(
          static_cast<double>(neighbor_y - current_y),
          static_cast<double>(neighbor_x - current_x));
      if (isRectangleFootprintBlocked(neighbor_x, neighbor_y, motion_yaw,
                                      costmap, footprint_mask)) {
        continue;
      }

      const int step_x = neighbor_x - current_x;
      const int step_y = neighbor_y - current_y;
      const bool diagonal = step_x != 0 && step_y != 0;
      // 对角移动时两侧正交栅格都必须可通行，禁止穿过墙角。
      if (diagonal &&
          (isRectangleFootprintBlocked(current_x + step_x, current_y,
                                       motion_yaw, costmap, footprint_mask) ||
           isRectangleFootprintBlocked(current_x, current_y + step_y,
                                       motion_yaw, costmap, footprint_mask))) {
        continue;
      }

      // 计算从当前节点到邻居节点的代价
      const double move_cost = diagonal ? diagonal_cost_ : straight_cost_;

      // 添加障碍物代价，让路径倾向于远离障碍物（降低权重）
      double obstacle_cost = calculateObstacleCost(
          neighbor_x, neighbor_y, costmap, footprint_mask);

      // 添加路径偏离代价，让路径倾向于跟随全局路径
      double deviation_cost = calculatePathDeviationCost(
          neighbor_x, neighbor_y, costmap, global_path_snapshot);

      const double tentative_g = search_g_costs_[entry.cell_index] + move_cost +
                                 obstacle_avoidance_weight_ * obstacle_cost +
                                 deviation_cost;

      if (tentative_g < search_g_costs_[neighbor_index]) {
        search_g_costs_[neighbor_index] = tentative_g;
        search_parent_indices_[neighbor_index] =
            static_cast<int32_t>(entry.cell_index);
        const double h_cost =
            calculateHeuristic(neighbor_x, neighbor_y, goal_x, goal_y);
        open_list.push(
            {neighbor_index, tentative_g, tentative_g + h_cost});
      }
    }
  }

  bool success = false;
  if (goal_found) {
    path = reconstructPath(goal_index, search_parent_indices_, costmap);
    success = true;
    RCLCPP_DEBUG(
        node_->get_logger(),
        "A* path planning successful, path length: %zu, searched_nodes=%zu",
        path.poses.size(), searched_nodes);
  } else {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "A* path planning failed: searched_nodes=%zu, visited_nodes=%zu, "
        "limited=%s",
        searched_nodes, searched_nodes, search_limited ? "true" : "false");
  }

  return success;
}

bool AStarPlanner::isObstacle(const geometry_msgs::msg::Point &point) {
  return isObstacle(point, 0.0);
}

bool AStarPlanner::isObstacle(const geometry_msgs::msg::Point &point,
                              double yaw) {
  CostmapConstPtr costmap_snapshot;
  std::shared_ptr<const std::vector<uint8_t>> footprint_mask_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!prepared_costmap_.valid()) return true;
    costmap_snapshot = prepared_costmap_.costmap;
    footprint_mask_snapshot = prepared_costmap_.footprint_mask;
  }
  int grid_x, grid_y;
  worldToGrid(point, *costmap_snapshot, grid_x, grid_y);
  return isRectangleFootprintBlocked(grid_x, grid_y, yaw, *costmap_snapshot,
                                     *footprint_mask_snapshot);
}

bool AStarPlanner::isObstacleWithDiagnostics(
    const geometry_msgs::msg::Point &point, const char *point_name,
    double yaw) {
  CostmapConstPtr costmap_snapshot;
  std::shared_ptr<const std::vector<uint8_t>> footprint_mask_snapshot;
  uint64_t snapshot_version = 0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!prepared_costmap_.valid()) {
      RCLCPP_INFO(node_->get_logger(),
                  "[AVOID_POINT_CHECK] point=%s world=(%.3f, %.3f), "
                  "blocked=1 reason=NO_PREPARED_COSTMAP",
                  point_name, point.x, point.y);
      return true;
    }
    costmap_snapshot = prepared_costmap_.costmap;
    footprint_mask_snapshot = prepared_costmap_.footprint_mask;
    snapshot_version = prepared_costmap_.version;
  }

  const auto &costmap = *costmap_snapshot;
  const auto &footprint_mask = *footprint_mask_snapshot;
  int grid_x = 0;
  int grid_y = 0;
  worldToGrid(point, costmap, grid_x, grid_y);
  const bool inside_map = isValidCell(grid_x, grid_y, costmap);
  int raw_cost = -998;
  bool footprint_blocked = false;
  if (inside_map) {
    const int index = grid_y * static_cast<int>(costmap.info.width) + grid_x;
    if (index >= 0 && index < static_cast<int>(costmap.data.size()) &&
        index < static_cast<int>(footprint_mask.size())) {
      raw_cost = static_cast<int>(costmap.data[index]);
      footprint_blocked = isRectangleFootprintBlocked(
          grid_x, grid_y, yaw, costmap, footprint_mask);
    }
  }

  const char *reason = "FREE";
  if (!inside_map) {
    reason = "OUT_OF_MAP";
  } else if (raw_cost < 0) {
    reason = "UNKNOWN";
  } else if (raw_cost >= obstacle_threshold_) {
    reason = "COST_THRESHOLD";
  } else if (footprint_blocked) {
    reason = "FOOTPRINT_COLLISION";
  }
  const bool blocked = !inside_map || raw_cost < 0 ||
                       raw_cost >= obstacle_threshold_ || footprint_blocked;

  RCLCPP_INFO(
      node_->get_logger(),
      "[AVOID_POINT_CHECK] point=%s world=(%.3f, %.3f), grid=(%d, %d), "
      "yaw=%.1f deg blocked=%d reason=%s raw_cost=%d threshold=%d "
      "footprint_blocked=%d, "
      "map_frame=%s map=%ux%u res=%.3f origin=(%.3f, %.3f) version=%lu",
      point_name, point.x, point.y, grid_x, grid_y, yaw * 180.0 / M_PI,
      blocked ? 1 : 0, reason,
      raw_cost, obstacle_threshold_, footprint_blocked ? 1 : 0,
      costmap.header.frame_id.c_str(), costmap.info.width, costmap.info.height,
      costmap.info.resolution, costmap.info.origin.position.x,
      costmap.info.origin.position.y,
      static_cast<unsigned long>(snapshot_version));
  return blocked;
}

bool AStarPlanner::isPathCollisionFree(const nav_msgs::msg::Path &path,
                                       bool allow_initial_escape) {
  CostmapConstPtr costmap_snapshot;
  std::shared_ptr<const std::vector<uint8_t>> footprint_mask_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!prepared_costmap_.valid()) return false;
    costmap_snapshot = prepared_costmap_.costmap;
    footprint_mask_snapshot = prepared_costmap_.footprint_mask;
  }

  const auto &costmap = *costmap_snapshot;
  const auto &footprint_mask = *footprint_mask_snapshot;
  if (path.poses.empty() || costmap.info.resolution <= 0.0) {
    return false;
  }

  // 按不大于半个栅格的间隔检查整条线段，避免平滑路径或稀疏路径的两个
  // 离散点均安全、但二者之间穿过障碍物。
  const double sample_step = costmap.info.resolution * 0.5;
  bool reached_strictly_free_sample = !allow_initial_escape;
  for (size_t pose_index = 0; pose_index < path.poses.size(); ++pose_index) {
    const auto &end = path.poses[pose_index].pose.position;
    const auto &start =
        pose_index == 0 ? end : path.poses[pose_index - 1].pose.position;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const auto &orientation = path.poses[pose_index].pose.orientation;
    const double sin_yaw =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw =
        1.0 - 2.0 * (orientation.y * orientation.y +
                     orientation.z * orientation.z);
    double path_yaw = std::atan2(sin_yaw, cos_yaw);
    if (std::hypot(dx, dy) >= 1e-8) {
      path_yaw = std::atan2(dy, dx);
    } else if (pose_index + 1 < path.poses.size()) {
      const auto &next = path.poses[pose_index + 1].pose.position;
      if (std::hypot(next.x - end.x, next.y - end.y) >= 1e-8) {
        path_yaw = std::atan2(next.y - end.y, next.x - end.x);
      }
    }
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(std::hypot(dx, dy) / sample_step)));
    for (int sample = 0; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / sample_count;
      geometry_msgs::msg::Point point;
      point.x = start.x + ratio * dx;
      point.y = start.y + ratio * dy;
      int grid_x, grid_y;
      worldToGrid(point, costmap, grid_x, grid_y);
      const bool footprint_blocked = isRectangleFootprintBlocked(
          grid_x, grid_y, path_yaw, costmap, footprint_mask);
      if (footprint_blocked) {
        // A*允许机器人从一个中心栅格安全、但带padding的矩形footprint与
        // 近障碍相交的起点向外脱困。复检必须采用相同语义：在首次进入
        // 严格安全区域前允许这个连续前缀；一旦已经安全，后续任何碰撞
        // 仍立即判失败。
        if (reached_strictly_free_sample ||
            !isStartPositionValid(grid_x, grid_y, costmap, footprint_mask)) {
          return false;
        }
      } else {
        reached_strictly_free_sample = true;
      }
    }
  }
  return reached_strictly_free_sample;
}

bool AStarPlanner::isRotationCollisionFree(
    const geometry_msgs::msg::Point &center, double start_yaw,
    double end_yaw) const {
  CostmapConstPtr costmap_snapshot;
  std::shared_ptr<const std::vector<uint8_t>> footprint_mask_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!prepared_costmap_.valid()) return false;
    costmap_snapshot = prepared_costmap_.costmap;
    footprint_mask_snapshot = prepared_costmap_.footprint_mask;
  }

  const auto &costmap = *costmap_snapshot;
  const auto &footprint_mask = *footprint_mask_snapshot;
  if (costmap.info.resolution <= 0.0) return false;

  int center_x = 0;
  int center_y = 0;
  worldToGrid(center, costmap, center_x, center_y);
  if (!isValidCell(center_x, center_y, costmap)) return false;

  const double yaw_delta = std::atan2(std::sin(end_yaw - start_yaw),
                                      std::cos(end_yaw - start_yaw));
  // 角点沿圆弧每移动不超过半个栅格就检查一次，并限制在1~5度之间。
  const double angular_step = std::clamp(
      costmap.info.resolution / std::max(2.0 * footprint_radius_, 1e-6),
      M_PI / 180.0, 5.0 * M_PI / 180.0);
  const int sample_count = std::max(
      1, static_cast<int>(std::ceil(std::abs(yaw_delta) / angular_step)));
  for (int sample = 0; sample <= sample_count; ++sample) {
    const double ratio = static_cast<double>(sample) / sample_count;
    const double yaw = start_yaw + ratio * yaw_delta;
    if (isRectangleFootprintBlocked(center_x, center_y, yaw, costmap,
                                    footprint_mask)) {
      return false;
    }
  }
  return true;
}

bool AStarPlanner::isObstacle(
    int grid_x, int grid_y, const nav_msgs::msg::OccupancyGrid &costmap,
    const std::vector<uint8_t> &footprint_mask) const {
  if (!isValidCell(grid_x, grid_y, costmap)) return true;

  const int index = grid_y * static_cast<int>(costmap.info.width) + grid_x;
  if (index >= 0 && index < static_cast<int>(costmap.data.size()) &&
      index < static_cast<int>(footprint_mask.size())) {
    return footprint_mask[index] != 0 ||
           costmap.data[index] >= obstacle_threshold_ ||
           costmap.data[index] < 0;
  }

  return true;  // 无效索引认为是障碍物
}

bool AStarPlanner::isRectangleFootprintBlocked(
    int center_x, int center_y, double yaw,
    const nav_msgs::msg::OccupancyGrid &costmap,
    const std::vector<uint8_t> &hard_obstacle_mask) const {
  if (!isValidCell(center_x, center_y, costmap) ||
      hard_obstacle_mask.size() != costmap.data.size() ||
      costmap.info.resolution <= 0.0) {
    return true;
  }

  const int center_index =
      center_y * static_cast<int>(costmap.info.width) + center_x;
  const int center_cost = static_cast<int>(costmap.data[center_index]);
  if (center_cost < 0 || center_cost >= obstacle_threshold_) {
    return true;
  }

  const double resolution = costmap.info.resolution;
  // 将栅格视为有面积的小方块，额外加半个栅格对角线，避免矩形边缘
  // 从障碍栅格角部穿过。
  const double cell_padding = resolution * std::sqrt(0.5);
  const double half_length = footprint_half_length_ + cell_padding;
  const double half_width = footprint_half_width_ + cell_padding;
  const int radius_cells =
      static_cast<int>(std::ceil(std::hypot(half_length, half_width) /
                                 resolution));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double world_dx = static_cast<double>(dx) * resolution;
      const double world_dy = static_cast<double>(dy) * resolution;
      const double local_x = cos_yaw * world_dx + sin_yaw * world_dy;
      const double local_y = -sin_yaw * world_dx + cos_yaw * world_dy;
      if (std::abs(local_x) > half_length ||
          std::abs(local_y) > half_width) {
        continue;
      }

      const int x = center_x + dx;
      const int y = center_y + dy;
      if (!isValidCell(x, y, costmap)) {
        return true;
      }
      const int index = y * static_cast<int>(costmap.info.width) + x;
      if (hard_obstacle_mask[index] != 0 || costmap.data[index] < 0) {
        return true;
      }
    }
  }
  return false;
}

void AStarPlanner::worldToGrid(const geometry_msgs::msg::Point &world_point,
                               const nav_msgs::msg::OccupancyGrid &costmap,
                               int &grid_x, int &grid_y) const {
  grid_x = static_cast<int>(
      std::floor((world_point.x - costmap.info.origin.position.x) /
                 costmap.info.resolution));
  grid_y = static_cast<int>(
      std::floor((world_point.y - costmap.info.origin.position.y) /
                 costmap.info.resolution));
}

void AStarPlanner::gridToWorld(int grid_x, int grid_y,
                               const nav_msgs::msg::OccupancyGrid &costmap,
                               geometry_msgs::msg::Point &world_point) const {
  world_point.x =
      costmap.info.origin.position.x + (grid_x + 0.5) * costmap.info.resolution;
  world_point.y =
      costmap.info.origin.position.y + (grid_y + 0.5) * costmap.info.resolution;
  world_point.z = costmap.info.origin.position.z;
}

void AStarPlanner::publishVisualization(const geometry_msgs::msg::Point &start,
                                        const geometry_msgs::msg::Point &goal,
                                        const nav_msgs::msg::Path &path) {
  // 发布路径
  nav_msgs::msg::Path vis_path = path;
  vis_path.header.frame_id = global_frame_;
  vis_path.header.stamp = node_->now();
  path_pub_->publish(vis_path);

  // 发布标记
  visualization_msgs::msg::MarkerArray marker_array;

  // 起点标记
  visualization_msgs::msg::Marker start_marker =
      createPointMarker(start, 0, 0.0, 1.0, 0.4);
  start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.3;
  marker_array.markers.push_back(start_marker);

  // 终点标记
  visualization_msgs::msg::Marker goal_marker =
      createPointMarker(goal, 1, 1.0, 0.0, 0.0);
  goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3;
  marker_array.markers.push_back(goal_marker);

  marker_pub_->publish(marker_array);
}

double AStarPlanner::calculateHeuristic(int x, int y, int goal_x,
                                        int goal_y) const {
  const double dx = std::abs(goal_x - x);
  const double dy = std::abs(goal_y - y);
  if (!allow_diagonal_) {
    return straight_cost_ * (dx + dy);
  }

  // 8邻域Octile启发；只包含基础移动代价，障碍和偏离代价留在g中。
  const double diagonal_steps = std::min(dx, dy);
  const double straight_steps = std::max(dx, dy) - diagonal_steps;
  return diagonal_cost_ * diagonal_steps + straight_cost_ * straight_steps;
}

double AStarPlanner::calculateObstacleCost(
    int x, int y, const nav_msgs::msg::OccupancyGrid &costmap,
    const std::vector<uint8_t> &footprint_mask) const {
  if (!isValidCell(x, y, costmap)) return 100.0;

  // 首先检查当前点的代价值
  int current_index = y * costmap.info.width + x;
  if (current_index >= 0 &&
      current_index < static_cast<int>(costmap.data.size())) {
    int8_t current_cost = costmap.data[current_index];

    // 可通行的软代价区域允许通过，但不鼓励路径贴近高代价栅格。
    if (current_cost > 0 && current_cost < obstacle_threshold_) {
      return static_cast<double>(current_cost) /
             std::max(1.0, static_cast<double>(obstacle_threshold_));
    }
  }

  double min_obstacle_distance = std::numeric_limits<double>::max();

  // 在安全边距范围内检查真实障碍物距离
  for (int dx = -safety_margin_; dx <= safety_margin_; ++dx) {
    for (int dy = -safety_margin_; dy <= safety_margin_; ++dy) {
      int check_x = x + dx;
      int check_y = y + dy;

      if (!isValidCell(check_x, check_y, costmap)) continue;

      int index = check_y * costmap.info.width + check_x;
      if (index >= 0 && index < static_cast<int>(costmap.data.size())) {
        // 只考虑真实的高代价障碍物
        if (footprint_mask[index] != 0 ||
            costmap.data[index] >= obstacle_threshold_ ||
            costmap.data[index] < 0) {
          double distance = std::sqrt(dx * dx + dy * dy);
          min_obstacle_distance = std::min(min_obstacle_distance, distance);
        }
      }
    }
  }

  // 根据到最近真实障碍物的距离计算代价
  if (min_obstacle_distance == std::numeric_limits<double>::max()) {
    return 0.0;                            // 没有附近的真实障碍物
  } else if (min_obstacle_distance < 2.0)  // 降低安全距离要求
  {
    // 距离真实障碍物较近，给予适中代价
    return 2.0 * (2.0 - min_obstacle_distance);
  } else {
    return 0.0;  // 距离足够远，不增加额外代价
  }
}

double AStarPlanner::calculatePathDeviationCost(
    int x, int y, const nav_msgs::msg::OccupancyGrid &costmap,
    const nav_msgs::msg::Path &global_path) const {
  if (path_deviation_weight_ <= 0.0 || global_path.poses.empty()) {
    return 0.0;
  }

  // Convert grid coordinates to world coordinates
  double world_x =
      (x + 0.5) * costmap.info.resolution + costmap.info.origin.position.x;
  double world_y =
      (y + 0.5) * costmap.info.resolution + costmap.info.origin.position.y;

  // Find minimum distance to global path
  double min_distance = std::numeric_limits<double>::max();

  for (const auto &pose : global_path.poses) {
    double dx = world_x - pose.pose.position.x;
    double dy = world_y - pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);

    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  // Apply weight to deviation cost
  return path_deviation_weight_ * min_distance;
}

bool AStarPlanner::isValidCell(
    int x, int y, const nav_msgs::msg::OccupancyGrid &costmap) const {
  return x >= 0 && x < static_cast<int>(costmap.info.width) && y >= 0 &&
         y < static_cast<int>(costmap.info.height);
}

bool AStarPlanner::isStartPositionValid(
    int grid_x, int grid_y, const nav_msgs::msg::OccupancyGrid &costmap,
    const std::vector<uint8_t> &footprint_mask) const {
  if (!isValidCell(grid_x, grid_y, costmap)) return false;

  const int index = grid_y * costmap.info.width + grid_x;
  if (index >= 0 && index < static_cast<int>(costmap.data.size()) &&
      index < static_cast<int>(footprint_mask.size())) {
    return footprint_mask[index] == 0 && costmap.data[index] >= 0 &&
           costmap.data[index] < start_obstacle_threshold_;
  }

  return false;  // 无效索引认为无效
}

std::vector<uint8_t> AStarPlanner::buildFootprintMask(
    const nav_msgs::msg::OccupancyGrid &costmap) const {
  const int width = static_cast<int>(costmap.info.width);
  const int height = static_cast<int>(costmap.info.height);
  std::vector<uint8_t> mask(costmap.data.size(), 0U);
  if (width <= 0 || height <= 0 || costmap.info.resolution <= 0.0 ||
      costmap.data.size() != static_cast<size_t>(width * height)) {
    return {};
  }

  // 这里只缓存硬障碍/未知栅格。车体外形不再预先按外接圆膨胀，而是在
  // 查询时根据运动方向旋转矩形footprint进行检查。
  for (int obstacle_y = 0; obstacle_y < height; ++obstacle_y) {
    for (int obstacle_x = 0; obstacle_x < width; ++obstacle_x) {
      const int obstacle_index = obstacle_y * width + obstacle_x;
      const int cost = costmap.data[obstacle_index];
      if (cost >= 0 && cost < footprint_obstacle_threshold_) {
        continue;
      }

      mask[obstacle_index] = 1U;
    }
  }

  return mask;
}

nav_msgs::msg::Path AStarPlanner::reconstructPath(
    size_t goal_index, const std::vector<int32_t> &parent_indices,
    const nav_msgs::msg::OccupancyGrid &costmap) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  std::vector<size_t> path_indices;
  int64_t current_index = static_cast<int64_t>(goal_index);

  // 从目标节点回溯到起始节点
  while (current_index >= 0) {
    path_indices.push_back(static_cast<size_t>(current_index));
    current_index = parent_indices[static_cast<size_t>(current_index)];
  }

  // 反转路径（从起点到终点）
  std::reverse(path_indices.begin(), path_indices.end());

  // 转换为世界坐标
  const size_t width = costmap.info.width;
  for (const size_t cell_index : path_indices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = node_->now();

    const int x = static_cast<int>(cell_index % width);
    const int y = static_cast<int>(cell_index / width);
    gridToWorld(x, y, costmap, pose.pose.position);
    pose.pose.orientation.w = 1.0;

    path.poses.push_back(pose);
  }

  return path;
}

visualization_msgs::msg::Marker AStarPlanner::createPointMarker(
    const geometry_msgs::msg::Point &point, int id, double r, double g,
    double b) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = node_->now();
  marker.ns = "astar_points";
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

}  // namespace pure_pursuit_local_planner
