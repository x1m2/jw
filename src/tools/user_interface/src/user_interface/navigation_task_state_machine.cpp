#include "user_interface/navigation_task_state_machine.hpp"

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <locale>
#include <sstream>

namespace user_interface {

static std::string resolveRosFindPath(const std::string &input) {
  const std::string prefix = "$(find ";

  if (input.rfind(prefix, 0) != 0) {
    return input;
  }

  const std::size_t end = input.find(')', prefix.size());

  if (end == std::string::npos) {
    return input;
  }

  const std::string package_name =
      input.substr(prefix.size(), end - prefix.size());

  std::string package_path;

  try {
    package_path = ament_index_cpp::get_package_share_directory(package_name);
  } catch (const std::exception &e) {
    return input;
  }

  if (package_path.empty()) {
    return input;
  }

  const std::string suffix = input.substr(end + 1);

  return package_path + suffix;
}

NavigationTaskStateMachine::NavigationTaskStateMachine()
    : Node("navigation_task_state_machine_node"),
      current_state_(NavigationTaskState::IDLE),
      previous_state_(NavigationTaskState::IDLE),
      current_task_index_(0),
      current_wait_delay_(0),
      nav_state_(false),
      nav_state_received_(false),
      target_point_published_(false),
      last_task_signal_(NavigationTaskSignal::STOP),
      loop_mode_(false),
      state_machine_frequency_(10.0) {
  RCLCPP_INFO(this->get_logger(), "NavigationTaskStateMachine 构造函数");
}

NavigationTaskStateMachine::~NavigationTaskStateMachine() {
  RCLCPP_INFO(this->get_logger(), "NavigationTaskStateMachine 析构函数");
}

bool NavigationTaskStateMachine::initialize() {
  try {
    // 加载参数
    loadParameters();

    // 初始化发布器
    task_action_pub_ =
        this->create_publisher<std_msgs::msg::Int32>("/task_action", 10);

    target_points_pub_ =
        this->create_publisher<geometry_msgs::msg::Pose>("/target_points", 10);

    // 初始化订阅器
    nav_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nav_state", 10,
        std::bind(&NavigationTaskStateMachine::navStateCallback, this,
                  std::placeholders::_1));

    nav_task_signal_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_task_signal", 10,
        std::bind(&NavigationTaskStateMachine::navTaskSignalCallback, this,
                  std::placeholders::_1));

    // 初始化状态机定时器
    state_machine_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / state_machine_frequency_),
        std::bind(&NavigationTaskStateMachine::processStateMachine, this));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&NavigationTaskStateMachine::poseCallback, this,
                  std::placeholders::_1));
    // 加载任务数据
    if (!loadAllTaskLists()) {
      RCLCPP_ERROR(this->get_logger(), "加载任务列表失败");
      return false;
    }

    if (!loadTargetPoints()) {
      RCLCPP_ERROR(this->get_logger(), "加载目标点失败");
      return false;
    }

    // 校验任务列表
    if (!validateTaskList()) {
      RCLCPP_ERROR(this->get_logger(), "任务列表校验失败");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "NavigationTaskStateMachine 初始化成功");

    RCLCPP_INFO(this->get_logger(), "加载了 %zu 个任务文件和 %zu 个目标点",
                task_lists_map_.size(), target_points_map_.size());

    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "NavigationTaskStateMachine 初始化失败: %s", e.what());

    return false;
  }
}

void NavigationTaskStateMachine::loadParameters() {
  // 声明参数
  this->declare_parameter<std::string>("task_folder_path",
                                       "$(find user_interface)/config/task");

  this->declare_parameter<std::string>(
      "target_points_file",
      "$(find user_interface)/config/generated_map_target_points.txt");

  this->declare_parameter<double>("state_machine_frequency", 10.0);
  this->declare_parameter<bool>("enable_skip_first_target", true);
  this->declare_parameter<double>("skip_first_target_threshold", 0.8);
  this->declare_parameter<std::string>("pose_topic",
                                       "/pose_stamped_localization");

  this->get_parameter("enable_skip_first_target", enable_skip_first_target_);
  this->get_parameter("skip_first_target_threshold",
                      skip_first_target_threshold_);
  this->get_parameter("pose_topic", pose_topic_);

  // 获取参数
  this->get_parameter("task_folder_path", task_folder_path_);

  this->get_parameter("target_points_file", target_points_file_);

  this->get_parameter("state_machine_frequency", state_machine_frequency_);

  task_folder_path_ = resolveRosFindPath(task_folder_path_);

  target_points_file_ = resolveRosFindPath(target_points_file_);

  RCLCPP_INFO(this->get_logger(), "任务文件夹路径: %s",
              task_folder_path_.c_str());

  RCLCPP_INFO(this->get_logger(), "目标点文件: %s",
              target_points_file_.c_str());

  RCLCPP_INFO(this->get_logger(), "状态机频率: %.1f Hz",
              state_machine_frequency_);
  RCLCPP_INFO(this->get_logger(), "是否启用跳过首点: %s",
              enable_skip_first_target_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "跳过首点距离阈值: %.2f m",
              skip_first_target_threshold_);
  RCLCPP_INFO(this->get_logger(), "状态机定位话题: %s", pose_topic_.c_str());
}

bool NavigationTaskStateMachine::loadAllTaskLists() {
  task_lists_map_.clear();

  DIR *dir = opendir(task_folder_path_.c_str());

  if (!dir) {
    RCLCPP_ERROR(this->get_logger(), "无法打开任务文件夹: %s",
                 task_folder_path_.c_str());

    return false;
  }

  struct dirent *entry;
  int task_file_count = 0;

  while ((entry = readdir(dir)) != NULL) {
    std::string filename = entry->d_name;

    // 跳过 "." ".." 和隐藏文件
    if (filename[0] == '.') {
      continue;
    }

    // 只处理 .txt 文件
    if (filename.length() < 4 ||
        filename.substr(filename.length() - 4) != ".txt") {
      continue;
    }

    std::string full_path = task_folder_path_ + "/" + filename;

    // 检查是否为普通文件
    struct stat st;

    if (stat(full_path.c_str(), &st) != 0 || !S_ISREG(st.st_mode)) {
      continue;
    }

    // 从文件加载任务列表
    std::vector<TaskItem> task_list;

    if (loadTaskListFromFile(full_path, task_list)) {
      // 去掉 .txt 后缀作为任务名称
      std::string task_name = filename.substr(0, filename.length() - 4);

      task_lists_map_[task_name] = task_list;

      task_file_count++;

      RCLCPP_INFO(this->get_logger(), "成功加载任务文件: %s (%zu 个任务)",
                  task_name.c_str(), task_list.size());
    }
  }

  closedir(dir);

  if (task_lists_map_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "未找到任何有效的任务文件");

    return false;
  }

  RCLCPP_INFO(this->get_logger(), "成功加载 %d 个任务文件", task_file_count);

  return true;
}

bool NavigationTaskStateMachine::loadTaskListFromFile(
    const std::string &file_path, std::vector<TaskItem> &task_list) {
  std::ifstream file(file_path);

  if (!file.is_open()) {
    RCLCPP_WARN(this->get_logger(), "无法打开任务列表文件: %s",
                file_path.c_str());

    return false;
  }

  task_list.clear();

  std::string line;

  int line_number = 0;

  while (std::getline(file, line)) {
    line_number++;

    // 跳过注释行和空行
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);

    TaskItem task;

    if (!(iss >> task.target_id >> task.wait_delay >> task.publish_data)) {
      RCLCPP_WARN(this->get_logger(), "文件 %s 第 %d 行格式错误: %s",
                  file_path.c_str(), line_number, line.c_str());

      continue;
    }

    task_list.push_back(task);

    RCLCPP_DEBUG(this->get_logger(), "加载任务: ID=%d, 延时=%d秒, 发布数据=%d",
                 task.target_id, task.wait_delay, task.publish_data);
  }

  file.close();

  if (task_list.empty()) {
    RCLCPP_WARN(this->get_logger(), "任务列表文件 %s 为空", file_path.c_str());

    return false;
  }

  return true;
}

bool NavigationTaskStateMachine::loadTargetPoints() {
  std::ifstream file(target_points_file_);

  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "无法打开目标点文件: %s",
                 target_points_file_.c_str());

    return false;
  }

  target_points_map_.clear();

  std::string line;

  int line_number = 0;

  while (std::getline(file, line)) {
    line_number++;

    // 跳过注释行和空行
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);

    TargetPoint point;

    if (!(iss >> point.id >> point.x >> point.y >> point.z >> point.qx >>
          point.qy >> point.qz >> point.qw >> point.label)) {
      RCLCPP_WARN(this->get_logger(), "目标点文件第 %d 行格式错误: %s",
                  line_number, line.c_str());

      continue;
    }

    target_points_map_[point.id] = point;

    RCLCPP_DEBUG(this->get_logger(),
                 "加载目标点: ID=%d, 位置=(%.3f,%.3f,%.3f), 标签=%s", point.id,
                 point.x, point.y, point.z, point.label.c_str());
  }

  file.close();

  if (target_points_map_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "目标点列表为空");

    return false;
  }

  RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个目标点",
              target_points_map_.size());

  return true;
}

bool NavigationTaskStateMachine::validateTaskList() {
  bool validation_passed = true;

  for (const auto &task_pair : task_lists_map_) {
    const std::string &task_name = task_pair.first;

    const std::vector<TaskItem> &task_list = task_pair.second;

    for (const auto &task : task_list) {
      if (target_points_map_.find(task.target_id) == target_points_map_.end()) {
        RCLCPP_ERROR(this->get_logger(),
                     "任务 %s 中的目标点ID %d 在目标点文件中不存在",
                     task_name.c_str(), task.target_id);

        validation_passed = false;
      }
    }
  }

  if (validation_passed) {
    RCLCPP_INFO(this->get_logger(), "所有任务列表校验通过");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "任务列表校验失败，请检查任务列表中的目标点ID");
  }

  return validation_passed;
}

void NavigationTaskStateMachine::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_pose_ = *msg;
  has_current_pose_ = true;

  RCLCPP_DEBUG(this->get_logger(), "状态机收到当前位姿: x=%.3f, y=%.3f",
               current_pose_.pose.position.x, current_pose_.pose.position.y);
}

void NavigationTaskStateMachine::navStateCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  nav_state_ = msg->data;

  nav_state_received_ = true;

  RCLCPP_DEBUG(this->get_logger(), "收到导航状态: %s",
               nav_state_ ? "已到达" : "未到达");
}

void NavigationTaskStateMachine::navTaskSignalCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  std::string signal = msg->data;

  RCLCPP_INFO(this->get_logger(), "收到任务信号: %s", signal.c_str());

  // STOP
  if (signal == "STOP") {
    RCLCPP_INFO(this->get_logger(), "停止任务");

    if (current_state_ != NavigationTaskState::IDLE) {
      previous_state_ = current_state_;

      current_state_ = NavigationTaskState::STOPPED;

      logStateTransition(previous_state_, current_state_);
    }

    return;
  } else if (signal == "CONTINUE") {
    RCLCPP_INFO(this->get_logger(), "继续执行任务");

    if (current_state_ == NavigationTaskState::STOPPED) {
      if (!nav_state_received_) {
        target_point_published_ = false;
      }

      previous_state_ = current_state_;

      current_state_ = NavigationTaskState::RUNNING;

      logStateTransition(previous_state_, current_state_);
    }

    return;
  } else if (signal == "GO_HOME") {
    RCLCPP_INFO(this->get_logger(), "回家");

    loop_mode_ = false;

    previous_state_ = current_state_;

    current_state_ = NavigationTaskState::GOING_HOME;

    logStateTransition(previous_state_, current_state_);

    publishHomeTargetPoint();

    return;
  }

  bool request_loop = false;

  std::string task_name = signal;

  const std::string loop_prefix = "LOOP:";

  if (signal.rfind(loop_prefix, 0) == 0) {
    request_loop = true;

    task_name = signal.substr(loop_prefix.size());
  }

  // 任务名称处理
  auto it = task_lists_map_.find(task_name);

  if (it != task_lists_map_.end()) {
    RCLCPP_INFO(this->get_logger(), "开始执行任务: %s (%zu 个任务点)%s",
                task_name.c_str(), it->second.size(),
                request_loop ? " [循环]" : "");

    if (current_state_ == NavigationTaskState::IDLE ||
        current_state_ == NavigationTaskState::STOPPED) {
      loop_mode_ = request_loop;

      current_task_name_ = task_name;

      current_task_list_ = it->second;

      // 根据当前位置判断从第几个任务点开始
      adjustStartIndexByCurrentPose();

      nav_state_received_ = false;

      nav_state_ = false;

      target_point_published_ = false;

      previous_state_ = current_state_;

      current_state_ = NavigationTaskState::RUNNING;

      logStateTransition(previous_state_, current_state_);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "当前状态不允许开始新任务，请先停止当前任务");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "未找到任务: %s", task_name.c_str());

    RCLCPP_INFO(this->get_logger(), "可用的任务有:");

    for (const auto &task_pair : task_lists_map_) {
      RCLCPP_INFO(this->get_logger(), "  - %s", task_pair.first.c_str());
    }
  }
}

void NavigationTaskStateMachine::processStateMachine() {
  switch (current_state_) {
    case NavigationTaskState::IDLE: {
      // 空闲状态
      break;
    }

    case NavigationTaskState::RUNNING: {
      if (current_task_list_.empty()) {
        RCLCPP_WARN(this->get_logger(), "当前任务列表为空，重置状态机");

        resetStateMachine();

        break;
      }

      if (current_task_index_ >= static_cast<int>(current_task_list_.size())) {
        if (loop_mode_) {
          RCLCPP_INFO(this->get_logger(), "任务 %s 完成，开始下一轮循环",
                      current_task_name_.c_str());

          current_task_index_ = 0;

          nav_state_received_ = false;

          nav_state_ = false;

          target_point_published_ = false;

          break;
        }

        RCLCPP_INFO(this->get_logger(), "任务 %s 完成，回到空闲状态",
                    current_task_name_.c_str());

        resetStateMachine();

        break;
      }

      const TaskItem &current_task = getCurrentTask();

      if (!nav_state_received_) {
        if (!target_point_published_) {
          publishTargetPoint(current_task.target_id);

          target_point_published_ = true;

          RCLCPP_INFO(this->get_logger(), "发布目标点 ID=%d, 标签=%s",
                      current_task.target_id,
                      target_points_map_[current_task.target_id].label.c_str());
        }
      } else if (nav_state_) {
        publishTaskAction(current_task.publish_data);

        RCLCPP_INFO(this->get_logger(), "到达目标点 ID=%d, 发布动作数据=%d",
                    current_task.target_id, current_task.publish_data);

        if (current_task.wait_delay > 0) {
          current_wait_delay_ = current_task.wait_delay;

          wait_start_time_ = this->now();

          previous_state_ = current_state_;

          current_state_ = NavigationTaskState::WAITING;

          logStateTransition(previous_state_, current_state_);

          RCLCPP_INFO(this->get_logger(), "开始等待 %d 秒",
                      current_wait_delay_);
        } else {
          moveToNextTask();
        }
      }

      break;
    }

    case NavigationTaskState::WAITING: {
      rclcpp::Duration elapsed = this->now() - wait_start_time_;

      if (elapsed.seconds() >= current_wait_delay_) {
        RCLCPP_INFO(this->get_logger(), "等待时间结束，继续下一个任务");

        moveToNextTask();

        previous_state_ = current_state_;

        current_state_ = NavigationTaskState::RUNNING;

        logStateTransition(previous_state_, current_state_);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "等待中... 剩余时间: %.1f 秒",
                     current_wait_delay_ - elapsed.seconds());
      }

      break;
    }

    case NavigationTaskState::STOPPED: {
      // 停止状态
      break;
    }

    case NavigationTaskState::GOING_HOME: {
      if (nav_state_received_ && nav_state_) {
        RCLCPP_INFO(this->get_logger(), "已到达家位置，重置状态机");

        resetStateMachine();
      }

      break;
    }
  }
}

void NavigationTaskStateMachine::publishTargetPoint(int target_id) {
  geometry_msgs::msg::Pose target_msg;

  auto it = target_points_map_.find(target_id);

  if (it != target_points_map_.end()) {
    const TargetPoint &point = it->second;

    target_msg.position.x = point.x;
    target_msg.position.y = point.y;
    target_msg.position.z = point.z;

    target_msg.orientation.x = point.qx;
    target_msg.orientation.y = point.qy;
    target_msg.orientation.z = point.qz;
    target_msg.orientation.w = point.qw;

    RCLCPP_INFO(this->get_logger(),
                "发布目标点: ID=%d, 位置=(%.3f,%.3f,%.3f), 标签=%s", target_id,
                point.x, point.y, point.z, point.label.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "目标点ID %d 不存在", target_id);

    return;
  }

  target_points_pub_->publish(target_msg);
}

void NavigationTaskStateMachine::publishHomeTargetPoint() {
  geometry_msgs::msg::Pose target_msg;

  target_msg.position.x = 0.0;
  target_msg.position.y = 0.0;
  target_msg.position.z = 0.0;

  target_msg.orientation.x = 0.0;
  target_msg.orientation.y = 0.0;
  target_msg.orientation.z = 0.0;
  target_msg.orientation.w = 1.0;

  target_points_pub_->publish(target_msg);

  RCLCPP_INFO(this->get_logger(), "发布回家目标点: (0, 0, 0)");
}

void NavigationTaskStateMachine::publishTaskAction(int action_data) {
  std_msgs::msg::Int32 msg;

  msg.data = action_data;

  task_action_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "发布任务动作: %d", action_data);
}

void NavigationTaskStateMachine::resetStateMachine() {
  previous_state_ = current_state_;

  current_state_ = NavigationTaskState::IDLE;

  loop_mode_ = false;

  current_task_name_.clear();

  current_task_list_.clear();

  current_task_index_ = 0;

  nav_state_received_ = false;

  nav_state_ = false;

  target_point_published_ = false;

  current_wait_delay_ = 0;

  logStateTransition(previous_state_, current_state_);

  RCLCPP_INFO(this->get_logger(), "状态机已重置到空闲状态");
}

void NavigationTaskStateMachine::moveToNextTask() {
  current_task_index_++;

  nav_state_received_ = false;

  nav_state_ = false;

  target_point_published_ = false;

  if (current_task_index_ >= static_cast<int>(current_task_list_.size())) {
    RCLCPP_INFO(this->get_logger(), "任务 %s 的所有任务点已完成",
                current_task_name_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "移动到下一个任务点，索引: %d/%zu",
                current_task_index_, current_task_list_.size());
  }
}

bool NavigationTaskStateMachine::isLastTask() const {
  return current_task_index_ >= static_cast<int>(current_task_list_.size()) - 1;
}

const TaskItem &NavigationTaskStateMachine::getCurrentTask() const {
  static TaskItem default_task;

  if (current_task_index_ >= 0 &&
      current_task_index_ < static_cast<int>(current_task_list_.size())) {
    return current_task_list_[current_task_index_];
  }

  return default_task;
}

void NavigationTaskStateMachine::logStateTransition(
    NavigationTaskState from_state, NavigationTaskState to_state) {
  RCLCPP_INFO(this->get_logger(), "状态转换: %s -> %s",
              stateToString(from_state).c_str(),
              stateToString(to_state).c_str());
}

std::string NavigationTaskStateMachine::stateToString(
    NavigationTaskState state) const {
  switch (state) {
    case NavigationTaskState::IDLE:
      return "IDLE";

    case NavigationTaskState::RUNNING:
      return "RUNNING";

    case NavigationTaskState::WAITING:
      return "WAITING";

    case NavigationTaskState::STOPPED:
      return "STOPPED";

    case NavigationTaskState::GOING_HOME:
      return "GOING_HOME";

    default:
      return "UNKNOWN";
  }
}

double NavigationTaskStateMachine::distance2D(double x1, double y1, double x2,
                                              double y2) const {
  const double dx = x1 - x2;
  const double dy = y1 - y2;
  return std::sqrt(dx * dx + dy * dy);
}

void NavigationTaskStateMachine::adjustStartIndexByCurrentPose() {
  current_task_index_ = 0;

  if (!enable_skip_first_target_) {
    RCLCPP_INFO(this->get_logger(),
                "[SKIP_FIRST_DISABLED] 不启用首点跳过，从第一个任务点开始");
    return;
  }

  if (!has_current_pose_) {
    RCLCPP_WARN(
        this->get_logger(),
        "[SKIP_FIRST_NO_POSE] 还没有当前定位位姿，默认从第一个任务点开始");
    return;
  }

  if (current_task_list_.empty()) {
    RCLCPP_WARN(this->get_logger(), "[SKIP_FIRST_EMPTY_TASK] 当前任务列表为空");
    return;
  }

  const TaskItem &first_task = current_task_list_[0];

  auto it = target_points_map_.find(first_task.target_id);
  if (it == target_points_map_.end()) {
    RCLCPP_WARN(this->get_logger(),
                "[SKIP_FIRST_NO_TARGET] 找不到第一个目标点 ID=%d",
                first_task.target_id);
    return;
  }

  const TargetPoint &first_point = it->second;

  double dist =
      distance2D(current_pose_.pose.position.x, current_pose_.pose.position.y,
                 first_point.x, first_point.y);

  if (dist <= skip_first_target_threshold_) {
    if (current_task_list_.size() >= 2) {
      current_task_index_ = 1;

      RCLCPP_WARN(
          this->get_logger(),
          "[SKIP_FIRST_TARGET] 当前离首点 ID=%d 很近, dist=%.3f <= %.3f, "
          "跳过首点, 从第二个任务点开始",
          first_task.target_id, dist, skip_first_target_threshold_);
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "[SKIP_FIRST_ONLY_ONE] 当前离首点很近, 但任务只有一个点, 不跳过");
    }
  } else {
    current_task_index_ = 0;

    RCLCPP_WARN(
        this->get_logger(),
        "[START_FROM_FIRST_TARGET] 当前离首点 ID=%d 较远, dist=%.3f > %.3f, "
        "从首点开始",
        first_task.target_id, dist, skip_first_target_threshold_);
  }
}

void NavigationTaskStateMachine::run() {
  RCLCPP_INFO(this->get_logger(), "导航任务状态机开始运行");

  rclcpp::spin(shared_from_this());
}

}  // namespace user_interface