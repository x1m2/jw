#ifndef NAVIGATION_TASK_STATE_MACHINE_HPP
#define NAVIGATION_TASK_STATE_MACHINE_HPP

#include <dirent.h>
#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
namespace user_interface {

/**
 * @brief 导航任务状态枚举
 */
enum class NavigationTaskState {
  IDLE,       // 空闲状态
  RUNNING,    // 运行状态
  WAITING,    // 等待状态（到达目标点后的延时等待）
  STOPPED,    // 停止状态
  GOING_HOME  // 回家状态
};

/**
 * @brief 导航任务信号枚举
 */
enum class NavigationTaskSignal {
  START_LOOP = 1,  // 开始循环导航
  STOP = 2,        // 停止任务
  CONTINUE = 3,    // 继续执行任务
  GO_HOME = 4      // 回家
};

/**
 * @brief 任务项结构体
 */
struct TaskItem {
  int target_id;     // 目标点ID
  int wait_delay;    // 等待延时（秒）
  int publish_data;  // 抵达后发布的数据

  TaskItem() : target_id(0), wait_delay(0), publish_data(0) {}

  TaskItem(int id, int delay, int data)
      : target_id(id), wait_delay(delay), publish_data(data) {}
};

/**
 * @brief 目标点结构体
 */
struct TargetPoint {
  int id;  // 目标点ID

  double x, y, z;  // 位置坐标

  double qx, qy, qz, qw;  // 四元数朝向

  std::string label;  // 标签

  TargetPoint() : id(0), x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(1) {}
};

/**
 * @brief 导航任务状态机类
 */
class NavigationTaskStateMachine : public rclcpp::Node {
 public:
  /**
   * @brief 构造函数
   */
  NavigationTaskStateMachine();

  /**
   * @brief 析构函数
   */
  ~NavigationTaskStateMachine();

  /**
   * @brief 初始化状态机
   * @return 成功返回true，失败返回false
   */
  bool initialize();

  /**
   * @brief 运行状态机主循环
   */
  void run();

 private:
  // ROS2相关
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav_state_sub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_task_signal_sub_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_action_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_points_pub_;

  rclcpp::TimerBase::SharedPtr state_machine_timer_;

  // 当前定位位姿
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // 状态机相关
  NavigationTaskState current_state_;

  NavigationTaskState previous_state_;

  // 任务相关
  std::map<std::string, std::vector<TaskItem>> task_lists_map_;

  std::vector<TaskItem> current_task_list_;

  std::string current_task_name_;

  std::map<int, TargetPoint> target_points_map_;

  int current_task_index_;

  rclcpp::Time wait_start_time_;

  int current_wait_delay_;

  // 导航状态
  bool nav_state_;

  bool nav_state_received_;

  bool target_point_published_;

  NavigationTaskSignal last_task_signal_;

  // 执行模式
  bool loop_mode_;

  // 配置文件路径
  std::string task_folder_path_;

  std::string target_points_file_;

  // 控制参数
  double state_machine_frequency_;

  geometry_msgs::msg::PoseStamped current_pose_;
  bool has_current_pose_ = false;

  // 跳过第一个任务点参数
  bool enable_skip_first_target_ = true;
  double skip_first_target_threshold_ = 0.8;
  std::string pose_topic_ = "/pose_stamped_localization";

  /**
   * @brief 加载参数
   */
  void loadParameters();

  /**
   * @brief 从文件夹加载所有任务列表
   * @return 成功返回true，失败返回false
   */
  bool loadAllTaskLists();

  /**
   * @brief 从单个文件加载任务列表
   * @param file_path 任务文件路径
   * @param task_list 输出任务列表
   * @return 成功返回true，失败返回false
   */
  bool loadTaskListFromFile(const std::string& file_path,
                            std::vector<TaskItem>& task_list);

  /**
   * @brief 从文件加载目标点
   * @return 成功返回true，失败返回false
   */
  bool loadTargetPoints();

  /**
   * @brief 校验任务列表中的目标点ID是否都存在
   * @return 校验通过返回true，否则返回false
   */
  bool validateTaskList();

  /**
   * @brief 导航状态回调函数
   * @param msg 导航状态消息
   */
  void navStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

  /**
   * @brief 导航任务信号回调函数
   * @param msg 导航任务信号消息
   */
  void navTaskSignalCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief 状态机定时器回调函数
   */
  void stateMachineTimerCallback();

  /**
   * @brief 处理状态机逻辑
   */
  void processStateMachine();

  /**
   * @brief 发布目标点
   * @param target_id 目标点ID
   */
  void publishTargetPoint(int target_id);

  /**
   * @brief 发布回家目标点 (0,0,0)
   */
  void publishHomeTargetPoint();

  /**
   * @brief 发布任务动作
   * @param action_data 动作数据
   */
  void publishTaskAction(int action_data);

  /**
   * @brief 重置状态机到空闲状态
   */
  void resetStateMachine();

  /**
   * @brief 切换到下一个任务
   */
  void moveToNextTask();

  /**
   * @brief 检查是否为最后一个任务
   * @return 是最后一个任务返回true
   */
  bool isLastTask() const;

  /**
   * @brief 获取当前任务项
   * @return 当前任务项引用
   */
  const TaskItem& getCurrentTask() const;

  /**
   * @brief 状态转换日志输出
   * @param from_state 起始状态
   * @param to_state 目标状态
   */
  void logStateTransition(NavigationTaskState from_state,
                          NavigationTaskState to_state);

  /**
   * @brief 状态枚举转字符串
   * @param state 状态枚举
   * @return 字符串
   */
  std::string stateToString(NavigationTaskState state) const;
  //跳过最近目标点
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void adjustStartIndexByCurrentPose();
  double distance2D(double x1, double y1, double x2, double y2) const;
};

}  // namespace user_interface

#endif  // NAVIGATION_TASK_STATE_MACHINE_HPP