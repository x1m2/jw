#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <slam_manager_interfaces/msg/task_target.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <plan_manage/controller.h>
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/pose_adapter.h>
#include <plan_manage/topic_config.h>
#include <bspline_opt/uniform_bspline.h>

namespace scan_planner {
class ClosedLoopController : public Controller {
 public:
  explicit ClosedLoopController(rclcpp::Node &node, SCANReplanFSM &planner) : node_(node) {
    auto number = [this](const char *name, double fallback) {
      if (!node_.has_parameter(name)) node_.declare_parameter<double>(name, fallback);
      return node_.get_parameter(name).as_double();
    };
    time_forward_ = number("closed_loop.time_forward", 0.8);
    heading_error_threshold_ = number("closed_loop.heading_error_threshold", 0.8);
    kp_pos_ = number("closed_loop.kp_pos", 0.8);
    kp_yaw_ = number("closed_loop.kp_yaw", 1.5);
    cross_track_gain_ = number("closed_loop.cross_track_gain", 0.6);
    cross_track_max_wz_ = number("closed_loop.cross_track_max_wz", 0.15);
    max_vx_ = number("closed_loop.max_vx", 0.75);
    max_vy_ = number("closed_loop.max_vy", 0.0);
    max_vyaw_ = std::min(number("closed_loop.max_vyaw", 1.0), 1.0);
    finish_dist_ = number("closed_loop.finish_dist", 0.1);
    yaw_tolerance_ = number("closed_loop.yaw_tolerance", 0.26);
    terminal_max_vx_ = number("closed_loop.terminal_max_vx", 0.3);
    terminal_min_vx_ = number("closed_loop.terminal_min_vx", 0.12);
    terminal_max_wz_ = number("closed_loop.terminal_max_wz", 0.45);
    final_yaw_gain_ = number("closed_loop.final_yaw_gain", 0.9);
    final_yaw_min_wz_ = number("closed_loop.final_yaw_min_wz", 0.35);
    odom_timeout_ = number("closed_loop.odom_timeout", 0.5);
    for (double value : {time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_, max_vx_, max_vyaw_,
        finish_dist_, yaw_tolerance_, terminal_max_vx_, terminal_max_wz_, final_yaw_gain_, final_yaw_min_wz_, odom_timeout_})
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid closed_loop parameter");
    if (!std::isfinite(max_vy_) || max_vy_ < 0) throw std::invalid_argument("max_vy must be nonnegative");
    if (!std::isfinite(cross_track_gain_) || cross_track_gain_ < 0 ||
        !std::isfinite(cross_track_max_wz_) || cross_track_max_wz_ < 0)
      throw std::invalid_argument("cross_track parameters must be finite and nonnegative");
    if (!std::isfinite(terminal_min_vx_) || terminal_min_vx_ < 0 ||
        terminal_min_vx_ > std::min(max_vx_, terminal_max_vx_))
      throw std::invalid_argument("terminal_min_vx must be between zero and min(max_vx, terminal_max_vx)");
    cmd_vel_pub_ = node_.create_publisher<geometry_msgs::msg::Twist>(topicName(node_, "cmd_vel", "/cmd_vel_planned"), 20);
    execution_frozen_pub_ = node_.create_publisher<std_msgs::msg::Bool>("planning/go2_execution_frozen", 10);
    nav_state_pub_ = node_.create_publisher<std_msgs::msg::Bool>("/nav_state", 10);
    paused_pub_ = node_.create_publisher<std_msgs::msg::Bool>("/pure_pursuit/paused", rclcpp::QoS(1).reliable().transient_local());
    odom_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
        topicName(node_, "body_pose", "/pose_stamped_localization"), rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          const auto odometry = adaptPose(node_, *message);
          if (!odometry) return;
          if (have_odom_ && rclcpp::Time(message->header.stamp) <= pose_stamp_) return;
          odom_pos_ << odometry->pose.pose.position.x, odometry->pose.pose.position.y, odometry->pose.pose.position.z;
          odom_yaw_ = tf2::getYaw(odometry->pose.pose.orientation);
          pose_stamp_ = rclcpp::Time(message->header.stamp);
          last_odom_time_ = node_.now(); have_odom_ = true;
        });
    task_sub_ = node_.create_subscription<slam_manager_interfaces::msg::TaskTarget>("/navigation/task_target", 10,
        [this](slam_manager_interfaces::msg::TaskTarget::ConstSharedPtr task) {
          if (!std::isfinite(task->target.x) || !std::isfinite(task->target.y) ||
              !std::isfinite(task->target.z) || !std::isfinite(task->target.theta)) return;
          const bool same_task = active_task_ && matchesGoal(task->target.x, task->target.y) &&
              task->use_topology == active_task_->use_topology &&
              (!task->use_topology || task->target.node_id == active_task_->target.node_id) &&
              std::abs(task->target.z - active_task_->target.z) < 1e-3 &&
              std::abs(normalizeAngle(task->target.theta - final_yaw_)) < 1e-3;
          if (same_task && !goal_completed_) {
            RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                "[scan_terminal_preserved] source=duplicate_task seq=%llu captured=%d terminal=%d target_yaw=%.3f",
                static_cast<unsigned long long>(task_sequence_), goal_position_reached_, final_yaw_active_, final_yaw_);
            return;
          }
          start_after_completion_ = goal_completed_;
          RCLCPP_INFO(node_.get_logger(),
              "[scan_terminal_reset] source=task reason=new_task seq=%llu previous_captured=%d previous_completed=%d goal=(%.3f,%.3f) target_yaw=%.3f",
              static_cast<unsigned long long>(++task_sequence_), goal_position_reached_, goal_completed_,
              task->target.x, task->target.y, normalizeAngle(task->target.theta));
          active_task_ = task;
          task_goal_ = Eigen::Vector2d(task->target.x, task->target.y);
          final_yaw_ = normalizeAngle(task->target.theta); have_task_ = true;
          have_final_goal_ = latest_path_stamp_ > 0 && (path_goal_ - task_goal_).norm() < 1e-3;
          receive_traj_ = false;
          goal_position_reached_ = false; goal_completed_ = false; final_yaw_active_ = false;
          last_terminal_turn_ = 0;
          publishState(false); publishStop();
          RCLCPP_INFO(node_.get_logger(), "[scan_task] goal=(%.3f,%.3f) yaw=%.3f matching_path=%d after_completed=%d",
              task_goal_.x(), task_goal_.y(), final_yaw_, have_final_goal_, start_after_completion_);
        });
    pause_service_ = node_.create_service<std_srvs::srv::SetBool>("/pure_pursuit/set_paused",
        [this](std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
          paused_ = req->data;
          std_msgs::msg::Bool message; message.data = paused_; paused_pub_->publish(message);
          if (paused_) publishStop();
          res->success = true; res->message = paused_ ? "navigation paused" : "navigation resumed";
        });
    navi_mode_ = node_.get_parameter("fsm.navi_mode").as_int();
    planner.setController(this);
    cmd_timer_ = node_.create_wall_timer(std::chrono::milliseconds(10), [this]() { cmdCallback(); });
    last_update_time_ = node_.now();
    std_msgs::msg::Bool initial; initial.data = false; paused_pub_->publish(initial); publishState(false);
    RCLCPP_INFO(node_.get_logger(),
        "[scan_controller] tracking=original max_vy=%.3f finish_dist=%.3f cross_track_gain=%.3f cross_track_max_wz=%.3f",
        max_vy_, finish_dist_, cross_track_gain_, cross_track_max_wz_);
  }

  void stop() override {
    std::scoped_lock lock(stop_mutex_);
    stopped_ = true; cmd_timer_->cancel(); publishStop();
  }
  void setTrajectory(const navigation_interfaces::msg::Bspline &message, int64_t path) override {
    if (stopped_ || (navi_mode_ == 3 && (!have_final_goal_ || path != latest_path_stamp_)) || goal_completed_ || final_yaw_active_) return;
    if (message.pos_pts.empty() || message.knots.empty() || message.order <= 0) return;
    Eigen::MatrixXd points(3, message.pos_pts.size());
    for (size_t i = 0; i < message.pos_pts.size(); ++i) points.col(i) << message.pos_pts[i].x, message.pos_pts[i].y, message.pos_pts[i].z;
    Eigen::VectorXd knots(message.knots.size());
    for (size_t i = 0; i < message.knots.size(); ++i) knots(i) = message.knots[i];
    UniformBspline position(points, message.order, 0.1); position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum(); traj_id_ = message.traj_id;
    exec_time_ = 0; last_update_time_ = node_.now(); receive_traj_ = true;
    if (start_after_completion_)
      RCLCPP_INFO(node_.get_logger(),
          "[scan_startup_trajectory] path_stamp=%ld traj=%ld action=first_trajectory_accepted",
          path, traj_id_);
    start_after_completion_ = false;
  }
  void setPath(const nav_msgs::msg::Path::ConstSharedPtr &path) override {
    const auto &goal = path->poses.back().pose.position;
    const bool preserve = matchesGoal(goal.x, goal.y) && (goal_position_reached_ || goal_completed_);
    path_goal_ = Eigen::Vector2d(goal.x, goal.y);
    latest_path_stamp_ = rclcpp::Time(path->header.stamp).nanoseconds();
    have_final_goal_ = matchesGoal(goal.x, goal.y);
    if (preserve) {
      receive_traj_ = false;
      RCLCPP_INFO(node_.get_logger(),
          "[scan_terminal_preserved] source=path seq=%llu path_stamp=%ld captured=%d completed=%d target_yaw=%.3f",
          static_cast<unsigned long long>(task_sequence_), latest_path_stamp_,
          goal_position_reached_, goal_completed_, final_yaw_);
      return;
    }
    RCLCPP_INFO(node_.get_logger(),
        "[scan_terminal_reset] source=path reason=tracking_path seq=%llu path_stamp=%ld matching_task=%d",
        static_cast<unsigned long long>(task_sequence_), latest_path_stamp_, have_final_goal_);
    receive_traj_ = false; goal_position_reached_ = false; goal_completed_ = false; final_yaw_active_ = false;
    last_terminal_turn_ = 0;
    publishState(false); publishStop();
  }
  void setTerminal(bool active) override { final_yaw_active_ = active && have_final_goal_ && !goal_completed_; }
  void setHold(bool hold) override { planner_hold_ = hold; if (hold) publishStop(); }
  bool completed() const override { return goal_completed_; }
  bool captured() const override { return goal_position_reached_; }
  bool paused() const override { return paused_; }
  bool matchesGoal(double x, double y) const override {
    return have_task_ && (Eigen::Vector2d(x, y) - task_goal_).norm() < 1e-3;
  }
  bool startingAfterCompletion() const override { return start_after_completion_; }
  double desiredYaw() const override { return final_yaw_; }
  double executionTime(int64_t trajectory) const override { return receive_traj_ && trajectory == traj_id_ ? exec_time_ : -1; }

 private:
  static double normalizeAngle(double value) { return std::remainder(value, 2 * M_PI); }
  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double maximum) {
    const double norm = value.norm();
    return norm <= maximum || norm < 1e-6 ? value : value / norm * maximum;
  }
  double estimateDesiredYaw(double at, const Eigen::Vector3d &desired) const {
    const double look = std::min(traj_duration_, at + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(look) - desired;
    if (direction.head<2>().squaredNorm() < 1e-4) direction = traj_[1].evaluateDeBoorT(at);
    return direction.head<2>().squaredNorm() < 1e-4 ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }
  void publishStop(double yaw = 0) {
    geometry_msgs::msg::Twist cmd; cmd.angular.z = std::clamp(yaw, -max_vyaw_, max_vyaw_); cmd_vel_pub_->publish(cmd);
  }
  void publishExecutionFrozen(bool value) { std_msgs::msg::Bool msg; msg.data = value; execution_frozen_pub_->publish(msg); }
  void publishState(bool completed) { std_msgs::msg::Bool msg; msg.data = completed; nav_state_pub_->publish(msg); }

  void cmdCallback() {
    std::scoped_lock lock(stop_mutex_);
    if (stopped_) return;
    const auto now = node_.now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;
    if (dt < 0 || dt > 0.2) dt = 0;
    if (goal_completed_) { publishExecutionFrozen(true); publishStop(); publishState(true); return; }
    const double source_age = have_odom_ ? (now - pose_stamp_).seconds() : odom_timeout_ + 1;
    const double receive_age = have_odom_ ? (now - last_odom_time_).seconds() : -1;
    const char *gate = paused_ ? "paused" : planner_hold_ ? "planner_hold" :
        !have_odom_ ? "no_pose" : receive_age > odom_timeout_ ? "pose_receive_timeout" :
        source_age > odom_timeout_ ? "pose_stamp_timeout" : source_age < -0.2 ? "pose_stamp_future" :
        (navi_mode_ == 3 && !have_final_goal_) ? "task_path_unmatched" : "none";
    if (last_gate_ != gate) {
      RCLCPP_INFO(node_.get_logger(),
          "[scan_control_gate] reason=%s previous=%s seq=%llu path_stamp=%ld source_age=%.3f receive_age=%.3f captured=%d terminal=%d",
          gate, last_gate_.c_str(), static_cast<unsigned long long>(task_sequence_), latest_path_stamp_,
          source_age, receive_age, goal_position_reached_, final_yaw_active_);
      last_gate_ = gate;
    }
    if (last_gate_ != "none") {
      publishExecutionFrozen(true); publishStop(); return;
    }
    if (final_yaw_active_) {
      const double distance = (task_goal_ - odom_pos_.head<2>()).norm();
      if (!goal_position_reached_ && distance <= finish_dist_) {
        goal_position_reached_ = true;
        receive_traj_ = false;
        // End the XY phase with an explicit stop. Yaw starts next control cycle.
        publishExecutionFrozen(true); publishStop();
        RCLCPP_INFO(node_.get_logger(),
            "[scan_terminal] phase=xy_reached seq=%llu path_stamp=%ld distance=%.3f tolerance=%.3f yaw=%.3f target_yaw=%.3f action=stop_then_rotate",
            static_cast<unsigned long long>(task_sequence_), latest_path_stamp_, distance, finish_dist_, odom_yaw_, final_yaw_);
        return;
      }
      geometry_msgs::msg::Twist command;
      if (goal_position_reached_) {
        const double error = normalizeAngle(final_yaw_ - odom_yaw_);
        if (std::abs(error) <= yaw_tolerance_) {
          goal_completed_ = true; publishState(true);
          RCLCPP_INFO(node_.get_logger(),
              "[scan_goal_completed] seq=%llu path_stamp=%ld distance=%.3f yaw=%.3f target_yaw=%.3f yaw_error=%.3f",
              static_cast<unsigned long long>(task_sequence_), latest_path_stamp_, distance, odom_yaw_, final_yaw_, error);
        } else command.angular.z = std::copysign(std::min({max_vyaw_, terminal_max_wz_,
            std::max(final_yaw_min_wz_, final_yaw_gain_ * std::abs(error))}), error);
        const int turn = (command.angular.z > 0) - (command.angular.z < 0);
        if (last_terminal_turn_ != 0 && turn != 0 && turn != last_terminal_turn_)
          RCLCPP_WARN(node_.get_logger(),
              "[scan_yaw_direction_change] seq=%llu yaw=%.3f target_yaw=%.3f error=%.3f wz=%.3f",
              static_cast<unsigned long long>(task_sequence_), odom_yaw_, final_yaw_, error, command.angular.z);
        if (turn != 0) last_terminal_turn_ = turn;
      } else {
        const Eigen::Vector2d delta = task_goal_ - odom_pos_.head<2>();
        double error = normalizeAngle(std::atan2(delta.y(), delta.x()) - odom_yaw_);
        double direction = 1;
        if (std::abs(error) > M_PI_2) { direction = -1; error = normalizeAngle(error + M_PI); }
        const double wz = std::min(max_vyaw_, terminal_max_wz_);
        command.angular.z = std::clamp(kp_yaw_ * error, -wz, wz);
        if (std::abs(error) < 0.5) command.linear.x = direction * std::min({max_vx_, terminal_max_vx_,
            std::max(terminal_min_vx_, kp_pos_ * distance)}) * std::cos(error);
      }
      publishExecutionFrozen(true); cmd_vel_pub_->publish(command);
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
          "[scan_terminal] phase=%s seq=%llu path_stamp=%ld distance=%.3f captured=%d completed=%d yaw=%.3f target_yaw=%.3f yaw_error=%.3f vx=%.3f wz=%.3f",
          goal_position_reached_ ? "yaw" : "xy", static_cast<unsigned long long>(task_sequence_), latest_path_stamp_,
          distance, goal_position_reached_, goal_completed_, odom_yaw_, final_yaw_,
          normalizeAngle(final_yaw_ - odom_yaw_), command.linear.x, command.angular.z);
      return;
    }
    if (!receive_traj_) {
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
          "[scan_trajectory_wait] seq=%llu path_stamp=%ld matching_task=%d",
          static_cast<unsigned long long>(task_sequence_), latest_path_stamp_, have_final_goal_);
      publishExecutionFrozen(false); publishStop(); return;
    }

    // Original closed-loop tracking equations and heading freeze.
    const double t_eval = std::min(exec_time_, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command = std::clamp(kp_yaw_ * yaw_error, -max_vyaw_, max_vyaw_);
    if (std::abs(yaw_error) > heading_error_threshold_) {
      RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
          "[scan_heading_align] traj=%ld yaw=%.3f target_yaw=%.3f error=%.3f wz=%.3f",
          traj_id_, odom_yaw_, normalizeAngle(odom_yaw_ + yaw_error), yaw_error, yaw_command);
      publishExecutionFrozen(true); publishStop(yaw_command); return;
    }
    publishExecutionFrozen(false);
    exec_time_ = std::min(traj_duration_, exec_time_ + dt);
    pos_des = traj_[0].evaluateDeBoorT(exec_time_);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(exec_time_);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_world = clampNorm(Eigen::Vector2d(vel_des.x(), vel_des.y()) + kp_pos_ * pos_error, std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_), s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), -max_vx_, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    const double trajectory_speed = vel_des.head<2>().norm();
    if (max_vy_ <= 1e-6 && command.linear.x > 0.05 && trajectory_speed > 0.05 &&
        exec_time_ < traj_duration_) {
      // 正值表示轨迹在左侧；只补偿横向误差，不把前后跟踪滞后变成转向。
      const double cross_track_error =
          (-vel_des.y() * pos_error.x() + vel_des.x() * pos_error.y()) / trajectory_speed;
      const double correction = std::clamp(cross_track_gain_ * cross_track_error,
                                           -cross_track_max_wz_, cross_track_max_wz_);
      command.angular.z = std::clamp(yaw_command + correction, -max_vyaw_, max_vyaw_);
      if (std::abs(correction) > 1e-3)
        RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
            "[scan_lateral_correction] traj=%ld error=%.3f base_wz=%.3f correction_wz=%.3f output_wz=%.3f vx=%.3f",
            traj_id_, cross_track_error, yaw_command, correction, command.angular.z, command.linear.x);
    }
    if (exec_time_ >= traj_duration_ && pos_error.norm() < finish_dist_) command = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(command);
    RCLCPP_INFO_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "[scan_control] tracking=original traj=%ld exec=%.3f/%.3f vx=%.3f vy=%.3f wz=%.3f",
        traj_id_, exec_time_, traj_duration_, command.linear.x, command.linear.y, command.angular.z);
  }

  rclcpp::Node &node_;
  std::mutex stop_mutex_;
  bool stopped_{false}, paused_{false}, planner_hold_{false}, have_odom_{false}, receive_traj_{false};
  bool have_task_{false}, have_final_goal_{false}, final_yaw_active_{false}, goal_completed_{false}, goal_position_reached_{false};
  bool start_after_completion_{false};
  int navi_mode_{3};
  int64_t latest_path_stamp_{0}, traj_id_{0};
  uint64_t task_sequence_{0};
  int last_terminal_turn_{0};
  std::string last_gate_;
  slam_manager_interfaces::msg::TaskTarget::ConstSharedPtr active_task_;
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector2d task_goal_{Eigen::Vector2d::Zero()}, path_goal_{Eigen::Vector2d::Zero()};
  double odom_yaw_{0}, final_yaw_{0}, exec_time_{0}, traj_duration_{0};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_, max_vx_, max_vy_, max_vyaw_, finish_dist_;
  double cross_track_gain_, cross_track_max_wz_;
  double yaw_tolerance_, terminal_max_vx_, terminal_min_vx_, terminal_max_wz_, final_yaw_gain_, final_yaw_min_wz_, odom_timeout_;
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME}, last_odom_time_{0, 0, RCL_ROS_TIME}, pose_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<UniformBspline> traj_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_, nav_state_pub_, paused_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
  rclcpp::Subscription<slam_manager_interfaces::msg::TaskTarget>::SharedPtr task_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_service_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

std::unique_ptr<Controller> createClosedLoopController(rclcpp::Node &node, SCANReplanFSM &planner) {
  return std::make_unique<ClosedLoopController>(node, planner);
}
} // namespace scan_planner
