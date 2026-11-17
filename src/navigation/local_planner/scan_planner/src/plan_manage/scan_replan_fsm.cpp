
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/reference_path_utils.h>
#include <plan_manage/topic_config.h>
#include <plan_manage/pose_adapter.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    replan_retry_interval_ = load_parameter<double>(node_, "fsm.replan_retry_interval", 1.0);
    stop_distance_ = load_parameter<double>(node_, "fsm.stop_distance", 0.30);
    if (!std::isfinite(stop_distance_) || stop_distance_ < 0.0)
      throw std::runtime_error("fsm.stop_distance must be a finite non-negative value");
    retry_not_before_ = node_->now();
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.0);
    terminal_enter_dist_ = load_parameter<double>(node_, "closed_loop.terminal_enter_dist", 0.5);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);
      preset_waypoints_.resize(waypoint_num_);
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        topicName(*node_, "body_pose", "/pose_stamped_localization"), rclcpp::SensorDataQoS(),
        [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          auto pose = adaptPose(*node_, *message);
          if (!pose) return;
          const double stamp = rclcpp::Time(pose->header.stamp).seconds();
          const Eigen::Vector3d position(pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z);
          const double dt = stamp - pose_stamp_;
          if (pose_stamp_ > 0 && dt <= 0) return;
          Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
          if (pose_stamp_ > 0 && dt < 0.5 && (position - previous_pose_).norm() < 0.5)
            velocity = 0.3 * (position - previous_pose_) / dt + 0.7 * odom_vel_;
          if (velocity.norm() > 1.0) velocity.normalize();
          previous_pose_ = position; pose_stamp_ = stamp;
          pose->twist.twist.linear.x = velocity.x(); pose->twist.twist.linear.y = velocity.y(); pose->twist.twist.linear.z = velocity.z();
          odometryCallback(pose);
          if (pending_path_) { auto path = pending_path_; pending_path_.reset(); pathCallback(path); }
        });
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<navigation_interfaces::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<navigation_interfaces::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    path_accepted_pub_ = node_->create_publisher<std_msgs::msg::Header>("/navigation/global_path_accepted", 10);
    replan_sub_ = node_->create_subscription<std_msgs::msg::Header>("/navigation/global_replan_request", 10,
        [this](std_msgs::msg::Header::ConstSharedPtr message) {
          if (message->frame_id != self_inflation_frame_id_) return;
          const auto stamp = rclcpp::Time(message->stamp).nanoseconds();
          if (stamp <= accepted_path_stamp_) {
            std_msgs::msg::Header accepted; accepted.frame_id = self_inflation_frame_id_;
            accepted.stamp = rclcpp::Time(accepted_path_stamp_); path_accepted_pub_->publish(accepted); return;
          }
          minimum_path_stamp_ = std::max(minimum_path_stamp_, stamp);
          pending_path_.reset(); awaiting_path_ = true;
          if (controller_ && (controller_->captured() || controller_->completed())) {
            controller_->setHold(true);
            RCLCPP_INFO(node_->get_logger(),
                "[scan_terminal_preserved] source=replan_request request_stamp=%ld captured=%d completed=%d action=hold_until_matching_path",
                stamp, controller_->captured(), controller_->completed());
            return;
          }
          have_target_ = false; terminal_active_ = false;
          recovery_retry_attempt_ = false;
          if (controller_) { controller_->setTerminal(false); controller_->setHold(true); }
          changeFSMExecState(WAIT_TARGET, "global_replan");
        });

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          topicName(*node_, "goal", "/move_base_simple/goal"), 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<navigation_interfaces::msg::PathWithPolicy>(
          topicName(*node_, "initial_path", "/smoothed_global_path_with_policy"), rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    active_waypoints_ = wps;
    current_wp_ = 0;
    trigger_ = true;
    init_pt_ = odom_pos_;

    if (planNextWaypoint())
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");
    }
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    trigger_ = true;
    init_pt_ = odom_pos_;

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.size() < 2)
    {
      RCLCPP_WARN(node_->get_logger(), "Reference path requires at least two points");
      return false;
    }

    end_pt_ = waypoints.back();
    std::vector<Eigen::Vector3d> reference_waypoints(waypoints.begin() + 1, waypoints.end());

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        waypoints.front(),
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero(),
        reference_waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];
    setStartStateFromOdomOrCurrentTraj();

    bool success = planner_manager_->planGlobalTraj(
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(
      const navigation_interfaces::msg::PathWithPolicy::ConstSharedPtr &msg)
  {
    if (!msg || msg->path.poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }
    const auto path = std::make_shared<nav_msgs::msg::Path>(msg->path);
    const bool has_path_policy = msg->edge_policies.size() == path->poses.size() &&
                                 !msg->edge_policies.empty();
    const bool avoid_obstacles = has_path_policy &&
        std::all_of(msg->edge_policies.begin(), msg->edge_policies.end(),
                    [](const auto &policy) { return policy.obstacle_avoidance_enabled; });
    if (!has_path_policy)
      RCLCPP_WARN(node_->get_logger(),
                  "Path policy is missing or mismatched; using stop-only mode for safety");

    if (!have_odom_)
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No odometry yet; ignoring initial_path");
      pending_path_ = msg;
      return;
    }

    const auto stamp = rclcpp::Time(path->header.stamp).nanoseconds();
    if (path->header.frame_id != self_inflation_frame_id_ || stamp <= 0 ||
        stamp < minimum_path_stamp_ || stamp <= accepted_path_stamp_) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "[scan_path_rejected] frame=%s stamp=%ld minimum=%ld accepted=%ld",
          path->header.frame_id.c_str(), stamp, minimum_path_stamp_, accepted_path_stamp_);
      return;
    }
    std::vector<Eigen::Vector3d> waypoints;
    std::string path_error;
    if (!prepareReferenceWaypoints(*path, body_height_, 0.5, waypoints, &path_error))
    {
      RCLCPP_WARN(node_->get_logger(), "Ignoring initial_path: %s", path_error.c_str());
      return;
    }
    if (controller_ && !controller_->matchesGoal(waypoints.back().x(), waypoints.back().y())) {
      pending_path_ = msg; awaiting_path_ = true;
      controller_->setHold(true);
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
          "[scan_path_waiting_task] path_stamp=%ld goal=(%.3f,%.3f) action=hold_until_matching_task",
          stamp, waypoints.back().x(), waypoints.back().y());
      return;
    }
    obstacle_avoidance_enabled_ = avoid_obstacles;
    stop_obstacle_hold_ = false;
    planner_manager_->setObstacleAvoidanceEnabled(obstacle_avoidance_enabled_);
    planner_manager_->setStopDistance(stop_distance_);
    RCLCPP_INFO(node_->get_logger(),
                "[scan_obstacle_policy] path_stamp=%ld mode=%s stop_distance=%.2f",
                stamp, obstacle_avoidance_enabled_ ? "avoid" : "stop", stop_distance_);
    pending_path_.reset();
    if (controller_ && (controller_->captured() || controller_->completed())) {
      end_pt_ = waypoints.back(); reference_points_ = waypoints; reference_index_ = 0;
      accepted_path_stamp_ = stamp; awaiting_path_ = false;
      controller_->setPath(path);
      terminal_active_ = !controller_->completed(); have_target_ = terminal_active_;
      controller_->setTerminal(terminal_active_);
      controller_->setHold(terminal_active_ && !terminalSafe());
      path_accepted_pub_->publish(path->header);
      RCLCPP_INFO(node_->get_logger(),
          "[scan_terminal_path] path_stamp=%ld captured=%d completed=%d action=ack_without_restarting_tracking",
          stamp, controller_->captured(), controller_->completed());
      if (controller_->completed()) changeFSMExecState(WAIT_TARGET, "completed_path_update");
      return;
    }

    trigger_ = true;
    end_pt_ = waypoints.back();
    reference_points_ = waypoints; reference_index_ = 0;
    terminal_active_ = false; awaiting_path_ = true;
    if (controller_) { controller_->setTerminal(false); controller_->setHold(true); }
    // A platform path is a new execution context. Do not seed it from a curve
    // frozen during the previous task's terminal yaw adjustment.
    planner_manager_->local_data_.start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    planner_manager_->local_data_.duration_ = 0.0;
    bool success = planGlobalTrajByWaypoints(waypoints);

    if (success)
    {
      accepted_path_stamp_ = stamp;
      awaiting_path_ = false;
      if (controller_) { controller_->setPath(path); controller_->setHold(false); }
      path_accepted_pub_->publish(path->header);
      // A new reference path always replaces the previous local trajectory state.
      replan_fail_count_ = 0;
      recovery_retry_attempt_ = false;
      retry_not_before_ = node_->now();
      changeFSMExecState(GEN_NEW_TRAJ, "path_accepted");
      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");
    }
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)
    {
      preset_started_ = true;
      planGlobalTrajbyGivenWps();
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    if (controller_ && planner_manager_->local_data_.traj_id_ > 0) {
      const double executed = controller_->executionTime(planner_manager_->local_data_.traj_id_);
      if (executed >= 0) {
        planner_manager_->local_data_.start_time_ = node_->now() - rclcpp::Duration::from_seconds(executed);
        last_freeze_update_time_ = node_->now(); return;
      }
    }
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    RCLCPP_INFO(node_->get_logger(), "[scan_fsm] source=%s from=%s to=%s",
        pos_call.c_str(), state_str[pre_s].c_str(), state_str[int(new_state)].c_str());
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[fsm]: state: " + state_str[int(exec_state_)] << endl;
  }

  bool SCANReplanFSM::terminalSafe()
  {
    if (!have_odom_ || reference_points_.empty()) return false;
    auto map = planner_manager_->grid_map_;
    Eigen::Vector3d goal = reference_points_.back(); goal.z() = odom_pos_.z();
    if (controller_->captured()) goal = odom_pos_;
    const Eigen::Vector3d delta = goal - odom_pos_;
    const double yaw = getOdomYaw();
    const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() / 0.025)));
    for (int i = 0; i <= samples; ++i)
      if (map->getInflateOccupancy(odom_pos_ + delta * (double(i) / samples), yaw) != 0) return false;
    double desired = controller_->captured() ? controller_->desiredYaw() : std::atan2(delta.y(), delta.x());
    double error = std::remainder(desired - yaw, 2 * M_PI);
    if (!controller_->captured() && std::abs(error) > M_PI_2) error = std::remainder(error + M_PI, 2 * M_PI);
    const double sweep = std::clamp(error, -0.2, 0.2);
    for (int i = 0; i <= 4; ++i)
      if (map->getInflateOccupancy(odom_pos_, yaw + sweep * i / 4) != 0) return false;
    return true;
  }

  bool SCANReplanFSM::terminalReady()
  {
    if (reference_points_.empty()) return false;
    double best = std::numeric_limits<double>::infinity();
    size_t selected = reference_index_;
    Eigen::Vector3d projection = reference_points_[reference_index_];
    for (size_t i = reference_index_; i + 1 < reference_points_.size() && i <= reference_index_ + 4; ++i) {
      const Eigen::Vector2d delta = (reference_points_[i + 1] - reference_points_[i]).head<2>();
      const double fraction = delta.squaredNorm() < 1e-12 ? 0 :
          std::clamp((odom_pos_ - reference_points_[i]).head<2>().dot(delta) / delta.squaredNorm(), 0.0, 1.0);
      Eigen::Vector3d candidate = reference_points_[i];
      candidate.head<2>() += fraction * delta;
      const double distance = (candidate - odom_pos_).head<2>().squaredNorm();
      if (distance < best) { best = distance; selected = i; projection = candidate; }
    }
    reference_index_ = selected;
    double remaining = (reference_points_[std::min(selected + 1, reference_points_.size() - 1)] - projection).head<2>().norm();
    for (size_t i = selected + 1; i + 1 < reference_points_.size(); ++i)
      remaining += (reference_points_[i + 1] - reference_points_[i]).head<2>().norm();
    return (reference_points_.back() - odom_pos_).head<2>().norm() <= terminal_enter_dist_ &&
        remaining <= terminal_enter_dist_ * 1.5 && terminalSafe();
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();
    if (awaiting_path_) {
      if (controller_) controller_->setHold(true);
      return;
    }
    if (controller_ && navi_mode_ == NAVI_MODE::REFERENCE_PATH && have_target_) {
      if (controller_->completed()) {
        have_target_ = false; terminal_active_ = false;
        changeFSMExecState(WAIT_TARGET, "controller_completed"); return;
      }
      if (controller_->paused()) return;
      if (terminal_active_ || terminalReady()) {
        if (!terminal_active_)
          RCLCPP_INFO(node_->get_logger(),
              "[scan_terminal_enter] path_stamp=%ld distance=%.3f captured=%d target_yaw=%.3f",
              accepted_path_stamp_, (reference_points_.back() - odom_pos_).head<2>().norm(),
              controller_->captured(), controller_->desiredYaw());
        terminal_active_ = true;
        controller_->setTerminal(true);
        controller_->setHold(!terminalSafe());
        return;
      }
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      if (node_->now() < retry_not_before_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        recovery_retry_attempt_ = false;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        if (!obstacle_avoidance_enabled_ || recovery_retry_attempt_)
        {
          replan_fail_count_ = max_replan_fail_count_;
          recovery_retry_attempt_ = false;
        }
        else
          replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        if (obstacle_avoidance_enabled_)
          replan_fail_count_++;
        else
          replan_fail_count_ = max_replan_fail_count_;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      if (isWaypointSequenceMode() &&
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;
        if (planNextWaypoint())
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if (controller_ && navi_mode_ == NAVI_MODE::REFERENCE_PATH) {
          // Platform task completion requires actual position and final yaw.
          changeFSMExecState(GEN_NEW_TRAJ, "trajectory_finished"); return;
        }
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())
        {
          current_wp_++;
          if (planNextWaypoint())
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            return;
          }
          replan_fail_count_++;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          return;
        }

        if (isWaypointSequenceMode())
        {
          active_waypoints_.clear();
          current_wp_ = 0;
        }

        have_target_ = false;

        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          need_hover_stop_ = false;
          if (have_target_)
          {
            recovery_retry_attempt_ = true;
            retry_not_before_ = node_->now() + rclcpp::Duration::from_seconds(replan_retry_interval_);
            RCLCPP_WARN(node_->get_logger(),
                        "Emergency stop complete; retaining target and retrying planning in %.1f s",
                        replan_retry_interval_);
          }
          else
          {
            trigger_ = false;
            RCLCPP_INFO(node_->get_logger(),
                        "Exiting EMERGENCY_STOP; switching to WAIT_TARGET without an active target");
          }
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and retry the active target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      recovery_retry_attempt_ = false;
      flag_escape_emergency_ = true;
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    rclcpp::Time time_now = node_->now();
    double t_cur = (time_now - info->start_time_).seconds();
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
    {
      start_pt_ = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      bool success = callReboundReplan(false, false);
      if (!success)
      {
        success = callReboundReplan(true, false);
        if (!success)
        {
          success = callReboundReplan(true, true);
          if (!success)
            return false;
        }
      }

      return true;
    }

    start_pt_ = odom_pos_;
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    if (!planner_manager_->planGlobalTraj(
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    if (controller_ && controller_->startingAfterCompletion())
    {
      const Eigen::Vector3d raw_velocity = start_vel_;
      Eigen::Vector2d route_direction = Eigen::Vector2d::Zero();
      for (size_t i = 1; i < reference_points_.size(); ++i) {
        route_direction = (reference_points_[i] - reference_points_.front()).head<2>();
        if (route_direction.norm() > 1e-3) { route_direction.normalize(); break; }
      }
      const double speed = raw_velocity.norm();
      const double projection = raw_velocity.head<2>().dot(route_direction);
      constexpr double startup_speed_limit = 0.1;
      const bool reverse_residual = speed <= startup_speed_limit && projection < -1e-3;
      if (reverse_residual) {
        start_vel_.setZero();
        // Clear the rotation residual in the velocity filter, retaining the
        // timestamp so out-of-order localization messages remain rejected.
        odom_vel_.setZero(); previous_pose_ = odom_pos_;
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "[scan_start_state] source=odometry after_completed=1 path_stamp=%ld raw=(%.3f,%.3f,%.3f) used=(%.3f,%.3f,%.3f) speed=%.3f limit=%.3f route=(%.3f,%.3f) projection=%.3f action=%s",
          accepted_path_stamp_, raw_velocity.x(), raw_velocity.y(), raw_velocity.z(),
          start_vel_.x(), start_vel_.y(), start_vel_.z(), speed, startup_speed_limit,
          route_direction.x(), route_direction.y(), projection,
          reverse_residual ? "zero_reverse_residual" : speed > startup_speed_limit ? "keep_moving" : "keep_route_velocity");
      return;
    }

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();
    if (awaiting_path_) {
      if (controller_) controller_->setHold(true);
      return;
    }
    if (terminal_active_ && controller_) {
      controller_->setHold(!terminalSafe()); return;
    }
    if (controller_ && controller_->paused()) return;

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    if (!obstacle_avoidance_enabled_)
    {
      if (!have_odom_)
        return;

      const bool blocked = map->getStopOccupancy(odom_pos_, getOdomYaw(), stop_distance_) != 0;
      if (blocked)
      {
        if (!stop_obstacle_hold_)
          RCLCPP_WARN(node_->get_logger(),
                      "[scan_stop] obstacle within %.2f m front clearance; holding current command",
                      stop_distance_);
        stop_obstacle_hold_ = true;
        if (controller_) controller_->setHold(true);
        return;
      }

      if (stop_obstacle_hold_)
      {
        stop_obstacle_hold_ = false;
        if (controller_) controller_->setHold(false);
        RCLCPP_INFO(node_->get_logger(),
                    "[scan_stop] front clearance restored; resuming the active trajectory");
      }
      return;
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      const double yaw = estimateYawFromSegment(pos, pos_next);
      const int occupancy = map->getInflateOccupancy(pos, yaw);
      if (occupancy != 0)
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      navigation_interfaces::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);
      if (controller_) controller_->setTrajectory(bspline, accepted_path_stamp_);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    navigation_interfaces::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);
    if (controller_) controller_->setTrajectory(bspline, accepted_path_stamp_);

    return true;
  }

  void SCANReplanFSM::getLocalTarget()
  {
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double max_acc = planner_manager_->pp_.max_acc_;
    const double duration = planner_manager_->global_data_.global_duration_;
    double t_step = max_vel > 1e-6 ? planning_horizon_ / 20.0 / max_vel : 0.01;
    t_step = std::max(t_step, 0.01);

    double t_proj = 0.0;
    double min_dist_to_start = 9999.0;
    for (double t = 0.0; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist_to_start = (pos_t - start_pt_).norm();
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        t_proj = t;
      }
    }

    double target_t = duration;
    double total_dist = 0.0;
    bool target_found = false;
    Eigen::Vector3d prev_pos = planner_manager_->global_data_.getPosition(t_proj);
    local_target_pt_ = end_pt_;

    for (double t = t_proj; t < duration; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      total_dist += (pos_t - prev_pos).norm();
      if (total_dist >= planning_horizon_)
      {
        local_target_pt_ = pos_t;
        target_t = t;
        target_found = true;
        break;
      }
      prev_pos = pos_t;
    }
    planner_manager_->global_data_.last_progress_time_ = target_found ? target_t : duration;

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (obstacle_avoidance_enabled_ && targetOccupancy(local_target_pt_) != 0)
    {
      bool found_free_target = false;
      double adjusted_t = target_t;

      for (double dt = 0.0; dt <= duration; dt += t_step)
      {
        double t_forward = target_t + dt;
        if (t_forward <= duration)
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);
          if (targetOccupancy(pt) == 0 && (pt - odom_pos_).norm() >= 0.2)
          {
            local_target_pt_ = pt;
            adjusted_t = t_forward;
            found_free_target = true;
            break;
          }
        }

        double t_backward = target_t - dt;
        if (t_backward >= std::max(0.0, t_proj))
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);
          if (targetOccupancy(pt) == 0 && (pt - odom_pos_).norm() >= 0.2)
          {
            local_target_pt_ = pt;
            adjusted_t = t_backward;
            found_free_target = true;
            break;
          }
        }
      }

      if (found_free_target)
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "Local target is in collision and no nearby free target was found");
      }
    }

    const double braking_distance = max_acc > 1e-6
        ? (max_vel * max_vel) / (2.0 * max_acc)
        : std::numeric_limits<double>::infinity();
    if ((end_pt_ - local_target_pt_).norm() < braking_distance)
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);
      if (local_target_vel_.norm() > max_vel)
        local_target_vel_ = local_target_vel_.normalized() * max_vel;
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
