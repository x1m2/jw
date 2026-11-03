#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <navigation_interfaces/msg/bspline.hpp>

namespace scan_planner
{
class SCANReplanFSM;

// Controllers share the planner node; they do not own a ROS node or executor.
class Controller
{
public:
  virtual ~Controller() = default;
  virtual void stop() = 0;
  virtual void setTrajectory(const navigation_interfaces::msg::Bspline &, int64_t) {}
  virtual void setPath(const nav_msgs::msg::Path::ConstSharedPtr &) {}
  virtual void setTerminal(bool) {}
  virtual void setHold(bool) {}
  virtual bool completed() const { return false; }
  virtual bool captured() const { return false; }
  virtual bool paused() const { return false; }
  virtual bool matchesGoal(double, double) const { return false; }
  virtual bool startingAfterCompletion() const { return false; }
  virtual double desiredYaw() const { return 0; }
  virtual double executionTime(int64_t) const { return -1; }
};

std::unique_ptr<Controller> createClosedLoopController(rclcpp::Node &node, SCANReplanFSM &planner);

}  // namespace scan_planner
