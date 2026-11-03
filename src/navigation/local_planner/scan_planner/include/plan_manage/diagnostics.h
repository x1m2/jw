#pragma once

#include <algorithm>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace scan_planner {
using DiagnosticClock = std::chrono::steady_clock;

// Diagnostics only: never use these timestamps to advance/control the robot.
struct PoseDiagnostics {
  DiagnosticClock::time_point received{}, reported{};
  double stamp{0.0};
  bool valid{false};
  double age() const {
    return valid ? std::chrono::duration<double>(DiagnosticClock::now() - received).count() : -1.0;
  }
  void record(rclcpp::Node &node, const char *owner,
              const geometry_msgs::msg::PoseStamped &msg) {
    const auto now = DiagnosticClock::now();
    const double gap = valid ? std::chrono::duration<double>(now - received).count() : -1.0;
    const double next_stamp = rclcpp::Time(msg.header.stamp).seconds();
    const double stamp_step = valid ? next_stamp - stamp : 0.0;
    stamp = next_stamp;
    received = now;
    valid = true;
    if (std::chrono::duration<double>(now - reported).count() < 1.0) return;
    reported = now;
    RCLCPP_INFO(node.get_logger(),
        "[scan_pose] owner=%s stamp=%.9f ros_receive=%.9f stamp_age=%.3f receive_gap=%.3f stamp_step=%.3f frame=%s xyz=(%.3f,%.3f,%.3f)",
        owner, stamp, node.now().seconds(), node.now().seconds() - stamp, gap, stamp_step,
        msg.header.frame_id.c_str(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  }
};

struct CallbackDiagnostics {
  DiagnosticClock::time_point previous{}, reported{};
  double max_ms{0.0}, max_gap_ms{0.0};
  unsigned count{0};
};

class CallbackScope {
 public:
  CallbackScope(rclcpp::Node &node, const char *name, CallbackDiagnostics &stats)
      : node_(node), name_(name), stats_(stats), start_(DiagnosticClock::now()) {
    if (stats_.previous.time_since_epoch().count() != 0)
      stats_.max_gap_ms = std::max(stats_.max_gap_ms,
          std::chrono::duration<double, std::milli>(start_ - stats_.previous).count());
    stats_.previous = start_;
  }
  ~CallbackScope() {
    const auto now = DiagnosticClock::now();
    stats_.max_ms = std::max(stats_.max_ms,
        std::chrono::duration<double, std::milli>(now - start_).count());
    ++stats_.count;
    if (std::chrono::duration<double>(now - stats_.reported).count() < 2.0) return;
    RCLCPP_INFO(node_.get_logger(), "[scan_callback] name=%s calls=%u max_ms=%.2f max_start_gap_ms=%.2f",
        name_, stats_.count, stats_.max_ms, stats_.max_gap_ms);
    stats_.reported = now;
    stats_.count = 0;
    stats_.max_ms = stats_.max_gap_ms = 0.0;
  }
 private:
  rclcpp::Node &node_;
  const char *name_;
  CallbackDiagnostics &stats_;
  DiagnosticClock::time_point start_;
};
}  // namespace scan_planner
