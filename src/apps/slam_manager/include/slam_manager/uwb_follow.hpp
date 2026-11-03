#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "slam_manager/worker_runtime.hpp"

namespace slam_manager {

// Follow has its own Worker chain. It deliberately never starts PGO,
// map localization, or the normal global_path_planner.
class UwbFollowController {
 public:
  UwbFollowController(rclcpp::Node *node,
                      rclcpp::CallbackGroup::SharedPtr callback_group);

  bool startFastLio(uint64_t session_id,
                    const std::shared_ptr<WorkerOperationToken> &token,
                    double timeout_sec, std::string &message);
  bool start(uint64_t session_id,
             const std::shared_ptr<WorkerOperationToken> &token,
             double timeout_sec, std::string &message);
  bool stop(double timeout_sec, std::string &message);
  bool active() const;
  bool plannerAttempted() const { return wasAttempted(&uwb_follow_planner_worker_); }

 private:
  struct PendingStart;
  bool startThrough(uint64_t session_id,
                    const std::shared_ptr<WorkerOperationToken> &token,
                    double timeout_sec, std::size_t worker_count,
                    std::string &message);
  bool wasAttempted(const WorkerClient *worker) const;
  WorkerClient fast_lio_worker_;
  WorkerClient obstacle_stop_worker_;
  WorkerClient path_smoother_worker_;
  WorkerClient pure_pursuit_worker_;
  WorkerClient uwb_follow_planner_worker_;
  std::array<WorkerClient *, 5> start_order_;
  std::array<WorkerClient *, 5> stop_order_;
  std::array<bool, 5> attempted_{};
  std::array<bool, 5> started_{};
  std::array<std::shared_ptr<PendingStart>, 5> pending_starts_{};
  uint64_t session_id_{0U};
};

}  // namespace slam_manager
