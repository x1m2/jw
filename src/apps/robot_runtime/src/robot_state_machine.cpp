#include "robot_runtime/robot_state_machine.hpp"

namespace robot_runtime {

RobotMode RobotStateMachine::mode() const {
  std::scoped_lock lock(mutex_);
  return mode_;
}

bool RobotStateMachine::initialize() {
  return transition(RobotMode::kBoot, RobotMode::kIdle);
}

bool RobotStateMachine::beginMapping() {
  return transition(RobotMode::kIdle, RobotMode::kStartingMapping);
}

bool RobotStateMachine::mappingStarted() {
  return transition(RobotMode::kStartingMapping, RobotMode::kMapping);
}

bool RobotStateMachine::beginStopMapping() {
  return transition(RobotMode::kMapping, RobotMode::kStoppingMapping);
}

bool RobotStateMachine::mappingStopped() {
  return transition(RobotMode::kStoppingMapping, RobotMode::kIdle);
}

bool RobotStateMachine::beginLocalization() {
  return transition(RobotMode::kIdle, RobotMode::kStartingLocalization);
}

bool RobotStateMachine::localizationStarted() {
  const RobotMode current = mode();
  // 定位成功信号可能早于 Worker 启动回调到达，因此允许幂等确认。
  if (current == RobotMode::kLocalizing || current == RobotMode::kReady) {
    return true;
  }
  return transition(RobotMode::kStartingLocalization,
                    RobotMode::kLocalizing);
}

bool RobotStateMachine::localizationReady() {
  // 重复上报 ready 视为成功，便于上层进行幂等调用。
  const RobotMode current = mode();
  if (current == RobotMode::kReady) return true;
  return transition(RobotMode::kStartingLocalization, RobotMode::kReady) ||
         transition(RobotMode::kLocalizing, RobotMode::kReady);
}

bool RobotStateMachine::localizationFailed() {
  std::scoped_lock lock(mutex_);
  if (mode_ != RobotMode::kStartingLocalization &&
      mode_ != RobotMode::kLocalizing) {
    return false;
  }
  mode_ = RobotMode::kIdle;
  return true;
}

bool RobotStateMachine::beginStopLocalization() {
  std::scoped_lock lock(mutex_);
  if (mode_ != RobotMode::kStartingLocalization &&
      mode_ != RobotMode::kLocalizing && mode_ != RobotMode::kReady) {
    return false;
  }
  mode_ = RobotMode::kStoppingLocalization;
  return true;
}

bool RobotStateMachine::localizationStopped() {
  return transition(RobotMode::kStoppingLocalization, RobotMode::kIdle);
}

bool RobotStateMachine::beginNavigation() {
  return transition(RobotMode::kReady, RobotMode::kStartingNavigation);
}

bool RobotStateMachine::navigationStarted() {
  return transition(RobotMode::kStartingNavigation, RobotMode::kNavigating);
}

bool RobotStateMachine::pauseNavigation() {
  return transition(RobotMode::kNavigating, RobotMode::kNavigationPaused);
}

bool RobotStateMachine::resumeNavigation() {
  return transition(RobotMode::kNavigationPaused, RobotMode::kNavigating);
}

bool RobotStateMachine::beginStopNavigation() {
  std::scoped_lock lock(mutex_);
  if (mode_ != RobotMode::kNavigating &&
      mode_ != RobotMode::kNavigationPaused) {
    return false;
  }
  mode_ = RobotMode::kStoppingNavigation;
  return true;
}

bool RobotStateMachine::navigationStopped() {
  return transition(RobotMode::kStoppingNavigation, RobotMode::kReady);
}

void RobotStateMachine::enterError() {
  // 任一模块发生严重错误时可从当前状态直接进入错误态。
  std::scoped_lock lock(mutex_);
  mode_ = RobotMode::kError;
}

bool RobotStateMachine::resetError() {
  return transition(RobotMode::kError, RobotMode::kIdle);
}

bool RobotStateMachine::transition(RobotMode expected, RobotMode target) {
  // 检查和赋值在同一把锁内完成，避免并发服务产生竞争。
  std::scoped_lock lock(mutex_);
  if (mode_ != expected) return false;
  mode_ = target;
  return true;
}

}  // namespace robot_runtime
