#include "slam_manager/slam_state_machine.hpp"

namespace slam_manager {

SlamMode SlamStateMachine::mode() const {
  std::scoped_lock lock(mutex_);
  return mode_;
}

bool SlamStateMachine::initialize() {
  return transition(SlamMode::kBoot, SlamMode::kIdle);
}

bool SlamStateMachine::beginLoadMap() {
  return transition(SlamMode::kIdle, SlamMode::kLoadingMap);
}

bool SlamStateMachine::loadMapFinished() {
  return transition(SlamMode::kLoadingMap, SlamMode::kIdle);
}

bool SlamStateMachine::beginMapping() {
  return transition(SlamMode::kIdle, SlamMode::kStartingMapping);
}

bool SlamStateMachine::mappingStarted() {
  return transition(SlamMode::kStartingMapping, SlamMode::kMapping);
}

bool SlamStateMachine::beginStopMapping() {
  return transition(SlamMode::kMapping, SlamMode::kStoppingMapping);
}

bool SlamStateMachine::mappingStopped() {
  return transition(SlamMode::kStoppingMapping, SlamMode::kIdle);
}

bool SlamStateMachine::beginLocalization() {
  return transition(SlamMode::kIdle, SlamMode::kStartingLocalization);
}

bool SlamStateMachine::localizationStarted() {
  const SlamMode current = mode();
  // 定位成功信号可能早于 Worker 启动回调到达，因此允许幂等确认。
  if (current == SlamMode::kLocalizing || current == SlamMode::kReady) {
    return true;
  }
  return transition(SlamMode::kStartingLocalization,
                    SlamMode::kLocalizing);
}

bool SlamStateMachine::localizationReady() {
  // 重复上报 ready 视为成功，便于上层进行幂等调用。
  const SlamMode current = mode();
  if (current == SlamMode::kReady) return true;
  return transition(SlamMode::kStartingLocalization, SlamMode::kReady) ||
         transition(SlamMode::kLocalizing, SlamMode::kReady);
}

bool SlamStateMachine::localizationFailed() {
  std::scoped_lock lock(mutex_);
  if (mode_ != SlamMode::kStartingLocalization &&
      mode_ != SlamMode::kLocalizing) {
    return false;
  }
  mode_ = SlamMode::kIdle;
  return true;
}

bool SlamStateMachine::beginNavigation() {
  return transition(SlamMode::kReady, SlamMode::kStartingNavigation);
}

bool SlamStateMachine::navigationStarted() {
  return transition(SlamMode::kStartingNavigation, SlamMode::kNavigating);
}

bool SlamStateMachine::pauseNavigation() {
  return transition(SlamMode::kNavigating, SlamMode::kNavigationPaused);
}

bool SlamStateMachine::resumeNavigation() {
  return transition(SlamMode::kNavigationPaused, SlamMode::kNavigating);
}

bool SlamStateMachine::beginStopNavigation() {
  std::scoped_lock lock(mutex_);
  if (mode_ != SlamMode::kNavigating &&
      mode_ != SlamMode::kNavigationPaused) {
    return false;
  }
  mode_ = SlamMode::kStoppingNavigation;
  return true;
}

bool SlamStateMachine::navigationStopped() {
  return transition(SlamMode::kStoppingNavigation, SlamMode::kReady);
}

void SlamStateMachine::enterError() {
  // 任一模块发生严重错误时可从当前状态直接进入错误态。
  std::scoped_lock lock(mutex_);
  mode_ = SlamMode::kError;
}

bool SlamStateMachine::transition(SlamMode expected, SlamMode target) {
  // 检查和赋值在同一把锁内完成，避免并发服务产生竞争。
  std::scoped_lock lock(mutex_);
  if (mode_ != expected) return false;
  mode_ = target;
  return true;
}

}  // namespace slam_manager
