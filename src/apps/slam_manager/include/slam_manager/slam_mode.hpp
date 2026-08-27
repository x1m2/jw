#pragma once

#include <cstdint>
#include <mutex>
#include <string_view>

namespace slam_manager {

// 机器人运行模式，包含建图和导航过程中的过渡状态。
enum class SlamMode : uint8_t {
  kBoot = 0,
  kIdle,
  kLoadingMap,
  kStartingMapping,
  kMapping,
  kStoppingMapping,
  kStartingLocalization,
  kLocalizing,
  kStartingNavigation,
  kNavigating,
  kNavigationPaused,
  kStoppingNavigation,
  kError,
};

// 转换成稳定的字符串，供状态话题和日志使用。
inline std::string_view toString(SlamMode mode) {
  switch (mode) {
    case SlamMode::kBoot:
      return "boot";
    case SlamMode::kIdle:
      return "idle";
    case SlamMode::kLoadingMap:
      return "loading_map";
    case SlamMode::kStartingMapping:
      return "starting_mapping";
    case SlamMode::kMapping:
      return "mapping";
    case SlamMode::kStoppingMapping:
      return "stopping_mapping";
    case SlamMode::kStartingLocalization:
      return "starting_localization";
    case SlamMode::kLocalizing:
      return "localizing";
    case SlamMode::kStartingNavigation:
      return "starting_navigation";
    case SlamMode::kNavigating:
      return "navigating";
    case SlamMode::kNavigationPaused:
      return "navigation_paused";
    case SlamMode::kStoppingNavigation:
      return "stopping_navigation";
    case SlamMode::kError:
      return "error";
  }
  return "unknown";
}

// 线程安全的机器人模式状态机，只允许预定义的合法状态流转。
class SlamStateMachine {
 public:
  SlamMode mode() const;

  bool initialize();
  bool beginLoadMap();
  bool loadMapFinished();
  bool beginMapping();
  bool mappingStarted();
  bool beginStopMapping();
  bool mappingStopped();
  bool beginLocalization();
  bool localizationStarted();
  bool localizationSucceeded();
  bool localizationFailed();
  bool beginNavigation();
  bool navigationStarted();
  bool pauseNavigation();
  bool resumeNavigation();
  bool beginStopNavigation();
  bool navigationStopped();
  void resetToIdleAfterTeardown();
  void enterError();

 private:
  // 仅当当前状态等于 expected 时才切换到 target。
  bool transition(SlamMode expected, SlamMode target);

  mutable std::mutex mutex_;
  SlamMode mode_{SlamMode::kBoot};
};

inline SlamMode SlamStateMachine::mode() const {
  std::scoped_lock lock(mutex_);
  return mode_;
}

inline bool SlamStateMachine::initialize() {
  return transition(SlamMode::kBoot, SlamMode::kIdle);
}

inline bool SlamStateMachine::beginLoadMap() {
  return transition(SlamMode::kIdle, SlamMode::kLoadingMap);
}

inline bool SlamStateMachine::loadMapFinished() {
  return transition(SlamMode::kLoadingMap, SlamMode::kIdle);
}

inline bool SlamStateMachine::beginMapping() {
  return transition(SlamMode::kIdle, SlamMode::kStartingMapping);
}

inline bool SlamStateMachine::mappingStarted() {
  return transition(SlamMode::kStartingMapping, SlamMode::kMapping);
}

inline bool SlamStateMachine::beginStopMapping() {
  return transition(SlamMode::kMapping, SlamMode::kStoppingMapping);
}

inline bool SlamStateMachine::mappingStopped() {
  return transition(SlamMode::kStoppingMapping, SlamMode::kIdle);
}

inline bool SlamStateMachine::beginLocalization() {
  return transition(SlamMode::kIdle, SlamMode::kStartingLocalization);
}

inline bool SlamStateMachine::localizationStarted() {
  const SlamMode current = mode();
  // 定位成功信号可能早于 Worker 启动回调到达，因此允许幂等确认。
  if (current == SlamMode::kLocalizing || current == SlamMode::kIdle) {
    return true;
  }
  return transition(SlamMode::kStartingLocalization, SlamMode::kLocalizing);
}

inline bool SlamStateMachine::localizationSucceeded() {
  // 重定位成功后回到空闲态，导航资格由独立成功标志判断。
  const SlamMode current = mode();
  if (current == SlamMode::kIdle) return true;
  return transition(SlamMode::kStartingLocalization, SlamMode::kIdle) ||
         transition(SlamMode::kLocalizing, SlamMode::kIdle);
}

inline bool SlamStateMachine::localizationFailed() {
  std::scoped_lock lock(mutex_);
  if (mode_ != SlamMode::kStartingLocalization &&
      mode_ != SlamMode::kLocalizing) {
    return false;
  }
  mode_ = SlamMode::kIdle;
  return true;
}

inline bool SlamStateMachine::beginNavigation() {
  return transition(SlamMode::kIdle, SlamMode::kStartingNavigation);
}

inline bool SlamStateMachine::navigationStarted() {
  return transition(SlamMode::kStartingNavigation, SlamMode::kNavigating);
}

inline bool SlamStateMachine::pauseNavigation() {
  return transition(SlamMode::kNavigating, SlamMode::kNavigationPaused);
}

inline bool SlamStateMachine::resumeNavigation() {
  return transition(SlamMode::kNavigationPaused, SlamMode::kNavigating);
}

inline bool SlamStateMachine::beginStopNavigation() {
  std::scoped_lock lock(mutex_);
  if (mode_ != SlamMode::kNavigating && mode_ != SlamMode::kNavigationPaused) {
    return false;
  }
  mode_ = SlamMode::kStoppingNavigation;
  return true;
}

inline bool SlamStateMachine::navigationStopped() {
  return transition(SlamMode::kStoppingNavigation, SlamMode::kIdle);
}

inline void SlamStateMachine::resetToIdleAfterTeardown() {
  // 只能在所有ManagedWorker均已停止并完成算法析构后调用。
  std::scoped_lock lock(mutex_);
  mode_ = SlamMode::kIdle;
}

inline void SlamStateMachine::enterError() {
  // 任一模块发生严重错误时可从当前状态直接进入错误态。
  std::scoped_lock lock(mutex_);
  mode_ = SlamMode::kError;
}

inline bool SlamStateMachine::transition(SlamMode expected, SlamMode target) {
  // 检查和赋值在同一把锁内完成，避免并发服务产生竞争。
  std::scoped_lock lock(mutex_);
  if (mode_ != expected) return false;
  mode_ = target;
  return true;
}

}  // namespace slam_manager
