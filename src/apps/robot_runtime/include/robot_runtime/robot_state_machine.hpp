#pragma once

#include <mutex>

#include "robot_runtime/robot_mode.hpp"

namespace robot_runtime {

// 线程安全的机器人模式状态机，只允许预定义的合法状态流转。
class RobotStateMachine {
 public:
  RobotMode mode() const;

  bool initialize();
  bool beginMapping();
  bool mappingStarted();
  bool beginStopMapping();
  bool mappingStopped();
  bool beginLocalization();
  bool localizationStarted();
  bool localizationReady();
  bool localizationFailed();
  bool beginStopLocalization();
  bool localizationStopped();
  bool beginNavigation();
  bool navigationStarted();
  bool pauseNavigation();
  bool resumeNavigation();
  bool beginStopNavigation();
  bool navigationStopped();
  void enterError();
  bool resetError();

 private:
  // 仅当当前状态等于 expected 时才切换到 target。
  bool transition(RobotMode expected, RobotMode target);

  mutable std::mutex mutex_;
  RobotMode mode_{RobotMode::kBoot};
};

}  // namespace robot_runtime
