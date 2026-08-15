#pragma once

#include <mutex>

#include "slam_manager/slam_mode.hpp"

namespace slam_manager {

// 线程安全的机器人模式状态机，只允许预定义的合法状态流转。
class SlamStateMachine {
 public:
  SlamMode mode() const;

  bool initialize();
  bool beginMapping();
  bool mappingStarted();
  bool beginStopMapping();
  bool mappingStopped();
  bool beginLocalization();
  bool localizationStarted();
  bool localizationReady();
  bool localizationFailed();
  bool beginNavigation();
  bool navigationStarted();
  bool pauseNavigation();
  bool resumeNavigation();
  bool beginStopNavigation();
  bool navigationStopped();
  void enterError();

 private:
  // 仅当当前状态等于 expected 时才切换到 target。
  bool transition(SlamMode expected, SlamMode target);

  mutable std::mutex mutex_;
  SlamMode mode_{SlamMode::kBoot};
};

}  // namespace slam_manager
