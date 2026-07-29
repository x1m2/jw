#pragma once

#include <cstdint>
#include <string_view>

namespace robot_runtime {

// 机器人运行模式，包含建图和导航过程中的过渡状态。
enum class RobotMode : uint8_t {
  kBoot = 0,
  kIdle,
  kStartingMapping,
  kMapping,
  kStoppingMapping,
  kStartingLocalization,
  kLocalizing,
  kReady,
  kStartingNavigation,
  kNavigating,
  kNavigationPaused,
  kStoppingNavigation,
  kError,
};

// 转换成稳定的字符串，供状态话题和日志使用。
inline std::string_view toString(RobotMode mode) {
  switch (mode) {
    case RobotMode::kBoot:
      return "boot";
    case RobotMode::kIdle:
      return "idle";
    case RobotMode::kStartingMapping:
      return "starting_mapping";
    case RobotMode::kMapping:
      return "mapping";
    case RobotMode::kStoppingMapping:
      return "stopping_mapping";
    case RobotMode::kStartingLocalization:
      return "starting_localization";
    case RobotMode::kLocalizing:
      return "localizing";
    case RobotMode::kReady:
      return "ready";
    case RobotMode::kStartingNavigation:
      return "starting_navigation";
    case RobotMode::kNavigating:
      return "navigating";
    case RobotMode::kNavigationPaused:
      return "navigation_paused";
    case RobotMode::kStoppingNavigation:
      return "stopping_navigation";
    case RobotMode::kError:
      return "error";
  }
  return "unknown";
}

}  // namespace robot_runtime
