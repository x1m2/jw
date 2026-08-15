#pragma once

#include <cstdint>
#include <string_view>

namespace slam_manager {

// 机器人运行模式，包含建图和导航过程中的过渡状态。
enum class SlamMode : uint8_t {
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
inline std::string_view toString(SlamMode mode) {
  switch (mode) {
    case SlamMode::kBoot:
      return "boot";
    case SlamMode::kIdle:
      return "idle";
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
    case SlamMode::kReady:
      return "ready";
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

}  // namespace slam_manager
