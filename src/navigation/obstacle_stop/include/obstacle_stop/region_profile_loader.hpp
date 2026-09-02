#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace obstacle_stop {

// 区域YAML只允许覆盖会改变本体停障行为的参数。
struct RegionObstacleStopOverrides {
  std::optional<double> hard_stop_distance;
  std::optional<double> slowdown_distance;
  std::optional<double> obstacle_ratio_threshold;
  std::optional<double> linear_velocity_factor;
  std::optional<double> angular_velocity_factor;
};

// 文件不存在是正常回退，found=false；文件存在但不可读或内容非法时返回false。
bool loadRegionObstacleStopOverrides(const std::filesystem::path &path,
                                     RegionObstacleStopOverrides &overrides,
                                     bool &found, std::string &message);

}  // namespace obstacle_stop
