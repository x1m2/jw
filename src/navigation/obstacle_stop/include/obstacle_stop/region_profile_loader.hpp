#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>

namespace obstacle_stop {

struct ZoneDistanceOverrides {
  std::optional<double> stop_distance;
  std::optional<double> slowdown_distance;
  std::optional<std::size_t> obstacle_cell_threshold;
};

// 区域YAML不改变传感器来源、区域开关或坐标基准。
struct RegionObstacleStopOverrides {
  std::array<ZoneDistanceOverrides, 4> zones;
  std::optional<double> linear_velocity_factor;
  std::optional<double> angular_velocity_factor;
};

// 文件不存在是正常回退，found=false；文件存在但不可读或内容非法时返回false。
bool loadRegionObstacleStopOverrides(const std::filesystem::path &path,
                                     RegionObstacleStopOverrides &overrides,
                                     bool &found, std::string &message);

}  // namespace obstacle_stop
