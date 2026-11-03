#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace obstacle_stop {

// 每帧的XY高度柱：计数不随高度层数增长，高度范围仅描述观测包络。
struct HeightCell {
  uint32_t point_count{0U};
  double z_min{std::numeric_limits<double>::infinity()};
  double z_max{-std::numeric_limits<double>::infinity()};

  void add(double z) {
    if (!std::isfinite(z)) return;
    if (point_count < std::numeric_limits<uint32_t>::max()) ++point_count;
    z_min = std::min(z_min, z);
    z_max = std::max(z_max, z);
  }

  bool occupied(std::size_t minimum_points) const {
    return point_count > 0U && point_count >= minimum_points;
  }
};

}  // namespace obstacle_stop
