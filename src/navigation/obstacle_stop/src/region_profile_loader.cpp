#include "obstacle_stop/region_profile_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <unordered_set>

namespace obstacle_stop {
namespace {

bool finiteNonNegative(const std::optional<double> &value) {
  return !value || (std::isfinite(*value) && *value >= 0.0);
}

bool finiteRatio(const std::optional<double> &value) {
  return !value || (std::isfinite(*value) && *value >= 0.0 && *value <= 1.0);
}

}  // namespace

bool loadRegionObstacleStopOverrides(const std::filesystem::path &path,
                                     RegionObstacleStopOverrides &overrides,
                                     bool &found, std::string &message) {
  overrides = {};
  found = false;

  std::error_code filesystem_error;
  const bool exists = std::filesystem::exists(path, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect region obstacle_stop profile " +
              path.string() + ": " + filesystem_error.message();
    return false;
  }
  if (!exists) return true;
  if (!std::filesystem::is_regular_file(path, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect region obstacle_stop profile " +
                        path.string() + ": " + filesystem_error.message()
                  : "region obstacle_stop profile is not a regular file: " +
                        path.string();
    return false;
  }

  try {
    const YAML::Node document = YAML::LoadFile(path.string());
    YAML::Node parameters;
    if (document["/**"] && document["/**"]["ros__parameters"]) {
      parameters = document["/**"]["ros__parameters"];
    } else if (document["obstacle_stop"] &&
               document["obstacle_stop"]["ros__parameters"]) {
      parameters = document["obstacle_stop"]["ros__parameters"];
    } else if (document["ros__parameters"]) {
      parameters = document["ros__parameters"];
    } else {
      parameters = document;
    }
    if (!parameters || !parameters.IsMap()) {
      message = "region obstacle_stop profile must contain a parameter map: " +
                path.string();
      return false;
    }

    const std::unordered_set<std::string> allowed{
        "hard_stop_distance", "slowdown_distance", "obstacle_ratio_threshold",
        "linear_velocity_factor", "angular_velocity_factor"};
    for (const auto &entry : parameters) {
      const std::string name = entry.first.as<std::string>();
      if (allowed.find(name) == allowed.end()) {
        message = "unsupported region obstacle_stop parameter '" + name +
                  "' in " + path.string();
        return false;
      }
    }

    const auto read = [&parameters](const char *name) -> std::optional<double> {
      if (!parameters[name]) return std::nullopt;
      return parameters[name].as<double>();
    };
    overrides.hard_stop_distance = read("hard_stop_distance");
    overrides.slowdown_distance = read("slowdown_distance");
    overrides.obstacle_ratio_threshold = read("obstacle_ratio_threshold");
    overrides.linear_velocity_factor = read("linear_velocity_factor");
    overrides.angular_velocity_factor = read("angular_velocity_factor");

    if (!finiteNonNegative(overrides.hard_stop_distance) ||
        !finiteNonNegative(overrides.slowdown_distance) ||
        !finiteRatio(overrides.obstacle_ratio_threshold) ||
        !finiteRatio(overrides.linear_velocity_factor) ||
        !finiteRatio(overrides.angular_velocity_factor)) {
      message = "invalid region obstacle_stop parameter in " + path.string();
      return false;
    }
  } catch (const YAML::Exception &error) {
    message = "failed to parse region obstacle_stop profile " + path.string() +
              ": " + error.what();
    return false;
  }

  found = true;
  message.clear();
  return true;
}

}  // namespace obstacle_stop
