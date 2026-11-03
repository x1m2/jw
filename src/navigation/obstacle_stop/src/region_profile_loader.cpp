#include "obstacle_stop/region_profile_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <unordered_set>

namespace obstacle_stop {
namespace {

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
        "linear_velocity_factor",
        "angular_velocity_factor", "zones"};
    for (const auto &entry : parameters) {
      const std::string name = entry.first.as<std::string>();
      if (allowed.find(name) == allowed.end()) {
        message = "unsupported region obstacle_stop parameter '" + name +
                  "' in " + path.string();
        return false;
      }
    }

    if (parameters["zones"]) {
      const auto zones = parameters["zones"];
      if (!zones.IsMap()) throw YAML::RepresentationException(zones.Mark(), "zones must be a map");
      const std::array<std::string,4> names{"front","rear","left","right"};
      const std::unordered_set<std::string> keys{
          "stop_distance", "slowdown_distance", "obstacle_cell_threshold"};
      for (const auto &entry : zones) {
        const auto name = entry.first.as<std::string>();
        const auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end() || !entry.second.IsMap()) {
          message = "invalid region zone '" + name + "' in " + path.string(); return false;
        }
        auto &target = overrides.zones[static_cast<std::size_t>(it-names.begin())];
        for (const auto &value : entry.second) {
          const auto key = value.first.as<std::string>();
          if ((name == "left" || name == "right") && key == "slowdown_distance") {
            message = "side zones support stopping only; remove " + name + ".slowdown_distance in " + path.string();
            return false;
          }
          if (!keys.count(key)) {
            message = "unsupported region zone parameter '" + name + "." + key + "' in " + path.string(); return false;
          }
          if (key == "obstacle_cell_threshold") {
            const auto threshold = value.second.as<int64_t>();
            if (threshold < 1) { message = "zone occupancy threshold must be at least 1"; return false; }
            target.obstacle_cell_threshold = static_cast<std::size_t>(threshold);
          } else {
            const double distance = value.second.as<double>();
            if (!std::isfinite(distance) || distance < 0.0) {
              message = "zone distances must be finite and nonnegative"; return false;
            }
            if (key == "stop_distance") target.stop_distance = distance;
            if (key == "slowdown_distance") target.slowdown_distance = distance;
          }
        }
      }
    }

    const auto read = [&parameters](const char *name) -> std::optional<double> {
      if (!parameters[name]) return std::nullopt;
      return parameters[name].as<double>();
    };
    overrides.linear_velocity_factor = read("linear_velocity_factor");
    overrides.angular_velocity_factor = read("angular_velocity_factor");

    if (!finiteRatio(overrides.linear_velocity_factor) ||
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
