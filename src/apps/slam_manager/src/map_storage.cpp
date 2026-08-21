#include "slam_manager/map_storage.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace slam_manager {

MapStorage::MapStorage(std::filesystem::path root_directory,
                       std::size_t max_maps)
    : root_directory_(std::move(root_directory)), max_maps_(max_maps) {
  if (root_directory_.empty() || !root_directory_.is_absolute()) {
    throw std::invalid_argument(
        "map storage root directory must be an absolute path");
  }
  if (max_maps_ == 0U) {
    throw std::invalid_argument("map storage max_maps must be greater than zero");
  }
}

const std::filesystem::path &MapStorage::rootDirectory() const {
  return root_directory_;
}

std::size_t MapStorage::maxMaps() const { return max_maps_; }

std::filesystem::path MapStorage::dataPath(const std::string &map_id) const {
  return root_directory_ / map_id;
}

std::filesystem::path MapStorage::mapPath(const std::string &map_id) const {
  return dataPath(map_id) / "map";
}

std::filesystem::path MapStorage::taskPath(const std::string &map_id) const {
  return dataPath(map_id) / "task";
}

bool MapStorage::countSavedMaps(std::size_t &count,
                                std::string &message) const {
  // 只有包含globalmap.pcd的非隐藏目录才计为一张完整地图。
  count = 0U;
  std::error_code filesystem_error;
  if (!std::filesystem::exists(root_directory_, filesystem_error)) {
    if (filesystem_error) {
      message = "failed to inspect map root: " + filesystem_error.message();
      return false;
    }
    return true;
  }

  std::filesystem::directory_iterator iterator(root_directory_,
                                                filesystem_error);
  const std::filesystem::directory_iterator end;
  if (filesystem_error) {
    message = "failed to list map root: " + filesystem_error.message();
    return false;
  }
  while (iterator != end) {
    const auto &entry = *iterator;
    const std::string name = entry.path().filename().string();
    if (!name.empty() && name.front() != '.' &&
        entry.is_directory(filesystem_error) &&
        std::filesystem::exists(entry.path() / "map" / "globalmap.pcd",
                                filesystem_error)) {
      ++count;
    }
    if (filesystem_error) {
      message = "failed to inspect map entry: " + filesystem_error.message();
      return false;
    }
    iterator.increment(filesystem_error);
    if (filesystem_error) {
      message = "failed to iterate map root: " + filesystem_error.message();
      return false;
    }
  }
  return true;
}

bool MapStorage::deleteMap(const std::string &map_id,
                           std::string &message) const {
  // 删除map_id数据目录；运行日志位于root/log下，不由MapStorage管理。
  const auto target = dataPath(map_id);
  std::error_code filesystem_error;
  const bool exists = std::filesystem::exists(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect map directory: " +
              filesystem_error.message();
    return false;
  }
  if (!exists) {
    message = "map does not exist: " + map_id;
    return false;
  }
  if (!std::filesystem::is_directory(target, filesystem_error) ||
      filesystem_error) {
    message = "map target is not a valid directory: " + target.string();
    return false;
  }

  const auto removed_count =
      std::filesystem::remove_all(target, filesystem_error);
  if (filesystem_error) {
    message = "failed to delete map directory: " +
              filesystem_error.message();
    return false;
  }
  message = "map deleted: " + map_id + ", removed entries=" +
            std::to_string(removed_count);
  return true;
}

bool MapStorage::finalize(const std::filesystem::path &map_path,
                          const std::string &map_id,
                          std::string &message) const {
  // 目录由GUI/launch预先创建；slam_manager只检查并写入保存结果。
  if (map_id.empty() || map_path.empty()) {
    message = "no active map task";
    return false;
  }

  const auto expected_map_path = mapPath(map_id);
  if (map_path != expected_map_path) {
    message = "unexpected map save path: " + map_path.string();
    return false;
  }

  std::error_code filesystem_error;
  if (!std::filesystem::exists(dataPath(map_id), filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect map data directory: " +
                        filesystem_error.message()
                  : "map data directory does not exist: " +
                        dataPath(map_id).string();
    return false;
  }
  if (!std::filesystem::is_directory(map_path, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect map directory: " +
                        filesystem_error.message()
                  : "map directory does not exist: " + map_path.string();
    return false;
  }
  const auto task_path = taskPath(map_id);
  if (!std::filesystem::is_directory(task_path, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect task directory: " +
                        filesystem_error.message()
                  : "task directory does not exist: " + task_path.string();
    return false;
  }

  // globalmap.pcd是地图保存成功的最低完整性条件。
  const auto global_map = map_path / "globalmap.pcd";
  if (!std::filesystem::exists(global_map, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect saved map: " +
                        filesystem_error.message()
                  : "PGO did not produce globalmap.pcd";
    return false;
  }
  if (!std::filesystem::is_regular_file(global_map, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect globalmap.pcd: " +
                        filesystem_error.message()
                  : "globalmap.pcd is not a regular file";
    return false;
  }

  // manifest属于地图保存结果文件，可以写入，但不创建任何目录。
  std::ofstream manifest(map_path / "manifest.yaml");
  if (!manifest.is_open()) {
    message = "failed to create manifest.yaml in existing map directory";
    return false;
  }
  manifest << "schema_version: 1\n"
           << "map_id: \"" << map_id << "\"\n"
           << "frame_id: \"map\"\n"
           << "status: \"ready\"\n"
           << "global_map: \"globalmap.pcd\"\n";
  manifest.close();
  if (!manifest) {
    message = "failed to write manifest.yaml";
    return false;
  }

  message = "map saved: map_id=" + map_id +
            ", map_path=" + global_map.string();
  return true;
}

}  // namespace slam_manager
