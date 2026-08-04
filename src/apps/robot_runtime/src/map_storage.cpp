#include "robot_runtime/map_storage.hpp"

#include <fstream>
#include <stdexcept>
#include <utility>

namespace robot_runtime {

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

std::filesystem::path MapStorage::logPath(const std::string &map_id) const {
  return dataPath(map_id) / "log";
}

std::filesystem::path MapStorage::taskPath(const std::string &map_id) const {
  return dataPath(map_id) / "task";
}

std::filesystem::path MapStorage::stagingPath(
    const std::string &map_id, uint64_t session_id) const {
  return root_directory_ / ".staging" /
         (map_id + "_" + std::to_string(session_id)) / "map";
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
  // 删除map_id目录时同时删除地图、日志和任务数据。
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

bool MapStorage::finalize(const std::filesystem::path &staging_path,
                          const std::string &map_id,
                          std::string &message) const {
  // 协议2.2：globalmap.pcd是地图保存成功的最低完整性条件。
  if (map_id.empty() || staging_path.empty()) {
    message = "no active map task";
    return false;
  }

  const auto global_map = staging_path / "globalmap.pcd";
  std::error_code filesystem_error;
  if (!std::filesystem::exists(global_map, filesystem_error) ||
      filesystem_error) {
    message = filesystem_error
                  ? "failed to inspect saved map: " +
                        filesystem_error.message()
                  : "PGO did not produce globalmap.pcd";
    return false;
  }

  std::ofstream manifest(staging_path / "manifest.yaml");
  if (!manifest.is_open()) {
    message = "failed to create manifest.yaml";
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

  const auto final_data_path = dataPath(map_id);
  const auto final_map_path = mapPath(map_id);
  const bool final_data_exists =
      std::filesystem::exists(final_data_path, filesystem_error);
  if (filesystem_error) {
    message = "failed to inspect existing map: " +
              filesystem_error.message();
    return false;
  }
  if (final_data_exists) {
    message = "map data directory appeared after mapping started; publish rejected";
    return false;
  }

  std::size_t saved_map_count = 0U;
  if (!countSavedMaps(saved_map_count, message)) return false;
  if (saved_map_count >= max_maps_) {
    message = "map limit reached before finalization: " +
              std::to_string(saved_map_count) + "/" +
              std::to_string(max_maps_);
    return false;
  }

  // staging_path指向暂存map目录，其父目录是本次完整的map_id数据目录。
  const auto staging_data_path = staging_path.parent_path();
  std::filesystem::create_directories(staging_data_path / "log",
                                      filesystem_error);
  if (!filesystem_error) {
    std::filesystem::create_directories(staging_data_path / "task",
                                        filesystem_error);
  }
  if (filesystem_error) {
    message = "failed to create map log/task directories: " +
              filesystem_error.message();
    return false;
  }

  // 同一文件系统内原子发布整个map_id目录，保证map、log、task同时可见。
  std::filesystem::rename(staging_data_path, final_data_path,
                          filesystem_error);
  if (filesystem_error) {
    message = "failed to publish saved map: " + filesystem_error.message();
    return false;
  }

  message = "map saved: map_id=" + map_id +
            ", map_path=" + (final_map_path / "globalmap.pcd").string();
  return true;
}

}  // namespace robot_runtime
