#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace slam_manager {

// 管理地图目录、暂存发布、数量限制和删除，不依赖ROS接口。
class MapStorage {
 public:
  MapStorage(std::filesystem::path root_directory, std::size_t max_maps);

  const std::filesystem::path &rootDirectory() const;
  std::size_t maxMaps() const;
  // map_id本身就是数据目录名（例如map_0），内部划分map、log和task。
  std::filesystem::path dataPath(const std::string &map_id) const;
  std::filesystem::path mapPath(const std::string &map_id) const;
  std::filesystem::path logPath(const std::string &map_id) const;
  std::filesystem::path taskPath(const std::string &map_id) const;
  std::filesystem::path stagingPath(const std::string &map_id,
                                    uint64_t session_id) const;

  bool countSavedMaps(std::size_t &count, std::string &message) const;
  bool deleteMap(const std::string &map_id, std::string &message) const;
  bool finalize(const std::filesystem::path &staging_path,
                const std::string &map_id, std::string &message) const;

 private:
  std::filesystem::path root_directory_;
  std::size_t max_maps_;
};

}  // namespace slam_manager
