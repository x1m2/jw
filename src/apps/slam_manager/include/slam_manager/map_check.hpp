#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

namespace slam_manager {

// 管理地图目录、暂存发布、数量限制和删除，不依赖ROS接口。
class MapStorage {
 public:
  MapStorage(std::filesystem::path root_directory, std::size_t max_maps);

  const std::filesystem::path &rootDirectory() const;
  std::size_t maxMaps() const;
  // map_id本身就是数据目录名，不额外添加map_前缀。
  std::filesystem::path dataPath(const std::string &map_id) const;
  std::filesystem::path mapPath(const std::string &map_id) const;
  std::filesystem::path taskPath(const std::string &map_id) const;

  bool countSavedMaps(std::size_t &count, std::string &message) const;
  // 开始建图前准备空目录：同名目录存在时整体覆盖，否则新建。
  bool prepareForMapping(const std::string &map_id,
                         std::string &message) const;
  bool deleteMap(const std::string &map_id, std::string &message) const;
  bool finalize(const std::filesystem::path &map_path,
                const std::string &map_id, std::string &message) const;

 private:
  std::filesystem::path root_directory_;
  std::size_t max_maps_;
};

// 负责3D-BBS saver子进程的启动、超时终止和输出文件校验。
class BbsVoxelGenerator {
 public:
  struct Config {
    std::filesystem::path executable;
    double min_level_resolution{0.5};
    int64_t max_level{6};
    double timeout_sec{120.0};
  };

  explicit BbsVoxelGenerator(Config config);

  bool generate(const std::filesystem::path &map_directory,
                const std::string &map_id, std::string &message,
                const std::function<bool()> &is_canceling = {}) const;

 private:
  Config config_;
};

}  // namespace slam_manager
