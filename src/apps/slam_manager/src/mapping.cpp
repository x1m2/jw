#include "slam_manager/mapping.hpp"

#include <utility>

namespace slam_manager {

MappingController::MappingController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : pgo_worker_(node, "/pgo_worker/set_state", callback_group),
      fast_lio_worker_(node, "/fast_lio_worker/set_state", callback_group) {}

void MappingController::start(uint64_t session_id, const std::string &map_id,
                              const std::string &map_staging_path,
                              std::shared_ptr<WorkerOperationToken> operation_token,
                              Completion completion) {
  // PGO 先准备好，再启动产生建图数据的 FastLIO。
  pgo_worker_.start(
      session_id, map_id, map_staging_path,
      [this, session_id, operation_token,
       completion = std::move(completion)](
          bool pgo_ok, const std::string &pgo_message) mutable {
        if (!operation_token->active()) {
          completion(false, "mapping start cancelled after PGO response");
          return;
        }
        if (!pgo_ok) {
          completion(false, "PGO start failed: " + pgo_message);
          return;
        }

        fast_lio_worker_.start(
            session_id,
            [operation_token, completion = std::move(completion)](
                bool fast_lio_ok,
                const std::string &fast_lio_message) mutable {
              if (!operation_token->active()) {
                completion(false,
                           "mapping start cancelled after FastLIO response");
                return;
              }
              if (fast_lio_ok) {
                completion(true, "mapping workers started");
                return;
              }
              completion(false, "FastLIO start failed: " + fast_lio_message);
            });
      });
}

void MappingController::stop(uint64_t session_id, Completion completion) {
  // 先停止 FastLIO 数据生产，再停止 PGO 数据处理。
  fast_lio_worker_.stop(
      session_id,
      [this, session_id, completion = std::move(completion)](
          bool fast_lio_ok, const std::string &fast_lio_message) mutable {
        pgo_worker_.stop(
            session_id,
            [completion = std::move(completion), fast_lio_ok,
             fast_lio_message](bool pgo_ok,
                               const std::string &pgo_message) mutable {
              if (fast_lio_ok && pgo_ok) {
                completion(true, "mapping workers stopped");
                return;
              }
              completion(false, "stop failed: FastLIO=" + fast_lio_message +
                                    ", PGO=" + pgo_message);
            });
      });
}

}  // namespace slam_manager
