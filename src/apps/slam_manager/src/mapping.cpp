#include "slam_manager/mapping.hpp"

#include <utility>

namespace slam_manager {

MappingController::MappingController(
    rclcpp::Node *node, rclcpp::CallbackGroup::SharedPtr callback_group)
    : pgo_worker_(node, "/pgo_worker/set_state", callback_group),
      fast_lio_worker_(node, "/fast_lio_worker/set_state", callback_group) {}

void MappingController::start(uint64_t session_id, const std::string &map_id,
                              std::shared_ptr<WorkerOperationToken> operation_token,
                              Completion completion) {
  // PGO 先准备好，再启动产生建图数据的 FastLIO。
  WorkerStartContext pgo_context;
  pgo_context.resource_id = map_id;
  pgo_worker_.start(
      session_id, pgo_context,
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

void MappingController::finish(uint64_t session_id,
                               const std::string &map_save_path,
                               Completion completion) {
  // 先停止FastLIO冻结输入，再显式保存，最后停止PGO。
  fast_lio_worker_.stop(
      session_id,
      [this, session_id, map_save_path, completion = std::move(completion)](
          bool fast_lio_ok, const std::string &fast_lio_message) mutable {
        if (!fast_lio_ok) {
          pgo_worker_.stop(
              session_id,
              [completion = std::move(completion), fast_lio_message](
                  bool pgo_ok, const std::string &pgo_message) mutable {
                completion(false, "FastLIO stop failed: " + fast_lio_message +
                                      "; PGO stop=" + pgo_message +
                                      (pgo_ok ? "" : " (failed)"));
              });
          return;
        }
        pgo_worker_.save(
            session_id, map_save_path,
            [this, session_id, completion = std::move(completion)](
                bool save_ok, const std::string &save_message) mutable {
              pgo_worker_.stop(
                  session_id,
                  [completion = std::move(completion), save_ok,
                   save_message](bool pgo_ok,
                                 const std::string &pgo_message) mutable {
                    if (save_ok && pgo_ok) {
                      completion(true, "map saved and mapping workers stopped");
                      return;
                    }
                    completion(false, "PGO save=" + save_message +
                                          "; PGO stop=" + pgo_message +
                                          (pgo_ok ? "" : " (failed)"));
                  });
            });
      });
}

void MappingController::cancel(uint64_t session_id, Completion completion) {
  // 取消时只按顺序停止两个Worker，不发送保存命令。
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
                completion(true, "mapping workers cancelled without saving");
                return;
              }
              completion(false, "cancel failed: FastLIO=" + fast_lio_message +
                                    ", PGO=" + pgo_message);
            });
      });
}

}  // namespace slam_manager
