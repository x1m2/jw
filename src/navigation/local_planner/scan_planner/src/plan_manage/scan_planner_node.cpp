#include <memory>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <plan_manage/scan_replan_fsm.h>
#include <plan_manage/controller.h>
#include <slam_manager/managed_worker.hpp>

namespace {
struct PlannerRuntime {
  scan_planner::SCANReplanFSM planner;
  std::unique_ptr<scan_planner::Controller> controller;
};
}
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto worker = std::make_shared<slam_manager::ManagedWorker>("scan_planner_worker", []() {
      auto node = std::make_shared<rclcpp::Node>("scan_planner_node");
      if (node->declare_parameter<std::string>("controller_mode", "closed_loop") != "closed_loop")
        throw std::invalid_argument("hardware baseline requires controller_mode=closed_loop");
      auto runtime = std::make_shared<PlannerRuntime>();
      runtime->planner.init(node.get());
      runtime->controller = scan_planner::createClosedLoopController(*node, runtime->planner);
      RCLCPP_INFO(node->get_logger(), "[scan_baseline] mapping=original planner=original tracking=original max_vy=%.3f terminal=platform_adapter callbacks=serialized",
          node->get_parameter("closed_loop.max_vy").as_double());
      slam_manager::ManagedWorker::AlgorithmInstance instance;
      instance.node = node; instance.owner = runtime;
      instance.before_stop = [runtime]() { runtime->controller->stop(); };
      return instance;
    }, 1U);
    rclcpp::spin(worker);
    worker.reset();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("scan_planner_worker"), "[scan_start_error] %s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
