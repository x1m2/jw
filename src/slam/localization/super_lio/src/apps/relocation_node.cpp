

#include <csignal>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros/ROSWrapper.h"
#include "lio/super_lio_reloc.h"


using namespace LI2Sup;

void SigHandle(int sig) {
  g_flag_run = false;
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  
  auto lio = std::make_shared<SuperLIOReLoc>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  auto timer = data_wrapper->create_wall_timer(
      std::chrono::milliseconds(2),
      [lio]()
      {
        Timer2 timer;
        if (lio->process())
        {
          LOG_INFO("location cost time = %.2lf ms", timer.elapsedLast());
        }
      },
      data_wrapper->getSensorCallbackGroup());

  rclcpp::spin(data_wrapper);

  lio->printTimeRecord();

  rclcpp::shutdown();
  return 0;
}
