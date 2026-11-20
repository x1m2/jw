

#ifndef SUPER_LIO_RELOCATION_H_
#define SUPER_LIO_RELOCATION_H_


#include "super_lio.h"
#include <future>


namespace LI2Sup{

class SuperLIOReLoc : public SuperLIO {
public:
  SuperLIOReLoc(){};
  ~SuperLIOReLoc() override;

  void init() override;
  bool process() override;
  void requestStop();

private:
  struct RelocTaskResult
  {
    bool ok = false;
    Eigen::Matrix4d imu_pose = Eigen::Matrix4d::Identity();
  };

  bool kf_init() override;
  void Output() override;
  void tracking_failed() override;

private:
  std::atomic<bool> stopping_{false};
  int imu_cout = 0, init_frame_count = 0, output_count_ = -1;
  BASIC::V3 mean_gyro = BASIC::V3::Zero(), mean_acce = BASIC::V3::Zero();
  BASIC::CloudPtr init_obs_data_;
  bool flg_get_init_guess_ = false;

  std::future<RelocTaskResult> reloc_future_;
  bool reloc_inflight_ = false;
};

} // namespace END.

#endif


