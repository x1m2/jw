

#ifndef SUPER_LIO_RELOCATION_H_
#define SUPER_LIO_RELOCATION_H_


#include "super_lio.h"
#include <future>


namespace LI2Sup{

class SuperLIOReLoc : public SuperLIO {
public:
  SuperLIOReLoc(){};
  ~SuperLIOReLoc(){};

  void init() override;
  bool process() override;

private:
  struct RelocTaskResult
  {
    bool ok = false;
    Eigen::Matrix4d imu_pose = Eigen::Matrix4d::Identity();
  };

  bool kf_init() override;
  bool map_init() override;
  bool RunBackend() override;
  void UpdateMap() override;
  void Output() override;

private:
  BASIC::CloudPtr init_obs_data_;
  bool flg_get_init_guess_ = false;

  std::future<RelocTaskResult> reloc_future_;
  bool reloc_inflight_ = false;
};

} // namespace END.

#endif


