

#ifndef SUPER_LIO_RELOCATION_H_
#define SUPER_LIO_RELOCATION_H_


#include "super_lio.h"


namespace LI2Sup{

class SuperLIOReLoc : public SuperLIO {
public:
  SuperLIOReLoc(){
    BASIC::V3 init_t = BASIC::V3(g_init_px, g_init_py, g_init_pz);
    Eigen::Matrix3d init_R = (Eigen::AngleAxisd(g_init_yaw / 180 * M_PI, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(g_init_pitch /180 * M_PI, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(g_init_roll / 180 * M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
                
    BASIC::M3 init_R2 = init_R.cast<BASIC::scalar>();             
      
    re_init_pose_ = BASIC::SE3(BASIC::SO3(init_R2), init_t);
  };
  ~SuperLIOReLoc(){};

  void init() override;

private:
  bool kf_init() override;
  bool map_init() override;
  void UpdateMap() override;
  void Output() override;

private:
  BASIC::CloudPtr init_obs_data_;
  bool flg_get_init_guess_ = false;
  BASIC::SE3 re_init_pose_;
};

} // namespace END.

#endif


