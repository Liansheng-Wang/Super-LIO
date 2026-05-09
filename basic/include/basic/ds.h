#ifndef DATA_STRUCTURE_H_H_
#define DATA_STRUCTURE_H_H_


#include "alias.h"
#include "Manifold.h"


namespace BASIC
{

  enum LID_TYPE
  {
    MID360 = 1,
    HESAI16,
    VELO16,
    VELO32,
    VEL_NCLT,
    LS16
  };

  
  struct RobotState {
    V3 p, v, a, j;
    scalar yaw;
    double rcv_time;
    bool rcv{false};
    SO3 q;
  };


  struct NavState
  {
    NavState() = default;
    explicit NavState(double time, const SO3& R = SO3(), const V3& t = V3::Zero(), const V3& v = V3::Zero())
        : timestamp(time), R(R), p(t), v(v){}
    
    SE3 GetSE3() const { return SE3(R, p); }
    double timestamp = 0;
    SO3 R = Eye3;
    V3  p = V3::Zero();
    V3  v = V3::Zero();
  };

  
  struct Pose_t
  {
    Pose_t() = default;
    explicit Pose_t(double time, const SO3& R = SO3(), const V3& t = V3::Zero())
        : timestamp(time), R(R), p(t) {}
    
    SE3 GetSE3() const { return SE3(R, p); }
    double timestamp = 0;
    SO3 R = Eye3;
    V3  p = V3::Zero();
  };


  struct ObsData
  {
    VV3 cloud;
    SE3 pose;
    double timestamp = 0;
  };

} // namespace name


#endif // DATA_STRUCTURE_H_H_