
#ifndef LI2Sup_DS_COMMON_H
#define LI2Sup_DS_COMMON_H

#include <queue>
#include <deque>
#include <string>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{

enum LID_TYPE
{
  LIVOX = 1,
  HESAI16 = 2,
  VELO16 = 3,
  VELO32 = 4,
  VEL_NCLT = 5,
  LS16 = 6,
  OUSTER = 7
};

static const std::array<std::string, 7> LID_TYPE_NAMES = {
  "INVALID",
  "LIVOX",
  "HESAI16",
  "VELO16",
  "VELO32",
  "VEL_NCLT",
  "LS16"
};


struct SysState {
  SysState() = default;

  explicit SysState(double time, const BASIC::SO3& R = BASIC::SO3(), const BASIC::V3& t = BASIC::V3::Zero(), const BASIC::V3& v = BASIC::V3::Zero(),
                    const BASIC::V3& bg = BASIC::V3::Zero(), const BASIC::V3& ba = BASIC::V3::Zero())
      : timestamp(time), R(R), p(t), v(v), bg(bg), ba(ba) {}

  SysState(double time, const BASIC::SE3& pose, const BASIC::V3& vel = BASIC::V3::Zero())
      : timestamp(time), R(pose.R()), p(pose.t()), v(vel) {}

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R, p); }

  friend std::ostream& operator<<(std::ostream& os, const SysState& s) {
    os  << "p: " << s.p.transpose() << ", v: " << s.v.transpose()
        << ", q: " << s.R.coeffs().transpose() << ", bg: " << s.bg.transpose()
        << ", ba: " << s.ba.transpose();
    return os;
  }

  double timestamp = 0;
  BASIC::SO3 R;
  BASIC::V3  p = BASIC::V3::Zero();
  BASIC::V3  v = BASIC::V3::Zero();
  BASIC::V3  bg = BASIC::V3::Zero();
  BASIC::V3  ba = BASIC::V3::Zero();
};


struct NavState
{
  NavState() = default;
  explicit NavState(double time, const BASIC::SO3& R = BASIC::SO3(), const BASIC::V3& t = BASIC::V3::Zero(), const BASIC::V3& v = BASIC::V3::Zero())
      : timestamp(time), R(R), p(t), v(v){}
  
  BASIC::SE3 GetSE3() const { return BASIC::SE3(R, p); }
  double timestamp = 0;
  BASIC::SO3 R = BASIC::Eye3;
  BASIC::V3  p = BASIC::V3::Zero();
  BASIC::V3  v = BASIC::V3::Zero();
};


struct DynamicState
{
  DynamicState() = default;
  explicit DynamicState(
    double __time, 
    const BASIC::M3& __R, 
    const BASIC::V3& __p, 
    const BASIC::V3& __v,
    const BASIC::V3& __w,
    const BASIC::V3& __a)
  : time(__time), R(__R), p(__p), v(__v), w(__w), a(__a)
  {}

  double time = 0;
  BASIC::M3  R = BASIC::M3::Identity();
  BASIC::V3  p = BASIC::V3::Zero();
  BASIC::V3  v = BASIC::V3::Zero();
  BASIC::V3  w = BASIC::V3::Zero();
  BASIC::V3  a = BASIC::V3::Zero();
};


struct Pose_t
{
  Pose_t() = default;
  explicit Pose_t(double time, const BASIC::SO3& R = BASIC::SO3(), const BASIC::V3& t = BASIC::V3::Zero())
      : timestamp(time), R(R), p(t) {}
  
  BASIC::SE3 GetSE3() const { return BASIC::SE3(R, p); }
  double timestamp = 0;
  BASIC::SO3 R = BASIC::Eye3;
  BASIC::V3  p = BASIC::V3::Zero();
};


struct IMUData{
  double secs = 0.0;
  BASIC::V3 acc = BASIC::V3::Zero();
  BASIC::V3 gyr = BASIC::V3::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


struct LidarData
{
  double start_time = 0.0;
  double end_time = 0.0;
  pcl::PointCloud<LI2Sup::PointXTZIT>::Ptr pc{nullptr};
};


struct MeasureGroup {
  LidarData lidar;
  std::deque<IMUData> imu;
};


}

#endif
