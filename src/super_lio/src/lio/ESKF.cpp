#include "lio/ESKF.h"

#include <glog/logging.h>

using namespace BASIC;

namespace LI2Sup{

namespace {
constexpr double kMaxImuPropagationDt = 0.2;
}


/// [1] SEfB: P195：（7.37a） P203: (7.76a) (7.77a)  P220: Table 7-2 
/// [2] VSLAM14: P71: (4.26)  P73: (4.32)
/// [3] Proportional Derivative (PD) Control on the Euclidean Group.  A_matrix
/// [1] = [2] <===> [3]

static inline M3d RightJacobianSO3(const V3& ang_vel, const scalar& dt) {
  const V3d w = ang_vel.template cast<double>();
  const double dt_d = static_cast<double>(dt);
  M3d Jr = M3d::Identity();
  if (!w.allFinite() || !std::isfinite(dt_d)) {
    return Jr;
  }
  const V3d phi = w * dt_d;
  const double theta = phi.norm();
  if (!std::isfinite(theta)) {
    return Jr;
  }
  M3d K;
  K << 0.0,     -phi.z(),  phi.y(),
       phi.z(),  0.0,     -phi.x(),
      -phi.y(),  phi.x(),  0.0;

  const M3d K2 = K * K;
  double A, B;
  // J_r(phi) = I - A * phi^ + B * phi^2
  // A = (1 - cos(theta)) / theta^2
  // B = (theta - sin(theta)) / theta^3
  if (theta < 1e-6) {  // Taylor expansion to avoid small angle numerical reduction
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    A = 0.5 - theta2 / 24.0 + theta4 / 720.0;
    B = 1.0 / 6.0 - theta2 / 120.0 + theta4 / 5040.0;
  } else {
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    A = (1.0 - std::cos(theta)) / theta2;
    B = (theta - std::sin(theta)) / theta3;
  }
  Jr = M3d::Identity() - A * K + B * K2;
  return Jr;
}


/// left jacobian
M3d A_matrix(const V3 & v){
  M3d res;
  double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
	double norm = std::sqrt(squaredNorm);
	if(norm < 1e-6){
		res = M3d::Identity();
	}
	else{
    M3d K;
    K << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
		res = M3d::Identity() + (1 - std::cos(norm)) / squaredNorm * K 
        + (1 - std::sin(norm) / norm) / squaredNorm * K * K;
	}
  return res;
}


void ESKF::SetInitialConditions(Options options, const V3& init_bg, 
                                const V3& init_ba, const float imu_scale,
                                const V3& gravity) 
{
  BuildNoise(options);
  options_ = options;
  bg_ = init_bg;
  ba_ = init_ba;
  g_ = gravity;
  imu_scale_ = imu_scale;

  P_ = 1e-4 * M18::Identity();
  P_.template block<3, 3>(0, 0) = 0.1 * M_PI / 180.0 * M3::Identity();   // r

}


void ESKF::SetX(const SysState& x) {
  last_imu_time_ = x.timestamp;      // TODO: The timestamp update is not strictly consistent.
  current_time_ = last_imu_time_;
  R_ = x.R;
  p_ = x.p;
  v_ = x.v;
  bg_ = x.bg;
  ba_ = x.ba;
  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
}


void ESKF::BuildNoise(const Options& options) {
  double et = options.gyro_var_;
  double ev = options.acce_var_;
  double eg = options.bias_gyro_var_;
  double ea = options.bias_acce_var_;

  double et2 = et;  // * et;
  double ev2 = ev;  // * ev;
  double eg2 = eg;  // * eg;
  double ea2 = ea;  // * ea;

  // set Q
  Q_.diagonal() << et2, et2, et2,    // ng: r
                   ev2, ev2, ev2,    // na: v  -> p
                   eg2, eg2, eg2,    // nbg: bg -> w -> r
                   ea2, ea2, ea2;    // nba: ba -> a -> v -> p
}

void ESKF::Update() {
  R_ = R_ * SO3::Exp(dx_.template block<3, 1>(0, 0));
  p_ += dx_.template block<3, 1>(3, 0);
  v_ += dx_.template block<3, 1>(6, 0);

  bg_ += dx_.template block<3, 1>(9, 0);
  ba_ += dx_.template block<3, 1>(12, 0);

  g_ += dx_.template block<3, 1>(15, 0);
  g_ = g_gravity_norm * (g_.normalized());

  fw_R_ = R_;
  fw_p_ = p_;
  fw_v_ = v_;
  forward_time_ = current_obs_time_;
}


bool ESKF::Predict(const IMUData& imu, DynamicState& state_imu, DynamicState& state_robot){
  if(!init_) {
    return false;
  }

  /// first time predict
  if(forward_time_ < 0){
    forward_time_ = imu.secs;
    forward_last_imu_ = imu;
    return false;
  }

  double dt = imu.secs - forward_time_;

  if (!std::isfinite(dt) || dt < 0) {
    return false;
  }
  if (dt > kMaxImuPropagationDt) {
    // A rosbag/session boundary can leave a large hole in the IMU stream.
    // There are no measurements with which to integrate that interval, so
    // keep the last corrected state and restart prediction at the new time.
    fw_R_ = R_;
    fw_p_ = p_;
    fw_v_ = v_;
    forward_time_ = imu.secs;
    forward_last_imu_ = imu;
    state_imu = DynamicState(imu.secs, fw_R_.R_, fw_p_, fw_v_,
                             body_omega_, global_acc_);
    state_robot = state_imu;
    LOG(WARNING) << " ---> [ESKF]: forward IMU gap " << dt
                 << " s; prediction time rebased without integration.";
    return false;
  }

  V3 acc = 0.5 * (imu.acc + forward_last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  
  V3 gyr = 0.5 * (imu.gyr + forward_last_imu_.gyr) - bg_;
  V3 new_p = fw_p_ + fw_v_ * dt + 0.5 * (fw_R_.R() * acc) * dt * dt + 0.5 * g_ * dt * dt;
  V3 new_v = fw_v_ + fw_R_.R() * acc * dt + g_ * dt;
  SO3 new_R = fw_R_ * SO3::Exp(gyr , dt);

  fw_R_ = new_R;
  fw_v_ = new_v;
  fw_p_ = new_p;

  state_imu.time = imu.secs;
  state_imu.R = fw_R_.R_;
  state_imu.p = fw_p_;
  state_imu.v = fw_v_;
  state_imu.w = gyr;
  state_imu.a = acc;
  forward_time_ = imu.secs;
  forward_last_imu_ = imu;

  new_R.R_ = fw_R_.R_ * g_odom_robo.R_;
  new_p = fw_R_.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + fw_p_;
  state_robot.time = imu.secs;
  state_robot.R = new_R.R_;
  state_robot.p = new_p;
  // todo: v, w, a

  return true;
}


bool ESKF::Predict(const IMUData& imu) {

  if(last_imu_time_ < 0){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  if(imu.secs <= last_obs_time_){
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }
  
  current_time_ = imu.secs;

  double dt;
  if(last_imu_time_ < last_obs_time_){
    dt = imu.secs - last_obs_time_;
  }else if (imu.secs > current_obs_time_){
    dt = current_obs_time_ - last_imu_time_;
    current_time_ = current_obs_time_;
  }else{
    dt = imu.secs - last_imu_time_;
  }

  if (!std::isfinite(dt) || dt < 0) {
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }
  if (dt > kMaxImuPropagationDt) {
    // Never integrate across missing IMU data.  In particular, consecutive
    // bag files commonly have a several-second timestamp gap; integrating
    // gravity/acceleration over that hole makes the nominal state diverge.
    LOG(WARNING) << " ---> [ESKF]: IMU gap " << dt
                 << " s; state held and propagation time rebased.";
    current_time_ = std::min(imu.secs, current_obs_time_);
    last_imu_time_ = imu.secs;
    last_imu_ = imu;
    return false;
  }

  V3 acc = 0.5 * (imu.acc + last_imu_.acc);
  acc = imu_scale_ * acc;
  acc = acc - ba_;
  body_omega_ = 0.5 * (imu.gyr + last_imu_.gyr) - bg_;
  M3 Jr_dt = (dt * RightJacobianSO3(body_omega_, dt)).cast<scalar>();   // J_l(-phi) = J_r(phi)

  M3 R_m3 = R_.R_;
  M3 R_dt = R_m3 * dt;

  F_X f_x = F_X::Identity();
  f_x.template block<3, 3>(0, 0) = SO3::Exp(-body_omega_, dt).R_;
  f_x.template block<3, 3>(0, 9) = - Jr_dt;
  f_x.template block<3, 3>(3, 6) = M3::Identity() * dt;
  f_x.template block<3, 3>(6, 0) = - R_m3 * SO3::hat(acc) * dt;
  f_x.template block<3, 3>(6, 12) = - R_dt;
  f_x.template block<3, 3>(6, 15) = M3::Identity() * dt;
  

  F_W f_w = F_W::Zero();
  f_w.template block<3, 3>(0, 0) = - Jr_dt;
  f_w.template block<3, 3>(6, 3) = - R_dt;                 // v -> na
  f_w.template block<3, 3>(9, 6) = M3::Identity() * dt;    // ba
  f_w.template block<3, 3>(12, 9) = M3::Identity() * dt;   // bg

  P_ = f_x * P_ * f_x.transpose() + f_w * Q_ * f_w.transpose();

  global_acc_ = R_.R() * acc + g_;
  p_ = p_ + v_ * dt + 0.5 * global_acc_ * dt * dt;
  v_ = v_ + global_acc_ * dt;
  R_ = R_ * SO3::Exp(body_omega_, dt);

  last_imu_time_ = imu.secs;
  last_imu_ = imu;
  return true;
}


const int STATE_DIM = 18;
bool ESKF::UpdateObserve(ESKF::ObsFunc obs) {
  // propagated state
  SO3 R_pred = R_;
  V3  p_pred = p_;
  V3  v_pred = v_;
  V3  bg_pred = bg_;
  V3  ba_pred = ba_;
  V3  g_pred = g_;

  M18 P_pred = P_;

  M6 HTVH;
  V6 HTVr;

  M18 Pk = M18::Zero();
  M18 Qk = M18::Zero();
  M18 K_x = M18::Zero();

  need_converge_ = false;

  for (int iter = 0; iter < options_.num_iterations_; ++iter) {
    if (iter > 2) {
      need_converge_ = true;
    }

    obs(GetKFState(), HTVH, HTVr);

    V18 dx_prior = V18::Zero();
    dx_prior.template block<3,1>(0,0)  = (R_pred.inverse() * R_).log_vee();
    dx_prior.template block<3,1>(3,0)  = p_  - p_pred;
    dx_prior.template block<3,1>(6,0)  = v_  - v_pred;
    dx_prior.template block<3,1>(9,0)  = bg_ - bg_pred;
    dx_prior.template block<3,1>(12,0) = ba_ - ba_pred;
    dx_prior.template block<3,1>(15,0) = g_  - g_pred;

    M18 G_prior = M18::Identity();

    M3 J_prior = M3::Identity()
               - 0.5 * SO3::hat(dx_prior.template block<3,1>(0,0));

    G_prior.template block<3,3>(0,0) = J_prior;

    Pk = G_prior * P_pred * G_prior.transpose();

    dx_prior = G_prior * dx_prior;

    // H^T R^{-1} H
    M18 HTRH = M18::Zero();
    HTRH.template block<6,6>(0,0) = HTVH;

    // information form
    M18 A = Pk.inverse() + HTRH;
    Qk = A.inverse();

    V18 b = V18::Zero();
    b.template head<6>() = HTVr;

    K_x = Qk * HTRH;

    // dx = K_h + (K_x - I) * dx_prior
    dx_ = Qk * b + (K_x - M18::Identity()) * dx_prior;

    Update();

    if (dx_.lpNorm<Eigen::Infinity>() < options_.quit_eps_ && iter > 0) {
      break;
    }
  }

  P_ = Qk;

  M18 G_reset = M18::Identity();
  M3 J_reset = M3::Identity()
             - 0.5 * SO3::hat(dx_.template block<3,1>(0,0));

  G_reset.template block<3,3>(0,0) = J_reset;

  P_ = G_reset * P_ * G_reset.transpose();

  P_ = 0.5 * (P_ + P_.transpose());

  dx_.setZero();

  last_obs_time_ = current_obs_time_;
  return true;
}

}
