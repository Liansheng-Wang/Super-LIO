
#include "basic/Manifold.h"


namespace BASIC {

template <>
inline float tolerance<float>() {
    return 1e-6f;
}
template <>
inline double tolerance<double>() {
    return 1e-11;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SKEW(const Eigen::MatrixBase<Derived> &v) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3>  m;
  m << typename Derived::Scalar(0), -v[2], v[1], 
      v[2], typename Derived::Scalar(0), -v[0], 
      -v[1], v[0], typename Derived::Scalar(0);
  return m;
}


template <typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang) {
  T ang_norm = ang.norm();
  if (ang_norm > 1e-10) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_axis);
    /// Roderigous Tranformation
    return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  } else {
    return Eye3;
  }
}


M3 A_matrix(const V3& v) {
  M3 res;
  double squaredNorm = v.squaredNorm();
  double norm = std::sqrt(squaredNorm);
  if (norm < tolerance<scalar>()) {
    res = M3::Identity();
  } else {
    res = M3::Identity() + (1 - std::cos(norm)) / squaredNorm * SO3::hat(v) +
          (1 - std::sin(norm) / norm) / squaredNorm * SO3::hat(v) * SO3::hat(v);
  }
  return res;
}


template <typename T>
inline Eigen::Matrix<T, 3, 3> Gamma_1(const Eigen::Matrix<T, 3, 1>& ang){
  T ang_norm = ang.norm();
  if (ang_norm > 1e-10) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_axis);
    return Eye3 + ((1.0 - std::cos(ang_norm)) * K + (ang_norm - std::sin(ang_norm))* K * K) / ang_norm;
  } else {
    return Eye3;
  }
}


// Second-order differential variable
template <typename T>
inline Eigen::Matrix<T, 3, 3> Gamma_2(const Eigen::Matrix<T, 3, 1>& ang){
  T ang_norm = ang.norm();
  if (ang_norm > 1e-5) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_axis);
    /// Roderigous Tranformation
    return 0.5 * Eye3 + ( 2.0 * (ang_norm - std::sin(ang_norm)) * K 
                          + ((ang_norm * ang_norm + 2 * std::cos(ang_norm) - 2) * K ) ) 
                        / ( 2 * ang_norm * ang_norm);
  } else {
    return 0.5 * Eye3;
  }
}


template <typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
  T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (ang_vel_norm > 0.0000001) {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K;

    K = SKEW(r_axis);

    T r_ang = ang_vel_norm * dt;

    return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  } else {
    return Eye3;
  }
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3) {
  T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm > 1e-10) {
    Eigen::Matrix<T, 3, 1> r_ang = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_ang);
    return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
  } else {
    return Eye3;
  }
}

/* Logrithm of a Rotation Matrix */
template <typename T>
inline Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
  T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Matrix<T, 3, 1> K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot) {
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  } else {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}


inline M4 hat6(const V6 &xi){
  M4 xi_hat;
  xi_hat  <<  0.0,   -xi(2),  xi(1), xi(3),
              xi(2),  0.0,   -xi(0), xi(4),
             -xi(1),  xi(0),  0.0,   xi(5),
              0.0,    0.0,    0.0,   0.0;
  return xi_hat;
}

inline V6 vee6(const M4 &xi_hat){
  V6 xi;
  xi << -xi_hat(1,2), xi_hat(0,2), -xi_hat(0,1),
          xi_hat(0,3), xi_hat(1,3), xi_hat(2,3);
  return xi;
}



/**
 * SO3 and SE3 ref to https://github.com/hku-mars/BALM/blob/master/src/compare_test/SE3/SE3.hpp
 */


SO3::SO3(const M3 &R):R_(R){}

SO3::SO3(const V3 &w):R_(Eye3){
  this->exp(hat(w));
}

SO3::SO3(const SO3 &R):R_(R.R()){}

M3 SO3::hat(const V3 &v){
  M3 m;
  m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return m;
}

V3 SO3::vee(const M3 &w_hat){
  V3 w;
  w << -w_hat(1,2), w_hat(0,2), -w_hat(0,1);
  return w;
}

SO3 SO3::Exp(const V3 &ang) {
  scalar norm = ang.norm();
  M3 I = Eye3;
  if (norm > 1e-10) {
    V3 r_ang = ang / norm;
    M3 K;
    K = hat(r_ang);
    return SO3(I + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K);
  } else {
    return SO3(I);
  }
}


M3 SO3::Exp_m3(const V3 &ang_vel, const scalar &dt){
  static const scalar coefficient[] = {1.0/6.0, 1.0/24.0, 1.0/120.0, 1.0/720.0};

  V3 phi = ang_vel * dt;  
  scalar squaredNorm = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
  scalar norm = std::sqrt(squaredNorm);
  scalar A, B;
  if (squaredNorm >= 1e-6) {
      A = sin(norm) / norm;
      B = (1.0 - cos(norm)) / squaredNorm;
  } else {
      // 小角稳定泰勒展开到 θ^4
      // A ≈ 1 - θ^2/6 + θ^4/120
      // B ≈ 1/2 - θ^2/24 + θ^4/720
      const scalar t4 = squaredNorm * squaredNorm;
      A = 1.0 - (squaredNorm * coefficient[0]) + (t4 * coefficient[2]);
      B = 0.5 - (squaredNorm * coefficient[1]) + (t4 * coefficient[3]);
  }

  M3 K;
  K << 0.0, -phi[2], phi[1], phi[2], 0.0, -phi[0], -phi[1], phi[0], 0.0;
  return M3::Identity() + A * K + B * K * K;
}


SO3 SO3::Exp(const V3 &ang_vel, const scalar &dt) {
  scalar ang_vel_norm = ang_vel.norm();
  M3 I = Eye3;
  if (ang_vel_norm > 1e-12) {
    V3 r_axis = ang_vel / ang_vel_norm;
    M3 K;
    K = hat(r_axis);
    scalar r_ang = ang_vel_norm * dt;
    return SO3(I + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K);
  } else {
    return SO3(I);
  }
}

SO3& SO3::operator=(const SO3& rhs){
  if (this == &rhs)
      return *this;
  R_ = rhs.R();
  return *this;
}

SO3 SO3::operator*(const SO3& rhs) const noexcept{
  M3 res = R_ * rhs.R();
  return SO3(res);
}

SO3 SO3::operator*(scalar s) const noexcept{
  M3 res = R_ * s;
  return SO3(res);
}

V3 SO3::operator*(const V3& rhs) const noexcept{
  return R_ * rhs;
}

void SO3::update(const V3 &dw){
  SO3 dR(dw);
  R_ = dR.R() * R_;
}

void SO3::updateRhs(const V3 &dw){
  SO3 dR(dw);
  R_ = R_ * dR.R();
}

void SO3::exp(const M3 &w_hat){
  V3 w = vee(w_hat);
  double o = w.norm();
  if ( o < 1e-12){
      R_ << Eye3 + w_hat;
      return;
  }
  double c1 = std::sin(o)/o;
  double c2 = (1 - std::cos(o))/o/o;
  R_ << Eye3 + c1 * w_hat + c2 * w_hat *w_hat;
}

V4 SO3::coeffs() const noexcept{
  return Quat(R_).coeffs();
}

M3 SO3::log(double *ro) const noexcept{
  M3 res;
  double tr = (R_.trace()-1)*0.5;
  double o;
  if (tr  < 1.0 - 1e-9 && tr > -1.0 + 1e-9 )
  {
    o = std::fabs(std::acos(tr));
    res << 0.5 * o / std::sin(o) * ( R_ - R_.transpose());
  }
  else if (tr >= 1.0 - 1e-9 )
  {
    o = 0.0;
    res << M3::Zero();
  }
  else
  {
    o = M_PI;
    V3 w;
    if( R_(0,0) > R_(1,1) && R_(0,0) > R_(2,2) )
    {
      w << R_(0,0) + 1.0,
          0.5 * ( R_(0,1) + R_(1,0)),
          0.5 * ( R_(0,2) + R_(2,0));
    }
    else if( R_(1,1) > R_(0,0) && R_(1,1) > R_(2,2) )
    {
      w << 0.5 * ( R_(1,0) + R_(0,1)),
            R_(1,1) + 1.0,
            0.5 * ( R_(1,2) + R_(2,1));
    }
    else
    {
      w << 0.5 * ( R_(2,0) + R_(0,2)),
            0.5 * ( R_(2,1) + R_(1,2)),
            R_(2,2) + 1.0;
    }
    double length = w.norm();
    if (length > 0.0)
    {
      w *= M_PI / length;
    }
    else
    {
      w << 0.0, 0.0, 0.0;
    }
    res = hat(w);
  }
  if (ro != nullptr) *ro = o;
  return res;
}

V3 SO3::log_vee() const noexcept{
  M3 w_hat = this->log();
  return vee(w_hat);
}

SO3 SO3::inverse(void) const noexcept{
  return SO3(R_.transpose());
}

M3 SO3::adjoint() const noexcept{
  return R_.transpose();
}

M3 SO3::R() const noexcept{
  return R_;
}

Quat SO3::quaternion() const noexcept{
  Quat res = Quat(R_);
  return res.normalized();
}

M3 SO3::matrix() const noexcept{
  return R_;
}

M3& SO3::ref2R(){
  return R_;
}

scalar SO3::distance(const SO3 &rhs) const noexcept{
  return (*this * rhs.inverse()).log_vee().norm();
}

void SO3::print(void) const noexcept{
  std::cout << R_ << std::endl;
}

void SO3::print_lie(void) const noexcept{
  V3 w =  this->log_vee();
  std::cout << w << std::endl;
}

scalar SO3::yaw() const noexcept{
  scalar sy = sqrt(R_(0, 0) * R_(0, 0) + R_(1, 0) * R_(1, 0));
  if(sy < 1e-6) 
    return 0;
  else
  return atan2(R_(1, 0), R_(0, 0));
}

////////////////////////////////////////////////////////////////////////
SE3::SE3(const M4 &T): T_(T) {
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}

SE3::SE3(const V6 &xi){
  this->exp(hat6(xi));
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}
  
SE3::SE3(const SE3 &T): T_(T.T()),R_(T.R()),t_(T.t()){}

SE3::SE3(const SO3 &R, const V3 &t):
T_(M4::Identity())
{
  T_.block<3,3>(0,0) = R.R();
  T_.block<3,1>(0,3) = t;
  R_ = R.R();
  t_ = t;
}

SE3::SE3(const M3 &R, const V3 &t):
T_(M4::Identity())
{
  T_.block<3,3>(0,0) = R;
  T_.block<3,1>(0,3) = t;
  R_ = R;
  t_ = t;
}

SE3::SE3(const Quat &q, const V3 &t):
T_(M4::Identity())
{
  T_.block<3,3>(0,0) = q.toRotationMatrix();
  T_.block<3,1>(0,3) = t;
  R_ = T_.topLeftCorner<3,3>();
  t_ = t;
}

SE3& SE3::operator=(const SE3& rhs){
  if (this == &rhs)
      return *this;
  T_ = rhs.T();
  R_ = rhs.R();
  t_ = rhs.t();
  return *this;
}

SE3 SE3::operator*(const SE3& rhs) const noexcept{
  M4 res = T_ * rhs.T();
  return SE3(res);
}

V3 SE3::operator*(const V3& point) const noexcept{
  return R_*point + t_;
}

V4 SE3::operator*(const V4& point) const noexcept{
  return T_ * point;
}

void SE3::update(const V6 &dxi){
  SE3 dT(dxi);
  T_ = dT.T() * T_;
}

void SE3::updateRhs(const V6 &dxi){
  SE3 dT(dxi);
  T_ = T_ * dT.T();
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}

void SE3::exp(const M4 &xi_hat){
  V6 xi = vee6(xi_hat);
  V3 w = xi.head<3>();
  V3 v = xi.tail<3>();
  SO3 rotation(w);
  M3 w_hat = xi_hat.topLeftCorner<3,3>();

  M3 V = M3::Identity();
  double o = w.norm();
  if ( o > 1e-12){
    double c2 = (1 - std::cos(o))/o/o;
    double c3 = (o - std::sin(o))/o/o/o;
    V += c2*w_hat + c3*w_hat*w_hat;
  }
  V3 t = V * v;

  T_  << rotation.R(), t,
          0,0,0,1;
  R_ = rotation.R();
  t_ = t;
}

M4 SE3::log(void) const noexcept{
  SO3 rotation(this->R());
  double o;
  M3 w_hat = rotation.log(&o);
  M3 Vinv = M3::Identity();
  if (o > 1e-12)
  {
    double c1 = std::sin(o);
    double c2 = (1 - std::cos(o))/o;
    double k1 = 1/o/o*(1 - 0.5*c1/c2);
    Vinv += -0.5*w_hat + k1* w_hat*w_hat;
  }
  V3 v = Vinv * T_.topRightCorner<3,1>();

  M4 xi_hat = M4::Zero();
  xi_hat << w_hat, v,
            0,0,0,0;
  return xi_hat;
}

V6 SE3::log_vee() const noexcept{
  M4 xi_hat = this->log();
  return vee6(xi_hat);
}

V3 SE3::transform(const V3 & p) const noexcept{
  return R_*p + t_;
}

SE3 SE3::inverse(void) const noexcept{
  M4 inv;
  M3 R = R_;
  R.transposeInPlace();
  inv << R, -R * this->t(),
          0,0,0,1;
  return SE3(inv);
}

M6 SE3::adjoint() const noexcept{
  M6 res(M6::Zero());
  M3 tx = SO3::hat( this->t() );
  res.topLeftCorner<3,3>() << R();
  res.bottomRightCorner<3,3>() << R();
  res.bottomLeftCorner<3,3>() << tx*R();
  return res;
}

M4 SE3::T() const noexcept{
  return T_;
}

M4 SE3::matrix() const noexcept{
  return T_;
}

M4& SE3::ref2T(){
  return T_;
}

SO3 SE3::so3() const noexcept{
  return SO3(R_);
}

M3 SE3::R() const noexcept{
  return R_;
}

V3 SE3::translation() const noexcept{
  return t_;
}

V3 SE3::t() const noexcept{
  return t_;
}

Quat SE3::quaternion() const noexcept{
  Quat res = Quat(R());
  return res.normalized();
}

double SE3::distance(const SE3 &rhs) const noexcept{
  return (*this * rhs.inverse()).log_vee().norm();
}

void SE3::print(void) const noexcept{
  std::cout << T_ << std::endl;
}

void SE3::print_lie(void) const noexcept{
  std::cout << this->log_vee() << std::endl;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

S2::S2(scalar length, V3 vec){
  length_ = length;
  vec_ = length_ * vec.normalized();
}

S2::S2(scalar length, scalar d1, scalar d2, scalar d3){
  V3 input = V3(d1, d2, d3);
  length_ = length;
  vec_ = length_ * input.normalized();
}


void S2::S2_Bx(M3_2& Bx) {
  V3 d = vec_.normalized();
  V3 v = (std::fabs(d.x()) < 0.9) ? V3(1, 0, 0) : V3(0, 1, 0);
  V3 b1 = (v - d * (d.dot(v))).normalized();
  V3 b2 = d.cross(b1);
  Bx.col(0) = b1;
  Bx.col(1) = b2;
}


void S2::S2_Mx(M3_2 &Mx, const V2 &delta){
  M3_2 Bx;
  S2_Bx(Bx);
  if (delta.norm() < tolerance<scalar>()) {
    Mx = - SO3::hat(vec_) * Bx;
  } else {
    V3 Bu = Bx * delta; 
    SO3 exp_delta = SO3::Exp(Bu);
    Mx = - exp_delta.R_ * SO3::hat(vec_) * A_matrix(Bu).transpose() * Bx;
  }
}


void S2::S2_Nx_yy(M2_3 &Nx) {
  M3_2 Bx;
  S2_Bx(Bx);
  Nx = 1 / length_ / length_ * Bx.transpose() * SO3::hat(vec_);
}


void S2::S2_hat(M3 &res) {
  M3 skew_vec;
  skew_vec << scalar(0), -vec_[2], vec_[1], vec_[2], scalar(0), -vec_[0], -vec_[1], vec_[0], scalar(0);
  res = skew_vec;
}


void S2::update (const V2& delta){
  M3_2 Bx;
  S2_Bx(Bx);
  V3 Bu = Bx * delta;
  vec_ = SO3::Exp(Bu).R_ * vec_;
}

}
