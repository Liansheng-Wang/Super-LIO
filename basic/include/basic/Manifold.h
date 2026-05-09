#ifndef Manifold_MATH_H_H_
#define Manifold_MATH_H_H_


#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "alias.h"


namespace BASIC {

template <class scalar>
inline scalar tolerance();

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> SKEW(const Eigen::MatrixBase<Derived> &v);

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang);

// left Jacobian
template <typename T>
Eigen::Matrix<T, 3, 3> Gamma_1(const Eigen::Matrix<T, 3, 1>& ang);

// Second-order differential variable
template <typename T>
Eigen::Matrix<T, 3, 3> Gamma_2(const Eigen::Matrix<T, 3, 1>& ang);

//// SO(3) left Jacobian
M3 A_matrix(const V3& v);

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt);

template <typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3);


/* Logrithm of a Rotation Matrix */
template <typename T>
Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R);

template <typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot);

M4 hat6(const V6 &xi);

V6 vee6(const M4 &xi_hat);


/**
 * SO3 and SE3 ref to https://github.com/hku-mars/BALM/blob/master/src/compare_test/SE3/SE3.hpp
 */
class SO3
{
public:
  SO3(const M3 &R = Eye3);

  SO3(const V3 &w);

  SO3(const SO3 &R);

  template<typename OtherDerived>
  SO3(const Eigen::MatrixBase<OtherDerived>& rhs):
  R_(rhs)
  {}

  static M3 hat(const V3 &v);

  static V3 vee(const M3 &w_hat);

  static SO3 Exp(const V3 &ang);

  static SO3 Exp(const V3 &ang_vel, const scalar &dt);

  static M3 Exp_m3(const V3 &ang_vel, const scalar &dt);

  SO3& operator=(const SO3& rhs);

  SO3 operator*(const SO3& rhs) const noexcept;

  SO3 operator*(scalar s) const noexcept;

  V3 operator*(const V3& rhs) const noexcept;

  template<typename OtherDerived>
  V3 operator*(const Eigen::MatrixBase<OtherDerived>& rhs) const noexcept{
    return R_ * rhs;
  }

  void update(const V3 &dw);

  void updateRhs(const V3 &dw);

  void exp(const M3 &w_hat);

  V4 coeffs() const noexcept;

  M3 log(double *ro = nullptr) const noexcept;

  V3 log_vee() const noexcept;

  SO3 inverse(void) const noexcept;

  M3 adjoint() const noexcept;

  M3 R() const noexcept;

  Quat quaternion() const noexcept;

  M3 matrix() const noexcept;

  M3& ref2R();

  scalar distance(const SO3 &rhs) const noexcept;

  void print(void) const noexcept;

  void print_lie(void) const noexcept;

  scalar yaw() const noexcept;

public:
  M3 R_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class SE3
{
public:
  SE3() : T_(M4::Identity()), R_(M3::Identity()), t_(V3::Zero()) {}

  SE3(const M4 &T);

  SE3(const V6 &xi);
  
  SE3(const SE3 &T);

  SE3(const SO3 &R, const V3 &t);

  SE3(const M3 &R, const V3 &t);

  SE3(const Quat &q, const V3 &t);

  template<typename OtherDerived>
  SE3(const Eigen::MatrixBase<OtherDerived>& rhs)  :
  T_(rhs)
  {}

  SE3& operator=(const SE3& rhs);

  SE3 operator*(const SE3& rhs) const noexcept;

  V3  operator*(const V3& point) const noexcept;

  V4  operator*(const V4& point) const noexcept;
  
  void update(const V6 &dxi);

  void updateRhs(const V6 &dxi);

  void exp(const M4 &xi_hat);

  M4 log(void) const noexcept;

  V6 log_vee() const noexcept;

  V3 transform(const V3 & p) const noexcept;

  SE3 inverse(void) const noexcept;

  M6 adjoint() const noexcept;

  M4 T() const noexcept;

  M4 matrix() const noexcept;

  M4& ref2T();

  SO3 so3() const noexcept;

  M3 R() const noexcept;

  V3 translation() const noexcept;

  V3 t() const noexcept;

  Quat quaternion() const noexcept;

  double distance(const SE3 &rhs) const noexcept;

  void print(void) const noexcept;

  void print_lie(void) const noexcept;

public:
  M4 T_;
  M3 R_;
  V3 t_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/// only can be used for gravity vector.
class S2
{
public:
  V3 vec_ = V3(0, 0, 1);
  scalar length_ = 1;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  S2() = default;

  S2(scalar length, V3 = V3(0, 0, 1));
    
  S2(scalar length, scalar d1, scalar d2, scalar d3);

  /// Construct the local basis matrix on S² (tangent space basis).
  void S2_Bx(M3_2& Bx);

  /// Used to compute Jacobians under perturbations in the tangent space.
  void S2_Mx(M3_2 &Mx, const V2 &delta);

  void S2_Nx_yy(M2_3 &Nx);

  void S2_hat(M3 &res);
  
  void update (const V2& delta);

};

}

#endif