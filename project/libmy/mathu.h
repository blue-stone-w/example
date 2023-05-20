#ifndef MY_MATH
#define MY_MATH

#include "lib3rd/cpp_header.h"

#include <Eigen/Dense>

namespace MyMath
{
/* constant */
constexpr static double degTorad = M_PI / 180, radTodeg = 180 / M_PI;

// auto p1 = cos(yaw) * cos(pitch) * curPlane[0]
//           + (cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll)) * curPlane[1]
//           + (sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)) * curPlane[2]
//           + x;
// auto p2 = sin(yaw) * cos(pitch) * curPlane[0]
//           + (cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll)) * curPlane[1]
//           + (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)) * curPlane[2]
//           + y;
// auto p3 = -sin(pitch) * curPlane[0]
//           + cos(pitch) * sin(roll) * curPlane[1]
//           + cos(pitch) * cos(roll) * curPlane[2]
//           + z;

/******** vector6d(xyzrpy)  aff3d ********/
template <typename S1, typename S2>
void vec6D2Aff(std::vector<S1> poseV6, Eigen::Transform<S2, 3, Eigen::Affine> &poseA3)
{
  // cos(yaw)*cos(pitch) cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll) sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll)
  // sin(yaw)*cos(pitch) cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll) sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)
  // -sin(pitch)         cos(pitch)*sin(roll)                             cos(pitch)*cos(roll)

  // poseA3(0, 0) = A * C; poseA3(0, 1) = A * DF - B * E; poseA3(0, 2) = B * F + A * DE; poseA3(0, 3) = poseV6[0];
  //
  // poseA3(1, 0) = B * C; poseA3(1, 1) = A * E + B * DF; poseA3(1, 2) = B * DE - A * F; poseA3(1, 3) = poseV6[1];
  //
  // poseA3(2, 0) = -D;    poseA3(2, 1) = C * F;          poseA3(2, 2) = C * E;          poseA3(2, 3) = poseV6[2];
  //
  // poseA3(3, 0) = 0;     poseA3(3, 1) = 0;              poseA3(3, 2) = 0;              poseA3(3, 3) = 1;

  auto A = cos(poseV6[5]), B = sin(poseV6[5]), C = cos(poseV6[4]), D = sin(poseV6[4]),
       E = cos(poseV6[3]), F = sin(poseV6[3]), DE = D * E, DF = D * F;

  poseA3(0, 0) = A * C;
  poseA3(0, 1) = A * DF - B * E;
  poseA3(0, 2) = B * F + A * DE;
  poseA3(0, 3) = poseV6[0];

  poseA3(1, 0) = B * C;
  poseA3(1, 1) = A * E + B * DF;
  poseA3(1, 2) = B * DE - A * F;
  poseA3(1, 3) = poseV6[1];

  poseA3(2, 0) = -D;
  poseA3(2, 1) = C * F;
  poseA3(2, 2) = C * E;
  poseA3(2, 3) = poseV6[2];

  poseA3(3, 0) = 0;
  poseA3(3, 1) = 0;
  poseA3(3, 2) = 0;
  poseA3(3, 3) = 1;
}

template <typename S1, typename S2>
static void aff2Pose6D(Eigen::Transform<S1, 3, Eigen::Affine> &poseA3, std::vector<S2> &poseV6)
{
  poseV6[0] = poseA3(0, 3);
  poseV6[1] = poseA3(1, 3);
  poseV6[2] = poseA3(2, 3);
  poseV6[3] = atan2(poseA3(2, 1), poseA3(2, 2));
  poseV6[4] = asin(-poseA3(2, 0));
  poseV6[5] = atan2(poseA3(1, 0), poseA3(0, 0));
}

template <typename S1, typename S2, typename S3>
static Eigen::Quaternion<S1> axis2Quat(const Eigen::Matrix<S2, 3, 1> &axis, S3 theta)
{
  Eigen::Quaternion<S1> q;

  if (theta < 1e-10)
  {
    q.w() = 1.0;
    q.x() = q.y() = q.z() = 0; // why not return q here?
  }

  double magnitude = sin(theta / 2.0f);

  q.w() = cos(theta / 2.0f);
  q.x() = axis(0) * magnitude;
  q.y() = axis(1) * magnitude;
  q.z() = axis(2) * magnitude;

  return q;
}
template <typename S1, typename S2, typename S3>
static Eigen::Quaternion<S1> vec2Quat(const Eigen::Matrix<S2, 3, 1> &vec)
{
  Eigen::Quaternion<S1> q;
  S3 theta = vec.norm();

  if (theta < 1e-10)
  {
    q.w() = 1.0;
    q.x() = q.y() = q.z() = 0;
    return q;
  }

  Eigen::Matrix<S2, 3, 1> tmp = vec / theta;
  return axis2Quat(tmp, theta);
}

template <typename S1>
static void enforceSymmetry(Eigen::Matrix<S1, Eigen::Dynamic, Eigen::Dynamic> &mat)
{
  // 自己对自己进行赋值的时候，先把结果存到临时变量,避免Eigen中的混叠（aliasing）问题
  mat = 0.5 * (mat + mat.transpose()).eval();
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> rpy2R(const Eigen::MatrixBase<Derived> &rpy)
{
  typedef typename Derived::Scalar Scalar_t;

  Scalar_t r = rpy(0);
  Scalar_t p = rpy(1);
  Scalar_t y = rpy(2);

  Eigen::Matrix<Scalar_t, 3, 3> Rz;
  Rz << cos(y), -sin(y), 0, sin(y), cos(y), 0, 0, 0, 1;
  Eigen::Matrix<Scalar_t, 3, 3> Ry;
  Ry << cos(p), 0., sin(p), 0., 1., 0., -sin(p), 0., cos(p);
  Eigen::Matrix<Scalar_t, 3, 3> Rx;
  Rx << 1., 0., 0., 0., cos(r), -sin(r), 0., sin(r), cos(r);

  return Rz * Ry * Rx;
}
template <typename S1, typename S2>
static Eigen::Matrix<S1, 3, 1> R2rpy(const Eigen::Matrix3d &R)
{
  Eigen::Matrix<S1, 3, 1> rpy;
  rpy(1) = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  rpy(0) = atan2(R(2, 1) / cos(rpy(1)), R(2, 2) / cos(rpy(1))); // roll
  rpy(2) = atan2(R(1, 0) / cos(rpy(1)), R(0, 0) / cos(rpy(1))); // yaw
  return rpy;
}
template <typename S1, typename S2>
static Eigen::Matrix<S1, 3, 1> Quternion2rpy(const Eigen::Quaternion<S2> &Q)
{
  return R2rpy(Q.toRotationMatrix());
}
template <typename S1, typename S2>
static Eigen::Quaternion<S1> rpy2Quat(const Eigen::Matrix<S2, 3, 1> &rpy)
{
  double halfYaw   = double(rpy(2)) * double(0.5);
  double halfPitch = double(rpy(1)) * double(0.5);
  double halfRoll  = double(rpy(0)) * double(0.5);
  double cosYaw    = cos(halfYaw);
  double sinYaw    = sin(halfYaw);
  double cosPitch  = cos(halfPitch);
  double sinPitch  = sin(halfPitch);
  double cosRoll   = cos(halfRoll);
  double sinRoll   = sin(halfRoll);
  Eigen::Quaternion<S1> Q;
  Q.x() = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  Q.y() = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  Q.z() = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  Q.w() = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  Q.normalized();
  return Q;
}
template <typename S1, typename S2>
static Eigen::Matrix<S1, 3, 1> QuaternionToEuler(Eigen::Quaternion<S2, 0> quat)
{
  double roll, yaw, pitch;
  double qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();
  yaw   = atan2f(2.f * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qx * qx)); // Z
  roll  = asinf(2.f * (qw * qx - qy * qz));                               // Y
  pitch = atan2f(2.f * (qw * qy + qz * qx), 1 - 2 * (qy * qy + qx * qx)); // X

  return Eigen::Matrix<S1, 3, 1>(roll, pitch, yaw);
}

/********  ???  ********/
// 向量的反对称阵
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skew(const Eigen::MatrixBase<Derived> &q)
{
  Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
  ans << typename Derived::Scalar(0), -q(2), q(1),
      q(2), typename Derived::Scalar(0), -q(0),
      -q(1), q(0), typename Derived::Scalar(0);
  return ans; // 反对称矩阵
  /****************
  ans =
  0   -q2   q1
  q2   0   -q0
  -q1   q0   0
  ****************/
}

template <typename Derived>
// ensure rotate in same direction???
static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
{
  // printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
  Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z()); // useless code???
  // printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
  return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
  // return q;
}

//???
template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
  typedef typename Derived::Scalar Scalar_t;
  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

template <class T>
static T rad2deg(const T &radians)
{
  return radians * radTodeg;
}

template <class T>
static T deg2rad(const T &degrees)
{
  return degrees * degTorad;
}

template <class Derived>
static Eigen::MatrixBase<Derived> rad2deg(const Eigen::MatrixBase<Derived> radians)
{
  return Eigen::MatrixBase<Derived>(rad2deg(radians(0)), rad2deg(radians(1)), rad2deg(radians(2)));
}

// deg vector
template <class Derived>
static Eigen::MatrixBase<Derived> deg2rag(const Eigen::MatrixBase<Derived> degrees)
{
  return Eigen::MatrixBase<Derived>(deg2rad(degrees(0)), deg2rad(degrees(1)), deg2rad(degrees(2)));
}

//???
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
{
  Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
  ans.template block<3, 1>(1, 0)     = qq.vec(), // (1, 0)???
      ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skew(qq.vec());
  return ans;
  /****************
  ans =
  w    -q0     -q1    -q2
  q0   w      -wq2    wq1
  q1   wq2     w      -wq0
  q2  -wq1    wq0      w
  ****************/
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
{
  Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
  Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
  ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
  ans.template block<3, 1>(1, 0)                             = pp.vec(),
                              ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skew(pp.vec());
  return ans;
  /****************
  ans =
  w    -q0     -q1    -q2
  q0    w       wq2   -wq1
  q1   -wq2     w      wq0
  q2    wq1    -wq0    w
  ****************/
}

//???
template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> Rleft(const Eigen::MatrixBase<Derived> axis)
{
  typedef typename Derived::Scalar Scalar_t;
  Eigen::Matrix<Scalar_t, 3, 3> ans;
  Scalar_t theta                  = axis.norm();              // axis is V3D
  Eigen::Matrix<Scalar_t, 3, 1> a = axis / theta;             // a is axis's unit vector
  double s                        = sin(theta) / theta;       //???
  double c                        = (1 - cos(theta)) / theta; //???
  ans                             = s * Eigen::Matrix<Scalar_t, 3, 3>::Identity() + (1 - s) * a * a.transpose() + c * skew(a);
}
template <typename S1, typename S2>
static Eigen::Matrix<S1, 3, 3> Rinvleft(const Eigen::Matrix<S2, 3, 1> axis)
{
  Eigen::Matrix<S1, 3, 3> ans;
  double theta = axis.norm();

  if (theta < 1e-10)
  {
    ans.setIdentity();
    return ans;
  }

  double half_theta             = theta / 2.0;
  Eigen::Matrix<double, 3, 1> a = axis / axis.norm(); // unit vector of axis
  double cot_half_theta         = cos(half_theta) / sin(half_theta);
  double s                      = half_theta * cot_half_theta;
  ans                           = s * Eigen::Matrix<S1, 3, 3>::Identity() + (1.0 - s) * a * a.transpose() - half_theta * skew(a);

  return ans;
}

template <class T>
T radInRange(T radians)
{
  while (radians < -M_PI)
  {
    radians += 2 * M_PI;
  }
  while (radians > M_PI)
  {
    radians -= 2 * M_PI;
  }
  return radians;
}
} // namespace MyMath

#endif
