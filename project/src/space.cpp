// #include "lib3rd/pcl_header.h"
#include <pcl/common/eigen.h>

/**** make a vector vertical ****/
void getVerticalVec()
{
  Eigen::Vector3d nor0(0, 0, 1), nor1(0.01, -0.02, 0.97);
  nor1.normalize();
  Eigen::Quaternion<double> quat = Eigen::Quaterniond::FromTwoVectors(nor0, nor1);
  double qw = quat.w(), qx = quat.x(), qy = quat.y(), qz = quat.z();
  double roll, yaw, pitch;

  yaw   = atan2f(2.f * (qw * qz + qx * qy), 1 - 2 * (qz * qz + qx * qx)); // Z
  roll  = asinf(2.f * (qw * qx - qy * qz));                               // Y
  pitch = atan2f(2.f * (qw * qy + qz * qx), 1 - 2 * (qy * qy + qx * qx)); // X

  std::cout << "r:" << roll << "; "
            << "p:" << pitch << "; "
            << "y:" << yaw << "; " << std::endl;

  Eigen::Matrix<double, 3, 3> Rz;
  Rz << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  Eigen::Matrix<double, 3, 3> Ry;
  Ry << cos(pitch), 0., sin(pitch), 0., 1., 0., -sin(pitch), 0., cos(pitch);
  Eigen::Matrix<double, 3, 3> Rx;
  Rx << 1., 0., 0., 0., cos(roll), -sin(roll), 0., sin(roll), cos(roll);

  Eigen::Matrix3d rot = Ry * Rx;

  Eigen::Vector3d t = rot.inverse() * nor1;
  std::cout << t << std::endl;
}

void trans()
{
  Eigen::Affine3f l2t = pcl::getTransformation(7.3, 0.4, 0.0, 0.0, 0.0, 1.6).inverse();
  Eigen::Affine3f c2l = pcl::getTransformation(0.28, -0.25, -0.58, -0.07, 0.01, 0.066);

  Eigen::Affine3f aff0(Eigen::Matrix<float, 4, 4>::Zero()); // right to left
  aff0(0, 0) = -1;
  aff0(1, 2) = -1;
  aff0(2, 1) = -1;
  aff0(3, 3) = 1;

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 1367.4187138648322, 0.0, 981.879780704955,
      0.0, 1379.561837746044, 520.767366835325,
      0.0, 0.0, 1.0;
  std::cout << cameraMatrix(0, 2) << std::endl;

  Eigen::Affine3f cm(Eigen::Matrix<float, 4, 4>::Zero());
  cm.matrix().block<3, 3>(0, 0) = cameraMatrix;

  Eigen::Affine3f t2img = cm * aff0 * c2l * l2t;
}

class Merge
{};

int main(int argc, char **argv) { trans(); }
