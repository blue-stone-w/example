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

void vecRotation()
{
  Eigen::AngleAxisd rotation_vector1(M_PI / 4, Eigen::Vector3d(0, 0, 1));
  Eigen::Vector3d eulerAngle1 = rotation_vector1.matrix().eulerAngles(0, 1, 2);
  // std::cout << std::fixed << std::setprecision(5) << eulerAngle1[0] << ", " << eulerAngle1[1] << ", " << eulerAngle1[2] << ", " << std::endl;
}

void cvCameraRT2Euler()
{
  // cv::Mat cameraMatrix, distCoeffs, R, T;;
  Eigen::Vector3d R(3), T(3); // calaulate from cv function
  double angle = sqrt(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]);
  Eigen::Vector3d axis(R[0] / angle, R[1] / angle, R[2] / angle);
  Eigen::AngleAxisd rotation_vector1(angle, axis);
  Eigen::Vector3d rpy = rotation_vector1.matrix().eulerAngles(0, 1, 2);

  // std::cout << std::fixed << std::setprecision(5) << std::showpos
  //           << T.at<double>(i, 0) << ", " << T.at<double>(i, 1) << ", " << T.at<double>(i, 2) << "; "
  //           << rpy[0] << ", " << rpy[1] << ", " << rpy[2] << std::endl;
  Eigen::Affine3f aff;

  Eigen::VectorXd euler(6); // x,y,z,roll,pitch,yaw

  euler[0] = T[0];
  euler[1] = T[1];
  euler[2] = T[2];
  euler[3] = rpy[0];
  euler[4] = rpy[1];
  euler[5] = rpy[2];

  aff = pcl::getTransformation(euler[0], euler[1], euler[2], euler[3], euler[4], euler[5]);
  // world frame and camera frame are definited by cv
  // poseC = aff * poseW;
}

class Merge
{};

int main(int argc, char **argv) { trans(); }
