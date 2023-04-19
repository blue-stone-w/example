#include <iostream>

#include <Eigen/Geometry>

#include <pcl/common/eigen.h>

int main(int argc, char **argv)
{
  Eigen::Affine3f l2t = pcl::getTransformation(7.3, 0.4, 0.0, 0.0, 0.0, 1.6).inverse();
  Eigen::Affine3f c2l = pcl::getTransformation(0.28, -0.25, -0.58, -0.07, 0.01, 0.066);

  Eigen::Affine3f aff0(Eigen::Matrix<float, 4, 4>::Zero()); // right to left
  aff0(0, 0) = -1;
  aff0(1, 2) = -1;
  aff0(2, 1) = -1;
  aff0(3, 3) = 1;

  std::cout << (aff0 * c2l * l2t).matrix() << std::endl;

  Eigen::Matrix3f cameraMatrix;
  cameraMatrix << 1367.4187138648322, 0.0, 981.879780704955,
      0.0, 1379.561837746044, 520.767366835325,
      0.0, 0.0, 1.0;
  std::cout << cameraMatrix(0, 2) << std::endl;

  Eigen::Affine3f cm(Eigen::Matrix<float, 4, 4>::Zero());
  cm.matrix().block<3, 3>(0, 0) = cameraMatrix;

  Eigen::Affine3f t2img = cm * aff0 * c2l * l2t;
  std::cout << t2img.matrix() << std::endl;

  Eigen::Vector4f pt1(15, 0, 0, 0);
  Eigen::Vector4f pt2;
  pt2 = t2img * pt1;
  pt2 /= pt2[2];
  Eigen::Vector4i pixel = pt2.cast<int>();

  std::cout << pixel << std::endl;


  return 0;
}
