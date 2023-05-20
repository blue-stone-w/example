#include <iostream>

#include <Eigen/Geometry>

#include <pcl/common/eigen.h>

int main(int argc, char **argv)
{
  float x = 0.28, y = -0.25, z = -0.58, roll = 0.07, pitch = 0.1, yaw = 0.066;
  Eigen::Affine3f aff = pcl::getTransformation(x, y, z, roll, pitch, yaw);
  Eigen::Vector4f curPlane(0, 0, 1, 1);

  auto p1 = cos(yaw) * cos(pitch) * curPlane[0]
            + (sin(yaw) * cos(pitch)) * curPlane[1]
            + (-sin(pitch)) * curPlane[2];

  auto p2 = (cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll)) * curPlane[0]
            + (cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll)) * curPlane[1]
            + (cos(pitch) * sin(roll)) * curPlane[2];

  auto p3 = (sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)) * curPlane[0]
            + (sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)) * curPlane[1]
            + (cos(pitch) * cos(roll)) * curPlane[2];
  auto p4 = curPlane[0] * x + curPlane[1] * y + curPlane[2] * z + 1;

  Eigen::Vector4f pt1(p1, p2, p3, p4);
  Eigen::Matrix<float, 1, 4> pt2 = curPlane.transpose() * aff.matrix();

  std::cout << pt1[0] << " -------------- " << pt2[0] << std::endl;
  std::cout << pt1[1] << " -------------- " << pt2[1] << std::endl;
  std::cout << pt1[2] << " -------------- " << pt2[2] << std::endl;
  std::cout << pt1[3] << " -------------- " << pt2[3] << std::endl;

  if (std::abs(p1 - pt2[0]) < 0.0001)
  {
    std::cout << "111111111111111111111" << std::endl;
  }
  if (std::abs(p2 - pt2[1]) < 0.0001)
  {
    std::cout << "222222222222222222222" << std::endl;
  }
  if (std::abs(p3 - pt2[2]) < 0.0001)
  {
    std::cout << "333333333333333333333" << std::endl;
  }

  return 0;
}
