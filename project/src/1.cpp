#include "libmy/mathu.h"

#include "lib3rd/cpp_header.h"

int main()
{
  std::vector<float> vec{0};
  Eigen::Transform<float, 3, Eigen::Affine> aff;
  MyMath::vec6D2Aff(vec, aff);
  std::cout << "op" << std::endl;
  return 0;
}
