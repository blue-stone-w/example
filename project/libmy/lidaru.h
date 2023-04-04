
#ifndef LIDAR_HEADER_H_
#define LIDAR_HEADER_H_

#include "lib3rd/cpp_header.h"
#include "lib3rd/pcl_header.h"

struct LidarParam
{
  std::string lidarID;
  std::string frame;
  int horizonScan, verticalScan;
  float angleResolution;
  int downsampleRate;
  float lidarMinRange, lidarMaxRange;
  Eigen::Affine3d lidar2truckAff, truck2lidarAff;
  std::string lidarTopic;
};

#endif