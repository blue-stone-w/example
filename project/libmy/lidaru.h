
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


#ifndef POINT_HEADER_H_
#define POINT_HEADER_H_

#include "lib3rd/pcl_header.h"

/******************************************************************************/
// PointTypePose
// 注册点云： User defined point structures can be registered using PCL macros.
/* A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp) */
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                  // enforce SSE padding for correct memory alignment

// User defined point structures can be registered using PCL macros. http://wiki.ros.org/pcl_ros/cturtle
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))
typedef PointXYZIRPYT PointTypePose;
/******************************************************************************/
// ls c32 point
struct LSPointXYZIRT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  std::uint16_t ring; // 距离矩阵中的每一行为一个ring
  double time;
  // double distance;
  // double azimuth;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
// 注册点云： User defined point structures can be registered using PCL macros.
POINT_CLOUD_REGISTER_POINT_STRUCT(LSPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, time, time))
/******************************************************************************/
// ouster os0 32
struct OusterPointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  std::uint8_t ring;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)(std::uint8_t, ring, ring)(std::uint32_t, range, range))

#endif
