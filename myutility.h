#ifndef MYUTILITY_H
#define MYUTILITY_H

/*
1. include utility header files
2. LidarParam for machanical lidar
3. point: a example about how to registere a point
5. TicToc, Timer
6. function feedback
*/

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv/cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


#include "glog/logging.h"

// custom header
// #include "global_definition.h"

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

/******************************************************************************/
// 注册点云： User defined point structures can be registered using PCL macros.
/* A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp) */
struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;                 // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

// User defined point structures can be registered using PCL macros. http://wiki.ros.org/pcl_ros/cturtle
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw) (double, time, time))


/******************************************************************************/
class CloudInfo
{
 public:
  // Cloud Info
  std_msgs::Header header;

  std::vector<int> startRingIndex; // a horizontal scan is a ring
  std::vector<int> endRingIndex;

  pcl::PointCloud<PointXYZIRT> cloudDeskewed;  // original deskewed cloud 
  pcl::PointCloud<PointType>   cloudEdge    ;  // extracted edge feature
  pcl::PointCloud<PointType>   cloudSurf    ;  // extracted surface feature
};


/******************************************************************************/
class TicToc {
 public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::system_clock::now();
  }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

class Timer {
 public:
  Timer() {
    restart();
  }

  // 重置计时器 reset timer
  void restart() {
    start_time = std::chrono::steady_clock::now();
  }

  // 结束计时,返回计时结果，单位为毫秒
  double elapsed(bool restart = false) {
    end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    if(restart) {
      this->restart();
    }
    return diff.count()*1000;
  }
 private:
  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point end_time  ;
};

// 在即将离开该函数返回上层调用时创建并写入相应信息
// 
enum ResultType { INFO, WARN, ERROR };
struct PerformResult {
  ResultType type = ResultType::INFO;       // 执行结果的类型
  std::string callChain; // 执行结果的接收地到发布地的整个链条(从上至下)
  std::string detail = ": ";    // 在产生执行结果的位置 添加 执行结果的详细情况
};
struct PerformResults {
  bool normal = true; // true means task can be performed(warn or ok); false means task wasn't performed and interrupted.
  std::deque<PerformResult> resultDeq; // 
};
// 执行结果合并的函数
static bool combinePerformResults(PerformResults& fatherResults, PerformResults childResults, std::string callChain) {
  fatherResults.normal = fatherResults.normal && childResults.normal;
  while(!childResults.resultDeq.empty()) {
    PerformResult result = childResults.resultDeq.front();
    result.callChain = callChain + result.callChain;
    fatherResults.resultDeq.push_back(result);
    childResults.resultDeq.pop_front();
  }
  return fatherResults.normal;
}

#endif
