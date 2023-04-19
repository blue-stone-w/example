// main.cc
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
std::string src_bag    = "/home/xxx/DataSet/2022-03-17/2022-03-17.bag";
std::string new_bag    = "/home/xxx/DataSet/2022-03-17/imu_pcd.bag";
std::string imu_topic  = "/imu/data_raw";
std::string pcd2_topic = "/hesai/pandar";

int main(int argc, char **argv)
{
  rosbag::Bag i_bag, o_bag;
  i_bag.open(src_bag, rosbag::bagmode::Read);
  o_bag.open(new_bag, rosbag::bagmode::Write);
  std::vector<std::string> topics;
  topics.push_back(std::string(imu_topic));
  topics.push_back(std::string(pcd2_topic));
  rosbag::View view(i_bag, rosbag::TopicQuery(topics));
  // rosbag::View view(i_bag); // read all topics
  for (auto m : view)
  {
    // 得到该帧数据的topic
    std::string topic = m.getTopic();

    sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
    if (imu == nullptr)
    {
      std::cout << "imu null " << std::endl;
    }
    else
    {
      std::cout << "imu stamp:" << imu->header.stamp << std::endl;
      o_bag.write(imu_topic, imu->header.stamp, imu);
    }

    sensor_msgs::PointCloud2::ConstPtr pcd = m.instantiate<sensor_msgs::PointCloud2>();
    if (pcd == nullptr)
    {
      std::cout << "pcd null " << std::endl;
    }
    else
    {
      std::cout << "pcd stamp:" << pcd->header.stamp << std::endl;
      o_bag.write(pcd2_topic, pcd->header.stamp, pcd);
    }
  }
  i_bag.close();
  o_bag.close();
  return 0;
}
