#include <iostream>
#include <pcl/io/pcl_io.h>//PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>//PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件

int main(int argc, char** argv){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl:PointXYZ>)// 创建点云（指针）

	if(pcl::io::loadPCDFile<PointXYZ>("model.pcd", *cloud)==-1){ //* 读入PCD格式的文件，如果文件不存在，返回-1
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return -1;
	}
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");//直接创造一个显示窗口
	viewer.showCloud(cloud);//在这个窗口显示点云
	while(!view.wasStopped)(){
	}
	return 0;
}
