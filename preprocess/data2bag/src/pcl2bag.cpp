#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "io_tools.h"

using namespace hx_slam::io;

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT>::Ptr PointPtr;

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh("~");

    std::string root_dir;
    double start_time = -1;
    double end_time = -1;

    if (!nh.getParam("root_dir", root_dir)) {
        ROS_ERROR("Failed to get parameter 'root_dir'");
        return 1;
    }

    if (!nh.getParam("start_time", start_time)) {
        ROS_ERROR("Failed to get parameter 'start_time'");
        return 1;
    }

    if (!nh.getParam("end_time", end_time)) {
        ROS_WARN("Failed to get parameter 'end_time', defaulting to -1");
        end_time = -1;  // 设置默认值
    }

    ROS_INFO("Root Directory: %s", root_dir.c_str());
    ROS_INFO("Start Time: %.9f", start_time);
    ROS_INFO("End Time: %.9f", end_time);

    std::string horizon_lidar_path = root_dir + "/horizon_lidar/";
    ros::Publisher pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/horizon_lidar/data", 100);
    ros::Rate loopRate(10);

    std::vector<std::string> lidar_paths = GetFileList(horizon_lidar_path);

    for (size_t i = 0; i < lidar_paths.size(); i++) {
        std::vector<std::string> spilts = StringSplit(lidar_paths[i], "/");
        std::string timestamp_str = StringSplit(spilts.back(), ".").front();
        std::stringstream ss; ss << timestamp_str;
        double timestamp; ss >> timestamp;

        timestamp = timestamp * 1e-9;

        if (start_time > 0 && timestamp < start_time) {
            continue;
        }

        if (end_time > 0 && timestamp >= end_time) {
            break;
        }

        pcl::PointCloud<velodyne_ros::Point> pl_orig;

        PointPtr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile(lidar_paths[i], *cloud) == -1) {
            std::cout << "点云数据读取失败: "<< lidar_paths[i] << std::endl;
            continue;
        }

        std::cout << "(" << i << "/" << lidar_paths.size() << ") " << lidar_paths[i] << " loading" << std::endl;

        for(int index=0; index < cloud->points.size(); index++){
            velodyne_ros::Point added_pt;
            added_pt.x = cloud->points[index].x;
            added_pt.y = cloud->points[index].y;
            added_pt.z = cloud->points[index].z;

            added_pt.intensity = cloud->points[index].intensity;
            auto tmp_cur =  cloud->points[index].curvature;
            added_pt.time = (cloud->points[index].curvature-static_cast<int>(tmp_cur)) * 10.0;

            pl_orig.points.push_back(added_pt);
            added_pt.ring = 1;
        }

        sensor_msgs::PointCloud2 laser_cloud;
        pcl::toROSMsg(pl_orig, laser_cloud);

        laser_cloud.header.stamp = ros::Time(timestamp);
        laser_cloud.header.frame_id = "base_link";

        pcd_pub.publish(laser_cloud);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}