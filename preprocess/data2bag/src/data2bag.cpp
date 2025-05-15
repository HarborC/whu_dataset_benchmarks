#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include "io_tools.h"

using namespace hx_slam::io;

void imu2Bag(rosbag::Bag& bag, std::string path, double start_time, double end_time);
void camera2Bag(rosbag::Bag& bag, std::string dir_name, double start_time, double end_time);
void lidar2Bag(rosbag::Bag& bag, std::string dir_name, double start_time, double end_time);
void gps2Bag(rosbag::Bag& bag, std::string path, double start_time, double end_time);

double G = 9.81;

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

int main(int argc, char** argv) {
    ros::init(argc, argv, "data2bag_node");
    ros::NodeHandle nh("~");

    std::string root_dir, save_dir;
    double start_time = -1;
    double end_time = -1;

    // 从参数服务器读取参数
    if (!nh.getParam("root_dir", root_dir)) {
        ROS_ERROR("Failed to get parameter 'root_dir'");
        return 1;
    }

    if (!nh.getParam("save_dir", save_dir)) {
        ROS_ERROR("Failed to get parameter 'save_dir'");
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

    // 输出读取的参数
    ROS_INFO("Root Directory: %s", root_dir.c_str());
    ROS_INFO("Save Directory: %s", save_dir.c_str());
    ROS_INFO("Start Time: %.9f", start_time);
    ROS_INFO("End Time: %.9f", end_time);

    int idx = 0;
    double duration = 300.0;
    while (start_time + idx * duration < end_time) {
        double s_t = start_time + idx * duration;
        double e_t = start_time + (idx + 1) * duration;
        if (end_time > 0 && e_t > end_time) {
            e_t = end_time;
        }

        std::string bag_path = save_dir + "/data" + std::to_string(idx) + ".bag";
        rosbag::Bag bag;
        bag.open(bag_path, rosbag::bagmode::Write);
        bag.setCompression(rosbag::compression::LZ4);

        std::cout << "start " << idx << std::endl;

        std::string imu_path = root_dir + "/horizon_imu.txt";
        imu2Bag(bag, imu_path, s_t, e_t);

        std::cout << "imu done" << std::endl;

        std::string gps_path = root_dir + "/gps.txt";
        gps2Bag(bag, gps_path, s_t, e_t);

        std::cout << "gps done" << std::endl;

        std::string horizon_lidar_path = root_dir + "/horizon_lidar/";
        lidar2Bag(bag, horizon_lidar_path, s_t, e_t);

        std::cout << "lidar done" << std::endl;

        std::string hkvison_image_path = root_dir + "/hkvison_image/";
        camera2Bag(bag, hkvison_image_path, s_t, e_t);

        std::cout << "camera done" << std::endl;

        bag.close();

        idx++;
    }
    
    return 0;
}

void imu2Bag(rosbag::Bag& bag, std::string path, double start_time, double end_time) {
    std::ifstream imu_file(path);
    std::string line;
    while (std::getline(imu_file, line)) {
        if (line[0] == '#' || line.empty()) {
            continue;
        }

        std::istringstream iss(line);
        double timestamp;
        double wx, wy, wz;
        double ax, ay, az;
        iss >> timestamp >> wx >> wy >> wz >> ax >> ay >> az;
        // std::cout << timestamp << " " << wx << " " << wy << " " << wz << " " << ax << " " << ay << " " << az << std::endl;

        timestamp = timestamp * 1e-9;

        if (start_time > 0 && timestamp < start_time) {
            continue;
        }

        if (end_time > 0 && timestamp >= end_time) {
            break;
        }

        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time(timestamp);
    
        imu_data.header.frame_id = "base_link";
        imu_data.angular_velocity.x = wx;
        imu_data.angular_velocity.y = wy;
        imu_data.angular_velocity.z = wz;
        imu_data.linear_acceleration.x = ax * G;
        imu_data.linear_acceleration.y = ay * G;
        imu_data.linear_acceleration.z = az * G;

        try {
            bag.write("/imu/horizon_data", imu_data.header.stamp, imu_data);
        } catch (const rosbag::BagIOException& e) {
            std::cerr << "Error writing to bag: " << e.what() << std::endl;
            return;
        }
    }

    imu_file.close();
}

void camera2Bag(rosbag::Bag& bag, std::string dir_name, double start_time, double end_time) {
    std::vector<std::string> img_paths = GetFileList(dir_name);

    for (size_t i = 0; i < img_paths.size(); i++) {
        std::vector<std::string> spilts = StringSplit(img_paths[i], "/");
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

        cv::Mat img = cv::imread(img_paths[i]);
        if (img.empty()) {
            std::cout << "image data read failed: " << img_paths[i] << std::endl;
            continue;
        }

        int width = img.cols;
        int height = img.rows;
        cv::resize(img, img, cv::Size(width/2, height/2), 0, 0, cv::INTER_AREA);

        std::cout << "(" << i << "/" << img_paths.size() << ") " << img_paths[i] << " loading" << std::endl;

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        msg->header.stamp = ros::Time(timestamp);
        msg->header.frame_id = "base_link";

        bag.write("/hkvison_camera/image_raw", msg->header.stamp, *msg);
    }
}

void lidar2Bag(rosbag::Bag& bag, std::string dir_name, double start_time, double end_time) {
    typedef pcl::PointXYZINormal PointT;
    typedef pcl::PointCloud<PointT>::Ptr PointPtr;

    std::vector<std::string> lidar_paths = GetFileList(dir_name);

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

        bag.write("/horizon_lidar/data", laser_cloud.header.stamp, laser_cloud);
    }
}

void gps2Bag(rosbag::Bag& bag, std::string path, double start_time, double end_time) {
    std::ifstream gps_file(path);
    std::string line;
    while (std::getline(gps_file, line)) {
        if (line[0] == '#' || line.empty()) {
            continue;
        }

        sensor_msgs::NavSatFix gps_data;

        std::stringstream ss;
        ss << line;
        double timestamp;
        double lat, lon, alt;
        double sat_num;
        double x_factor, y_factor, z_factor;
        ss >> timestamp >> lat >> lon >> alt >> sat_num >> x_factor >> y_factor >> z_factor;

        if (start_time > 0 && timestamp < start_time) {
            continue;
        }

        if (end_time > 0 && timestamp >= end_time) {
            break;
        }

        double latitude = int(lat / 100);
        latitude = latitude + (lat - latitude * 100) / 60;
        double longitude = int(lon / 100);
        longitude = longitude + (lon - longitude * 100) / 60;

        gps_data.header.frame_id = "base_link";
        gps_data.header.stamp = ros::Time(timestamp);
        gps_data.latitude = latitude;
        gps_data.longitude = longitude;
        gps_data.altitude = alt;
        
        if (std::abs(gps_data.altitude) < 1e-9)
          continue;

        gps_data.position_covariance = {x_factor, 0, 0, 0, y_factor, 0, 0, 0, z_factor};

        bag.write("/gps/rtk", gps_data.header.stamp, gps_data);
    }
    gps_file.close();
}