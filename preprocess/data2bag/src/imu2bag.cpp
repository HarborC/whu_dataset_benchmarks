#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>
#include <iostream>
#include <vector>

#include "io_tools.h"

using namespace hx_slam::io;

double G = 9.81;

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_node");
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

    std::string imu_path = root_dir + "/horizon_imu.txt";

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/horizon_data", 1000000);
    ros::Rate loopRate(200);

    std::ifstream imu_file(imu_path);
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

        imu_pub.publish(imu_data);
        ros::spinOnce();
        loopRate.sleep();
    }

    imu_file.close();

    return 0;
}
