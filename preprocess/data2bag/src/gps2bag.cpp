#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <fstream>
#include <iostream>
#include <vector>

#include "io_tools.h"

using namespace hx_slam::io;

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_node");
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

    std::string gps_path = root_dir + "/gps.txt";

    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/rtk", 1000000);
    ros::Rate loopRate(200);

    std::ifstream gps_file(gps_path);
    std::string line;
    while (ros::ok() && std::getline(gps_file, line)) {
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

        gps_pub.publish(gps_data);
        ros::spinOnce();
        loopRate.sleep();
    }

    gps_file.close();

    return 0;
}
