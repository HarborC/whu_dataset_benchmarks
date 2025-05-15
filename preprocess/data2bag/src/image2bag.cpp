#include <ros/ros.h>
#include <rosbag/bag.h>
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

#include "io_tools.h"

using namespace hx_slam::io;

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera_node");
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

  std::string hkvison_image_path = root_dir + "/hkvison_image/";
  ros::Publisher camera_pub = nh.advertise<sensor_msgs::Image>("/hkvison_camera/image_raw", 100);
  ros::Rate loopRate(10);

  std::vector<std::string> img_paths = GetFileList(hkvison_image_path);

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

      camera_pub.publish(msg);

      ros::spinOnce();
      loopRate.sleep();
  }

  return 0;
}
