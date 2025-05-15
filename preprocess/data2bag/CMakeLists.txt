cmake_minimum_required(VERSION 3.0.2)
project(data2bag)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  rosbag
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
)

catkin_package()

add_executable(data2bag_node src/data2bag.cpp)
target_link_libraries(data2bag_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(pcl2bag_node src/pcl2bag.cpp)
target_link_libraries(pcl2bag_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(imu2bag_node src/imu2bag.cpp)
target_link_libraries(imu2bag_node ${catkin_LIBRARIES})

add_executable(gps2bag_node src/gps2bag.cpp)
target_link_libraries(gps2bag_node ${catkin_LIBRARIES})

add_executable(image2bag_node src/image2bag.cpp)
target_link_libraries(image2bag_node ${catkin_LIBRARIES} ${OpenCV_LIBS})