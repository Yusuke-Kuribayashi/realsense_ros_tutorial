cmake_minimum_required(VERSION 3.0.2)
project(realsense_ros_tutorial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package( PCL 1.10 REQUIRED QUIET)
find_package(realsense2 CONFIG REQUIRED)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS 
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sample_code src/sample_code.cpp)
target_link_libraries(sample_code ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${realsense2_LIBRARY})
