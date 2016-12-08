cmake_minimum_required(VERSION 2.8.3)
project(pluto_calibration)

find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pluto_costmap_generator
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ransac_icp_calibration src/ransac_icp_calibration.cpp)

target_link_libraries(ransac_icp_calibration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
