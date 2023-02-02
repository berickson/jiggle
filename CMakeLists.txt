cmake_minimum_required(VERSION 3.5)
project(jiggle)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   rosbag
#   sensor_msgs
#   tf
#   tf2
#   tf2_ros
#   hector_mapping
#   hector_trajectory_server
# )

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(jiggle src/jiggle.cpp)


add_executable(lidar_odom src/lidar_odom.cpp)

ament_target_dependencies(lidar_odom
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  laser_geometry
  tf2
  tf2_ros
)

install(TARGETS
  lidar_odom
  DESTINATION lib/${PROJECT_NAME})

ament_package()

