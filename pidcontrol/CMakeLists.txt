cmake_minimum_required(VERSION 2.8.3)
project(pidcontrol)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
  tf
  sensor_msgs
  std_msgs
  tf2_msgs
)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)
find_package(cmake_modules REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


add_executable(pidcontrol src/rospid.cpp)
target_link_libraries(pidcontrol ${catkin_LIBRARIES})



