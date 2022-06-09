cmake_minimum_required(VERSION 3.0.2)
project(gps_demo)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

set(ALL_TARGET_LIBRARIES "")

include(cmake/geographic.cmake)
include(cmake/YAML.cmake)
include(cmake/glog.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")

add_executable(gps_demo_node ${ALL_SRCS})
target_link_libraries(gps_demo_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

