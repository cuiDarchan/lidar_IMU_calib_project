cmake_minimum_required(VERSION 3.0.2)
project(lidar_imu_calib_based_on_map)
set(CMAKE_BUILD_TYPE Release) # this is Important ! ! ! ! !

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11) #添加编译选项，方式1
#add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fopenmp") # 设备编译选项，方式2

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag             # use rosbag::Bag
  sensor_msgs
  message_filters
  geometry_msgs
  nav_msgs
  tf2_ros
  image_transport
  cv_bridge
  tf
)

catkin_package() # ** 不填加会找不到可执行文件
set(ALL_TARGET_LIBRARIES "")
include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/Ros.cmake)
include(cmake/Eigen.cmake)
include(cmake/PCL.cmake)
include(cmake/yaml-cpp.cmake)

add_executable(align_to_map_node src/align_to_map_node.cpp src/align_to_map.cpp)
target_link_libraries(align_to_map_node ${ALL_TARGET_LIBRARIES})