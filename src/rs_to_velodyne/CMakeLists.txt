cmake_minimum_required(VERSION 3.8)
# 添加以下行来设置 CMP0074 策略
cmake_policy(SET CMP0074 NEW)
project(rs_to_velodyne)

# 添加 C++14 支持
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 添加编译优化选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
)

add_executable(rs_to_velodyne src/rs_to_velodyne.cpp)
ament_target_dependencies(rs_to_velodyne
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(rs_to_velodyne
  ${PCL_LIBRARIES}
)

install(TARGETS
  rs_to_velodyne
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
