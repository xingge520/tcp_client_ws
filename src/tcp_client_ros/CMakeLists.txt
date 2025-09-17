cmake_minimum_required(VERSION 3.0.2)
project(tcp_client_ros)

## 找 ROS 包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  object_information_msgs
)

## OpenCV
find_package(OpenCV REQUIRED)

## nlohmann/json (header-only)
find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tcp_client_lib
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs cv_bridge object_information_msgs
  DEPENDS OpenCV nlohmann_json
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

## 编译库
add_library(tcp_client_lib
  src/TcpClient.cpp
  src/DataSender.cpp
)

target_link_libraries(tcp_client_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json
)

## 编译可执行文件
add_executable(tcp_client_node src/main.cpp)
target_link_libraries(tcp_client_node
  tcp_client_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json
)
