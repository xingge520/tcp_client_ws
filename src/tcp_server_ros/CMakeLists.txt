cmake_minimum_required(VERSION 3.0.2)
project(tcp_server_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tcp_server_lib
  CATKIN_DEPENDS roscpp
  DEPENDS OpenCV nlohmann_json
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
)

add_library(tcp_server_lib
  src/TcpServerNode.cpp
)

target_link_libraries(tcp_server_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json
)

add_executable(tcp_server_node src/main.cpp)
target_link_libraries(tcp_server_node
  tcp_server_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json
)
