cmake_minimum_required(VERSION 3.0.2)
project(gps_trajectory)

## 使用 C++11（推荐）
add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")


## ROS 依赖包
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  visualization_msgs
  serial_port
)

## GeographicLib 依赖
find_package(GeographicLib REQUIRED)

###################################
## catkin package 配置
###################################
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES gps_trajectory
  # CATKIN_DEPENDS nav_msgs roscpp sensor_msgs tf2 visualization_msgs
  # DEPENDS GeographicLib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}  # 添加头文件路径
)

## 可执行程序
add_executable(gps_trajectory_node src/gps_trajectory_node.cpp)
add_executable(utm_trajectory_node src/utm_trajectory_node.cpp)

## 链接库
target_link_libraries(gps_trajectory_node
  ${catkin_LIBRARIES}
)

target_link_libraries(utm_trajectory_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}   # 添加 GeographicLib 链接库
)
