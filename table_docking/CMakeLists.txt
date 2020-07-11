cmake_minimum_required(VERSION 3.0.2)
project(table_docking)

add_compile_options(-std=c++14)

link_libraries(stdc++fs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  monitoring_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs visualization_msgs monitoring_core
#  DEPENDS system_lib
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_definitions(${catkin_DEFINITIONS})

add_executable(${PROJECT_NAME}_node
  src/table_docking_node.cpp
  src/table_docking.cpp
  src/node_monitor.cpp
  src/ping_monitor.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
