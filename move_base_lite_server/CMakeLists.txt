cmake_minimum_required(VERSION 2.8.3)
project(move_base_lite_server)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  grid_map_planner_lib
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_base_lite_server
#  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs grid_map_planner_lib move_base_msgs nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(move_base_lite_ros
  src/move_base_lite_ros.cpp
)

target_link_libraries(move_base_lite_ros
  ${catkin_LIBRARIES}
)

#add_dependencies(move_base_lite_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(move_base_lite_node src/move_base_lite_node.cpp)

target_link_libraries(move_base_lite_node
  #${catkin_LIBRARIES}
  move_base_lite_ros
)
