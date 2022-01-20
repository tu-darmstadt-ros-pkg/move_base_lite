cmake_minimum_required(VERSION 3.0.2)
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
  hector_move_base_msgs
  move_base_lite_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/MoveBaseLite.cfg
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

add_dependencies(move_base_lite_ros ${PROJECT_NAME}_gencfg)

#add_dependencies(move_base_lite_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(move_base_lite_node src/move_base_lite_node.cpp)

target_link_libraries(move_base_lite_node
  #${catkin_LIBRARIES}
  move_base_lite_ros
)

# Mark executables and/or libraries for installation
install(TARGETS move_base_lite_node move_base_lite_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

