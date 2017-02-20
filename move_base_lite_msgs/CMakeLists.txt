cmake_minimum_required(VERSION 2.8.3)
project(move_base_lite_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  message_generation
  actionlib_msgs
  geometry_msgs
  nav_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    ErrorCodes.msg
    FollowPathOptions.msg
  )

add_action_files(
  DIRECTORY
    action
  FILES
    FollowPath.action
    MoveBase.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    nav_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES move_base_lite_msgs
#  CATKIN_DEPENDS actionlib_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


