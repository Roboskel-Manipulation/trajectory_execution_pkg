cmake_minimum_required(VERSION 2.8.3)
project(use_case_traj_exec)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  trajectory_execution_msgs
  control_trajectory_execution
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES use_case
#  CATKIN_DEPENDS roscpp trajectory_execution_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable (control_trajectory_execution_action_client src/control_trajectory_execution_action_client.cpp)
target_link_libraries (control_trajectory_execution_action_client ${catkin_LIBRARIES})
