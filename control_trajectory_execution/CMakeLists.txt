cmake_minimum_required(VERSION 2.8.3)
project(control_trajectory_execution)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	std_msgs
	roscpp
	rospy
	tf
	tf_conversions
	visualization_msgs
	keypoint_3d_matching_msgs
	cartesian_state_msgs
	trajectory_custom_msgs
	actionlib
	actionlib_msgs
)

add_action_files(
	DIRECTORY action
	FILES control_tracking.action
)

generate_messages(
	DEPENDENCIES
	actionlib_msgs std_msgs trajectory_custom_msgs cartesian_state_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall")

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    include
#  LIBRARIES cpr_closed_loop_controller
  CATKIN_DEPENDS 
#  DEPENDS system_lib
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


add_executable (control_trajectory_execution_action_server src/control_trajectory_execution_action_server.cpp)
target_link_libraries (control_trajectory_execution_action_server ${catkin_LIBRARIES})
add_dependencies (control_trajectory_execution_action_server ${control_trajectory_execution_EXPORTED_TARGETS})
