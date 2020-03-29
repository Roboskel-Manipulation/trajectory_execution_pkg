cmake_minimum_required(VERSION 2.8.3)
project(moveit_trajectory_execution)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  geometric_shapes
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  pcl_conversions
  pcl_ros
  pluginlib
  rosbag
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  tf
  trajectory_execution_msgs
  visualization_msgs
  std_msgs
  actionlib
  actionlib_msgs
  )

add_action_files(
  DIRECTORY action
  FILES moveit_tracking.action
  )

generate_messages(
  DEPENDENCIES
  actionlib_msgs trajectory_execution_msgs)

find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
  LIBRARIES
    interactivity_utils
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    include 
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)


include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} include)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


add_executable (moveit_trajectory_execution_action_server src/moveit_trajectory_execution_action_server.cpp)
target_link_libraries (moveit_trajectory_execution_action_server ${catkin_LIBRARIES})
add_dependencies (moveit_trajectory_execution_action_server ${moveit_trajectory_execution_EXPORTED_TARGETS})

# add_executable (moveit_trajectory_execution_action_client src/moveit_trajectory_execution_action_client.cpp)
# target_link_libraries (moveit_trajectory_execution_action_client ${catkin_LIBRARIES})
# add_dependencies (moveit_trajectory_execution_action_client ${moveit_trajectory_execution_EXPORTED_TARGETS})
