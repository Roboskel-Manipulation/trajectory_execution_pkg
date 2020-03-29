# ROS metapackage for robot trajectory execution

This ROS metapackage provides the functionality for trajectory execution by Manos (Roboskel's UR3) 
using two different methods. The first one is based on MoveIt! (https://github.com/ThanasisTs/trajectory_execution_pkg/tree/master/moveit_trajectory_execution) and the second one on a Velocity 
Controller (https://github.com/ThanasisTs/trajectory_execution_pkg/tree/master/control_trajectory_execution).

The given trajectory consists of 3D waypoints in the form of `trajectory_execution_msgs/PointArray` ROS
message (https://github.com/ThanasisTs/trajectory_execution_pkg/tree/master/trajectory_execution_msgs).

## Launch files
To check the functionality of the package using Manos in a simulated environment, first run in a tab of
your favourite terminal `roslaunch manos_gazebo manos_gazebo.launch` and specify the arguents for Moveit! or
Velocity Control interface (check https://github.com/ThanasisTs/manos/tree/devel  for more info)
In another tab run `roslaunch trajectory_execution_launch trajectory_execution_launch.launch`
 
 The arguments of the launch file are the following:
 * `control`: Set to true if you want to use the Velocity Controller method, otherwise set it to false
 for MoveIt! (default true)
 * `clean`: Set to true if you want to clean the input trajectory (default true)
 * `smooth`: Set to true if you want to smooth the input trajectory (default false)
 
 For the last two arguments check and their functionality check https://github.com/ThanasisTs/trajectory_preprocessing_pkg.     Lastly in a third tab run `rosrun trajectory_execution example_trajectory.py` and observe the results in Rviz (or Gazebo)
 
 
