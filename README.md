# ROS metapackage for robot trajectory execution

This ROS metapackage provides the functionality for trajectory execution by Manos (Roboskel's UR3) 
using two different methods. The first one is based on MoveIt! (check [this](https://github.com/Roboskel-Manipulation/trajectory_execution_pkg/tree/master/moveit_trajectory_execution)) and the second one on a Velocity 
Controller (check [this](https://github.com/Roboskel-Manipulation/trajectory_execution_pkg/tree/master/control_trajectory_execution)).

The given trajectory consists of 3D waypoints in the form of `trajectory_execution_msgs/PointArray` ROS
message (check [this](https://github.com/Roboskel-Manipulation/trajectory_execution_pkg/tree/master/trajectory_execution_msgs)).

## Launch files
To check the functionality of the package using Manos in a simulated environment, first run in a tab of
your favourite terminal `roslaunch manos_gazebo manos_gazebo.launch` and specify the arguents for Moveit! or
Velocity Control interface (check [this](https://github.com/Roboskel-Manipulation/manos/tree/devel) for more info)
In another tab run `roslaunch trajectory_execution_launch trajectory_execution_launch.launch`
 
 The argument of the launch file is the following:
 * `control`: Set to true if you want to use the Velocity Controller method, otherwise set it to false
 for MoveIt! (default true)
 
