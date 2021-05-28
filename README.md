# trajectory_replication

## Description
This ROS metapackage provides two packages for trajectory execution by a UR3 cobot using two different methods. The first one is based on [MoveIt!](https://moveit.ros.org/) and the second one on a Cartesian Velocity Controller([CVC](https://github.com/Roboskel-Manipulation/manos/tree/updated_driver/manos_cartesian_control)).

## Input Trajectory
The input trajectory consists of a list of 3D waypoints in the form of [`trajectory_custom_msgs/PointStampedArray`](https://github.com/Roboskel-Manipulation/trajectory_custom_msgs/blob/main/msg/PointStampedArray.msg) message.

## Launch files
* Moveit!: `roslaunch moveit_trajectory_execution moveit_trajectory_execution.launch`

* CVC: `roslaunch control_trajectory_execution control_trajectory_execution.launch`
  * Argument:  _smooth_: True if the input trajectory is a Bezier Curve.
