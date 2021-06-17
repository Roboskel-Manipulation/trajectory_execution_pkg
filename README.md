# trajectory_replication

## Description
This ROS metapackage provides two packages for trajectory execution by a UR3 cobot using two different methods. The first one is based on [MoveIt!](https://moveit.ros.org/) and the second one on a Cartesian Velocity Controller([CVC](https://github.com/Roboskel-Manipulation/manos/tree/updated_driver/manos_cartesian_control)).

## Input Trajectory
The input trajectory consists of a list of 3D waypoints in the form of [`trajectory_custom_msgs/PointStampedArray`](https://github.com/Roboskel-Manipulation/trajectory_custom_msgs/blob/main/msg/PointStampedArray.msg) message.

## control_trajectory_execution

This package provides an action server for 3D cartesian trajectory execution. It accepts a list of 3D cartesian points and publishes each one of them at appropriate publishing rate so that the overall robot motion lasts the same duration as the input motion.
* It accepts a list of 3D waypoints (trajectory_custom_msgs/PointArray ROS message).
* It computes the appropriate publishing rates.
* It subscribes to a topic which provides the current pose and twist of the end effector (cartesian_state_msgs/PoseTwist ROS message). Used only for visualization purposes.
* It publishes the waypoints to the /trajectory_points topic.

**Note**: To see how to utilize the CVC for generating velocity commands for the robot motion, check the [cartesian_trajectory_tracking](https://github.com/Roboskel-Manipulation/cartesian_trajectory_tracking) package.

## moveit_trajectory_execution

This package utilises MoveIt! in order for an end effector to track a given cartesian trajectory.
* It accepts a list of 3D waypoints (trajectory_execution_msgs/PointArray ROS message)
* It transforms the 3D list into a 7D list by adding a 4D quaternion in each point
* It moves the end effector to the first point of the trajectory using the built-in functions of MoveIt! plan() and execute()
* It calls the MoveIt! service computeCartesianPath()
* It executes the configuration-space trajectory (response of the computeCartesianPath service)


## Launch files
* Moveit!: `roslaunch moveit_trajectory_execution moveit_trajectory_execution.launch`

* CVC: `roslaunch control_trajectory_execution control_trajectory_execution.launch`
  * Argument:  _smooth_: True if the input trajectory is a Bezier Curve.


