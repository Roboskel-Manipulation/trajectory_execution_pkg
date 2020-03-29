# Trajectory execution using MoveIt!

This package utilises MoveIt! in order for an end effector to track a given cartesian trajectory.

## Functionality
* It accepts a list of 3D waypoints (trajectory_execution_msgs/PointArray ROS message)
* It transforms the 3D list into a 7D list by adding a 4D quaternion in each point
* It moves the end effector to the first point of the trajectory using the built-in functions
of MoveIt! `plan()` and `execute()`
* It calls the MoveIt! service `computeCartesianPath()`
* It executes the configuration-space trajectory (response of the `computeCartesianPath` service)
