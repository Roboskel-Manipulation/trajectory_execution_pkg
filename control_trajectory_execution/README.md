# Trajectory execution using Velocity Controllers

This package utilises a Velocity Controller based on 
[this](https://github.com/epfl-lasa/ridgeback_ur5_controller/tree/devel/ur5_cartesian_velocity_control) repo.
Check out also [this](https://github.com/ThanasisTs/manos_control/tree/master/manos_cartesian_velocity_control) repo
for a detailed description of the controller

## Functionality
* It subscribes to a topic which provides the current pose and twist of the end effector
(`cartesian_state_msgs/PoseTwist` ROS message)
* It accepts a list of 3D waypoints (`trajectory_execution_msgs/PointArray` ROS message)
* It moves to the first point of the trajectory by moving along the line connecting its current
pose and the first point of the trajectory
* It executes the given trajectory by producing linear velocities based on its current position
and the waypoints of the trajectory.
