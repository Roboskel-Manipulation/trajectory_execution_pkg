# Trajectory execution using Velocity Controllers

This package provides an action server for offline 3D trajectory execution. The functionality for the execution is in [this](https://github.com/ThanasisTs/reactive_control) package.

## Functionality
* It subscribes to a topic which provides the current pose and twist of the end effector
(`cartesian_state_msgs/PoseTwist` ROS message)
* It accepts a list of 3D waypoints (`trajectory_custom_msgs/PointArray` ROS message)
* It maps the 3D waypoints to the robot workspace by adding a predifined offset specified in the `config/control_trajectory_execution.yaml` file
* Publishes the waypoints to the `trajectory_points` topic at a frequency specified in the `config/control_trajectory_execution_action_server.yaml` file
