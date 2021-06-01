# control_trajectory_execution

This package provides an action server for 3D cartesian trajectory execution. It accepts a list of 3D cartesian points and publishes each one of them at appropriate publishing rate so that the overall robot motion lasts the same duration as the input motion.

## Functionality
* It accepts a list of 3D waypoints (`trajectory_custom_msgs/PointArray` ROS message).
* It computes the appropriate publishing rates.
* It subscribes to a topic which provides the current pose and twist of the end effector
(`cartesian_state_msgs/PoseTwist` ROS message). Used only for visualization purposes.
* It publishes the waypoints to the `/trajectory_points` topic.
