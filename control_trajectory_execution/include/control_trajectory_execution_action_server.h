#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_trajectory_execution/control_trackingAction.h>
#include <trajectory_custom_msgs/PointArray.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

bool sim, init_flag = false, final_flag = false, time_flag = true;
float init_gain, D, K;
float sleep_rate, lim;
float last_time, control_cycle_duration;

