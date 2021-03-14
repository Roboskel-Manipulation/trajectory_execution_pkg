#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_trajectory_execution/control_trackingAction.h>
#include <trajectory_custom_msgs/PointStampedArray.h>
#include <cartesian_state_msgs/PoseTwist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

bool init_flag = false, smooth;
float sleep_rate;
std::string ee_state_topic, ee_command_topic;
