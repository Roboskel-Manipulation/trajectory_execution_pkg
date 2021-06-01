#include "control_trajectory_execution_action_server.h"

class ControlTrackingAction{
protected:
	ros::NodeHandle nh_;
	ros::Subscriber vel_sub;
	ros::Publisher vel_pub;
	ros::Publisher vis_pub;
	ros::Publisher control_points_pub, ee_state_pub;
	actionlib::SimpleActionServer<control_trajectory_execution::control_trackingAction> as;
	std::string action_name;
	control_trajectory_execution::control_trackingFeedback feedback;
	control_trajectory_execution::control_trackingResult result;
	trajectory_custom_msgs::PointStampedArray control_points;
	geometry_msgs::PointPtr ee_pos = boost::make_shared<geometry_msgs::Point>();
	geometry_msgs::TwistPtr vel = boost::make_shared<geometry_msgs::Twist>();
	geometry_msgs::PointPtr acc = boost::make_shared<geometry_msgs::Point>();
	geometry_msgs::TwistPtr zero_vel = boost::make_shared<geometry_msgs::Twist>();
	visualization_msgs::Marker marker;

public:
	ControlTrackingAction(std::string name, ros::NodeHandle nh):
	nh_(nh),
	as(nh_, name, boost::bind(&ControlTrackingAction::executeCB, this, _1), false),
	action_name(name){
		vel_sub = nh.subscribe(ee_state_topic, 10, &ControlTrackingAction::ee_state_callback, this);
		vel_pub = nh.advertise<geometry_msgs::Twist>(ee_command_topic, 10);			
		vis_pub = nh.advertise<visualization_msgs::Marker>("/trajectory_visualization", 10);
		ee_state_pub = nh.advertise<cartesian_state_msgs::PoseTwist>("/ee_state_action_server_topic", 10);
		control_points_pub = nh.advertise<geometry_msgs::PointStamped>("/trajectory_points", 10);
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time::now();
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.01;
		marker.color.a = 1.0;
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
		marker.lifetime = ros::Duration(100);

		zero_vel->linear.x = 0;
		zero_vel->linear.y = 0;
		zero_vel->linear.z = 0;
		as.start();
	}

	~ControlTrackingAction(void){}

	void executeCB(const control_trajectory_execution::control_trackingGoalConstPtr &goal){
		ROS_INFO("Received the waypoints");
		control_points.points.clear();
		for (int i=0; i<goal->waypoints.points.size(); i++){
			control_points.points.push_back(goal->waypoints.points[i]);
		}

		ROS_INFO("Number of control points: %d", control_points.points.size());
		//
		// Move to the first point of the trajectory
		//
		control_points_pub.publish(control_points.points[0]);
		// ros::Duration(8).sleep();

		init_flag = true;
		// Publish trajectory points
		double start_time = ros::Time::now().toSec();
		if (smooth){
			sleep_rate = (control_points.points[control_points.points.size()-1].header.stamp.toSec() - control_points.points[0].header.stamp.toSec())/control_points.points.size();
			ROS_INFO("Publish rate: %f", sleep_rate);
 			for (int i=1; i<control_points.points.size(); ++i){
				control_points_pub.publish(control_points.points[i]);
				ros::Duration(sleep_rate).sleep();
			}
		}
		else{
			for (int i=1; i<control_points.points.size(); ++i){
				if (i != control_points.points.size() -1){
					sleep_rate = control_points.points[i+1].header.stamp.toSec() - control_points.points[i].header.stamp.toSec();
				}
				else{
					sleep_rate = 0;
				}
				control_points_pub.publish(control_points.points[i]);
				ros::Duration(sleep_rate).sleep();
			}
		}

		double end_time = ros::Time::now().toSec();
		ROS_INFO("The motion lasted %f secs", end_time - start_time);
		// vel_pub.publish(*zero_vel);
		result.success = true;
		as.setSucceeded(result);
	}


	void ee_state_callback (const cartesian_state_msgs::PoseTwist::ConstPtr msg){
		ee_pos->x = msg->pose.position.x;
		ee_pos->y = msg->pose.position.y;
		ee_pos->z = msg->pose.position.z;

		if (init_flag)
			ee_state_pub.publish(*msg);
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "control_trajectory_execution_action_server");
	ros::NodeHandle nh;
	nh.param("control_trajectory_execution_action_server/smooth", smooth, false);
	nh.param("cartesian_trajectory_tracking/state_topic", ee_state_topic, std::string("/ur3_cartesian_velocity_controller/ee_state"));
	nh.param("cartesian_trajectory_tracking/command_topic", ee_command_topic, std::string("/ur3_cartesian_velocity_controller/command_cart_vel"));

	ros::AsyncSpinner spinner(4);
	spinner.start();


	ControlTrackingAction control_tracking("control_tracking", nh);
	ros::waitForShutdown();
}