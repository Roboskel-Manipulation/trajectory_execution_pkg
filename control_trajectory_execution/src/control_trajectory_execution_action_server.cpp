#include "control_trajectory_execution_action_server.h"

class ControlTrackingAction{
protected:
	ros::NodeHandle nh_;
	ros::Subscriber vel_sub;
	ros::Publisher vel_pub;
	ros::Publisher vis_pub;
	actionlib::SimpleActionServer<control_trajectory_execution::control_trackingAction> as;
	std::string action_name;
	control_trajectory_execution::control_trackingFeedback feedback;
	control_trajectory_execution::control_trackingResult result;
	trajectory_execution_msgs::PointArray control_points;
	geometry_msgs::PointPtr ee_pos = boost::make_shared<geometry_msgs::Point>();
	geometry_msgs::TwistPtr vel = boost::make_shared<geometry_msgs::Twist>();
	geometry_msgs::TwistPtr zero_vel = boost::make_shared<geometry_msgs::Twist>();
	visualization_msgs::Marker marker;

public:
	ControlTrackingAction(std::string name, ros::NodeHandle nh):
	nh_(nh),
	as(nh_, name, boost::bind(&ControlTrackingAction::executeCB, this, _1), false),
	action_name(name){
		vel_sub = nh.subscribe("/manos_cartesian_velocity_controller/ee_state", 10, &ControlTrackingAction::ee_state_callback, this);
		vel_pub = nh.advertise<geometry_msgs::Twist>("/manos_cartesian_velocity_controller/command_cart_vel", 10);			
		vis_pub = nh.advertise<visualization_msgs::Marker>("/trajectory_visualization", 10);
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
		
		for (int i=0; i<goal->waypoints.points.size(); i++){
			control_points.points.push_back(goal->waypoints.points[i]);
		}

		//
		// Move to the first point of the trajectory
		//
		while (not init_flag){
			vel->linear.x = init_gain*(control_points.points[0].x - ee_pos->x);
			vel->linear.y = init_gain*(control_points.points[0].y - ee_pos->y);
			vel->linear.z = init_gain*(control_points.points[0].z - ee_pos->z);
			vel_pub.publish(*vel);
			ros::Duration(0.2).sleep();
			if (abs(control_points.points[0].x - ee_pos->x) < 0.005 and abs(control_points.points[0].y - ee_pos->y) < 0.005 and abs(control_points.points[0].z - ee_pos->z) < 0.005){
				ROS_INFO("Reached initial position");
				init_flag = true;
			}		
		}
	
		//
		// Execute the trajectory
		//	
		if (init_flag){
			ros::Duration(0.5).sleep();
			double start_time = ros::Time::now().toSec();
			for (short int i=1; i<control_points.points.size(); i++){
				vel->linear.x = Dt*(control_points.points[i].x - ee_pos->x);
				vel->linear.y = Dt*(control_points.points[i].y - ee_pos->y);
				vel->linear.z = Dt*(control_points.points[i].z - ee_pos->z);
				vel_pub.publish(*vel);
				feedback.ee_pos.points.push_back(control_points.points[i]);
				feedback.percentage = (float)feedback.ee_pos.points.size()/control_points.points.size();
				as.publishFeedback(feedback);
				if (i == control_points.points.size()-1){
					while (not final_flag){
						vel->linear.x = Dt*(control_points.points[i].x - ee_pos->x);
						vel->linear.y = Dt*(control_points.points[i].y - ee_pos->y);
						vel->linear.z = Dt*(control_points.points[i].z - ee_pos->z);
						vel_pub.publish(*vel);
						ros::Duration(dt).sleep();
						if (abs(control_points.points[i].x - ee_pos->x) < 0.005 and abs(control_points.points[i].y - ee_pos->y) < 0.005 and abs(control_points.points[i].z - ee_pos->z) < 0.005){
							ROS_INFO("Reached final point");
							final_flag = true;
						}
					}
				}
				else{
					ros::Duration(dt).sleep();
				}
			}
			ROS_INFO("Published all velocities");
			double end_time = ros::Time::now().toSec();
			ROS_INFO("The motion lasted %f secs", end_time - start_time);
			vel_pub.publish(*zero_vel);
			result.success = true;
			as.setSucceeded(result);

		}
	}


	void ee_state_callback (const trajectory_execution_msgs::PoseTwist::ConstPtr msg){
		ee_pos->x = msg->pose.position.x;
		ee_pos->y = msg->pose.position.y;
		ee_pos->z = msg->pose.position.z;
		if (init_flag){
			marker.points.push_back(*ee_pos);
			marker.scale.x = 0.01;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
		  	vis_pub.publish(marker);
		}
	}
};


int main(int argc, char** argv){
	ros::init(argc, argv, "control_trajectory_execution_action_server");
	ros::NodeHandle nh;
	nh.param("control_trajectory_execution_action_server/dt", dt, 0.0f);
	nh.param("control_trajectory_execution_action_server/Dt", Dt, 0.0f);
	nh.param("control_trajectory_execution_action_server/init_gain", init_gain, 0.0f);
	nh.param("control_trajectory_execution_action_server/sim", sim, true);
	std::cout << sim << std::endl;
	ros::AsyncSpinner spinner(2);
	spinner.start();


	ControlTrackingAction control_tracking("control_tracking", nh);
	ros::waitForShutdown();
}