#include "moveit_trajectory_execution_action_server.h"

void event_handler(int sig){
	std::cout << "Received signal " << sig << std::endl;
	std::cout << "Gonna exit..." << std::endl;
	exit(-1);
}


class MoveItTrackingAction{
protected:
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<moveit_trajectory_execution::moveit_trackingAction> as;
	std::string action_name;
	ros::Publisher vis_pub;
	moveit_trajectory_execution::moveit_trackingFeedback feedback;
	moveit_trajectory_execution::moveit_trackingResult result;


 public:
 	MoveItTrackingAction(std::string name):as(nh, name, boost::bind(&MoveItTrackingAction::executeCB, this, _1), false),
 	action_name(name){
		vis_pub = nh.advertise<visualization_msgs::Marker>("ee_visualization", 1);
 		as.start();
 	}
 	
 	~MoveItTrackingAction(void){}

	void executeCB(const moveit_trajectory_execution::moveit_trackingGoalConstPtr& goal){
		for (int i=0; i<goal->waypoints.points.size(); i++){
			control_points.points.push_back(goal->waypoints.points[i]);
		}

		moveit::planning_interface::MoveGroupInterface mg("arm");
		mg.setPoseReferenceFrame("base_link");

		int num_points = control_points.points.size();
		ROS_INFO("The number of points are %d", num_points);
		

		//
		// Visualize the trajectory in Rviz
		//
		visualization_msgs::Marker marker;
		marker.header.frame_id = "base_link";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0);
		for (short int i=0; i<num_points; i++){
			marker.id = i;
			marker.points.push_back(control_points.points[i]);
			marker.scale.x = 0.01;
			marker.color.a = 1.0; // Don't forget to set the alpha!
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
		}
	 	vis_pub.publish(marker);


		//
		// Transform from 3D points to 6D Poses
		//
		geometry_msgs::Pose target_pose;
		geometry_msgs::Pose init_pose;

		init_pose.position.x = control_points.points[0].x;
		init_pose.position.y = control_points.points[0].y;
		init_pose.position.z = control_points.points[0].z;

		//
		// Looking straight
		//
		// init_pose.orientation.x = 0.707;
		// init_pose.orientation.y = 0.707;
		// init_pose.orientation.z = 0;
		// init_pose.orientation.w = 0;
		
		//
		// Looking downwards
		//
		init_pose.orientation.x = 0.51;
		init_pose.orientation.y = 0.511;
		init_pose.orientation.z = -0.49;
		init_pose.orientation.w = 0.489;


		//
		// Change ee's yaw angle for each waypoint
		//
		tf::Quaternion q(init_pose.orientation.x, init_pose.orientation.y,  init_pose.orientation.z,  init_pose.orientation.w);
		tf::Quaternion q_new;
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		mg.setPoseReferenceFrame("base_link");

		for (auto i=1; i<num_points; i++){
			target_pose.position.x = control_points.points[i].x;	
			target_pose.position.y = control_points.points[i].y;	
			target_pose.position.z = control_points.points[i].z;		
			yaw += (float(1)/float(num_points))*M_PI;
			q_new.setRPY(roll, pitch, yaw);	

			target_pose.orientation.x = q_new.getX();
			target_pose.orientation.y = q_new.getY();
			target_pose.orientation.z = q_new.getZ();
			target_pose.orientation.w = q_new.getW();
			target_points.poses.push_back(target_pose);
		}

		//
		// Move to the first point of the trajectory
		//
		moveit::planning_interface::MoveGroupInterface::Plan init_plan;
		mg.setPoseTarget(init_pose);
		mg.setStartStateToCurrentState();
		mg.setPlanningTime(100.0);

		while (mg.plan(init_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS){
			ROS_INFO("Replanning...");
		}
		ROS_INFO("Moved to the first point of the trajectory");
		mg.execute(init_plan);
		
		//
		// IK computation and trajectory generation in configuration space
		//
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;
		mg.setMaxVelocityScalingFactor(0.001);
		mg.setStartStateToCurrentState();

		moveit_msgs::RobotTrajectory trajectory;
		const double jump_threshold = 0.0;
		const double eef_step = 0.01;

		bool avoid_collisions = true;
		double fraction = mg.computeCartesianPath(target_points.poses, eef_step, jump_threshold, trajectory, avoid_collisions);

		ROS_INFO("The portion of the trajectory that will be executed is %f", fraction*100);
		my_plan.trajectory_ = trajectory;
		moveit::planning_interface::MoveItErrorCode ex_feedback = mg.asyncExecute(my_plan);

		mg.setPoseReferenceFrame("base_link");
		
		while (ex_feedback != moveit::planning_interface::MoveItErrorCode::SUCCESS){
			geometry_msgs::Pose current_pose = mg.getCurrentPose().pose;
			geometry_msgs::PoseArray cp_poses;
			cp_poses.poses.push_back(current_pose);
			feedback.percentage = cp_poses.poses.size();
			feedback.ee_pose = cp_poses;			
		}
		
		result.success = (fraction != 1); 
		as.setSucceeded(result);

		my_plan.trajectory_.joint_trajectory.points.clear();
		target_points.poses.clear();
	}
};



int main(int argc, char** argv){

	ros::init(argc, argv, "moveit_trajectory_execution_action_server");
	ros::NodeHandle nh;
	
	nh.param("control_trajectory_execution_action_server/init_q_x", init_q_x, 0.0f);
	nh.param("control_trajectory_execution_action_server/init_q_y", init_q_y, 0.0f);
	nh.param("control_trajectory_execution_action_server/init_q_z", init_q_z, 0.0f);
	nh.param("control_trajectory_execution_action_server/init_q_w", init_q_w, 0.0f);

	ros::AsyncSpinner spinner(2);
	spinner.start();
	signal(SIGINT, event_handler);


	MoveItTrackingAction moveit_tracking("moveit_tracking");

	ros::waitForShutdown();
}
