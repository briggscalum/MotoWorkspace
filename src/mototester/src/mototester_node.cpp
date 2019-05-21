#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/PositionConstraint.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>  
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#define PI 3.14159265


class PointGoal {
  
	public:

	double x;
	double y;
	double time;

};

float fabric_x = 0;
float fabric_y = 0;
float fabric_orien = 0;

void poseUpdater(std_msgs::Float32MultiArray msg)
{
  	fabric_x =  msg.data[0];
  	fabric_y =  msg.data[1];
  	fabric_orien =  msg.data[2];
}

int main(int argc, char **argv)
{
	int test = 3;
	ros::init(argc, argv, "test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();


	// motoman_msgs::DynamicJointTrajectory command;

	// ros::Publisher command_publisher = node_handle.advertise<motoman_msgs::DynamicJointTrajectory>("/sda10f/sda10f_r1_controller/joint_path_command", 1, true);

	// command.header.frame_id = "/torso_base_link	";
	// command.header.seq = 1;
	// command.joint_names = {"arm_left_joint_1_s", "arm_left_joint_2_l", "arm_left_joint_3_e", "arm_left_joint_4_u", "arm_left_joint_5_r", "arm_left_joint_6_b", "arm_left_joint_7_t"};
	// command.points.groups.positions = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};	

	// command_publisher.publish(command);

	sleep(1.0);

	ros::Subscriber posedub = node_handle.subscribe("fabric_pose", 1000, poseUpdater);
 	ros::Publisher gripper_pub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);

 	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output reset;
 	reset.rACT = 0;
 	reset.rGTO = 0;
	reset.rATR = 0;
	reset.rPR = 0;
	reset.rSP = 0;
	reset.rFR = 0;

	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output open;
 	open.rACT = 1;
 	open.rGTO = 1;
	open.rATR = 0;
	open.rPR = 0;
	open.rSP = 255;
	open.rFR = 150;

	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output close;
 	close.rACT = 1;
 	close.rGTO = 1;
	close.rATR = 0;
	close.rPR = 255;
	close.rSP = 255;
	close.rFR = 200;



	// gripper_pub.publish(reset);

	// sleep(2.0);
	// gripper_pub.publish(close);

	// sleep(2.0);
	// gripper_pub.publish(open);

	// sleep(2.0);

	moveit::planning_interface::MoveGroupInterface group("arm_left");
	moveit::planning_interface::MoveGroupInterface torso_group("torso");

	//moveit::planning_interface::MoveGroupInterface arm_left_group("arm_left");
	moveit::planning_interface::MoveGroupInterface arm_right_group("arm_right");


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	//arm_left_group.setPlannerId("RRTConnectkConfigDefault");
	arm_right_group.setPlannerId("RRTConnectkConfigDefault");
	group.setPlannerId("RRTConnectkConfigDefault");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//group.setPlannerId("PRMstarkConfigDefault");

	group.setGoalJointTolerance(0.001);
	group.setNumPlanningAttempts(10);
	torso_group.setEndEffectorLink("torso_link_b1");
	group.setEndEffectorLink("arm_left_link_tool0");
	arm_right_group.setEndEffectorLink("arm_right_link_tool0");

	geometry_msgs::Pose left_flat;
	geometry_msgs::Pose right_home;

	// right_home.position.x = 0.4;   
	// right_home.position.y = -0.1;
	// right_home.position.z = 1.0;
	// right_home.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/4,0);  

	left_flat.position.x = 0;  
	left_flat.position.y = 0.8;
	left_flat.position.z = 0.8;
	left_flat.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2 - M_PI/8,0,0);  

	// tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/4,0); behind robot flat

	// ADD COLLISION

	// END COLLISION ADDER

	// std::vector<PointGoal> path(100);

	// //Generate a Dummy Path
 //    for (int i = 0; i < 100; i++)
 //    {
 //      path[i].x = sin(i/100.0*2.0*PI)/10.0;
 //      path[i].time = i/10;
 //    }


	std::vector<double> group_variable_values;
	std::vector<double> torso_variable_values;
	//std::vector<double> left_start_position = {-1.4351, -1.4266, 0, -1.7675, 0, 0.6670, -1.4645};
	std::vector<double> left_alt_start_position = {1.6808, -0.6219, -0.1952, -1.674, 0, 0.08568, 1.52};
	std::vector<double> left_alt_pregrip_position = {1.802564, -0.526207, -0.236598, -1.471528, 0, -0.154917, 1.5345};

	std::vector<double> left_alt_grip_position = {1.890188, -0.407766, -0.314982, -1.142800, 0, -0.42265, 1.540177};

	std::vector<double> left_alt_postgrip_position = {1.694776, -0.782688, -0.155478, -1.552859, 0, -0.359225, 1.572601};
	//std::vector<double> left_grip_position = {-1.226, 0.3801, -0.3696, 1.077, 0, 0.41692, -1.5115};
	
	// std::vector<double> left_preprepre_sew_position = {2.156794, -0.670111, -0.943899, -1.064476, 0, -1.469128, -1.1475};
	// std::vector<double> left_prepre_sew_position = {2.033547, -0.779688, -0.916789, -1.642225, 0, -1.022603, -1.071741};
	// std::vector<double> left_pre_sew_position = {2.104537, -0.830992, -1.100775, -1.840762, 0, -0.808256, -0.919216};
	// std::vector<double> left_alt_sew_position = {2.32439, -0.8409, -1.349660 -1.544886, 0, -0.954030, -0.799565};


	std::vector<double> left_prepre_sew_position = {2.6565, -0.738203, -1.512687, -0.421708, 0, -1.829020, -0.898371};
	std::vector<double> left_pre_sew_position = {2.339, -0.7352, -1.214, -1.074, 0, -1.316, -0.8973};
	std::vector<double> left_alt_sew_position = {2.2955, -0.8602, -1.3303, -1.652, 0, -0.8620, -0.7915};



	//group.getCurrentState()->copyJointGroupPositions(
    //group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	
	if(test == 0) {

	    group.setJointValueTarget(left_alt_start_position);
		group.plan(my_plan);
		group.execute(my_plan);


		sleep(1.0);

		group.setJointValueTarget(left_alt_pregrip_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		group.setJointValueTarget(left_alt_grip_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		sleep(1.0);



		group.setJointValueTarget(left_alt_postgrip_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		// group.setJointValueTarget(left_preprepre_sew_position);
		// group.plan(my_plan);
		// group.execute(my_plan);

		// sleep(1.0);


		group.setJointValueTarget(left_prepre_sew_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);


		group.setJointValueTarget(left_pre_sew_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		group.setJointValueTarget(left_alt_sew_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

	}

	// Sewing Allignment 270, 226 0.46

 //  group.setJointValueTarget(left_pre_sew_position);
	// group.plan(my_plan);
	// group.execute(my_plan);

	// ROS_INFO("Joint 1 %f", group_variable_values[0]);
	// ROS_INFO("Joint 2 %f", group_variable_values[1]);
	// ROS_INFO("Joint 3 %f", group_variable_values[2]);
	// ROS_INFO("Joint 4 %f", group_variable_values[3]);
	// ROS_INFO("Joint 5 %f", group_variable_values[4]);
	// ROS_INFO("Joint 6 %f", group_variable_values[5]);
	// ROS_INFO("Joint 7 %f", group_variable_values[6]);

	// left_arm	
	//left_start_position[0] = 0.0;
	//left_start_position[1] = 0.0;
	//left_start_position[2] = 0.0;
	//left_start_position[3] = 0.0;
	//left_start_position[4] = 0.0;
	//left_start_position[5] = 0.0;
	//left_start_position[6] = 0.0;


	// //group_variable_values[10] = group_variable_values[10] + 0.1;
	// group.setJointValueTarget(left_start_position);
	// group.plan(my_plan);
	// group.execute(my_plan);

	// geometry_msgs::Pose left_sew_start;
	// left_sew_start.position.x = 0.63566;   
	// left_sew_start.position.y = 0.093692;
	// left_sew_start.position.z = 0.88518;
	// left_sew_start.orientation.x = 0.35885;      
	// left_sew_start.orientation.y = -0.60485;
	// left_sew_start.orientation.z = -0.37386;
	// left_sew_start.orientation.w = 0.60466;  

	int grabcounter = 1;

	if(test == 1) {

   	    sleep(2.0);

		grabcounter = grabcounter + 1;
		//right_home.position.y = -0.25 - (fabric_x-360)*0.001;
		//right_home.position.x = -0.4 - (fabric_y-240)*0.00125;
		//right_home.position.z = right_home.position.z - 0.05;

		//group.setPoseTarget(left_sew_start, "arm_left_link_7_t");
		group.setPoseTarget(left_flat, "arm_left_link_tool0");

		group.plan(my_plan);
	    group.execute(my_plan);

   	    sleep(2.0);

   	    left_flat.position.y = 1.0;

   	    group.setPoseTarget(left_flat, "arm_left_link_tool0");

   	    group.plan(my_plan);
	    group.execute(my_plan);

	    gripper_pub.publish(close);

   	    sleep(2.0);



		left_flat.position.y = 0.8;
		left_flat.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2 - M_PI/8,0,0);  
		left_flat.position.z = 1.0;

		group.setPoseTarget(left_flat, "arm_left_link_tool0");


		group.plan(my_plan);
	    group.execute(my_plan);

   	    sleep(2.0);
   	}




	if(test == 2) {

		geometry_msgs::PoseStamped test_pose;
		test_pose = group.getCurrentPose();
		
		// moveit_msgs::Constraints arm_left_constraints;
		// moveit_msgs::PositionConstraint arm_left_position_constraints;
		// 	arm_left_position_constraints.link_name = "arm_left_link_tool0";
		// 	arm_left_position_constraints.target_point_offset.y = 0.001;
		// 	arm_left_position_constraints.target_point_offset.x = 0.001;
		// 	arm_left_position_constraints.weight= 1000.0;


		// arm_left_constraints.position_constraints = {arm_left_position_constraints}	;

		//group.setPathConstraints(arm_left_constraints);
		//test_pose.position.x = 0.5;  
		//test_pose.position.y = 0.5;
		//test_pose.pose.position.z = test_pose.pose.position.z 

	    ROS_INFO("X Position %f", test_pose.pose.position.x);
   	    ROS_INFO("Y Position %f", test_pose.pose.position.y);
	    ROS_INFO("Z Position %f", test_pose.pose.position.z);



	    ROS_INFO("Joint 1 %f", group_variable_values[0]);
		ROS_INFO("Joint 2 %f", group_variable_values[1]);
		ROS_INFO("Joint 3 %f", group_variable_values[2]);
		ROS_INFO("Joint 4 %f", group_variable_values[3]);
		ROS_INFO("Joint 5 %f", group_variable_values[4]);
		ROS_INFO("Joint 6 %f", group_variable_values[5]);
		ROS_INFO("Joint 7 %f", group_variable_values[6]);

  	    sleep(10.0);
   	    
   	}

   	if(test == 3) {


   		group.setMaxVelocityScalingFactor(0.4);
   		moveit_msgs::RobotTrajectory trajectory;
  		std::vector<double> left_start_position = {1.030,-1.4833, 2.896, 2.318, -0.897, 0.593, -0.717};
  

  		// moveit_msgs::Constraints arm_left_constraints;
		// moveit_msgs::PositionConstraint arm_left_position_constraints;
		// arm_left_position_constraints.link_name = "arm_left_link_tool0";
		// arm_left_position_constraints.header.frame_id = "/torso_base_link";
		// arm_left_position_constraints.weight= 100000.0;
		
		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		std::vector<double> left_pre_grab = {2.758473, -0.905713, 2.652617, 1.955613, -0.450778, -0.712207, -0.339732};
		
		group.setJointValueTarget(left_pre_grab);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);


		geometry_msgs::PoseStamped preGrabPose;
		geometry_msgs::PoseStamped grabPose;

		preGrabPose = group.getCurrentPose();
		grabPose = group.getCurrentPose();

		grabPose.pose.position.y = grabPose.pose.position.y + 0.17;
		grabPose.pose.position.x = grabPose.pose.position.x - 0.01;

		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(preGrabPose.pose);
		waypoints.push_back(grabPose.pose);

		group.setMaxVelocityScalingFactor(0.5);

		group.setPlanningTime(7.0);

		double fraction = group.computeCartesianPath(waypoints, 0.01, 0, trajectory);

    	// First to create a RobotTrajectory object
    	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_left");

    	// Second get a RobotTrajectory from trajectory
    	rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);

    	// Thrid create a IterativeParabolicTimeParameterization object
   		trajectory_processing::IterativeParabolicTimeParameterization iptp;

   		ROS_INFO("number of points %d", (int)rt.getWayPointCount());

   		int trajlen = (int)rt.getWayPointCount();

   		for( int i = 1; i < trajlen; i++) {
   			rt.setWayPointDurationFromPrevious (i, 0.1);
   		}

   		rt.getRobotTrajectoryMsg(trajectory);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    	cart_plan.trajectory_ = trajectory;
    	group.execute(cart_plan);


    	sleep(2.0);

		gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		sleep(2.0);

		std::vector<double> left_post_grab = { 2.3934, -0.822231, 2.940899, 1.63835, -1.55050, -0.51859, 0.473729};

		group.setJointValueTarget(left_post_grab);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);


  		//std::vector<double> left_mid_position = {1.684,-0.8745, 1.618, 2.046, -0.1423, 0.999, 3.12};
  

  //  		moveit_msgs::Constraints arm_left_constraints;
		// moveit_msgs::PositionConstraint arm_left_position_constraints;
		// arm_left_position_constraints.link_name = "arm_left_link_tool0";
		// arm_left_position_constraints.header.frame_id = "/torso_base_link";
		// arm_left_position_constraints.weight= 100000.0;
		
	 //   	geometry_msgs::Pose left_start_pose;
	 //   		left_start_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,0);  
		// 	left_start_pose.position.x = 0.30;   
		// 	left_start_pose.position.y = 0.797189;
		// 	left_start_pose.position.z = 0.85;

		// group.setPoseTarget(left_start_pose);


		// group.setJointValueTarget(left_mid_position);
		// group.plan(my_plan);
		// group.execute(my_plan);

		// sleep(3.0);


		group.setMaxVelocityScalingFactor(0.2);

		// Attempt 1

		std::vector<double> left_just_pre_sew = {2.650497,-0.489527,1.358955,0.761918, -0.032624, 1.743939, 2.685309};
		
		group.setJointValueTarget(left_just_pre_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		std::vector<double> left_just_above_sew = {2.712010,-0.543950, 1.314090, 0.874825, 0.009432, 1.589310, 2.712379};
		
		group.setJointValueTarget(left_just_above_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		std::vector<double> left_tilted = {2.665224, -0.531388, 1.318388, 1.067499, -0.009128,1.403775, 2.712981};
		
		group.setJointValueTarget(left_tilted);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);


		std::vector<double> left_under_foot = {2.673439, -0.553307, 1.246899, 1.303732, -0.030376, 1.173526, 2.566982};
		
		group.setJointValueTarget(left_under_foot);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		
		std::vector<double> left_sew = {2.825593, -0.603775, 1.136042, 1.283638, 0.079296, 1.150045, 2.518556};
		
		group.setJointValueTarget(left_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(5.0);

		group.setMaxVelocityScalingFactor(0.02);
		
		

		float xoff = 284.0 - fabric_x;
		float yoff = 265.0 - fabric_y;
		float angleoff = 0.389 - fabric_orien;


		ROS_INFO("X Offset in pixels: %f", xoff);
		ROS_INFO("y Offset in pixels: %f", yoff);
		ROS_INFO("Anlge Offset in radians: %f", angleoff);
		ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
		ROS_INFO("y Offset in millimeters: %f", yoff * 0.5);

		sleep(5.0);

		geometry_msgs::PoseStamped sew_pose;
		sew_pose = group.getCurrentPose();
		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,M_PI-angleoff);

		sleep(1.0);

		moveit_msgs::Constraints arm_left_constraints;
		moveit_msgs::PositionConstraint arm_left_position_constraints;
		arm_left_position_constraints.link_name = "arm_left_link_tool0";
		arm_left_position_constraints.header.frame_id = "/torso_base_link";
		arm_left_position_constraints.weight= 100000.0;

		arm_left_position_constraints.target_point_offset.z = 0.001;
		arm_left_position_constraints.target_point_offset.x = 0.001;
		arm_left_position_constraints.target_point_offset.y= 0.001;
			
		arm_left_constraints.position_constraints = {arm_left_position_constraints};

		group.setPathConstraints(arm_left_constraints);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		sleep(10.0);
		group.execute(my_plan);
		sleep(1.0);
		group.clearPathConstraints();


		//while(xoff > 10 || xoff < -10 || (yoff > 10) || (yoff < -10)) {


			xoff = 284.0 - fabric_x;
			yoff = 265.0 - fabric_y;
			//angleoff = 0.407 - fabric_orien;



			ROS_INFO("X Offset in pixels: %f", xoff);
			ROS_INFO("Y Offset in pixels: %f", yoff);
			ROS_INFO("Anlge Offset in radians: %f", angleoff);
			ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
			ROS_INFO("Y Offset in millimeters: %f", yoff * 0.5);


			//geometry_msgs::PoseStamped sew_pose;
			sew_pose = group.getCurrentPose();

			sew_pose.pose.position.x = sew_pose.pose.position.x + yoff*0.00018;
			sew_pose.pose.position.y = sew_pose.pose.position.y + xoff*0.00018;	

			sleep(1.0);

			group.setPoseTarget(sew_pose);
			group.plan(my_plan);
			group.execute(my_plan);


			sleep(1.0);
		//}
   	}

	if(test == 4) {

		group.setMaxVelocityScalingFactor(0.05);
		
		sleep(2.0);


		float xoff = 284.0 - fabric_x;
		float yoff = 265.0 - fabric_y;
		float angleoff = 0.389 - fabric_orien;

		ROS_INFO("X Offset in pixels: %f", xoff);
		ROS_INFO("y Offset in pixels: %f", yoff);
		ROS_INFO("Anlge Offset in radians: %f", angleoff);
		ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
		ROS_INFO("y Offset in millimeters: %f", yoff * 0.5);

		sleep(5.0);

		geometry_msgs::PoseStamped sew_pose;
		sew_pose = group.getCurrentPose();
		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,M_PI-angleoff);

		sleep(1.0);

		moveit_msgs::Constraints arm_left_constraints;
		moveit_msgs::PositionConstraint arm_left_position_constraints;
		arm_left_position_constraints.link_name = "arm_left_link_tool0";
		arm_left_position_constraints.header.frame_id = "/torso_base_link";
		arm_left_position_constraints.weight= 100000.0;

		arm_left_position_constraints.target_point_offset.z = 0.001;
		arm_left_position_constraints.target_point_offset.x = 0.001;
		arm_left_position_constraints.target_point_offset.y= 0.001;
			
		arm_left_constraints.position_constraints = {arm_left_position_constraints};

		group.setPathConstraints(arm_left_constraints);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		sleep(4.0);
		group.execute(my_plan);
		sleep(2.0);
		group.clearPathConstraints();


		//while(xoff > 10 || xoff < -10 || (yoff > 10) || (yoff < -10)) {



			xoff = 284.0 - fabric_x;
			yoff = 265.0 - fabric_y;
			

			ROS_INFO("X Offset in pixels: %f", xoff);
			ROS_INFO("Y Offset in pixels: %f", yoff);
			ROS_INFO("Anlge Offset in radians: %f", angleoff);
			ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
			ROS_INFO("Y Offset in millimeters: %f", yoff * 0.5);


			//geometry_msgs::PoseStamped sew_pose;
			sew_pose = group.getCurrentPose();

			sew_pose.pose.position.x = sew_pose.pose.position.x + yoff*0.00018;
			sew_pose.pose.position.y = sew_pose.pose.position.y + xoff*0.00018;	

			sleep(1.0);

			group.setPoseTarget(sew_pose);
			group.plan(my_plan);
			group.execute(my_plan);


			sleep(1.0);
		//}
	}
  
	// Show off Robot Pickup demo

  	if(test == 5) {

  		std::vector<double> left_start_position = {-1.2725,0.7272, 2.606, -2.349, -2.7848, -1.284230, 3.12};
  		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		geometry_msgs::PoseStamped goal_pose;
		goal_pose = group.getCurrentPose();

		ROS_INFO("%f", ((390.0 - fabric_x)*0.00125));
		ROS_INFO("%f", ((200.0 - fabric_y)*0.00125));

		goal_pose.pose.position.z = 0.832870;
		goal_pose.pose.position.x = 0.254483; + (390.0 - fabric_x)*0.00125;
		goal_pose.pose.position.y = 1.058344; + (200.0 - fabric_y)*0.00125;

		group.setPoseTarget(goal_pose, "arm_left_link_tool0");


		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		sleep(1.0);

		sleep(2.0);

		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		sleep(1.0);

		gripper_pub.publish(open);
		gripper_pub.publish(open);
		gripper_pub.publish(open);

		sleep(1.0);


		// Vision Center
		// x = 380 y = 206
		// x inverted, y not inverted
		// pixel = 1.25 mm

  	}


	if(test == 6) {


		group.setMaxVelocityScalingFactor(0.2);
  		std::vector<double> left_start_position = {-1.4688,0.7039, -0.403209, 2.2110, 0.642783, -0.4479, -2.00};
  		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

	}

	if(test == 7) {


	    moveit_msgs::RobotTrajectory trajectory;

		std::vector<double> left_start_position = {-1.4688,0.7039, -0.403209, 2.2110, 0.642783, -0.4479, -2.00};
  		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);


	    geometry_msgs::Pose Pose1;
	    Pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,0); 
	  
		Pose1.position.x = 0.4;   
		Pose1.position.y = 0.870986;
		Pose1.position.z = 0.808254;

	    geometry_msgs::Pose Pose2;

	    Pose2.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,0); 
	  
		Pose2.position.x = -0.3;   
		Pose2.position.y = 0.870986;
		Pose2.position.z = 0.808254;


		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(Pose1);
		waypoints.push_back(Pose2);

		group.setPlanningTime(2.0);

		group.setPoseTarget(Pose1);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		// group.setPoseTarget(Pose2);
		// group.plan(my_plan);
		// group.execute(my_plan);

    	double fraction = group.computeCartesianPath(waypoints, 0.01,  // eef_step
    	                                             0,   // jump_threshold
    	                                             trajectory);

    	    // First to create a RobotTrajectory object
    	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_left");

    		// Second get a RobotTrajectory from trajectory
    	rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);

    	// Thrid create a IterativeParabolicTimeParameterization object
   		trajectory_processing::IterativeParabolicTimeParameterization iptp;

   		ROS_INFO("number of points %d", (int)rt.getWayPointCount());

   		int trajlen = (int)rt.getWayPointCount();
   		for( int i = 0; i < trajlen; i++) {
   			rt.setWayPointDurationFromPrevious (i, 0.02+(trajlen-1-i)*0.001);
   		}

   		rt.getRobotTrajectoryMsg(trajectory);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    	cart_plan.trajectory_ = trajectory;
    	group.execute(cart_plan);


	}

	if(test == 8) {

		//group.setMaxVelocityScalingFactor(0.2);


		moveit_msgs::RobotTrajectory trajectory;

		// std::vector<double> left_start_position = {-1.4688,0.7039, -0.403209, 2.2110, 0.642783, -0.4479, -2.00};
  // 		group.setJointValueTarget(left_start_position);
		// group.plan(my_plan);
		// group.execute(my_plan);


		geometry_msgs::PoseStamped Pose1;
		geometry_msgs::PoseStamped Pose2;
		geometry_msgs::PoseStamped Pose3;

		Pose1 = group.getCurrentPose();
		Pose2 = group.getCurrentPose();
		Pose3 = group.getCurrentPose();

		sleep(4.0);


		Pose2.pose.position.y = Pose2.pose.position.y + 0.1;
		Pose2.pose.position.z = Pose2.pose.position.z + 0.15;
		Pose2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,0);

		Pose3.pose.position.y = Pose2.pose.position.y;
		Pose3.pose.position.z = Pose2.pose.position.z - 0.12;
		Pose3.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,0);

		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(Pose1.pose);
		waypoints.push_back(Pose2.pose);
		waypoints.push_back(Pose3.pose);


		group.setPlanningTime(2.0);

		sleep(2.0);

		double fraction = group.computeCartesianPath(waypoints, 0.01,  // eef_step
    	                                             0,   // jump_threshold
    	                                             trajectory);

    	    // First to create a RobotTrajectory object
    	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_left");

    		// Second get a RobotTrajectory from trajectory
    	rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);

    	// Thrid create a IterativeParabolicTimeParameterization object
   		trajectory_processing::IterativeParabolicTimeParameterization iptp;

   		ROS_INFO("number of points %d", (int)rt.getWayPointCount());

   		int trajlen = (int)rt.getWayPointCount();

   		for( int i = 0; i < trajlen; i++) {
   			rt.setWayPointDurationFromPrevious (i, -0.02+(trajlen-i)*0.002);
   			if(i>trajlen/2)
   				rt.setWayPointDurationFromPrevious (i, 0.1);
   		}

   		rt.getRobotTrajectoryMsg(trajectory);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    	cart_plan.trajectory_ = trajectory;
    	group.execute(cart_plan);

	}

	// moveit_msgs::Constraints arm_left_constraints;
	// moveit_msgs::PositionConstraint arm_left_position_constraints;
	// arm_left_position_constraints.link_name = "arm_left_link_tool0";
	// arm_left_position_constraints.header.frame_id = "/torso_base_link";
	// arm_left_position_constraints.weight= 100000.0;

	sleep(1.0);

	ROS_INFO("Test Complete");

    sleep(1.0);

	ros::shutdown();
	return 0;
}

