#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
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
#include <iostream>



float fabric_x = 0;
float fabric_y = 0;
float fabric_orien = 0;
float goal_x = 587.0;
float goal_y = 554.0;
float goal_orien = 0.45;
float fine_orien = 0.0;
float fine_x = 0;
float fine_y = 0;

void poseUpdater(std_msgs::Float32MultiArray msg)
{
  	fabric_x =  msg.data[0];
  	fabric_y =  msg.data[1];
  	fabric_orien =  msg.data[2];
  	fine_orien = msg.data[3];
  	fine_x = msg.data[4];
  	fine_y = msg.data[5];
}

int main(int argc, char **argv)
{
	int test = 1;
	ros::init(argc, argv, "test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	sleep(1.0);

	ros::Subscriber posedub = node_handle.subscribe("fabric_pose", 1000, poseUpdater);
 	ros::Publisher gripper_pub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);
 	ros::Publisher clamp_pub = node_handle.advertise<std_msgs::Empty>("clamp_pedal", 1);
 	ros::Publisher start_pub = node_handle.advertise<std_msgs::Empty>("start_pedal", 1);

 	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output reset;
 	reset.rACT = 0; reset.rGTO = 0; reset.rATR = 0; reset.rPR = 0; reset.rSP = 0; reset.rFR = 0;

	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output open;
 	open.rACT = 1; open.rGTO = 1; open.rATR = 0; open.rPR = 50; open.rSP = 255; open.rFR = 150;

 	std_msgs::Empty Ping;



	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output close;
 	close.rACT = 1; close.rGTO = 1;	close.rATR = 0; close.rPR = 255; close.rSP = 255; close.rFR = 200;

 	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output release;
 	release.rACT = 1; release.rGTO = 1;	release.rATR = 0; release.rPR = 215; release.rSP = 255; release.rFR = 200;

	moveit::planning_interface::MoveGroupInterface group("arm_left");
	moveit::planning_interface::MoveGroupInterface torso_group("torso");
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
	arm_right_group.setEndEffectorLink("arm_right_link_7_t");



	std::vector<double> right_group_variable_values;
	std::vector<double> group_variable_values;
	std::vector<double> torso_variable_values;

	group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    torso_group.getCurrentState()->copyJointGroupPositions(
    torso_group.getCurrentState()->getRobotModel()->getJointModelGroup(torso_group.getName()), torso_variable_values);

	arm_right_group.getCurrentState()->copyJointGroupPositions(
    arm_right_group.getCurrentState()->getRobotModel()->getJointModelGroup(arm_right_group.getName()), right_group_variable_values);
    geometry_msgs::PoseStamped sew_pose;
    sleep(2);

	if(test == 1) {

		//group.setPlanningTime(10.0);

		group.setGoalJointTolerance(0.001);
		torso_group.setGoalJointTolerance(0.001);

		group.setMaxVelocityScalingFactor(0.2);
		torso_group.setMaxVelocityScalingFactor(0.2);


     	torso_group.setJointValueTarget({0.289002});
		torso_group.plan(my_plan);
		torso_group.execute(my_plan);

		sleep(1.0);

		std::vector<double> start_pos = {-1.104773, 0.758963, 1.874357, -1.709933, 0.269328, -1.131166, -0.690442};
		group.setJointValueTarget(start_pos);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		gripper_pub.publish(open);
		gripper_pub.publish(open);
		gripper_pub.publish(open);

		std::vector<double> pre_grab =  {-0.428202, 0.420311, 1.135222, -1.520615, 0.562379, -0.099132, -0.368607};
		
		group.setJointValueTarget(pre_grab);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(3.0);


   		moveit_msgs::RobotTrajectory trajectory;
		geometry_msgs::PoseStamped preGrabPose;
		geometry_msgs::PoseStamped grabPose;

		preGrabPose = group.getCurrentPose();
		grabPose = group.getCurrentPose();

		grabPose.pose.position.x = grabPose.pose.position.x + 0.13;
		grabPose.pose.position.y = grabPose.pose.position.y + 0.01;

		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(preGrabPose.pose);
		waypoints.push_back(grabPose.pose);

		group.setMaxVelocityScalingFactor(0.2);

		double fraction = group.computeCartesianPath(waypoints, 0.04, 0, trajectory);

    	// First to create a RobotTrajectory object
    	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_left");

    	// Second get a RobotTrajectory from trajectory
    	rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);

    	// Thrid create a IterativeParabolicTimeParameterization object
   		trajectory_processing::IterativeParabolicTimeParameterization iptp;

   		ROS_INFO("number of points %d", (int)rt.getWayPointCount());

   		int trajlen = (int)rt.getWayPointCount();
		

		for( int i = 1; i < trajlen; i++) {
   			rt.setWayPointDurationFromPrevious (i, 0.3);
   		}

   		rt.getRobotTrajectoryMsg(trajectory);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    	cart_plan.trajectory_ = trajectory;
    	group.execute(cart_plan);

    	sleep(1.0);

    	gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		sleep(1.0);


   		moveit_msgs::RobotTrajectory trajectory2;
		preGrabPose = group.getCurrentPose();
		grabPose = group.getCurrentPose();

		grabPose.pose.position.z = grabPose.pose.position.z + 0.10;

		std::vector<geometry_msgs::Pose> waypoints2;
		waypoints2.push_back(preGrabPose.pose);
		waypoints2.push_back(grabPose.pose);


		fraction = group.computeCartesianPath(waypoints2, 0.03, 0, trajectory2);

    	// First to create a RobotTrajectory object
    	robot_trajectory::RobotTrajectory rt2(group.getCurrentState()->getRobotModel(), "arm_left");

    	// Second get a RobotTrajectory from trajectory
    	rt2.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory2);

    	// Thrid create a IterativeParabolicTimeParameterization object
   		trajectory_processing::IterativeParabolicTimeParameterization iptp2;

   		ROS_INFO("number of points %d", (int)rt2.getWayPointCount());

   		trajlen = (int)rt2.getWayPointCount();

   		for( int i = 1; i < trajlen; i++) {
   			rt2.setWayPointDurationFromPrevious (i, 0.3);
   		}

   		rt2.getRobotTrajectoryMsg(trajectory2);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan2;
    	cart_plan2.trajectory_ = trajectory2;
    	group.execute(cart_plan2);

		sleep(2.0);

		group.setJointValueTarget(start_pos);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);



		// std::vector<double> post_grab = {-0.646624, 0.949381, 1.494978, -1.234475, 0.310137, -1.211981, -0.563091};
		// //std::vector<double> post_grab = {-1.254029, 0.610950, 1.888877, -1.760919, 0.502705, -0.246014, -0.109544};
		
		// group.setJointValueTarget(post_grab);
		// group.plan(my_plan);
		// group.execute(my_plan);

		//sleep(1.0);

		std::vector<double> left_pre_slide = {-0.527996, 0.709739, 1.505777, -1.494583, 0.445128, -0.968352, -0.805972};
		group.setJointValueTarget(left_pre_slide);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);
 
		// {-0.286087, 1.211214, 1.255206, -1.131865, 0.425991, -1.152399, -0.656965}


		std::vector<double> left_post_slide = {-0.081403, 1.274806, 1.208306, -0.736706, 0.417379, -1.407921, -0.634256};
		group.setJointValueTarget(left_post_slide);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		do
		{
			std::cout << '\n' << "Press any Key to Continue";
		} while(std::cin.get() != '\n');

		//
		clamp_pub.publish(Ping);

		sew_pose = group.getCurrentPose();

		group.setMaxVelocityScalingFactor(0.02);
		group.setGoalJointTolerance(0.00001);

		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);

		sleep(1.0);

		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,tf::getYaw(sew_pose.pose.orientation)-fine_orien);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);


		sew_pose.pose.position.x = sew_pose.pose.position.x + fine_y*0.000095;
		sew_pose.pose.position.y = sew_pose.pose.position.y - fine_x*0.00009;	

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);


		//


		do
		{
			std::cout << '\n' << "Press any Key to Continue";
		} while(std::cin.get() != '\n');

		clamp_pub.publish(Ping);

		geometry_msgs::PoseStamped releasePose = group.getCurrentPose();
		releasePose.pose.position.z = releasePose.pose.position.z + 0.01;
		group.setPoseTarget(releasePose.pose);
		group.plan(my_plan);
		group.execute(my_plan);


		sleep(1.0);

		gripper_pub.publish(release);
		gripper_pub.publish(release);
		gripper_pub.publish(release);

		sleep(1.0);

		// publish Start

		do
		{
			std::cout << '\n' << "Press any Key to Continue";
		} while(std::cin.get() != '\n');

		clamp_pub.publish(Ping);		


		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		
		sleep(1.0);

		group.setMaxVelocityScalingFactor(0.3);

		std::vector<double> left_pre_spin = {-0.577356, 1.017097, 1.390288, -1.676352, 0.374808, -0.938538, -0.793730};
		group.setJointValueTarget(left_pre_spin);
		group.plan(my_plan);
		group.execute(my_plan);
		

		sleep(1.0);


		group.setMaxVelocityScalingFactor(1.0);

		std::vector<double> left_spin = {-1.178165, 1.421353, 1.191402, -1.607475, 0.536316, -1.371288, -0.278463};
		group.setJointValueTarget(left_spin);
		group.plan(my_plan);
		group.execute(my_plan);

		
		sleep(1.0);
		

		torso_group.setJointValueTarget({-0.884527});
		torso_group.plan(my_plan);
		torso_group.execute(my_plan);

		sleep(1.0);
		

		std::vector<double> left_above_machine_right =  {-0.645993, 1.464338, 1.472378, -1.380613, -0.502675, -1.419525, -2.011410};
		group.setJointValueTarget(left_above_machine_right);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		
		std::vector<double> right_sew = {-0.497078, 1.332876, 1.786358, -1.766493, -0.718905, -1.066329, -1.913416};
		group.setJointValueTarget(right_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		do
		{
			std::cout << '\n' << "Press any Key to Continue";
		} while(std::cin.get() != '\n');

		//

		clamp_pub.publish(Ping);		

		sew_pose = group.getCurrentPose();

		group.setMaxVelocityScalingFactor(0.02);
		group.setGoalJointTolerance(0.00001);

		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);

		sleep(1.0);

		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,tf::getYaw(sew_pose.pose.orientation)+fine_orien);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);


		sew_pose.pose.position.x = sew_pose.pose.position.x + fine_y*0.000095;
		sew_pose.pose.position.y = sew_pose.pose.position.y + fine_x*0.00009;	

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);


		// P

		do
		{
			std::cout << '\n' << "Press any Key to Continue";
		} while(std::cin.get() != '\n');


		clamp_pub.publish(Ping);
		releasePose = group.getCurrentPose();
		releasePose.pose.position.z = releasePose.pose.position.z + 0.01;
		group.setPoseTarget(releasePose.pose);
		group.plan(my_plan);
		group.execute(my_plan);



		sleep(1.0);



		gripper_pub.publish(release);
		gripper_pub.publish(release);
		gripper_pub.publish(release);

		sleep(1.0);

		// Publish Start

		do
		{
			std::cout << '\n' << "Press any Key to Continue";
		} while(std::cin.get() != '\n');

		clamp_pub.publish(Ping);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		
		sleep(1.0);

		group.setMaxVelocityScalingFactor(0.2);

		std::vector<double> right_post_sew = {-0.635614, 1.497216, 1.651262, -2.099519, -0.639761, -0.992546, -1.701275};
		group.setJointValueTarget(right_post_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);


		group.setJointValueTarget(left_spin);
		group.plan(my_plan);
		group.execute(my_plan);

		
		sleep(1.0);

     	torso_group.setJointValueTarget({0.289002});
		torso_group.plan(my_plan);
		torso_group.execute(my_plan);

		sleep(1.0);

		return 1; 

	}
	
	if(test == 2) {

	geometry_msgs::PoseStamped test_pose;
	test_pose = arm_right_group.getCurrentPose();
	
    ROS_INFO("X Position %f", test_pose.pose.position.x);
	ROS_INFO("Y Position %f", test_pose.pose.position.y);
    ROS_INFO("Z Position %f", test_pose.pose.position.z);

    ROS_INFO("Joint 1 %f", right_group_variable_values[0]);
	ROS_INFO("Joint 2 %f", right_group_variable_values[1]);
	ROS_INFO("Joint 3 %f", right_group_variable_values[2]);
	ROS_INFO("Joint 4 %f", right_group_variable_values[3]);
	ROS_INFO("Joint 5 %f", right_group_variable_values[4]);
	ROS_INFO("Joint 6 %f", right_group_variable_values[5]);
	ROS_INFO("Joint 7 %f", right_group_variable_values[6]);

	ROS_INFO("Left Joints: {%f, %f, %f, %f, %f, %f, %f}", group_variable_values[0], group_variable_values[1], group_variable_values[2], 
			group_variable_values[3], group_variable_values[4], group_variable_values[5], group_variable_values[6]);
	ROS_INFO("Right Joints: {%f, %f, %f, %f, %f, %f, %f}", right_group_variable_values[0], right_group_variable_values[1], right_group_variable_values[2], 
			right_group_variable_values[3], right_group_variable_values[4], right_group_variable_values[5], right_group_variable_values[6]);
	ROS_INFO("Torso Joint: {%f}", torso_variable_values[0]);
   	sleep(1.0);
   	}

   	if(test == 3) {


   		group.setMaxVelocityScalingFactor(0.5);
   		group.setPlanningTime(10.0);
   		moveit_msgs::RobotTrajectory trajectory;
  		std::vector<double> left_start_position = {1.030,-1.4833, 2.896, 2.318, -0.897, 0.593, -0.717};
  

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

		std::vector<geometry_msgs::Pose> waypoints;
		waypoints.push_back(preGrabPose.pose);
		waypoints.push_back(grabPose.pose);


		

		double fraction = group.computeCartesianPath(waypoints, 0.05, 0, trajectory);

    	// First to create a RobotTrajectory object
    	robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_left");

    	// Second get a RobotTrajectory from trajectory
    	rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory);

    	// Thrid create a IterativeParabolicTimeParameterization object
   		trajectory_processing::IterativeParabolicTimeParameterization iptp;

   		ROS_INFO("number of points %d", (int)rt.getWayPointCount());

   		int trajlen = (int)rt.getWayPointCount();

   		for( int i = 1; i < trajlen; i++) {
   			rt.setWayPointDurationFromPrevious (i, 0.2);
   		}

   		rt.getRobotTrajectoryMsg(trajectory);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
    	cart_plan.trajectory_ = trajectory;
    	group.execute(cart_plan);


    	//sleep(2.0);

		gripper_pub.publish(close);
		gripper_pub.publish(close);
		gripper_pub.publish(close);

		sleep(1.0);

		std::vector<double> left_post_grab = { 2.3934, -0.822231, 2.940899, 1.63835, -1.55050, -0.51859, 0.473729};

		group.setJointValueTarget(left_post_grab);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		group.setJointValueTarget(left_start_position);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);


		group.setMaxVelocityScalingFactor(0.2);


		std::vector<double> left_above_platform = {2.422718, -0.260845, 1.421150, 1.041862, -0.117524, 1.497682, 2.628672};
		
		group.setJointValueTarget(left_above_platform);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

		std::vector<double> left_on_platform ={2.893582, -0.458030, 1.225651, 0.894022, 0.096777, 1.502754, 2.434789};
		
		group.setJointValueTarget(left_on_platform);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);

	
		std::vector<double> left_sew = {2.870913, -0.678207, 1.143393, 1.212422, 0.124116, 1.197173, 2.518376};
		
		group.setJointValueTarget(left_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		test = 4;
   	}

	if(test == 4) {

		group.setMaxVelocityScalingFactor(0.02);
		group.setGoalJointTolerance(0.00001);


		sleep(4.0);

		float xoff = goal_x - fabric_x;
		float yoff = goal_y - fabric_y;
		float angleoff = goal_orien - fabric_orien;
		
		ROS_INFO("X Offset in pixels: %f", xoff);
		ROS_INFO("y Offset in pixels: %f", yoff);
		ROS_INFO("Anlge Offset in radians: %f", angleoff);

		// ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
		// ROS_INFO("y Offset in millimeters: %f", yoff * 0.5);

		
		// ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
		// ROS_INFO("y Offset in millimeters: %f", yoff * 0.5);
	
		sew_pose = group.getCurrentPose();
		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,tf::getYaw(sew_pose.pose.orientation)-angleoff);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);
		
		sleep(6.0);

		xoff = goal_x - fabric_x;
		yoff = goal_y - fabric_y;

		sew_pose.pose.position.x = sew_pose.pose.position.x + yoff*0.000068;
		sew_pose.pose.position.y = sew_pose.pose.position.y + xoff*0.00005;	

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);

		ROS_INFO("Begin Step 2");
		
		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);
		test = 5;

		// moveit_msgs::Constraints arm_left_constraints;
		// moveit_msgs::PositionConstraint arm_left_position_constraints;
		// arm_left_position_constraints.link_name = "arm_left_link_tool0";
		// arm_left_position_constraints.header.frame_id = "/torso_base_link";
		// arm_left_position_constraints.weight= 100000.0;

		// arm_left_position_constraints.target_point_offset.z = 0.001;
		// arm_left_position_constraints.target_point_offset.x = 0.001;
		// arm_left_position_constraints.target_point_offset.y= 0.001;
			
		// arm_left_constraints.position_constraints = {arm_left_position_constraints};

		// group.setPathConstraints(arm_left_constraints);

		// group.setPoseTarget(sew_pose.pose);
		// group.plan(my_plan);
		// sleep(2.0);
		// group.execute(my_plan);
		// sleep(2.0);
		// group.clearPathConstraints();


		// while((xoff) > 3 || (xoff < -3)|| (yoff > 3) || (yoff < -3)) {


		// 	xoff = goal_x - fabric_x;
		// 	yoff = goal_y - fabric_y;
			

		// 	ROS_INFO("X Offset in pixels: %f", xoff);
		// 	ROS_INFO("Y Offset in pixels: %f", yoff);
		// 	ROS_INFO("Anlge Offset in radians: %f", angleoff);
		// 	ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
		// 	ROS_INFO("Y Offset in millimeters: %f", yoff * 0.5);


		// 	//geometry_msgs::PoseStamped sew_pose;
		// 	sew_pose = group.getCurrentPose();

		// 	sew_pose.pose.position.x = sew_pose.pose.position.x + yoff*0.00018;
		// 	sew_pose.pose.position.y = sew_pose.pose.position.y + xoff*0.00018;	

		// 	sleep(1.0);

		// 	group.setPoseTarget(sew_pose);
		// 	group.plan(my_plan);
		// 	group.execute(my_plan);


		// 	sleep(1.0);
		// }
	}

	if(test == 5) {
	

		sew_pose = group.getCurrentPose();

		group.setMaxVelocityScalingFactor(0.02);
		group.setGoalJointTolerance(0.00001);


		sleep(2.0);

		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);

		sleep(2.0);

		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,tf::getYaw(sew_pose.pose.orientation)-fine_orien);

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(4.0);

		ROS_INFO("X Offset in pixels: %f", fine_x);
		ROS_INFO("y Offset in pixels: %f", fine_y);
		ROS_INFO("Anlge Offset in radians: %f", fine_orien);

		sleep(2.0);

		sew_pose.pose.position.x = sew_pose.pose.position.x + fine_y*0.000092;
		sew_pose.pose.position.y = sew_pose.pose.position.y - fine_x*0.000090;	

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);
		return 1;

	}
  

	if(test == 6) {

		sleep(1.0);

		group.setMaxVelocityScalingFactor(0.1);

		torso_group.setJointValueTarget({-0.884527});
		torso_group.plan(my_plan);
		torso_group.execute(my_plan);

		sleep(1.0);
		

		std::vector<double> left_above_machine_right =  {-0.645993, 1.464338, 1.472378, -1.380613, -0.502675, -1.419525, -2.011410};
		group.setJointValueTarget(left_above_machine_right);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(1.0);

		group.setMaxVelocityScalingFactor(0.02);


		std::vector<double> right_sew = {-0.493925, 1.326519, 1.789426, -1.752869, -0.720622, -1.071448, -1.922890};
		group.setJointValueTarget(right_sew);
		group.plan(my_plan);
		group.execute(my_plan);

		sleep(2.0);


		group.setMaxVelocityScalingFactor(0.1);

		std::vector<double> right_post_sew = {-0.635614, 1.497216, 1.651262, -2.099519, -0.639761, -0.992546, -1.701275};
		group.setJointValueTarget(right_post_sew);
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
  		// group.setJointValueTarget(left_start_position);
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


	if(test == 9) {

		std::vector<double> right_start_position = 	{0.613899, -0.726459, -0.983373, 1.330265, -0.653947, 1.275680, 1.884841};
		arm_right_group.setJointValueTarget(right_start_position);
		arm_right_group.plan(my_plan);
		arm_right_group.execute(my_plan);

		sleep(2.0);
		



	    moveit_msgs::RobotTrajectory trajectory;


		geometry_msgs::PoseStamped Pose1 = arm_right_group.getCurrentPose();
		geometry_msgs::PoseStamped Pose2 = arm_right_group.getCurrentPose();
		geometry_msgs::PoseStamped Pose3 = arm_right_group.getCurrentPose();
		geometry_msgs::PoseStamped Pose4 = arm_right_group.getCurrentPose();
		geometry_msgs::PoseStamped Pose5 = arm_right_group.getCurrentPose();

		

		
		while(true) {

		std::vector<geometry_msgs::Pose> waypoints;

		Pose2.pose.position.y = Pose1.pose.position.y + 0.2;
		Pose3.pose.position.z = Pose1.pose.position.z + 0.2;
		Pose4.pose.position.y = Pose1.pose.position.y - 0.2;
		Pose5.pose.position.z = Pose1.pose.position.z - 0.2;

		waypoints.push_back(Pose1.pose);
		waypoints.push_back(Pose2.pose);
		waypoints.push_back(Pose3.pose);
		waypoints.push_back(Pose4.pose);
		waypoints.push_back(Pose5.pose);
		waypoints.push_back(Pose2.pose);
		waypoints.push_back(Pose3.pose);
		waypoints.push_back(Pose4.pose);
		waypoints.push_back(Pose5.pose);
		waypoints.push_back(Pose2.pose);
		waypoints.push_back(Pose3.pose);
		waypoints.push_back(Pose4.pose);
		waypoints.push_back(Pose5.pose);
		waypoints.push_back(Pose2.pose);
		waypoints.push_back(Pose3.pose);
		waypoints.push_back(Pose4.pose);
		waypoints.push_back(Pose5.pose);
		waypoints.push_back(Pose2.pose);
		waypoints.push_back(Pose3.pose);
		waypoints.push_back(Pose4.pose);
		waypoints.push_back(Pose5.pose);

		//ROS_INFO("%i", waypoints.size());

		arm_right_group.setPlanningTime(5.0);

		sleep(2.0);

		double fraction = arm_right_group.computeCartesianPath(waypoints, 0.01,  // eef_step
		                                             0,   // jump_threshold
		                                             trajectory);

		    // First to create a RobotTrajectory object
		robot_trajectory::RobotTrajectory rt(arm_right_group.getCurrentState()->getRobotModel(), "arm_right");

			// Second get a RobotTrajectory from trajectory
		rt.setRobotTrajectoryMsg(*arm_right_group.getCurrentState(), trajectory);

		// Thrid create a IterativeParabolicTimeParameterization object
		trajectory_processing::IterativeParabolicTimeParameterization iptp;

		ROS_INFO("number of points %d", (int)rt.getWayPointCount());

		int trajlen = (int)rt.getWayPointCount();

		for( int i = 0; i < trajlen; i++) {
			rt.setWayPointDurationFromPrevious (i, 0.1);
		}

		rt.getRobotTrajectoryMsg(trajectory);
		moveit::planning_interface::MoveGroupInterface::Plan cart_plan;
		cart_plan.trajectory_ = trajectory;
		arm_right_group.execute(cart_plan);

		}
	// moveit_msgs::Constraints arm_left_constraints;
	// moveit_msgs::PositionConstraint arm_left_position_constraints;
	// arm_left_position_constraints.link_name = "arm_left_link_tool0";
	// arm_left_position_constraints.header.frame_id = "/torso_base_link";
	// arm_left_position_constraints.weight= 100000.0;

		sleep(1.0);
	}
	if(test == 10){

		sleep(1.0);
		std::vector<double> targetpose = {-0.527996, 0.709739, 1.505777, -1.494583, 0.445128, -0.968352, -0.805972};
		group.setJointValueTarget(targetpose);
		group.plan(my_plan);
		group.execute(my_plan);
	}


	// Flatten
	if(test == 11){

		sleep(1.0);
		sew_pose = group.getCurrentPose();
		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,tf::getYaw(sew_pose.pose.orientation));

		group.setPoseTarget(sew_pose.pose);
		group.plan(my_plan);
		group.execute(my_plan);
	}

	if(test == 12){

	sleep(1);
	clamp_pub.publish(Ping);
	sleep(1);
	start_pub.publish(Ping);

	}
	ROS_INFO("Test Complete");

    sleep(1.0);

	ros::shutdown();
	return 0;
}

