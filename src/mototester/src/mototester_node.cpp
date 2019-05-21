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


float fabric_x = 0;
float fabric_y = 0;
float fabric_orien = 0;
float goal_x = 284.0;
float goal_y = 265.0;
float goal_orien = 0.389;

void poseUpdater(std_msgs::Float32MultiArray msg)
{
  	fabric_x =  msg.data[0];
  	fabric_y =  msg.data[1];
  	fabric_orien =  msg.data[2];
}

int main(int argc, char **argv)
{
	int test = 4;
	ros::init(argc, argv, "test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	sleep(1.0);

	ros::Subscriber posedub = node_handle.subscribe("fabric_pose", 1000, poseUpdater);
 	ros::Publisher gripper_pub = node_handle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput", 10);

 	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output reset;
 	reset.rACT = 0; reset.rGTO = 0; reset.rATR = 0; reset.rPR = 0; reset.rSP = 0; reset.rFR = 0;

	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output open;
 	open.rACT = 1; open.rGTO = 1; open.rATR = 0; open.rPR = 0; open.rSP = 255; open.rFR = 150;

	robotiq_2f_gripper_control::Robotiq2FGripper_robot_output close;
 	close.rACT = 1; close.rGTO = 1;	close.rATR = 0; close.rPR = 255; close.rSP = 255; close.rFR = 200;


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
	arm_right_group.setEndEffectorLink("arm_right_link_tool0");


	std::vector<double> group_variable_values;
	std::vector<double> torso_variable_values;

	group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	
	if(test == 2) {

	geometry_msgs::PoseStamped test_pose;
	test_pose = group.getCurrentPose();
	
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

	ROS_INFO("Joints: {%f, %f, %f, %f, %f, %f, %f}", group_variable_values[0], group_variable_values[1], group_variable_values[2], 
			group_variable_values[3], group_variable_values[4], group_variable_values[5], group_variable_values[6]	);
	sleep(1.0);
   	    
   	}

   	if(test == 3) {


   		group.setMaxVelocityScalingFactor(0.4);
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

		group.setMaxVelocityScalingFactor(0.5);

		group.setPlanningTime(5.0);

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


		group.setMaxVelocityScalingFactor(0.2);


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
		
		

		float xoff = goal_x - fabric_x;
		float yoff = goal_y - fabric_y;
		float angleoff = goal_orien - fabric_orien;


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


			xoff = goal_x - fabric_x;
			yoff = goal_y - fabric_y;
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


		float xoff = goal_x - fabric_x;
		float yoff = goal_y - fabric_y;
		float angleoff = goal_orien - fabric_orien;
		angleoff = 0.1;

		ROS_INFO("X Offset in pixels: %f", xoff);
		ROS_INFO("y Offset in pixels: %f", yoff);
		ROS_INFO("Anlge Offset in radians: %f", angleoff);
		ROS_INFO("X Offset in millimeters: %f", xoff * 0.5);
		ROS_INFO("y Offset in millimeters: %f", yoff * 0.5);

		sleep(5.0);

		geometry_msgs::PoseStamped sew_pose;
		sew_pose = group.getCurrentPose();
		sew_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2,0,tf::getYaw(sew_pose.pose.orientation)-angleoff);

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


			xoff = goal_x - fabric_x;
			yoff = goal_y - fabric_y;
			

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

