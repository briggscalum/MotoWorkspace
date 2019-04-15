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
#include <moveit_msgs/PositionConstraint.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>  
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
int test_num = 1;

void poseUpdater(std_msgs::Float32MultiArray msg)
{
  	fabric_x =  msg.data[0];
  	fabric_y =  msg.data[1];
  	fabric_orien =  msg.data[2];
}

int main(int argc, char **argv)
{
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
	close.rFR = 150;



	sleep(1.0);
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

	int test = 2;
	
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

		moveit_msgs::Constraints arm_left_constraints;
		moveit_msgs::PositionConstraint arm_left_position_constraints;
			arm_left_position_constraints.link_name = "arm_left_link_tool0";
			//arm_left_position_constraints.target_point_offset.y = 0.001;
			arm_left_position_constraints.target_point_offset.x = 0.001;
			arm_left_position_constraints.weight= 1000.0;


		arm_left_constraints.position_constraints = {arm_left_position_constraints};

		group.setPathConstraints(arm_left_constraints);
		//test_pose.position.x = 0.5;  
		//test_pose.position.y = 0.5;
		test_pose.pose.position.z = test_pose.pose.position.z + 0.5;

		group.setPoseTarget(test_pose);
		group.plan(my_plan);
	    group.execute(my_plan);

   	    sleep(2.0);
   	    
   	}


   	 //    right_home.position.x = right_home.position.x - 0.01;
   	 //    right_home.position.x = right_home.position.z - 0.01;

   	 //    arm_right_group.setPoseTarget(right_home, "arm_right_link_7_t");
   	 //    arm_right_group.plan(my_plan);
	    // arm_right_group.execute(my_plan);

	   	// ROS_INFO("Count: %i", grabcounter);


	    // if(grabcounter%5 == 0){

	    // 	right_home.position.x = -0.55;
	    // 	arm_right_group.setPoseTarget(right_home, "arm_right_link_7_t");
	    // 	arm_right_group.plan(my_plan);
	    // 	arm_right_group.execute(my_plan);

	    // 	sleep(5.0);

	    // 	ROS_INFO("Grabbing");

	    // 	right_home.position.x = -0.45;
	    // 	arm_right_group.setPoseTarget(right_home, "arm_right_link_7_t");
	    // 	arm_right_group.plan(my_plan);
	    // 	arm_right_group.execute(my_plan);

	    // 	sleep(3.0);

	    // }

	 // sleep(2.0);

	 //    std::vector<double> right_group_variable_values;
		// arm_right_group.getCurrentState()->copyJointGroupPositions(
	 //    arm_right_group.getCurrentState()->getRobotModel()->getJointModelGroup(arm_right_group.getName()), right_group_variable_values);

	 //    right_group_variable_values[6] = -0.3;
		// arm_right_group.setJointValueTarget(right_group_variable_values);
		// arm_right_group.plan(my_plan);
		// arm_right_group.execute(my_plan);
	//}


    //group.plan(my_plan);
    //group.execute(my_plan);


	sleep(5.0);

	ROS_INFO("Get Ready to Rumble");

    sleep(2.0);


    int starttime = ros::Time::now().toSec();
    int currenttime = starttime;
    int i = 0;
	// while(i < 50) {
	// 	group_variable_values[2] = group_variable_values[2] + 0.03;
	// 	group.setJointValueTarget(group_variable_values);
	// 	group.plan(my_plan);
 //    	group.asyncExecute(my_plan);
 //    	ROS_INFO("Move at time: %f", ros::Time::now().toSec());
 //    	ros::Duration(0.5).sleep();
 //    	i++;
	// }



	ros::shutdown();
	return 0;
}

