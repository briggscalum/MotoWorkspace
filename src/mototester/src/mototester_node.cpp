#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <motoman_msgs/DynamicJointTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#define PI 3.14159265


class PointGoal {
  
public:

double x;
double y;
double time;

};

int test_num = 1;

void poseUpdater(std_msgs::Float32MultiArray msg)
{
  	ROS_INFO("I heard: [%f]", msg.data[0]);
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

	// sleep(5.0);

	ros::Subscriber posedub = node_handle.subscribe("fabric_pose", 1000, poseUpdater);

	moveit::planning_interface::MoveGroupInterface group("arm_left");
	//moveit::planning_interface::MoveGroupInterface arm_left_group("arm_left");
	moveit::planning_interface::MoveGroupInterface arm_right_group("arm_right");


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	//arm_left_group.setPlannerId("RRTConnectkConfigDefault");
	arm_right_group.setPlannerId("RRTConnectkConfigDefault");
	group.setPlannerId("RRTConnectkConfigDefault");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	//group.setPlannerId("PRMstarkConfigDefault");

	group.setGoalJointTolerance(0.001);
	group.setEndEffectorLink("arm_left_link_7_t");
	//group.setEndEffectorLink("arm_right_link_7_t");

	geometry_msgs::Pose left_flat;



	std::vector<PointGoal> path(100);

	//Generate a Dummy Path
    for (int i = 0; i < 100; i++)
    {
      path[i].x = sin(i/100.0*2.0*PI)/10.0;
      path[i].time = i/10;
    }


	std::vector<double> group_variable_values;

	group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);


	// left_arm	
	group_variable_values[0] = 1.0;
	group_variable_values[1] = 0.0;
	group_variable_values[2] = 0.0;
	group_variable_values[3] = 0.0;
	group_variable_values[4] = 0.0;
	group_variable_values[5] = 0.0;
	group_variable_values[6] = 0.0;



	// group_variable_values[10] = group_variable_values[10] + 0.1;

	
	//group.setJointValueTarget(group_variable_values);

	geometry_msgs::Pose left_sew_start;
	left_sew_start.position.x = 0.63566;   
	left_sew_start.position.y = 0.093692;
	left_sew_start.position.z = 0.88518;
	left_sew_start.orientation.x = 0.35885;      
	left_sew_start.orientation.y = -0.60485;
	left_sew_start.orientation.z = -0.37386;
	left_sew_start.orientation.w = 0.60466;  


	group.setPoseTarget(left_sew_start, "arm_left_link_7_t");

	group.plan(my_plan);
    group.execute(my_plan);


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

