#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#define PI 3.14159265


int test_num = 1;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("sda10f");
	moveit::planning_interface::MoveGroupInterface arm_left_group("arm_left");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	arm_left_group.setPlannerId("RRTConnectkConfigDefault");

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	group.setPlannerId("PRMstarkConfigDefault");

	group.setGoalJointTolerance(0.001);
	group.setEndEffectorLink("arm_left_link_7_t");

	geometry_msgs::Pose left_flat;

	ROS_INFO("Bar");


	std::vector<double> group_variable_values;

	group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

	//left_arm
	group_variable_values[1] = 1.5707;
	group_variable_values[2] = 1.5707;
	group_variable_values[3] = 1.5707;
	group_variable_values[4] = 1.5707;
	group_variable_values[5] = 1.5707;
	group_variable_values[6] = 0.0;
	group_variable_values[7] = 0.0;

	//right_arm
	group_variable_values[8] = 1.5707;
	group_variable_values[9] = 1.5707;
	group_variable_values[10] = 0;
	group_variable_values[11] = 1.5707;
	group_variable_values[12] = 1.5707;
	group_variable_values[13] = 0.0;
	group_variable_values[14] = 0.0;

	ROS_INFO("Foo");
	group.setJointValueTarget(group_variable_values);
	group.plan(my_plan);
    group.execute(my_plan);


	sleep(5.0);

	ros::shutdown();
	return 0;
}

