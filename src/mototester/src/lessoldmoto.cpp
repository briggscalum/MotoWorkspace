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

class PointGoal {
  
public:

double x;
double y;
double time;

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "test");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface group("sda10f");
	moveit::planning_interface::MoveGroupInterface arm_left_group("arm_left");
	moveit::planning_interface::MoveGroupInterface arm_right_group("arm_right");


	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	arm_left_group.setPlannerId("RRTConnectkConfigDefault");
	arm_right_group.setPlannerId("RRTConnectkConfigDefault");


	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	group.setPlannerId("PRMstarkConfigDefault");

	group.setGoalJointTolerance(0.001);
	group.setEndEffectorLink("arm_left_link_7_t");
	group.setEndEffectorLink("arm_right_link_7_t");

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

	// //left_arm
	// group_variable_values[1] = 1.5707;
	// group_variable_values[2] = 1.5707;
	// group_variable_values[3] = 1.5707;
	// group_variable_values[4] = 1.5707;
	// group_variable_values[5] = 1.5707;
	// group_variable_values[6] = 0.0;
	// group_variable_values[7] = 0.0;

	//right_arm
	group_variable_values[8] = 1.5707;
	group_variable_values[9] = 1.5707;
	group_variable_values[10] = 0;
	group_variable_values[11] = 1.5707;
	group_variable_values[12] = 1.5707;
	group_variable_values[13] = 0.0;
	group_variable_values[14] = 0.0;

	

	sleep(2.0);

    ROS_INFO("Get Ready to Rumble");

    sleep(2.0);

    int starttime = ros::Time::now().toSec();
    int currenttime = starttime;
    int i = 0;



    while(i < 100)
    {
      //group_variable_values[1] = 1.57 + path[i].x;
      group_variable_values[8] = 1.57 + path[i].x;
      group.setJointValueTarget(group_variable_values);
      
      group.asyncExecute(my_plan);

      if(currenttime > path[i].time){
        i++;
        ROS_INFO("%i: moving to %f", i, (path[i].x + 1.57));
        group.plan(my_plan);
      }
      
      currenttime = ros::Time::now().toSec() - starttime;
      
      //moveit::planning_interface::MoveGroup::Plan my_plan;
      //group.plan(my_plan);
    }


		

	sleep(5.0);

	ros::shutdown();
	return 0;
}

	