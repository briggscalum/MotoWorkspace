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
#include <motoman_msgs/DynamicJointTrajectory.h>

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
	arm_right_group.setEndEffectorLink("arm_right_link_7_t");

	geometry_msgs::Pose left_flat;


	geometry_msgs::Pose right_home;

	right_home.position.x = -0.4;   
	right_home.position.y = -0.25;
	right_home.position.z = 1.4;
	right_home.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,M_PI/4,0);  

	// ADD COLLISION

	// moveit_msgs::CollisionObject collision_object;
	// moveit_msgs::CollisionObject collision_object_2;
	// collision_object.header.frame_id = arm_right_group.getPlanningFrame();

	// /* The id of the object is used to identify it. */
	// collision_object.id = "box1";
	// collision_object_2.id = "box2";

	// /* Define a box to add to the world. */
	// shape_msgs::SolidPrimitive primitive;
	// primitive.type = primitive.BOX;
	// primitive.dimensions.resize(3);
	// primitive.dimensions[0] = 1.0;
	// primitive.dimensions[1] = 4.0;
	// primitive.dimensions[2] = 4.0;

	// shape_msgs::SolidPrimitive primitive_2;
	// primitive_2.type = primitive.BOX;
	// primitive_2.dimensions.resize(3);
	// primitive_2.dimensions[0] = 1.0;
	// primitive_2.dimensions[1] = 2.0;
	// primitive_2.dimensions[2] = 1.2;

	// /* A pose for the box (specified relative to frame_id) */
	// geometry_msgs::Pose box_pose;
	// box_pose.orientation.w = 1.0;
	// box_pose.position.x =  1;
	// box_pose.position.y = -0.5;
	// box_pose.position.z =  1.0;

	// geometry_msgs::Pose box2_pose;
	// box2_pose.orientation.w = 1.0;
	// box2_pose.position.x =  -0.9;
	// box2_pose.position.y =  0;
	// box2_pose.position.z =  0.60;

	// collision_object_2.primitives.push_back(primitive_2);
	// collision_object_2.primitive_poses.push_back(box2_pose);
	// collision_object_2.operation = collision_object_2.ADD;


	// collision_object.primitives.push_back(primitive);
	// collision_object.primitive_poses.push_back(box_pose);
	// collision_object.operation = collision_object.ADD;

	// std::vector<moveit_msgs::CollisionObject> collision_objects;
	// collision_objects.push_back(collision_object);
	// collision_objects.push_back(collision_object_2);

	// planning_scene_interface.applyCollisionObjects(collision_objects);

 //    sleep(2.0);

	// END COLLISION ADDER



	// std::vector<PointGoal> path(100);

	// //Generate a Dummy Path
 //    for (int i = 0; i < 100; i++)
 //    {
 //      path[i].x = sin(i/100.0*2.0*PI)/10.0;
 //      path[i].time = i/10;
 //    }


	std::vector<double> group_variable_values;

	group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);


	// left_arm	
	group_variable_values[0] = 0.0;
	group_variable_values[1] = 0.0;
	group_variable_values[2] = 0.0;
	group_variable_values[3] = 0.0;
	group_variable_values[4] = 0.0;
	group_variable_values[5] = 0.0;
	group_variable_values[6] = M_PI/2;



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

	int grabcounter = 1;

	while(1) {

		grabcounter = grabcounter + 1;
	    sleep(2.0);

		right_home.position.y = -0.25 - (fabric_x-360)*0.001;
		right_home.position.x = -0.4 - (fabric_y-240)*0.00125;
		//group.setPoseTarget(left_sew_start, "arm_left_link_7_t");
		arm_right_group.setPoseTarget(right_home, "arm_right_link_7_t");

		arm_right_group.plan(my_plan);
	    arm_right_group.execute(my_plan);

   	    sleep(2.0);

   	 //    right_home.position.x = right_home.position.x - 0.01;
   	 //    right_home.position.x = right_home.position.z - 0.01;

   	 //    arm_right_group.setPoseTarget(right_home, "arm_right_link_7_t");
   	 //    arm_right_group.plan(my_plan);
	    // arm_right_group.execute(my_plan);

	    sleep(1.0);


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
	}


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

