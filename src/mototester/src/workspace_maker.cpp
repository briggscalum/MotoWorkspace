#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "workspace");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//moveit::planning_interface::MoveGroupInterface group("arm_left");
	//moveit::planning_interface::MoveGroupInterface arm_left_group("arm_left");
	//moveit::planning_interface::MoveGroupInterface arm_right_group("arm_right");

	// ADD COLLISION

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::CollisionObject collision_object;
	moveit_msgs::CollisionObject collision_object_2;
    //#collision_object.header.frame_id = arm_right_group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = "box1";
	collision_object_2.id = "box2";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 1.0;
	primitive.dimensions[1] = 4.0;
	primitive.dimensions[2] = 4.0;

	shape_msgs::SolidPrimitive primitive_2;
	primitive_2.type = primitive.BOX;
	primitive_2.dimensions.resize(3);
	primitive_2.dimensions[0] = 1.0;
	primitive_2.dimensions[1] = 2.0;
	primitive_2.dimensions[2] = 1.2;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  1;
	box_pose.position.y = -0.5;
	box_pose.position.z =  1.0;

	geometry_msgs::Pose box2_pose;
	box2_pose.orientation.w = 1.0;
	box2_pose.position.x =  -0.9;
	box2_pose.position.y =  0;
	box2_pose.position.z =  0.60;

	collision_object_2.primitives.push_back(primitive_2);
	collision_object_2.primitive_poses.push_back(box2_pose);
	collision_object_2.operation = collision_object_2.ADD;


	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	collision_objects.push_back(collision_object_2);

	planning_scene_interface.applyCollisionObjects(collision_objects);

    sleep(2.0);

	ros::spin();
	return 0;
}

