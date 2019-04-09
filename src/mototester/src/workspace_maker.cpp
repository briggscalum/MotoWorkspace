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
	moveit_msgs::CollisionObject collision_object_3;


    //#collision_object.header.frame_id = arm_right_group.getPlanningFrame();

	/* The id of the object is used to identify it. */
	collision_object.id = "box1";
	collision_object_2.id = "box2";
	collision_object_3.id = "sewing_machine";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 1.0;
	primitive.dimensions[1] = 4.0;
	primitive.dimensions[2] = 2;

	shape_msgs::SolidPrimitive primitive_2;
	primitive_2.type = primitive.BOX;
	primitive_2.dimensions.resize(3);
	primitive_2.dimensions[0] = 2.0;
	primitive_2.dimensions[1] = 1.5;
	primitive_2.dimensions[2] = 0.74;

	shape_msgs::SolidPrimitive primitive_3;
	primitive_3.type = primitive.BOX;
	primitive_3.dimensions.resize(3);
	primitive_3.dimensions[0] = 0.8;
	primitive_3.dimensions[1] = 0.5;
	primitive_3.dimensions[2] = 0.4;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  1.1;
	box_pose.position.y = -0.5;
	box_pose.position.z =  1.0;

	geometry_msgs::Pose box2_pose;
	box2_pose.orientation.w = 1.0;
	box2_pose.position.x =  0.0;
	box2_pose.position.y =  1.3;
	box2_pose.position.z =  0.35;

	geometry_msgs::Pose sewing_pose;
	sewing_pose.orientation.w = 1.0;
	sewing_pose.position.x =  0.9;
	sewing_pose.position.y =  0.0;
	sewing_pose.position.z =  0.85;

	collision_object_3.primitives.push_back(primitive_3);
	collision_object_3.primitive_poses.push_back(sewing_pose);
	collision_object_3.operation = collision_object_3.ADD;

	collision_object_2.primitives.push_back(primitive_2);
	collision_object_2.primitive_poses.push_back(box2_pose);
	collision_object_2.operation = collision_object_2.ADD;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);
	collision_objects.push_back(collision_object_2);
	collision_objects.push_back(collision_object_3);

	planning_scene_interface.applyCollisionObjects(collision_objects);

    sleep(2.0);

	ros::spin();
	return 0;
}

