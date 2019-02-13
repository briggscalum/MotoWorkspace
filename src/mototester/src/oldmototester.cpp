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

  arm_left_group.setPlannerId("RRTConnectkConfigDefault");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  group.setPlannerId("PRMstarkConfigDefault");

  group.setGoalJointTolerance(0.001);
  group.setEndEffectorLink("arm_left_link_7_t");


  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  geometry_msgs::Pose left_flat;
  left_flat.position.x = 0.873586;
  left_flat.position.y = 0.257035;
  left_flat.position.z = 0.76;
  left_flat.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, 0.4, -M_PI / 2);

  ROS_INFO("Quaternion: %f, %f, %f, %f,", left_flat.orientation.x, left_flat.orientation.y, left_flat.orientation.z,
           left_flat.orientation.w);


   geometry_msgs::Pose left_flat_zero;
  left_flat_zero.position.x = 0.19253;
  left_flat_zero.position.y = 1.1196;
  left_flat_zero.position.z = 1.2175;
  left_flat_zero.orientation.x = 0.70683;
  left_flat_zero.orientation.y = 0;
  left_flat_zero.orientation.z = 0;
  left_flat_zero.orientation.w = 0.70739;

  geometry_msgs::Pose right_flat_zero;
  right_flat_zero.position.x = 0.194269;
  right_flat_zero.position.y = -1.11964;
  right_flat_zero.position.z = 1.2175;
  right_flat_zero.orientation.x = -0.00056286;
  right_flat_zero.orientation.y = 0.70682;
  right_flat_zero.orientation.z = 0.70739;
  right_flat_zero.orientation.w = -0.00056331;

  geometry_msgs::Pose left_front_home;
  left_flat_zero.position.x = 0.56862;
  left_flat_zero.position.y = 0.14813;
  left_flat_zero.position.z = 0.71106;
  left_flat_zero.orientation.x = 0.84705;
  left_flat_zero.orientation.y = 0.15545;
  left_flat_zero.orientation.z = -0.49672;
  left_flat_zero.orientation.w = -0.10774;

  geometry_msgs::Pose right_front_home;
  right_flat_zero.position.x = 0.5641;
  right_flat_zero.position.y = -0.13935;
  right_flat_zero.position.z = 0.69895;
  right_flat_zero.orientation.x = 0.18639;
  right_flat_zero.orientation.y = 0.84103;
  right_flat_zero.orientation.z = -0.11821;
  right_flat_zero.orientation.w = -0.49391;

  if (test_num == 1)
  {

  	
    //Add collision Object test
    moveit_msgs::CollisionObject back_wall;
    back_wall.header.frame_id = group.getPlanningFrame();

    back_wall.id = "back_wall";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.25;
    primitive.dimensions[1] = 5;
    primitive.dimensions[2] = 5;

    geometry_msgs::Pose back_wall_pose;
    back_wall_pose.orientation.w = 1.0;
    back_wall_pose.position.x = -1.125;
    back_wall_pose.position.y = 0;
    back_wall_pose.position.z = 2.5;

    back_wall.primitives.push_back(primitive);
    back_wall.primitive_poses.push_back(back_wall_pose);
    back_wall.operation = back_wall.ADD;

    //====================================================

    moveit_msgs::CollisionObject floor;
    floor.header.frame_id = group.getPlanningFrame();

    floor.id = "floor";

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 5;
    primitive.dimensions[1] = 5;
    primitive.dimensions[2] = 0.25;

    geometry_msgs::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.x = 1.25;
    floor_pose.position.y = 0;
    floor_pose.position.z = -0.125;

    floor.primitives.push_back(primitive);
    floor.primitive_poses.push_back(floor_pose);
    floor.operation = floor.ADD;

    //======================================================================================

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(back_wall);
    collision_objects.push_back(floor);

    ROS_INFO("Added collision surfaces to world");
    planning_scene_interface.applyCollisionObjects(collision_objects);

    sleep(2.0);

    test_num = 2;

  }
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //

  ROS_INFO("Orientation Tolerance: %f", group.getGoalOrientationTolerance());

  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End-effector: %s", group.getEndEffectorLink().c_str());

  if (test_num == 2)
  {
    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.

    group.setPoseTarget(right_flat_zero, "arm_right_link_7_t");
    group.setPoseTarget(left_flat_zero, "arm_left_link_7_t");

    // Now, we call the planner to compute the plan
    // and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(10.0);

    group.move();
  }

  sleep(5.0);

  ros::shutdown();
  return 0;
}

