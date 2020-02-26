/**************************************************)
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Author: Piotr Jagiełło
 * email: p.jagiello@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/ 

#ifndef ARIA_DEMO_PICKANDPLACE_H
#define ARIA_DEMO_PICKANDPLACE_H
// ROS
#include <ros/ros.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//  class definition
class PickAndPlace {
public:
    PickAndPlace();

    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface); // spawning object in workspace

    void openGripper(trajectory_msgs::JointTrajectory &gripper_joint_trajectory); // set gripper in open position

    void closeGripper(trajectory_msgs::JointTrajectory &gripper_joint_trajectory); // set gripper in close position

//    pick object from table
    void pickFromTable1(moveit::planning_interface::MoveGroupInterface &group);

    void pickFromTable2(moveit::planning_interface::MoveGroupInterface &group);

//    place object on table
    void placeOnTable2(moveit::planning_interface::MoveGroupInterface &group);

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; // moveit planning interface
    std::vector<moveit_msgs::CollisionObject> collision_objects; // objects
    std::vector<moveit_msgs::Grasp> grasps_table_1, grasps_table_2; //grasps vectors
    std::vector<moveit_msgs::PlaceLocation> place_location_table_1, place_location_table_2; // place locations
    tf2::Quaternion orientation_pick_table_1, orientation_pick_table_2, orientation_place_table_1, orientation_place_table_2; // orientation used in pick and place methods
};


#endif //ARIA_DEMO_PICKANDPLACE_H
