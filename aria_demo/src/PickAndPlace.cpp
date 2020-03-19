/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Author: Piotr Jagiełło
 * email: p.jagiello@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include "aria_demo/PickAndPlace.h"

int main(int argc, char *argv[]) {
//    ROS init
    ros::init(argc, argv, "demo_pick_and_place");
    ros::AsyncSpinner spinner(4);
    spinner.start();

//    create instance fo class
    PickAndPlace andPlace;

//    create group and initial configuration
    moveit::planning_interface::MoveGroupInterface group_arm("arm");
    group_arm.setPlannerId("RRTConnect");
    group_arm.setPlanningTime(10);

    andPlace.pickFromTable1(group_arm); // pick object
    ros::Duration(1.0).sleep();
    andPlace.placeOnTable2(group_arm); //place object
    ros::Duration(1.0).sleep();
//    andPlace.pickFromTable2(group_arm);
    return 0;
}

PickAndPlace::PickAndPlace() {
    collision_objects.resize(3); // 4 objects (2 tables, 1 object to replace and floor)
//    planning_scene_interface = new (moveit::planning_interface::PlanningSceneInterface);
    addCollisionObjects(planning_scene_interface); // add objects in arm workspace
    nh.param<std::string>("/arm_type", arm_type, "aria_v2");
    std::cout << "object spawned";
}

void PickAndPlace::addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
//    definition of 1st table
    collision_objects[0].id = "table_1";
    collision_objects[0].header.frame_id = "base_link";
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.4;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.2;
//    definition of 1st table position
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0.5;
    collision_objects[0].primitive_poses[0].position.z = (collision_objects[0].primitives[0].dimensions[2]) / 2;
//    add object
    collision_objects[0].operation = collision_objects[0].ADD;

//    definition of 2nd table
    collision_objects[1].id = "table_2";
    collision_objects[1].header.frame_id = "base_link";
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.4;
    collision_objects[1].primitives[0].dimensions[2] = 0.1;
    //    definition of 2nd table position
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = -0.5;
    collision_objects[1].primitive_poses[0].position.z = (collision_objects[1].primitives[0].dimensions[2]) / 2;
//    add object
    collision_objects[1].operation = collision_objects[1].ADD;

//    definition of moveable object
    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.035;
    collision_objects[2].primitives[0].dimensions[1] = 0.035;
    collision_objects[2].primitives[0].dimensions[2] = 0.15;
//    definition of moveable object position
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0;
    collision_objects[2].primitive_poses[0].position.y = -0.5;
    collision_objects[2].primitive_poses[0].position.z =
            (collision_objects[1].primitives[0].dimensions[2] + collision_objects[2].primitives[0].dimensions[2] / 2) +
            0.001;
    collision_objects[2].operation = collision_objects[2].ADD;

////    definition of floor
//    collision_objects[3].id = "floor";
//    collision_objects[3].header.frame_id = "base_link";
//    collision_objects[3].primitives.resize(1);
//    collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].CYLINDER;
//    collision_objects[3].primitives[0].dimensions.resize(2);
//    collision_objects[3].primitives[0].dimensions[0] = 0.001;
//    collision_objects[3].primitives[0].dimensions[1] = 1;
//
//    //    definition of floor position
//    collision_objects[3].primitive_poses.resize(1);
//    collision_objects[3].primitive_poses[0].position.x = 0;
//    collision_objects[3].primitive_poses[0].position.y = 0;
//    collision_objects[3].primitive_poses[0].position.z = (collision_objects[3].primitives[0].dimensions[2]) / 2;
//    collision_objects[2].operation = collision_objects[2].ADD;


    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void PickAndPlace::openGripper(trajectory_msgs::JointTrajectory &gripper_joint_trajectory) {
//    resize joint names vector and set name
    gripper_joint_trajectory.joint_names.resize(1);
    if (arm_type == "aria_v2")
        gripper_joint_trajectory.joint_names[0] = "finger_joint_1";
    else if (arm_type == "aria_v1")
        gripper_joint_trajectory.joint_names[0] = "FJoint1";

//    resize points and position vectors, set values
    gripper_joint_trajectory.points.resize(1);
    gripper_joint_trajectory.points[0].positions.resize(1);
    gripper_joint_trajectory.points[0].positions[0] = 1;

//    set time duration for openning gripper
    gripper_joint_trajectory.points[0].time_from_start = ros::Duration(1);
}

void PickAndPlace::closeGripper(trajectory_msgs::JointTrajectory &gripper_joint_trajectory) {
//    resize joint names vector and set name
    gripper_joint_trajectory.joint_names.resize(1);
    if (arm_type == "aria_v2")
        gripper_joint_trajectory.joint_names[0] = "finger_joint_1";
    else if (arm_type == "aria_v1")
        gripper_joint_trajectory.joint_names[0] = "FJoint1";

//    resize points and position vectors, set values
    gripper_joint_trajectory.points.resize(1);
    gripper_joint_trajectory.points[0].positions.resize(1);
    gripper_joint_trajectory.points[0].positions[0] = 0.25;

//    set duration time for closing gripper
    gripper_joint_trajectory.points[0].time_from_start = ros::Duration(1);
}

void PickAndPlace::pickFromTable1(moveit::planning_interface::MoveGroupInterface &group) {
    grasps_table_1.resize(1);
//    grasps pose
    grasps_table_1[0].grasp_pose.header.frame_id = "base_link";
    orientation_pick_table_1.setRPY(M_PI / 2, -M_PI, 0);
    grasps_table_1[0].grasp_pose.pose.orientation = tf2::toMsg(orientation_pick_table_1);
    grasps_table_1[0].grasp_pose.pose.position.x = 0;
    grasps_table_1[0].grasp_pose.pose.position.y = -0.47;
    grasps_table_1[0].grasp_pose.pose.position.z = 0.187;
//    pre grasps approach
    grasps_table_1[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps_table_1[0].pre_grasp_approach.direction.vector.y = -1;
    grasps_table_1[0].pre_grasp_approach.min_distance = 0.095;
    grasps_table_1[0].pre_grasp_approach.desired_distance = 0.115;
//    post grasps pose
    grasps_table_1[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps_table_1[0].post_grasp_retreat.direction.vector.z = 1;
    grasps_table_1[0].post_grasp_retreat.min_distance = 0.1;
    grasps_table_1[0].post_grasp_retreat.desired_distance = 0.15;

    openGripper(grasps_table_1[0].pre_grasp_posture);
    closeGripper(grasps_table_1[0].grasp_posture);

    group.setSupportSurfaceName("table_1");
    group.setNumPlanningAttempts(2);
    group.pick("object", grasps_table_1[0]);
}

void PickAndPlace::pickFromTable2(moveit::planning_interface::MoveGroupInterface &group) {
    grasps_table_2.resize(1);
//    grasps pose
    grasps_table_2[0].grasp_pose.header.frame_id = "base_link";
    orientation_pick_table_2 = orientation_place_table_2;
    grasps_table_2[0].grasp_pose.pose.orientation = tf2::toMsg(orientation_pick_table_2);
    grasps_table_2[0].grasp_pose.pose.position.x = 0;
    grasps_table_2[0].grasp_pose.pose.position.y = 0.47;
    grasps_table_2[0].grasp_pose.pose.position.z = (collision_objects[0].primitives[0].dimensions[2] +
                                                    (collision_objects[2].primitives[0].dimensions[2]) / 2) + 0.1;
//    pre grasps approach
    grasps_table_2[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    grasps_table_2[0].pre_grasp_approach.direction.vector.y = 1.0;
    grasps_table_2[0].pre_grasp_approach.min_distance = 0.07;
    grasps_table_2[0].pre_grasp_approach.desired_distance = 0.115;

//    post grasps pose
    grasps_table_2[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    grasps_table_2[0].post_grasp_retreat.direction.vector.z = 1;
    grasps_table_2[0].post_grasp_retreat.min_distance = 0.1;
    grasps_table_2[0].post_grasp_retreat.desired_distance = 0.15;

//    openGripper(grasps_table_2[0].pre_grasp_posture);
    closeGripper(grasps_table_2[0].grasp_posture);

    group.setSupportSurfaceName("table_2");
    group.setNumPlanningAttempts(2000);
    group.pick("object", grasps_table_2[0]);
}

void PickAndPlace::placeOnTable2(moveit::planning_interface::MoveGroupInterface &group) {
    place_location_table_2.resize(1);
//    location place
    place_location_table_2[0].place_pose.header.frame_id = "base_link";
    orientation_place_table_2.setRPY(0, 0, -M_PI);
    place_location_table_2[0].place_pose.pose.orientation = tf2::toMsg(orientation_place_table_2);
    place_location_table_2[0].place_pose.pose.position.x = 0;
    place_location_table_2[0].place_pose.pose.position.y = 0.47;
    place_location_table_2[0].place_pose.pose.position.z = collision_objects[0].primitives[0].dimensions[2] +
                                                           (collision_objects[2].primitives[0].dimensions[2]) / 2;
//    pre place approach
    place_location_table_2[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location_table_2[0].pre_place_approach.direction.vector.z = -1.0;
    place_location_table_2[0].pre_place_approach.min_distance = 0.0857;
    place_location_table_2[0].pre_place_approach.desired_distance = 0.121;
//    post place retreat
    place_location_table_2[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location_table_2[0].post_place_retreat.direction.vector.y = -1.0;
    place_location_table_2[0].post_place_retreat.min_distance = 0.08;
    place_location_table_2[0].post_place_retreat.desired_distance = 0.1;

    openGripper(place_location_table_2[0].post_place_posture);
    group.setSupportSurfaceName("table_2");
    group.setNumPlanningAttempts(2);
    group.place("object", place_location_table_2);
}
