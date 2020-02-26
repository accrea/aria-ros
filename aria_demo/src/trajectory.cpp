/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Author: Piotr Jagiełło
 * email: p.jagiello@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[]) {
//    ROS init
    ros::init(argc, argv, "demo_trajectory");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

//    creating move group
    moveit::planning_interface::MoveGroupInterface move_group_arm("arm");

//    std::vector<double> joint_pose_init, joint_pose_end;
//    joint_pose_init.resize(6);
//    joint_pose_end.resize(6);
//
//    joint_pose_init = move_group_arm.getCurrentJointValues();

    geometry_msgs::Pose pose;
    geometry_msgs::PoseStamped pose_stamped;
    std::vector<geometry_msgs::Pose> waypoints; // vector of poses
    std::vector<double> joint_pos; // vector of joint position values

//    move to initial position
    move_group_arm.setNamedTarget("Init Position");
    move_group_arm.move();

//    get current end effector pose
    pose_stamped = move_group_arm.getCurrentPose("eef_link");
    pose = pose_stamped.pose;

//    pose.position.x = 0;
//    pose.position.y = 0;
//    pose.position.z = 0.9;
//    pose.orientation.x = 0;
//    pose.orientation.y = 0;
//    pose.orientation.z = 0;
//    pose.orientation.w = 1;
//    waypoints.push_back(pose);

//    pose.position.z += 0.3;

//      move through x axiz forward
    pose.position.x += 0.1;
    waypoints.push_back(pose);

//    set helix center
    double x_center = pose.position.x;
    double y_center = pose.position.y;

//    variables for calculate helix path
    double radius = 0.01, resolution = 1, angle = 0, d_angle = 0;
    const int rev_count = 7;
    d_angle = resolution * 3.14 / 180;

    for (int j = 0; j < rev_count * 360; ++j) {
        angle += d_angle;
        pose.position.x = x_center + radius * cos(angle);
        pose.position.y = y_center + radius * sin(angle);
        radius += 0.00005;
        pose.position.z += 0.00005;
//        push back computed pose to vector of poses
        waypoints.push_back(pose);
    }

    double eef_resolution = std::min(0.01, 0.02);
    double jump_threshold = 0.0;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_arm.computeCartesianPath(waypoints, eef_resolution, jump_threshold, trajectory);

//    execute plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_arm.execute(plan);

    return 0;
}
