/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#ifndef ARIA_DRIVER_ROS_UDP_H
#define ARIA_DRIVER_ROS_UDP_H

#include "ros/ros.h"
//ROS messages
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"

#include <string>

//UDP Aria Client

#include "AriaClient.h"

class RosUDP {
public:
    RosUDP();

    ~RosUDP();

    void rosInit();

    void connectionInit();

    int getAndPublishData();

    ros::NodeHandle nh_;

private:

//     ROS publishers
    ros::Publisher arm_status_pub_; // publish arm status
    ros::Publisher arm_fault_code_pub_; // publish arm fault code
    ros::Publisher joint_mode_pub_; //publish joint modes
    ros::Publisher joint_fault_code_pub_; //publihs joint fault codes
    ros::Publisher joint_state_pub_; // publish joint states(pos, vel, eff)
    ros::Publisher end_effector_pose_pub_; // publish end effector pose
    ros::Publisher gripper_pos_pub_; //publish gripper position

//    ROS subscribers
    ros::Subscriber arm_mode_sub_; // arm mode
    ros::Subscriber joint_state_sub_;  // joint states
    ros::Subscriber end_effector_pose_sub_; // end effector_pose
    ros::Subscriber gripper_position_sub_; //gripper position

//    ROS messages
    std_msgs::UInt8 arm_status_msg_; // message to publish arm status
    std_msgs::UInt16 arm_fault_code_msg_; // messgae to publish arm fault code
    std_msgs::UInt8MultiArray joint_mode_msg_; //message to publish joint modes
    std_msgs::UInt16MultiArray joint_fault_codes_msg_; //message to publish joint fault codes
    sensor_msgs::JointState joint_states_msg_; //message to publish joint states(pos, vel, eff)
    geometry_msgs::Pose end_effector_pose_msg_; //message to publish end effector pose
    std_msgs::Float32 gripper_pos_msg_; // message to publish gripper position

//    ROS callbacks
    void armModeCallback(const std_msgs::UInt8::ConstPtr &msg); // callback method to change arm mode

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg); // callback method to set position, velocity and effort of each joint

    void endEffectorPosCallback(const geometry_msgs::Pose::ConstPtr &pose); // callback method to set end effector pose(position, orientation) in cartessian space

    void gripperPositionCallback(const std_msgs::Float32::ConstPtr &position); // callback method to change gripper position (open, close)

//    actual end effecor pose
    float EEPosAct_[3], EEQuatAct_[4];
//    demand end effector pose
    float EEPosDem_[3], EEQuatDem_[4];
    //    connection status
    int connection_status;
};

#endif //ARIA_DRIVER_ROS_UDP_H
