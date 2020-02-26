/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#ifndef SRC_ROS_UDP_H
#define SRC_ROS_UDP_H

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

#include "arm_udp_frame.h"
#include "udp_socket.h"

class RosUdp {
public:
    RosUdp();

    ~RosUdp();

    bool udpInit(); // init UDP connection

    void rosInit(); // startup NodeHandlers, publishers and subscribers

    bool armInit(); // read initial data from Aria arm

    int readData(); // read data from hardware

    void writeData(); //send datat to hardware

    ros::NodeHandle nh, nh_private;

private:
//    UDP connection variables
    UDPSocket udp_socket;
    int port_in;
    int port_out;
    std::string remote_ip;
//    send/receive buffers
    arm_udp_frame_send_t udp_frame_send;
    arm_udp_frame_recv_t udp_frame_recv;

//    ROS publishers
    ros::Publisher arm_status_pub; // publish arm status
    ros::Publisher arm_fault_code_pub; // publish arm fault code
    ros::Publisher joint_mode_pub; //publish joint modes
    ros::Publisher joint_fault_code_pub; //publihs joint fault codes
    ros::Publisher joint_state_pub; // publish joint states(pos, vel, eff)
    ros::Publisher end_effector_pose_pub; // publish end effector pose
    ros::Publisher gripper_pos_pub; //publish gripper position

//    ROS subscribers
    ros::Subscriber arm_mode_sub; // arm mode
    ros::Subscriber joint_state_sub;  // joint states
    ros::Subscriber end_effector_pose_sub; // end effector_pose
    ros::Subscriber gripper_position_sub; //gripper position
    ros::Subscriber cartessain_pos_sub; // tmp sub

//    ROS messages
    std_msgs::UInt8 arm_status_msg; //message to publish arm status
    std_msgs::UInt16 arm_fault_code_msg; // messgae to publish arm fault code
    std_msgs::UInt8MultiArray joint_mode_msg; //message to publish joint modes
    std_msgs::UInt16MultiArray joint_fault_codes_msg; //message to publish joint fault codes
    sensor_msgs::JointState joint_states_msg; //message to publish joint states(pos, vel, eff)
    geometry_msgs::Pose end_effector_pose_msg; //message to publish end effector pose
    std_msgs::Float32 gripper_pos_msg; // message to publish gripper position

//    ROS callbacks
    void armModeCallback(const std_msgs::UInt8::ConstPtr &msg); // callback method to change arm mode

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg); // callback method to set position of each joint

    void endEffectorPosCallback(const geometry_msgs::Pose::ConstPtr &pose); // callback method to set end effector pose(position, orientation) in cartessian space

    void gripperPositionCallback(const std_msgs::Float32::ConstPtr &position); // callback method to change gripper position (open, close)
};

#endif //SRC_ROS_UDP_H
