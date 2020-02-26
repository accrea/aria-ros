/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#ifndef ARIA_V2_ROS_UDP_ROS_CONTROL_UDP_H
#define ARIA_V2_ROS_UDP_ROS_CONTROL_UDP_H

#include "ros/ros.h"
#include "udp_socket.h"
#include "arm_udp_frame.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

class RosControlUdp : public hardware_interface::RobotHW {
public:
    RosControlUdp();

    ~RosControlUdp();

    int read(); // read data from hardware

    void write(); // send data to hardware

    ros::NodeHandle nh; // class NodeHandler

    ros::Subscriber arm_mode_sub; // ROS subscriber to change arm mode
    ros::Subscriber cartessian_pose_sub; // ROS callback, allow to control arm in cartessian mode
private:
//      ROS hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;

    int num_joints, frame_count;
    std::vector<std::string> joint_names; // joint names vector
    std::vector<double> joint_position, joint_velocity, joint_effort, joint_position_command;
    geometry_msgs::Pose gripper_pose;
    bool arm_init;

//    UDP connection variables
    UDPSocket udpSocket;
    int port_in, port_out;
    std::string remote_ip;
    arm_udp_frame_send_t udp_frame_send;
    arm_udp_frame_recv_t udp_frame_recv;

//      ROS callbacks
    void armModeCallback(const std_msgs::UInt8::ConstPtr &msg);
    void cartessianPoseCallback(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr &msg);
};

#endif //ARIA_V2_ROS_UDP_ROS_CONTROL_UDP_H
