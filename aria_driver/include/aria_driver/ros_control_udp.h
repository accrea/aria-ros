//
// Created by accrea on 4/28/20.
//

#ifndef ARIA_DRIVER_ROS_CONTROL_UDP_H
#define ARIA_DRIVER_ROS_CONTROL_UDP_H

#include "ros/ros.h"
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt8.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

#include "AriaClient.h"

class RosControlUDP : public hardware_interface::RobotHW {
public:
    RosControlUDP();

    ~RosControlUDP();

    void read();

    void write();

    void connectionInit();

    ros::NodeHandle nh_; // class NodeHandler

    ros::Subscriber arm_mode_sub_; // ROS subscriber to change arm mode
    ros::Subscriber cartessian_pose_sub_; // ROS callback, allow to control arm in cartessian mode

    int connection_status_;
private:
//    ROS hardware interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

//    ROS callbacks
    void armModeCallback(const std_msgs::UInt8::ConstPtr &msg);

    void cartessianPoseCallback(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr &msg);

    int num_joints_, frame_count_;
    float EEPosAct_[3], EEQuatAct_[4];
    float EEPosDem_[3], EEQuatDem_[4];
    std::vector<std::string> joint_names_; // joint names vector
    std::vector<double> joint_position_, joint_velocity_, joint_effort_, joint_position_command_;
    bool arm_init_;
};

#endif //ARIA_DRIVER_ROS_CONTROL_UDP_H
