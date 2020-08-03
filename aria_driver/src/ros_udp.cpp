/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/
#include <aria_driver/ros_udp.h>

RosUDP::RosUDP() {
    connectionInit();
    rosInit();
}

RosUDP::~RosUDP() = default;

void RosUDP::rosInit() {
//    Init ROS publishers,
    arm_status_pub_ = nh_.advertise<std_msgs::UInt8>("out/arm_status", 10);
    arm_fault_code_pub_ = nh_.advertise<std_msgs::UInt16>("out/arm_fault_code", 10);
    joint_mode_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("out/joint_mode", 10);
    joint_fault_code_pub_ = nh_.advertise<std_msgs::UInt16MultiArray>("out/joint_fault_code", 10);
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("out/joint_states", 10);
    end_effector_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("out/end_effector_pose", 10);
    gripper_pos_pub_ = nh_.advertise<std_msgs::Float32>("out/gripper_position", 10);

//    Init ROS subscribers,
    arm_mode_sub_ = nh_.subscribe("in/arm_mode", 10, &RosUDP::armModeCallback, this);
    joint_state_sub_ = nh_.subscribe("in/joint_states", 10, &RosUDP::jointStatesCallback, this);
    end_effector_pose_sub_ = nh_.subscribe("/in/end_effector_pose", 10, &RosUDP::endEffectorPosCallback, this);
    gripper_position_sub_ = nh_.subscribe("in/gripper_position", 10, &RosUDP::gripperPositionCallback, this);

//      resize vectors
    joint_states_msg_.name.resize(7);
    joint_states_msg_.position.resize(7);
    joint_states_msg_.velocity.resize(7);
    joint_states_msg_.effort.resize(7);
    joint_states_msg_.name[0] = "arm_joint_1";
    joint_states_msg_.name[1] = "arm_joint_2";
    joint_states_msg_.name[2] = "arm_joint_3";
    joint_states_msg_.name[3] = "arm_joint_4";
    joint_states_msg_.name[4] = "arm_joint_5";
    joint_states_msg_.name[5] = "arm_joint_6";
    joint_states_msg_.name[6] = "gripper";

    joint_fault_codes_msg_.data.resize(7);
    joint_mode_msg_.data.resize(7);
}

void RosUDP::connectionInit() {
    AriaClient_SetAddress("192.168.9.9", 7777);
    AriaClient_Init();
    do {
        connection_status = AriaClient_Connect();
    } while (connection_status != 0);

    AriaClient_StartCommunication();
}

int RosUDP::getAndPublishData() {
//    get current time
    joint_states_msg_.header.stamp = ros::Time::now();
    for (int j = 0; j < 7; ++j) {
//        get actual joint pos, vel, anf eff
        joint_states_msg_.position[j] = AriaClient_GetJointPos(j);
        joint_states_msg_.velocity[j] = AriaClient_GetJointVel(j);
        joint_states_msg_.effort[j] = AriaClient_GetJointTrq(j);
//        get joint modes
        joint_mode_msg_.data[j] = AriaClient_GetJointMode(j);
//        get fault codes
        joint_fault_codes_msg_.data[j] = AriaClient_GetJointFaultCode(j);
    }
//    get arm status
    arm_status_msg_.data = AriaClient_GetArmStatus();
//    pobierz kod błędu ramienia
    arm_fault_code_msg_.data = AriaClient_GetArmFaultCode();
//    end effector position
    AriaClient_GetArmEEPosition(&EEPosAct_[0]);
    end_effector_pose_msg_.position.x = EEPosAct_[0];
    end_effector_pose_msg_.position.y = EEPosAct_[1];
    end_effector_pose_msg_.position.z = EEPosAct_[2];
//    end effector orientation
    AriaClient_GetArmEEQuaternion(&EEQuatAct_[0]);
    end_effector_pose_msg_.orientation.x = EEQuatAct_[0];
    end_effector_pose_msg_.orientation.y = EEQuatAct_[1];
    end_effector_pose_msg_.orientation.z = EEQuatAct_[2];
    end_effector_pose_msg_.orientation.w = EEQuatAct_[3];
//    end effector position
    gripper_pos_msg_.data = AriaClient_GetArmGripperPos();

//    publish messages
    arm_status_pub_.publish(arm_status_msg_);
    arm_fault_code_pub_.publish(arm_fault_code_msg_);
    joint_mode_pub_.publish(joint_mode_msg_);
    joint_fault_code_pub_.publish(joint_fault_codes_msg_);
    joint_state_pub_.publish(joint_states_msg_);
    end_effector_pose_pub_.publish(end_effector_pose_msg_);
    gripper_pos_pub_.publish(gripper_pos_msg_);
}


void RosUDP::armModeCallback(const std_msgs::UInt8::ConstPtr &msg) {
    do {
        AriaClient_SetArmControlMode(msg->data);
        AriaClient_WaitForMode(msg->data);
    } while (msg->data != AriaClient_GetArmStatus());
}

void RosUDP::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
//    change demand joint positions
    if (msg->position.size() != 7)
        ROS_ERROR("Invalid number of position values in incoming ROS Message");
    else {
        for (int j = 0; j < 7; ++j) {
            AriaClient_SetJointPos(j, msg->position[j]);
        }
    }

//    change demand joint velocities
    if (msg->velocity.size() != 7)
        ROS_ERROR("Invalid number of velocity values in incoming ROS Message");
    else {
        for (int j = 0; j < 7; ++j) {
            AriaClient_SetJointVel(j, msg->velocity[j]);
        }
    }

//   change demand joint torques
    if (msg->effort.size() != 7)
        ROS_ERROR("Invalid number of effort values in incoming ROS Message");
    else {
        for (int j = 0; j < 7; ++j) {
            AriaClient_SetJointTrq(j, msg->effort[j]);
        }
    }
}

void RosUDP::endEffectorPosCallback(const geometry_msgs::Pose::ConstPtr &pose) {
//    write values from message to tables
    EEPosDem_[0] = pose->position.x;
    EEPosDem_[1] = pose->position.y;
    EEPosDem_[2] = pose->position.z;
    EEQuatDem_[0] = pose->orientation.x;
    EEQuatDem_[1] = pose->orientation.y;
    EEQuatDem_[2] = pose->orientation.z;
    EEQuatDem_[3] = pose->orientation.w;

    AriaClient_SetArmEEPosition(&EEPosDem_[0]);
    AriaClient_SetArmEEQuaternion(&EEQuatDem_[0]);
}

void RosUDP::gripperPositionCallback(const std_msgs::Float32::ConstPtr &position) {
    AriaClient_SetArmGripperPos(position->data);
}