/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Author: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include "aria_driver/ros_udp.h"

RosUdp::RosUdp() : nh_private("~") {}

RosUdp::~RosUdp() {
}

void RosUdp::rosInit() {
//    Loading ROS parameters
    nh_private.param("/port_in", port_in, 7777);
    nh_private.param("port_out", port_out, 7777);
    nh_private.param<std::string>("/remote_ip", remote_ip, "192.168.9.9");

//    Init ROS publishers,
    arm_status_pub = nh.advertise<std_msgs::UInt8>("out/arm_status", 10);
    arm_fault_code_pub = nh.advertise<std_msgs::UInt8>("out/arm_fault_code", 10);
    joint_mode_pub = nh.advertise<std_msgs::UInt8MultiArray>("out/joint_mode", 10);
    joint_fault_code_pub = nh.advertise<std_msgs::UInt16MultiArray>("out/joint_fault_code", 10);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("out/joint_states", 10);
    end_effector_pose_pub = nh.advertise<geometry_msgs::Pose>("out/end_effector_pose", 10);
    gripper_pos_pub = nh.advertise<std_msgs::Float32>("out/gripper_position", 10);

//    Init ROS subscribers,
    arm_mode_sub = nh.subscribe("in/arm_mode", 10, &RosUdp::armModeCallback, this);
    joint_state_sub = nh.subscribe("in/joint_states", 10, &RosUdp::jointStatesCallback, this);
    end_effector_pose_sub = nh.subscribe("/in/end_effector_pose", 10, &RosUdp::endEffectorPosCallback, this);
    gripper_position_sub = nh.subscribe("in/gripper_position", 10, &RosUdp::gripperPositionCallback, this);

//      resize vectors
    joint_states_msg.name.resize(kArmNumberOfJoints);
    joint_states_msg.position.resize(kArmNumberOfJoints);
    joint_states_msg.velocity.resize(kArmNumberOfJoints);
    joint_states_msg.effort.resize(kArmNumberOfJoints);
    joint_states_msg.name[0] = "arm_joint_1";
    joint_states_msg.name[1] = "arm_joint_2";
    joint_states_msg.name[2] = "arm_joint_3";
    joint_states_msg.name[3] = "arm_joint_4";
    joint_states_msg.name[4] = "arm_joint_5";
    joint_states_msg.name[5] = "arm_joint_6";
    joint_states_msg.name[6] = "gripper";

    joint_fault_codes_msg.data.resize(kArmNumberOfJoints);
    joint_mode_msg.data.resize(kArmNumberOfJoints);
}

bool RosUdp::udpInit() {
//    methods to init UDP connection
    memset(&udp_frame_send, 0, sizeof(udp_frame_send));
    memset(&udp_frame_recv, 0, sizeof(udp_frame_recv));
    udp_socket.configureNetwork(port_in, remote_ip, port_out);
    udp_socket.open();
}

bool RosUdp::armInit() {

    uint8_t status = 0;
//      first handshake with Aria arm
    do {
        writeData();
        status = readData();
    } while (status != 0);

    std::cout << "Successful handshake going to next init step" << std::endl;

    udp_frame_send.sControlModeDem = 40; // HOLD POSITION
    writeData();
    status = readData();

    do {
        if (status == 0) {
//            if read status is ok, rewrite received joints positions to send frame (stay where you are)
            for (int k = 0; k < kArmNumberOfJoints; ++k) {
                udp_frame_send.sJointPosDem[k] = udp_frame_recv.sJointPosAct[k];
//                display actual joints position
                std::cout << "J[" << k << "] = " << udp_frame_recv.sJointPosAct[k] << " ";
            }
            std::cout << std::endl;
            udp_frame_send.sCartesianEEPositionDEM[0] = udp_frame_recv.sEEPosAct[0];
            udp_frame_send.sCartesianEEPositionDEM[1] = udp_frame_recv.sEEPosAct[1];
            udp_frame_send.sCartesianEEPositionDEM[2] = udp_frame_recv.sEEPosAct[2];
            udp_frame_send.sCartesianEEQuaternionDEM[0] = udp_frame_recv.sEEQuatAct[0];
            udp_frame_send.sCartesianEEQuaternionDEM[1] = udp_frame_recv.sEEQuatAct[1];
            udp_frame_send.sCartesianEEQuaternionDEM[2] = udp_frame_recv.sEEQuatAct[2];
            udp_frame_send.sCartesianEEQuaternionDEM[3] = udp_frame_recv.sEEQuatAct[3];
//            display actual gripper pose
            std::cout << " pos x :" << udp_frame_send.sCartesianEEPositionDEM[0] << ", pos y :"
                      << udp_frame_send.sCartesianEEPositionDEM[1]
                      << ", pos z: " << udp_frame_send.sCartesianEEPositionDEM[2] << ", orien x: "
                      << udp_frame_send.sCartesianEEQuaternionDEM[0]
                      << ", orien y: " << udp_frame_send.sCartesianEEQuaternionDEM[1] << " orien, z: "
                      << udp_frame_send.sCartesianEEQuaternionDEM[2] << ", orien w: "
                      << udp_frame_send.sCartesianEEQuaternionDEM[3] << std::endl;
        } else std::cout << "UDP receive error: " << int(status) << std::endl;

        writeData();
        status = readData();
    } while ((udp_frame_recv.sArmStatus != udp_frame_send.sControlModeDem) || (status != 0));

//    check actual joint positions
//    for (int m = 0; m < 10; ++m) {
//        writeData();
//        readData();
//        for (int l = 0; l < kArmNumberOfJoints; ++l) {
//            if ((fabs(udp_frame_send.sJointPosDem[l] - udp_frame_recv.sJointPosAct[l]) > 0.01) ||
//                udp_frame_send.sJointPosDem[l] == 0) {
//                ROS_ERROR("Joints position fault");
//                return false;
//            }
//        }
//    }
    return true;
//    }
}

int RosUdp::readData() {

    int status = 0;
    int ret_value;

//    receive data and publish to ROS
    if (ret_value = udp_socket.receiveData(&udp_frame_recv, sizeof(udp_frame_recv)) > 0) {

        uint32_t crc_value;
//          calculate CRC value
        udp_socket.CRC32_calculate(&udp_frame_recv.data_bytes, sizeof(udp_frame_recv.data_bytes), &crc_value);

        if (crc_value == udp_frame_recv.CRC) {
//             Here crc is ok
//             joints states and fault codes
            joint_states_msg.header.stamp = ros::Time::now();
            for (int j = 0; j < kArmNumberOfJoints; ++j) {
                joint_states_msg.position[j] = udp_frame_recv.sJointPosAct[j];
                joint_states_msg.velocity[j] = udp_frame_recv.sJointVelAct[j];
                joint_states_msg.effort[j] = udp_frame_recv.sJointTrqAct[j];
                joint_mode_msg.data[j] = udp_frame_recv.sJointModeAct[j];
                joint_fault_codes_msg.data[j] = udp_frame_recv.sJointFaultCodeAct[j];
            }
            //        arm status
            arm_status_msg.data = udp_frame_recv.sArmStatus;
            //        arm fault code
            arm_fault_code_msg.data = udp_frame_recv.sArmFaultCode;
            //        end effector_pose
            end_effector_pose_msg.position.x = udp_frame_recv.sEEPosAct[0];
            end_effector_pose_msg.position.y = udp_frame_recv.sEEPosAct[1];
            end_effector_pose_msg.position.z = udp_frame_recv.sEEPosAct[2];
            end_effector_pose_msg.orientation.x = udp_frame_recv.sEEQuatAct[0];
            end_effector_pose_msg.orientation.y = udp_frame_recv.sEEQuatAct[1];
            end_effector_pose_msg.orientation.z = udp_frame_recv.sEEQuatAct[2];
            end_effector_pose_msg.orientation.w = udp_frame_recv.sEEQuatAct[3];
            //        gripper position
            gripper_pos_msg.data = udp_frame_recv.sGripperPosAct;
            //        publish messages to ROS
            arm_status_pub.publish(arm_status_msg);
            arm_fault_code_pub.publish(arm_fault_code_msg);
            joint_mode_pub.publish(joint_mode_msg);
            joint_fault_code_pub.publish(joint_fault_codes_msg);
            joint_state_pub.publish(joint_states_msg);
            end_effector_pose_pub.publish(end_effector_pose_msg);
            gripper_pos_pub.publish(gripper_pos_msg);

            status = 0;
        } else {
            ROS_ERROR("CRC ERROR");
            status = 1;
        }
    } else {
        ROS_ERROR("RECEIVE DATA ERROR, CODE: %d", ret_value);
        status = 2;
    }
//      return status of readed data
    return status;
}

void RosUdp::writeData() {
// calculate crc
    uint32_t crc_value;
    udp_socket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
    udp_frame_send.CRC = crc_value;

//    send updated data
    udp_socket.sendData(&udp_frame_send, sizeof(udp_frame_send));
}

void RosUdp::armModeCallback(const std_msgs::UInt8::ConstPtr &msg) {
//    update desired control mode from incoming topic
    uint8_t status = 0;
    do {
        writeData();
        status = readData();
        if (status == 0) {
            udp_frame_send.sControlModeDem = msg->data;
            writeData();
            status = readData();
        }
//         wait until received data id OK and received status is equal to demand status
    } while ((udp_frame_recv.sArmStatus != udp_frame_send.sControlModeDem) || (status != 0));
    ROS_INFO("ARM STATUS = %d", udp_frame_recv.sArmStatus);
}

void RosUdp::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg) {
//    update desired positions from incoming topic
    for (int j = 0; j < kArmNumberOfJoints; ++j) {
        udp_frame_send.sJointPosDem[j] = msg->position[j];
        udp_frame_send.sJointVelDem[j] = msg->velocity[j];
        udp_frame_send.sJointTrqDem[j] = msg->effort[j];
    }
}

void RosUdp::gripperPositionCallback(const std_msgs::Float32::ConstPtr &position) {
//    update gripper position from incoming topic
    udp_frame_send.sGripperPositionDEM = position->data;
}

void RosUdp::endEffectorPosCallback(const geometry_msgs::Pose::ConstPtr &pose) {
//    update end effector pose from incoming topic
    udp_frame_send.sCartesianEEPositionDEM[0] = pose->position.x;
    udp_frame_send.sCartesianEEPositionDEM[1] = pose->position.y;
    udp_frame_send.sCartesianEEPositionDEM[2] = pose->position.z;
    udp_frame_send.sCartesianEEQuaternionDEM[0] = pose->orientation.x;
    udp_frame_send.sCartesianEEQuaternionDEM[1] = pose->orientation.y;
    udp_frame_send.sCartesianEEQuaternionDEM[2] = pose->orientation.z;
    udp_frame_send.sCartesianEEQuaternionDEM[3] = pose->orientation.w;
}
