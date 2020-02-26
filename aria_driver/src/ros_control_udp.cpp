/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include "aria_driver/ros_control_udp.h"

RosControlUdp::RosControlUdp() {
    //    init ROS params
    nh.param("/port_in", port_in, 7777);
    nh.param("/port_out", port_out, 7777);
    nh.param<std::string>("/remote_ip", remote_ip, "192.168.9.9");

//    init ROS subscribers
    arm_mode_sub = nh.subscribe("in/arm_mode", 10, &RosControlUdp::armModeCallback, this);
    cartessian_pose_sub = nh.subscribe(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 1,
            &RosControlUdp::cartessianPoseCallback, this);

    //    resize vectors
    joint_names.push_back("arm_joint_1");
    joint_names.push_back("arm_joint_2");
    joint_names.push_back("arm_joint_3");
    joint_names.push_back("arm_joint_4");
    joint_names.push_back("arm_joint_5");
    joint_names.push_back("arm_joint_6");
    joint_names.push_back("finger_joint_1");
    joint_names.push_back("finger_joint_2");
    joint_names.push_back("finger_joint_3");
    num_joints = joint_names.size();
    joint_position.resize(num_joints);
    joint_velocity.resize(num_joints);
    joint_effort.resize(num_joints);
    joint_position_command.resize(num_joints);

    for (int j = 0; j < num_joints; ++j) {
//    connect joint_state_interface
        hardware_interface::JointStateHandle joint_state_handle(joint_names[j], &joint_position[j],
                                                                &joint_velocity[j],
                                                                &joint_effort[j]);
        joint_state_interface.registerHandle(joint_state_handle);
//    connect position_joint_interface
        hardware_interface::JointHandle joint_position_handle(joint_state_handle, &joint_position_command[j]);
        position_joint_interface.registerHandle(joint_position_handle);
    }
//    register interfaces
    registerInterface(&joint_state_interface);
    registerInterface(&position_joint_interface);

    //    configure and open UDP connection
    memset(&udp_frame_send, 0, sizeof(udp_frame_send));
    memset(&udp_frame_recv, 0, sizeof(udp_frame_recv));
    udpSocket.configureNetwork(port_in, remote_ip, port_out);
    udpSocket.open();

    arm_init = false;
    frame_count = 0;
}

RosControlUdp::~RosControlUdp() {

}

int RosControlUdp::read() {
    int status = 0;
    int ret_value;

//    receive data from hardware
    if (ret_value = udpSocket.receiveData(&udp_frame_recv, sizeof(udp_frame_recv)) > 0) {

//        calculate CRC value
        uint32_t crc_value;
        udpSocket.CRC32_calculate(&udp_frame_recv.data_bytes, sizeof(udp_frame_recv.data_bytes), &crc_value);

//        if crc values are equal, read incoming data from Aria arm
        if (crc_value == udp_frame_recv.CRC) {
//            joint position
            joint_position[0] = udp_frame_recv.sJointPosAct[0];
            joint_position[1] = udp_frame_recv.sJointPosAct[1];
            joint_position[2] = udp_frame_recv.sJointPosAct[2];
            joint_position[3] = udp_frame_recv.sJointPosAct[3];
            joint_position[4] = udp_frame_recv.sJointPosAct[4];
            joint_position[5] = udp_frame_recv.sJointPosAct[5];
            joint_position[6] = udp_frame_recv.sJointPosAct[6];
            joint_position[7] = udp_frame_recv.sJointPosAct[6];
            joint_position[8] = udp_frame_recv.sJointPosAct[6];
//            gripper pose
//            gripper_pose.position.x = udp_frame_recv.sEEPosAct[0];
//            gripper_pose.position.y = udp_frame_recv.sEEPosAct[1];
//            gripper_pose.position.z = udp_frame_recv.sEEPosAct[2];
//            gripper_pose.orientation.x = udp_frame_recv.sEEQuatAct[0];
//            gripper_pose.orientation.y = udp_frame_recv.sEEQuatAct[1];
//            gripper_pose.orientation.z = udp_frame_recv.sEEQuatAct[2];
//            gripper_pose.orientation.w = udp_frame_recv.sEEQuatAct[3];
//            for (int j = 0; j < kArmNumberOfJoints; ++j) {
//                std::cout << "J[" << j+1 << "]: " << joint_position[j] << " " ;
//            }
//            std::cout << std::endl;
            status = 0;
        } else {
            ROS_ERROR("CRC ERROR");
            status = 1;
        }
    } else {
        ROS_ERROR("RECEIVE DATA ERROR, CODE: %d", ret_value);
        status = 2;
    }
//    return status of readed data
    return status;
}

void RosControlUdp::write() {
//    calculate CRC
    uint32_t crc_value;
    udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
    udp_frame_send.CRC = crc_value;

    int status = 0;
//      arm not initialized
    if (!arm_init) {
//        fist handshake with Aria arm
        do {
            udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
            status = read();
        } while (status != 0);

        std::cout << "Successful handshake going to next init step" << std::endl;

//        send HOLD POSITION mode
        udp_frame_send.sControlModeDem = 40;
        do {
            udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
            udp_frame_send.CRC = crc_value;
            udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
            status = read();
        } while (status != 0);
        do {
            if (status == 0) {
//                if read status is ok, rewrite received joints positions and gripper pose to send frame (stay where you are)
                for (int k = 0; k < kArmNumberOfJoints; ++k) {
                    udp_frame_send.sJointPosDem[k] = udp_frame_recv.sJointPosAct[k];
//                    display artual joints positions
                    std::cout << "J[" << k << "] = " << udp_frame_send.sJointPosDem[k] << ", ";
                }
                std::cout << std::endl;
                udp_frame_send.sCartesianEEPositionDEM[0] = udp_frame_recv.sEEPosAct[0];
                udp_frame_send.sCartesianEEPositionDEM[1] = udp_frame_recv.sEEPosAct[1];
                udp_frame_send.sCartesianEEPositionDEM[2] = udp_frame_recv.sEEPosAct[2];
                udp_frame_send.sCartesianEEQuaternionDEM[0] = udp_frame_recv.sEEQuatAct[0];
                udp_frame_send.sCartesianEEQuaternionDEM[1] = udp_frame_recv.sEEQuatAct[1];
                udp_frame_send.sCartesianEEQuaternionDEM[2] = udp_frame_recv.sEEQuatAct[2];
                udp_frame_send.sCartesianEEQuaternionDEM[3] = udp_frame_recv.sEEQuatAct[3];
//                display actual end effector pose
                std::cout << " Pos x :" << udp_frame_send.sCartesianEEPositionDEM[0] << ", pos y :"
                          << udp_frame_send.sCartesianEEPositionDEM[1]
                          << ", pos z: " << udp_frame_send.sCartesianEEPositionDEM[2] << ", orien x: "
                          << udp_frame_send.sCartesianEEQuaternionDEM[0]
                          << ", orien y: " << udp_frame_send.sCartesianEEQuaternionDEM[1] << ", orien, z: "
                          << udp_frame_send.sCartesianEEQuaternionDEM[2] << ", orien w: "
                          << udp_frame_send.sCartesianEEQuaternionDEM[3] << std::endl;
            } else std::cout << "UDP receive error: " << int(status) << std::endl;

            udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
            udp_frame_send.CRC = crc_value;

            udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
            status = read();
        } while ((udp_frame_recv.sArmStatus != udp_frame_send.sControlModeDem) || (status != 0));

        do {

            udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
            udp_frame_send.CRC = crc_value;

            udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
            status = read();
//            if read status is ok, try to switch arm in position control mode
            if (status == 0) {
                udp_frame_send.sControlModeDem = 50;

                udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
                udp_frame_send.CRC = crc_value;

                udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
                status = read();
            }
        } while ((udp_frame_recv.sArmStatus != udp_frame_send.sControlModeDem) || (status != 0));
//        display Aria arm status
        ROS_INFO("ARM STATUS = %d", udp_frame_recv.sArmStatus);

//        change init_arm value to "true"
        arm_init = true;
    }
//    arm initialized
    else {
//        if Aria arm is in position control mode send values from joint_position_command vectors, wait some time to initialize values in vector
        if (udp_frame_recv.sArmStatus == 50 && frame_count > 350) {
            for (int j = 0; j < kArmNumberOfJoints; ++j) {
                udp_frame_send.sJointPosDem[j] = joint_position_command[j];
            }
            udp_frame_send.sCartesianEEPositionDEM[0] = udp_frame_recv.sEEPosAct[0];
            udp_frame_send.sCartesianEEPositionDEM[1] = udp_frame_recv.sEEPosAct[1];
            udp_frame_send.sCartesianEEPositionDEM[2] = udp_frame_recv.sEEPosAct[2];
            udp_frame_send.sCartesianEEQuaternionDEM[0] = udp_frame_recv.sEEQuatAct[0];
            udp_frame_send.sCartesianEEQuaternionDEM[1] = udp_frame_recv.sEEQuatAct[1];
            udp_frame_send.sCartesianEEQuaternionDEM[2] = udp_frame_recv.sEEQuatAct[2];
            udp_frame_send.sCartesianEEQuaternionDEM[3] = udp_frame_recv.sEEQuatAct[3];
        }
        frame_count++;
        if (frame_count == 350)
            ROS_INFO("Arm init complete");
        udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
        udp_frame_send.CRC = crc_value;
        udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));

    }
//     display sended values
//    for (int k = 0; k < kArmNumberOfJoints; ++k) {
//        std::cout << "J[" << k + 1 << "]: " << udp_frame_send.sJointPosDem[k] << " ";
//    }
//    std::cout << std::endl;

//    std::cout << " Pos x :" << udp_frame_recv.sEEPosAct[0] << ", pos y :"
//              << udp_frame_recv.sEEPosAct[1]
//              << ", pos z: " << udp_frame_recv.sEEPosAct[2] << ", orien x: "
//              << udp_frame_recv.sEEQuatAct[0]
//              << ", orien y: " << udp_frame_recv.sEEQuatAct[1] << ", orien, z: "
//              << udp_frame_recv.sEEQuatAct[2] << ", orien w: "
//              << udp_frame_recv.sEEQuatAct[3] << std::endl;
}

void RosControlUdp::armModeCallback(const std_msgs::UInt8::ConstPtr &msg) {
//    callback to change arm mode
    uint8_t internal_status = 0;
    do {
        uint32_t crc_value;
        udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
        udp_frame_send.CRC = crc_value;
        udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
        internal_status = read();
//        if read status is ok, try to switch arm in demand control mode (value from incoming topic)
        if (internal_status == 0) {
            udp_frame_send.sControlModeDem = msg->data;

            udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
            udp_frame_send.CRC = crc_value;

            udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
            internal_status = read();
        }
        //         wait until received data id OK and received status is equal to demand status
    } while ((udp_frame_recv.sArmStatus != udp_frame_send.sControlModeDem) || (internal_status != 0));
//    display changed Aria arm status
    ROS_INFO("ARM STATUS = %d", udp_frame_recv.sArmStatus);
}

void RosControlUdp::cartessianPoseCallback(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr &msg) {
//    if ( (frame_count > 350) && (msg->poses.size() == 1) ) {
//    callback method, allows to control Aria arm using interactive marker displayed in MoveIt!
//    Aria arm must be in cartessian control mode
    if ((udp_frame_recv.sArmStatus == 100) && (msg->poses.size() == 1) && (msg->type == 1)) {

//        set cartessian pose goal
        udp_frame_send.sCartesianEEPositionDEM[0] = msg->poses[0].pose.position.x;
        udp_frame_send.sCartesianEEPositionDEM[1] = msg->poses[0].pose.position.y;
        udp_frame_send.sCartesianEEPositionDEM[2] = msg->poses[0].pose.position.z;
        udp_frame_send.sCartesianEEQuaternionDEM[0] = msg->poses[0].pose.orientation.w;
        udp_frame_send.sCartesianEEQuaternionDEM[1] = msg->poses[0].pose.orientation.x;
        udp_frame_send.sCartesianEEQuaternionDEM[2] = msg->poses[0].pose.orientation.y;
        udp_frame_send.sCartesianEEQuaternionDEM[3] = msg->poses[0].pose.orientation.z;
//        std::cout << "pos x:" << msg->poses[0].pose.position.x << " pos y:" << msg->poses[0].pose.position.y
//                  << " pos z:"
//                  << msg->poses[0].pose.position.z << " orien x:" << msg->poses[0].pose.orientation.x << " orien y:"
//                  << msg->poses[0].pose.orientation.y << " orien z:" << msg->poses[0].pose.orientation.z << " orien w:"
//                  << msg->poses[0].pose.orientation.w << std::endl;
//        set actual joint positions
        for (int j = 0; j < kArmNumberOfJoints; ++j) {
            udp_frame_send.sJointPosDem[j] = udp_frame_recv.sJointPosAct[j];
        }

        uint32_t crc_value;
        udpSocket.CRC32_calculate(&udp_frame_send.data_bytes, sizeof(udp_frame_send.data_bytes), &crc_value);
        udp_frame_send.CRC = crc_value;

        udpSocket.sendData(&udp_frame_send, sizeof(udp_frame_send));
        read();
    }
//    ROS_INFO("cartessianPoseCallback");
}
