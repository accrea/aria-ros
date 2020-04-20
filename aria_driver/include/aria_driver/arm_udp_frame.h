/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#ifndef ARIA_ROBOT_HW_ARM_UDP_FRAME_H
#define ARIA_ROBOT_HW_ARM_UDP_FRAME_H

// UDP frame used in communication with Aria arm
const unsigned int kArmNumberOfJoints = 7;

// UDP inputs to ARIA arm
typedef struct __attribute__((__packed__))
{
    union __attribute__((__packed__))
    {
        struct __attribute__((__packed__))
        {
            uint8_t sControlModeDem;
            float sJointPosDem[kArmNumberOfJoints];
            float sJointVelDem[kArmNumberOfJoints];
            float sJointTrqDem[kArmNumberOfJoints];
            float sCartesianEEPositionDEM[3];
            float sCartesianEEQuaternionDEM[4];
            float sGripperPositionDEM;
        };
        uint8_t data_bytes[117];
    };
    uint32_t CRC;
}
arm_udp_frame_send_t;

// UDP outputs from ARIA arm
typedef struct __attribute__((__packed__))
{
    union __attribute__((__packed__))
    {
        struct __attribute__((__packed__))
        {
            uint8_t sArmStatus;
            uint16_t sArmFaultCode;
            uint8_t sJointModeAct[kArmNumberOfJoints];
            uint16_t sJointFaultCodeAct[kArmNumberOfJoints];
            float sJointPosAct[kArmNumberOfJoints];
            float sJointVelAct[kArmNumberOfJoints];
            float sJointTrqAct[kArmNumberOfJoints];
            float sEEPosAct[3];
            float sEEQuatAct[4];
            float sGripperPosAct;
        };
        uint8_t data_bytes[140];
    };
    uint32_t CRC;
}
arm_udp_frame_recv_t;

#endif  // ARIA_ROBOT_HW_ARM_UDP_FRAME_H
