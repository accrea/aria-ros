/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include "aria_driver/ros_udp.h"

int main(int argc, char **argv) {
    const int LOOP_RATE = 100;

    ros::init(argc, argv, "aria_v2_driver");
    RosUdp ros_udp;
//    ros::start();
    ros::Rate loop_rate(LOOP_RATE);

    ros_udp.rosInit();
    ros_udp.udpInit();

    if (ros_udp.armInit()) {
        ROS_INFO("ARM INIT COMPLETE");
        while (ros_udp.nh.ok()) {
            ros_udp.writeData();
            ros_udp.readData();
            ros::spinOnce();
            loop_rate.sleep();
        }
    } else ROS_ERROR("Aria arm init failed");

    return 0;
}
