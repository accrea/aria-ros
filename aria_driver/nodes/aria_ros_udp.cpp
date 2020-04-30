/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include <aria_driver/ros_udp.h>

int main(int argc, char **argv) {
    const int LOOP_RATE = 100;

    ros::init(argc, argv, "ros_udp");
    RosUDP ros_udp;
//    ros::start();
    ros::Rate loop_rate(LOOP_RATE);

//    AriaClient_StartCommunication();

    while (ros_udp.nh_.ok()) {
        if (AriaClient_isConnected() > 0) {
            ros_udp.getAndPublishData();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    AriaClient_StopCommunication();

    return 0;
}