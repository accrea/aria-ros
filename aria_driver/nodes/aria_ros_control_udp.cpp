/**************************************************
 *
 * Copyright (C) ACCREA 2020, All rights reserved
 * Authors: Piotr Jagiełło, Damian Muszyński
 * email: p.jagiello@accrea.com, d.muszynski@accrea.com
 * website: engineering.accrea.com
 *
 *************************************************/

#include "aria_driver/ros_control_udp.h"
#include <controller_manager/controller_manager.h>


int main(int argc, char *argv[]) {
    const int LOOP_HZ = 100;

    ros::init(argc, argv, "ros_control_udp");

    RosControlUdp rosControlUdp;
    controller_manager::ControllerManager controllerManager(&rosControlUdp);

    ros::AsyncSpinner spinner(1);
    spinner.start();

//    ros::Time prev_time = ros::Time::now();
    ros::Rate loop_rate(LOOP_HZ);

    while (rosControlUdp.nh.ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period(0.1);

        rosControlUdp.write();
        controllerManager.update(time, period);
        rosControlUdp.read();

        loop_rate.sleep();
    }

    return 0;
}
