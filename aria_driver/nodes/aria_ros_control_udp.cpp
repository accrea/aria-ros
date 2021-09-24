//
// Created by accrea on 4/28/20.
//

#include "aria_driver/ros_control_udp.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv){
    const int LOOP_HZ = 100;

    ros::init(argc, argv, "ros_control_udp");

    RosControlUDP rosControlUDP;
    controller_manager::ControllerManager controllerManager(&rosControlUDP);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate loop_rate(LOOP_HZ);



    while(rosControlUDP.nh_.ok()){
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        if (AriaClient_isConnected() > 0){
            rosControlUDP.write();
            controllerManager.update(time, period);
            rosControlUDP.read();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    AriaClient_StopCommunication();

    return 0;
}