//
// Created by accrea on 4/28/20.
//

#include "aria_driver/ros_control_udp.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv){
    const int LOOP_HZ = 50;
    static const double BILLION = 1000000000.0;

    ros::init(argc, argv, "ros_control_udp");

    RosControlUDP rosControlUDP;
    controller_manager::ControllerManager controllerManager(&rosControlUDP);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    struct timespec current_time_;
    struct timespec last_time_;
    ros::Rate loop_rate(LOOP_HZ);
    ros::Duration elapsed_time_;
    ros::Duration desired_update_period_ = ros::Duration(1 / LOOP_HZ);
    double cycle_time_error_threshold_ = 0.1;

    while(rosControlUDP.nh_.ok()){
        clock_gettime(CLOCK_MONOTONIC, &current_time_);
  	elapsed_time_ = ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
  	last_time_ = current_time_;
  	
  	const double cycle_time_error = (elapsed_time_ - desired_update_period_).toSec();
        if (cycle_time_error > cycle_time_error_threshold_)
        {
         ROS_WARN_STREAM_NAMED("CONTROL_LOOP", "Cycle time exceeded error threshold by: " << cycle_time_error << ", cycle time: " << elapsed_time_ << ", threshold: " << cycle_time_error_threshold_);
        }
  	
        if (AriaClient_isConnected() > 0){
            rosControlUDP.write();
            controllerManager.update(ros::Time::now(), elapsed_time_);
            rosControlUDP.read();
        }
        //ros::spinOnce();
        loop_rate.sleep();
    }

    AriaClient_StopCommunication();

    return 0;
}
