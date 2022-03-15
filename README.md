# ROS SUPPORT FOR ARIA ARM
- [Basic Informations](#Basic-Informations)
- [Brief Description](#Brief-description)
- [File Contents](#File-contents)
- [Installation](#installation)
- [How to Use](#How-to-Use)

# Basic Informations
This repository contains ROS (Robot Operation System) support for ACCREA Aria Arm. 

Package has been tested on Ubuntu 18.04 LTS with ROS Melodic.



# Brief Description
The aria-ros package provides interface to control Aria robotic arm using ROS Topics and ROS Control. Full description of ROS Control is available [here](http://wiki.ros.org/ros_control). 



# File Contents
- `aria_startup`: contains connection configuration file and launch files to bringup connection with real robotic arm,
- `aria_demo`: demo files, works with MoveIt! (Pick and Place demo and Helix Trajectory demo),
- `aria_description`: contains URDF and meshes,
- `aria_driver`: provides interface between ROS and Aria arm, uses UDP protocol. It's not recommended to make any changes here,
- `aria_gazebo`: support for visualization in Gazebo with use of rqt_controller,
- `aria_moveit`: contains files needed to work with MoveIt!. It allows to move the arm in simulated and real environment.

# Installation
We assume you are using Ubuntu 18.04 LTS and ROS Melodic. If you're not sure what version of ROS do you have, type `rosversion -d` in terminal. Full installation guide for ROS Melodic is available [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

To add `aria-ros` package to your workspace, run the following commands:
```
cd ~/path_to_your_workspace/src
git clone https://github.com/accrea/aria-ros
cd ~/path_to_your_workspace
catkin_make
```

Also, make sure you have installed `ros-control`, `controller-manager` and `Moveit` (if not, run in terminal `sudo apt-get install ros-<rosdistro>-ros-control`, `sudo apt-get install ros-<rosdistro>-joint-trajectory-controller` `sudo apt-get install ros-<rosdistro>-controller-manager`, `sudo apt-get install ros-<rosdistro>-moveit*` and `sudo apt-get install ros-<rosdistro>-joint-state-controller`).

# How to use

### Work with real arm
To work with real robotic arm, run in terminal:

    roslaunch aria_startup aria_startup.launch

By default it will startup connection between Aria arm and ROS without possibility to use ROS Control. In order to use Aria arm with ROS Control please run:

    roslaunch aria_startup aria_startup.launch use_ros_control:=true

If you want to use additionally `rqt_joint_trajectory_controller`  run the following commend: 

    roslaunch aria_startup aria_startup.launch use_ros_control:=true use_joint_trajectory_controller_gui:=true

In order to use MoveIt! run in the next shell: 

    roslaunch aria_v2_moveit_config moveit_planning_execution.launch

### Arm simulation

- To simulate Aria arm in Gazebo, run:

        roslaunch aria_gazebo aria_gazebo.launch
    
    It will bringup simulated arm with `rqt_joint_trajectory controller` similar than ros-control with real robot. You can use sliders to change joints positions.


- To simulate arm movement in Rviz (without using MoveIt!):

        roslaunch aria_description display_arm.launch

    Then will appear virtual arm in initial position and simple GUI to change joints positions.

- To simulate arm movement in Rviz with use of MoveIt!:

        roslaunch aria_v2_moveit_config demo.launch
    

If you're working with virtual arm in MoveIt!, you can check demo files placed in `aria_demo` directory. In next shell type `rosrun aria_demo demo_pick_and_place` to run pick and place demo or type `rosrun aria_demo demo_trajectory` to execute helix trajectory. 
