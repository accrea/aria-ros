//
// Created by accrea on 4/28/20.
//

#include <aria_driver/ros_control_udp.h>

RosControlUDP::RosControlUDP() {

    connectionInit();

//    get joint names from ROS param server
    nh_.getParam("/joint_names", joint_names_);

//    init ROS subscribers
    arm_mode_sub_ = nh_.subscribe("/in/arm_mode", 10, &RosControlUDP::armModeCallback, this);
    cartessian_pose_sub_ = nh_.subscribe(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update", 10,
            &RosControlUDP::cartessianPoseCallback, this);

//    resize vectors
    num_joints_ = joint_names_.size();
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);

    for (int j = 0; j < num_joints_; ++j) {
//    connect joint_state_interface
        hardware_interface::JointStateHandle joint_state_handle(joint_names_[j], &joint_position_[j],
                                                                &joint_velocity_[j],
                                                                &joint_effort_[j]);
        joint_state_interface_.registerHandle(joint_state_handle);
//    connect position_joint_interface
        hardware_interface::JointHandle joint_position_handle(joint_state_handle, &joint_position_command_[j]);
        position_joint_interface_.registerHandle(joint_position_handle);
    }
//    register interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    arm_init_ = false;
    frame_count_ = 0;
}

RosControlUDP::~RosControlUDP() = default;

void RosControlUDP::connectionInit(){
    AriaClient_SetAddress("192.168.9.9", 7777);
    AriaClient_Init();
    do {
        connection_status_ = AriaClient_Connect();
    } while (connection_status_ != 0);

    AriaClient_StartCommunication();
}

void RosControlUDP::read() {
    if (AriaClient_isConnected() > 0) {
        for (int j = 0; j < 7; ++j) {
            joint_position_[j] = AriaClient_GetJointPos(j);
        }
        AriaClient_GetArmEEPosition(&EEPosAct_[0]);
        AriaClient_GetArmEEQuaternion(&EEQuatAct_[0]);
    }
}

void RosControlUDP::write() {

    if (!arm_init_) {

        do {
            AriaClient_SetArmControlMode(50);
        } while (AriaClient_GetArmStatus() != 50);

        arm_init_ = true;
    } else {
        if ((AriaClient_GetArmStatus() == 50) && (frame_count_ > 350)) {
            for (int j = 0; j < 7; ++j) {
                AriaClient_SetJointPos(j, joint_position_command_[j]);
            }
        }
        frame_count_++;
    }
}

void RosControlUDP::armModeCallback(const std_msgs::UInt8::ConstPtr &msg) {
    do {
        AriaClient_SetArmControlMode(msg->data);
    } while (msg->data != AriaClient_GetArmStatus());
}

void RosControlUDP::cartessianPoseCallback(const visualization_msgs::InteractiveMarkerUpdate::ConstPtr &msg) {
    if( (AriaClient_GetArmStatus() == 100) && (msg->poses.size() == 1) && (msg->type == 1) ){
        EEPosDem_[0] = msg->poses[0].pose.position.x;
        EEPosDem_[1] = msg->poses[0].pose.position.y;
        EEPosDem_[2] = msg->poses[0].pose.position.z;

        EEQuatDem_[0] = msg->poses[0].pose.orientation.w;
        EEQuatDem_[1] = msg->poses[0].pose.orientation.x;
        EEQuatDem_[2] = msg->poses[0].pose.orientation.y;
        EEQuatDem_[3] = msg->poses[0].pose.orientation.z;

        AriaClient_SetArmEEPosition(&EEPosDem_[0]);
        AriaClient_SetArmEEQuaternion(&EEQuatDem_[0]);
    }
}
