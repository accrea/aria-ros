cmake_minimum_required(VERSION 2.8.3)
project(aria_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
#        aria_description
        geometry_msgs
        roscpp
        rospy
        sensor_msgs
        std_msgs
        controller_manager
        )

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(INCLUDE_DIRS include)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(aria_ros nodes/aria_ros_udp.cpp src/ros_udp.cpp src/udp_socket.cpp)
add_executable(aria_ros_control nodes/aria_ros_control_udp.cpp src/ros_control_udp.cpp src/udp_socket.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(aria_ros ${catkin_LIBRARIES})
target_link_libraries(aria_ros_control ${catkin_LIBRARIES})
