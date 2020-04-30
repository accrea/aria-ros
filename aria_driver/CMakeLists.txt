cmake_minimum_required(VERSION 2.8.3)
project(aria_driver)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

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

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(FindPkgConfig)
find_package(AriaClient REQUIRED)

catkin_package(INCLUDE_DIRS include)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${ARIACLIENT_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(aria_ros nodes/aria_ros_udp.cpp src/ros_udp.cpp)
add_executable(aria_ros_control nodes/aria_ros_control_udp.cpp src/ros_control_udp.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(aria_ros ${catkin_LIBRARIES} ${ARIACLIENT_LIBRARY})
target_link_libraries(aria_ros_control ${catkin_LIBRARIES} ${ARIACLIENT_LIBRARY})


#add_executable(aria_ros_new nodes/aria_ros_udp.cpp src/ros_udp.cpp)
#target_link_libraries(aria_ros_new ${catkin_LIBRARIES} ${ARIACLIENT_LIBRARY})
#
#add_executable(aria_ros_control_new nodes/aria_ros_control_udp.cpp src/ros_control_udp.cpp)
#target_link_libraries(aria_ros_control_new ${catkin_LIBRARIES} ${ARIACLIENT_LIBRARY})