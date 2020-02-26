cmake_minimum_required(VERSION 2.8.3)
project(aria_demo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
        interactive_markers
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        moveit_ros_perception
        roscpp
        rospy
        std_msgs
        )
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system date_time thread)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES aria_demo
        CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        moveit_core
        moveit_visual_tools
        moveit_ros_planning_interface
        interactive_markers
        DEPENDS EIGEN3
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
)


## C++ executable

add_executable(demo_pick_and_place src/PickAndPlace.cpp)
add_executable(demo_trajectory src/trajectory.cpp)

## Libraries to link a library or executable
target_link_libraries(demo_pick_and_place
        ${catkin_LIBRARIES} ${Boost_LIBRARIES}
        )
target_link_libraries(demo_trajectory
        ${catkin_LIBRARIES} ${Boost_LIBRARIES}
        )
