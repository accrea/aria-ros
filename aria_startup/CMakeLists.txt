cmake_minimum_required(VERSION 2.8.3)
project(aria_startup)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  aria_description
  roscpp
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
