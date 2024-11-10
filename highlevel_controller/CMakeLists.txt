cmake_minimum_required(VERSION 3.0.2)

# Project name
project(highlevel_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_controller
)

catkin_package(
 CATKIN_DEPENDS
        roscpp
        std_msgs
        message_runtime
)


