cmake_minimum_required(VERSION 3.0.2)

project(keyboard_controller)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS
                geometry_msgs
                roscpp
                std_msgs
        DEPENDS
)

## package locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Add executable and libraries
add_executable(keyboard_controller src/keyboard_controller.cpp)
target_link_libraries(keyboard_controller ${catkin_LIBRARIES})

