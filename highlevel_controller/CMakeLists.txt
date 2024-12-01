cmake_minimum_required(VERSION 3.0.2)

# Project name
project(highlevel_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_controller
        actionlib
        highlevel_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
                    ${EIGEN_INCLUDE_DIRS}
        CATKIN_DEPENDS
            roscpp
            std_msgs
            message_runtime
            actionlib
            highlevel_msgs
)

include_directories(
        include
        ${EIGEN_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(action_client
        src/client.cpp
        src/action_client.cpp
)

target_link_libraries(action_client ${catkin_LIBRARIES} )



