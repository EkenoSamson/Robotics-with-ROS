cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project name
project(highlevel_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_controller
        actionlib
        highlevel_msgs
        control_msgs
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
            control_msgs
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

add_executable(pick_and_place_client
        src/pick_and_place_client.cpp
        src/pick_and_place_main.cpp
)

target_link_libraries(pick_and_place_client ${catkin_LIBRARIES})


