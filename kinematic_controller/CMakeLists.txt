cmake_minimum_required(VERSION 3.0.2)
project(kinematic_controller)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Find dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

## Add Boost-related definition for Pinocchio
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

# Declare catkin package
catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
        ${PINOCCHIO_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp std_msgs geometry_msgs sensor_msgs
        DEPENDS
)

# Include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        SYSTEM ${EIGEN3_INCLUDE_DIR}
        ${PINOCCHIO_INCLUDE_DIRS}
)

# Add executable
add_executable(inverse_kinematic_controller src/kinematic_controller.cpp)

# Link the executable with necessary libraries
target_link_libraries(inverse_kinematic_controller
        ${catkin_LIBRARIES}
        ${PINOCCHIO_LIBRARIES}
)

# Specify install targets
install(TARGETS inverse_kinematic_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
