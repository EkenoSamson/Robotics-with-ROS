# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(dynamic_controller)

# Set the C++ standard to 14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Uncomment to add compiler warnings (if needed)
# add_definitions(-Wall -Werror) # Remove or comment to avoid strict compilation rules


## Find catkin macros and libraries
# REQUIRED: roscpp and std_msgs
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        highlevel_msgs
        potential_field_planner
        cubic_polynomial_planner
)


## Find Eigen and Pinocchio
# Use PkgConfig to find Eigen3 libraries
find_package(PkgConfig REQUIRED)
pkg_search_module(Eigen3 REQUIRED eigen3)

# Find Pinocchio library
find_package(pinocchio REQUIRED)

# Add a definition for Pinocchio to manage BOOST MPL size limitation
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")


## Catkin Package Configuration
catkin_package(
        INCLUDE_DIRS
        include  # Add the include directory for the headers
        ${Eigen3_INCLUDE_DIRS}  # Include Eigen headers
        ${PINOCCHIO_INCLUDE_DIRS}  # Include Pinocchio headers
        LIBRARIES
        # Define if you have any libraries to export
        CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        highlevel_msgs
        potential_field_planner
        cubic_polynomial_planner
        DEPENDS
        Eigen3
        pinocchio
)

#############
## Build ##
#############

## Specify additional locations of header files
## The include directory should come before others
include_directories(
        include  # Add the project's include directory
        ${catkin_INCLUDE_DIRS}  # Include directories from catkin dependencies
        SYSTEM ${Eigen3_INCLUDE_DIRS}  # Eigen include directories from pkg_search_module
        ${PINOCCHIO_INCLUDE_DIRS}  # Pinocchio include directories
)

## Add Executable: joint_controller_node
# Add an executable for the node 'joint_controller' node
add_executable(joint_controller
        src/dynamic_controller.cpp  # The main source file
        src/controller.cpp          # Controller logic implementation
)

# Add an executable for the node 'pose_controller' node
add_executable(pose_controller
        src/pose_controller_main.cpp     # The main source file
        src/pose_controller.cpp          # Controller logic implementation
)


# Link the libraries to the executable 'joint_controller_node'
target_link_libraries(joint_controller
        ${catkin_LIBRARIES}  # Link catkin libraries
        ${PINOCCHIO_LIBRARIES}  # Link Pinocchio libraries
)

# Link the libraries to the executable 'pose_controller' node
target_link_libraries(pose_controller
        ${catkin_LIBRARIES}  # Link catkin libraries
        ${PINOCCHIO_LIBRARIES}  # Link Pinocchio libraries
)



#############
## Install ##
#############

# Mark the header files for installation
# Install all .hpp files in the include directory
install(
        DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp"
)

# Mark executables and libraries for installation
install(
        TARGETS joint_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
        TARGETS pose_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

