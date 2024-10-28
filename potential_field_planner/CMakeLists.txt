# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(potential_field_planner)

# Set the C++ standard to 14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Uncomment to add compiler warnings (if needed)
# add_definitions(-Wall -Werror) # Remove or comment to avoid strict compilation rules

## Find catkin macros and libraries
# REQUIRED: roscpp and geometry_msgs
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
)

## Find Eigen and Pinocchio
# Use PkgConfig to find Eigen3 and Pinocchio libraries
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
        ${Eigen3_INCLUDE_DIRS}  # Eigen include directories from pkg_search_module
        ${PINOCCHIO_INCLUDE_DIRS}  # Pinocchio include directories
)

## Add Executable: joint_space_planner
# Add an executable for the node 'joint_space_planner'
add_executable(joint_planner
        src/joint_space_planner.cpp
        src/potential_field_planner.cpp  # Add your planner implementation
)

# Link the libraries to the executable 'joint_space_planner'
target_link_libraries(joint_planner
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
        TARGETS joint_planner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
