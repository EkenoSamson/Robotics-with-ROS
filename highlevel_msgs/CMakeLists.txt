# Minimum CMake version required
cmake_minimum_required(VERSION 3.0.2)

# Declare ROS package name
project(highlevel_msgs)

## Find catkin macros and libraries
## This also finds other ROS packages that are required at build and runtime
find_package(catkin REQUIRED COMPONENTS
		roscpp  							# ROS C++ client library
		std_msgs  							# Standard ROS message types
		message_generation  				# Required to generate custom messages and services
		actionlib_msgs						# For generating ROS action types
)

################################################
## Declare ROS messages, services, and actions ##
################################################

# Generate services from srv files in the 'srv' directory
add_service_files(
		FILES
		MoveTo.srv  						# for setting trajectory
		SetKD.srv							# for setting stiffness and damping
)

# Generate actions from action files
add_action_files(
		DIRECTORY action
		FILES PoseCommand.action
)

## Generate added messages and services
## 'std_msgs' dependency is required here because we're using standard message types in our service(float64 and bool)
generate_messages(
		DEPENDENCIES
		std_msgs  							# std_msgs is included because we're using float64 and bool
		actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

# Declare the catkin package
## This tells other packages what to depend on if they rely on this package.
catkin_package(
		INCLUDE_DIRS
		LIBRARIES
		CATKIN_DEPENDS
			roscpp std_msgs message_runtime actionlib_msgs # Dependencies needed at runtime
		DEPENDS
)

###########
## Build ##
###########

# Specify additional locations of header files
## Header files that are part of this package (if any) are located in the 'include' directory
include_directories(
		include
		${catkin_INCLUDE_DIRS}  # Include directories from catkin dependencies (e.g., roscpp)
)
