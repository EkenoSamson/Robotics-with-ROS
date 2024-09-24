cmake_minimum_required(VERSION 3.0.2)
project(highlevel_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	genmsg
	std_msgs
)


## Generate services in the 'srv' folder
add_service_files(
	FILES
	MoveTo.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS
                genmsg
		message_generation
                roscpp
		std_msgs
        DEPENDS
)

## package locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
