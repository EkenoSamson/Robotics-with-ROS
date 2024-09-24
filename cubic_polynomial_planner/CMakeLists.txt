cmake_minimum_required(VERSION 3.0.2)
project(cubic_polynomial_planner)


find_package(catkin REQUIRED COMPONENTS
	highlevel_msgs
	roscpp
	std_msgs
	geometry_msgs
)

catkin_package(
        INCLUDE_DIRS
		${catkin_INCLUDE_DIRS}
        LIBRARIES
        CATKIN_DEPENDS
                roscpp
                std_msgs
		highlevel_msgs
        DEPENDS
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


## Add executable and libraries
add_executable(pose_planner src/pose_planner.cpp)
target_link_libraries(pose_planner ${catkin_LIBRARIES})


