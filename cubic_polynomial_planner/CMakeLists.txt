cmake_minimum_required(VERSION 3.0.2)
project(cubic_polynomial_planner)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## to display warning and errors
add_definitions(-Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
		highlevel_msgs
		roscpp
		std_msgs
		geometry_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
		INCLUDE_DIRS
				include
		${catkin_INCLUDE_DIRS}
        LIBRARIES
        CATKIN_DEPENDS
                roscpp
                std_msgs
				highlevel_msgs
				geometry_msgs
        DEPENDS
)


include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${EIGEN_INCLUDE_DIRS}
)


## Add executable and libraries
add_executable(pose_planner src/pose_planner.cpp)
target_link_libraries(pose_planner ${catkin_LIBRARIES})


