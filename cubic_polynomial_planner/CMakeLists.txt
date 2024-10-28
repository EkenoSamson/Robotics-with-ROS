cmake_minimum_required(VERSION 3.0.2)
project(cubic_polynomial_planner)

## Find catkin macros and libraries
## 'roscpp' is for C++ ROS client library, 'std_msgs' for standard messages,
## 'highlevel_msgs' for custom services, and 'message_generation' for generating message/service files
find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		geometry_msgs
		highlevel_msgs
		message_generation  					# Required to generate custom messages and services
)

find_package(Eigen3 REQUIRED)					# Locate Eigen3

###################################
## catkin specific configuration ##
###################################

## catkin_package() exports your package's dependencies for other ROS packages that rely on it
catkin_package(
		INCLUDE_DIRS include  											# The include directory contains your header files
		LIBRARIES cubic_polynomial_planner  							# Library to link with your source code
		CATKIN_DEPENDS
					geometry_msgs roscpp std_msgs
					highlevel_msgs message_runtime  	# Packages needed at runtime
		DEPENDS
)

###########
## Build ##
###########

## Specify additional locations for header files
## This tells CMake where to find the package's own header files and the headers of dependencies
include_directories(
		include  									# Include your package's header files in the 'include' folder
		${catkin_INCLUDE_DIRS}  					# Include directories from all dependencies (roscpp, std_msgs, etc.)
		${EIGEN3_INCLUDE_DIRS}						# Include Eigen's header files
)

## Declare a C++ library
## The library will be built from cubic_polynomial_planner.cpp (the core logic)
add_library(cubic_polynomial_planner
		src/cubic_polynomial_planner.cpp  			# Source file for your main logic
)

## Link the library with catkin libraries
## This links the library with ROS libraries like roscpp and other catkin dependencies
target_link_libraries(cubic_polynomial_planner
		${catkin_LIBRARIES}  						# Links the catkin libraries to your library
)

## Declare a C++ executable
## This executable will be built from main.cpp and will use the cubic_polynomial_planner library
add_executable(pose_planner src/main.cpp)  			# Main entry point of your program

## Link your executable to your library and catkin dependencies
## This ensures your node uses the cubic_polynomial_planner library and all necessary ROS libraries
target_link_libraries(pose_planner
		cubic_polynomial_planner  					# Link the executable to the cubic_polynomial_planner library
		${catkin_LIBRARIES}  						# Link the catkin libraries to the executable
)

#############
## Install ##
#############

## Mark the library for installation
install(TARGETS cubic_polynomial_planner
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark executables for installation
## This ensures the planner_node executable gets installed properly
install(TARGETS pose_planner
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g., launch files)
## This installs any launch files located in the 'launch/' directory
install(DIRECTORY launch/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############

## Unit tests can be added here (optional)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cubic_polynomial_planner.cpp)
# target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
