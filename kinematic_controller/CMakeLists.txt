cmake_minimum_required(VERSION 3.0.2)
project(kinematic_controller)

## Use C++17
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Find dependencies
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        sensor_msgs
        cubic_polynomial_planner
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
        roscpp std_msgs geometry_msgs sensor_msgs cubic_polynomial_planner
        DEPENDS pinocchio
)

# Include directories
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        SYSTEM ${EIGEN3_INCLUDE_DIR}
        ${PINOCCHIO_INCLUDE_DIRS}
)


## Declare a C++ library
## The library will be built from kinematic_controller.cpp (the core logic)
add_library(kinematic_controller
        src/kinematic_controller.cpp  			# Source file for your main logic
)

## Link the library with catkin libraries
## This links the library with ROS libraries like roscpp and other catkin dependencies
target_link_libraries(kinematic_controller
        ${catkin_LIBRARIES}  						# Links the catkin libraries to your library
        ${PINOCCHIO_LIBRARIES}					    # Links Pinocchio libraries
)

## Declare a C++ executable
## This executable will be built from main.cpp and will use the kinematic_controller library
add_executable(inverse_kinematic_controller src/main.cpp)  			# Main entry point of your program

## Link your executable to your library and catkin dependencies
## This ensures your node uses the cubic_polynomial_planner library and all necessary ROS libraries
target_link_libraries(inverse_kinematic_controller
        kinematic_controller  					    # Link the executable to the kinematic_controller library
        ${catkin_LIBRARIES}  						# Link the catkin libraries to the executable
)

#############
## Install ##
#############

## Mark the library for installation
install(TARGETS kinematic_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark executables for installation
## This ensures the inverse_kinematic_controller executable gets installed properly
install(TARGETS inverse_kinematic_controller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinematic_controller.cpp)
# target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})