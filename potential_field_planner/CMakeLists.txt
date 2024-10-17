cmake_minimum_required(VERSION 3.0.2)
project(potential_field_planner)

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


# Declare catkin package
catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN_INCLUDE_DIRS}
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
)

# Add executable
add_executable(joint_planner src/potential_field_planner.cpp)

# Link the executable with necessary libraries
target_link_libraries(joint_planner
        ${catkin_LIBRARIES}
)

# Specify install targets
install(TARGETS joint_planner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
