#ifndef POTENTIAL_FIELD_PLANNER_HPP
#define POTENTIAL_FIELD_PLANNER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>  // Eigen library for vector and matrix operations

class PotentialFieldPlanner {
public:
    PotentialFieldPlanner(ros::NodeHandle& nh);  // Constructor

    // Function to compute joint velocities based on potential field forces
    void computeJointVelocities();

private:
    void readParameters(ros::NodeHandle& nh);  // Function to read all ROS parameters
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);  // Callback for joint state feedback
    void publishJointVelocities(const Eigen::VectorXd& joint_velocities);  // Helper to publish velocities

    // ROS subscriber and publisher
    ros::Subscriber joint_state_sub_;
    ros::Publisher velocity_pub_;

    // Parameters for potential field computation
    double k_att_;  // Scalar value for attractive potential constant
    double max_velocity_;  // Scalar value for max velocity per joint
    Eigen::VectorXd default_joint_positions_;  // Vector for default joint positions
    Eigen::VectorXd max_velocity_vector_;  // Vector for max velocity per joint

    // Current joint state
    Eigen::VectorXd current_positions_;
    bool received_joint_state_;  // Flag to check if feedback has been received
};

#endif  // POTENTIAL_FIELD_PLANNER_HPP
