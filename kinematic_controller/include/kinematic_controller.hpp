// kinematic_controller.hpp
// created by Ekeno

#ifndef KINEMATIC_CONTROLLER_HPP
#define KINEMATIC_CONTROLLER_HPP

// Include necessary Pinocchio libraries for kinematics
#include <pinocchio/multibody/model.hpp>       // Model representation in Pinocchio
#include <pinocchio/multibody/data.hpp>        // Data storage for kinematics calculations
#include <pinocchio/parsers/urdf.hpp>          // URDF parser for loading robot models
#include <pinocchio/algorithm/kinematics.hpp>  // Algorithms for kinematics
#include <pinocchio/algorithm/jacobian.hpp>    // Algorithms for Jacobian computation

// Include ROS libraries
#include <ros/ros.h>                           // ROS core
#include <geometry_msgs/Pose.h>                // Message for publishing the pose
#include <geometry_msgs/Twist.h>               // Message for publishing the twist
#include <sensor_msgs/JointState.h>            // Message for subscribing to joint states
#include <std_msgs/Float64MultiArray.h>        // Message for publishing joint positions
#include <string>                              // Standard string library
#include <Eigen/Dense>                         // Eigen library for matrix/vector operations


class KIN
{
public:
    // Constructor: initializes the ROS node handle and sets up ROS communication
    KIN(ros::NodeHandle& nh);

    // Destructor: currently empty, but can be used for cleanup if needed in the future
    ~KIN() {}

    // Initializes the kinematic controller by loading the URDF file into Pinocchio
    bool init();

    // Main update function that computes forward kinematics and publishes the results
    void update();

    // Public parameters to store the publish rate and URDF file name
    int publish_rate_;                      // Publish rate for ROS loop
    std::string urdf_file_name;             // URDF file path used to load the model

    // Add flags
    bool joint_states_received_ = false;  // Flag for joint states
    bool reference_pose_received_ = false;  // Flag for reference pose

private:
    // Helper function to read ROS parameters like URDF file and publish rate from parameter server
    bool readParameters();

    // Callback function to receive joint states (position and velocity) from the robot
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

    // Callback function to receive reference pose for task space planning
    void referencePoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

    // Method to compute inverse kinematics using the Jacobian
    void computeInverseKinematics();
    void computeForwardKinematics();

    // ROS communication members
    ros::NodeHandle nh_;                    // Node handle for interacting with ROS
    ros::Subscriber joint_states_sub_;      // Subscriber for joint states
    ros::Subscriber reference_pose_sub_;    // Subscriber for reference pose
    ros::Publisher joint_command_pub_;      // Publisher for joint command velocities
    ros::Publisher end_effector_pose_pub_;  // Publisher for end-effector pose
    ros::Publisher end_effector_twist_pub_; // Publisher for end-effector twist (velocity)

    // Kinematic variables
    pinocchio::Model model_;                // Pinocchio model of the robot
    pinocchio::Data data_;                  // Data structure for storing kinematic computations
    Eigen::MatrixXd jacobian_;              // Matrix to store the computed Jacobian
    Eigen::VectorXd task_space_pos_;        // End-effector position in task space
    Eigen::VectorXd task_space_vel_;        // End-effector velocity in task space
    Eigen::VectorXd joint_positions_;       // Joint positions vector
    Eigen::VectorXd joint_velocities_;      // Joint velocities vector
    Eigen::VectorXd reference_pos_;         // Reference position from task space planner

    // Internal variables for handling kinematics
    int num_joints_ = 7;                    // Number of joints in the manipulator (e.g., 7 for Kinova Gen3)
    int ee_id_;                             // Index of the end-effector joint (ID for computing Jacobian)
    bool model_loaded_;                     // Flag to check if the URDF model was successfully loaded

    // Parameters for inverse kinematics
    double k_att_;                          // Control gain for linear motion
    double max_velocity_;                   // Maximum allowable velocity

    // Joint position command message
    std_msgs::Float64MultiArray joint_position_msg_; // Message to publish joint positions

    // Subscriber and Publisher topics
    std::string joint_states_topic_;                // joint states topic
    std::string feedback_pose_topic_;               // feedback pose topic
    std::string feedback_twist_topic_;              // feedback twist topic

};

#endif // KINEMATIC_CONTROLLER_HPP
