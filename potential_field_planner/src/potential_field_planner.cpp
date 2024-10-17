#include "potential_field_planner.hpp"
#include <std_msgs/Float64MultiArray.h>

// Constructor: Initializes the ROS node and sets up parameters, publishers, and subscribers
PotentialFieldPlanner::PotentialFieldPlanner(ros::NodeHandle& nh)
    : received_joint_state_(false)  // Initialize the feedback flag to false
{
    // Read parameters from the ROS parameter server
    readParameters(nh);

    // Subscribe to the joint states topic to receive feedback on the robot's joint positions
    joint_state_sub_ = nh.subscribe("/gen3/joint_states", 10, &PotentialFieldPlanner::jointStateCallback, this);

    // Publisher for the joint velocities, using Float64MultiArray to send the velocity commands
    velocity_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/gen3/reference/velocity", 10);
}

// Function to read parameters from the ROS parameter server
void PotentialFieldPlanner::readParameters(ros::NodeHandle& nh) {
    // Load scalar values for k_att and max_velocity
    nh.getParam("/gen3/linear/k_att", k_att_);
    nh.getParam("/gen3/linear/max_velocity", max_velocity_);

    // Load the default joint positions and max velocity as Eigen vectors directly
    std::vector<double> default_positions, max_velocity_vec;

    nh.getParam("/gen3/joint/default", default_positions);
    nh.getParam("/gen3/joint/max_velocity", max_velocity_vec);

    // Convert std::vector to Eigen::VectorXd
    default_joint_positions_ = Eigen::Map<Eigen::VectorXd>(default_positions.data(), default_positions.size());
    max_velocity_vector_ = Eigen::Map<Eigen::VectorXd>(max_velocity_vec.data(), max_velocity_vec.size());
}

// Callback function for receiving joint state feedback
void PotentialFieldPlanner::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Resize the Eigen vector to match the size of the incoming message
    current_positions_.resize(msg->position.size());

    // Copy the joint positions from the message into the Eigen vector
    for (size_t i = 0; i < msg->position.size(); ++i) {
        current_positions_[i] = msg->position[i];
    }

    // Log the size of current_positions_ for debugging
    ROS_INFO_STREAM("Received joint states with " << current_positions_.size() << " positions.");

    // Mark that the joint state has been received
    received_joint_state_ = true;
}

// Function to compute joint velocities based on the potential field forces
void PotentialFieldPlanner::computeJointVelocities() {
//    if (!received_joint_state_) {
//        ROS_WARN("Joint state feedback not received yet. Skipping velocity computation.");
//        return;
//    }
//
//    // Check that the size of the current_positions_ vector matches the default_joint_positions_
//    if (current_positions_.size() != default_joint_positions_.size()) {
//        ROS_ERROR("Size mismatch between current and default joint positions. Skipping velocity computation.");
//        return;
//    }

    // Compute the error between current and target positions
    Eigen::VectorXd error = default_joint_positions_ - current_positions_;

    // Compute the joint velocities based on the attractive potential
    Eigen::VectorXd joint_velocities = k_att_ * error;

    // Normalize the joint velocities if they exceed the maximum allowed velocity
    for (int i = 0; i < joint_velocities.size(); ++i) {
        if (std::abs(joint_velocities[i]) > max_velocity_) {
            joint_velocities[i] = std::copysign(max_velocity_, joint_velocities[i]);
        }
    }

    // Publish the computed joint velocities
    publishJointVelocities(joint_velocities);
}


// Helper function to publish joint velocities
void PotentialFieldPlanner::publishJointVelocities(const Eigen::VectorXd& joint_velocities) {
    // Create a message to hold the computed velocities
    std_msgs::Float64MultiArray joint_velocity_msg;

    // Assign the computed velocities to the message data
    joint_velocity_msg.data.assign(joint_velocities.data(), joint_velocities.data() + joint_velocities.size());

    // Publish the computed joint velocities to the appropriate topic
    velocity_pub_.publish(joint_velocity_msg);
}

// Main function to run the node
int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "potential_field_planner");
    ros::NodeHandle nh;

    // Instantiate the PotentialFieldPlanner class
    PotentialFieldPlanner planner_node(nh);

    // Set the loop rate based on the ROS parameter
    int publish_rate;
    nh.getParam("/publish_rate", publish_rate);
    ros::Rate loop_rate(publish_rate);

    // Main loop
    while (ros::ok()) {
        // Spin once to process incoming messages
        ros::spinOnce();

        // Compute and publish joint velocities
        planner_node.computeJointVelocities();

        // Sleep for the remainder of the loop
        loop_rate.sleep();
    }

    return 0;
}
