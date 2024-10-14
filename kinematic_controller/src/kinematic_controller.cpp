/**
* kinematic_controller.cpp
* created by Ekeno
*/
#include "kinematic_controller.hpp"

// Constructor for the KIN class
KIN::KIN(ros::NodeHandle& nh) : nh_(nh), model_loaded_(false)
{
    // Initialize Eigen vectors and matrices to zero
    task_space_vel_ = Eigen::VectorXd::Zero(3);           // 3D vector for end-effector velocity
    task_space_pos_ = Eigen::VectorXd::Zero(3);           // 3D vector for end-effector position
    joint_positions_ = Eigen::VectorXd::Zero(num_joints_);// Vector for joint positions (size: num_joints_)
    joint_velocities_ = Eigen::VectorXd::Zero(num_joints_);// Vector for joint velocities (size: num_joints_)
    jacobian_ = Eigen::MatrixXd::Zero(6, num_joints_);    // 6xN Jacobian matrix (6 for spatial velocity, N = num_joints_)

    // Attempt to read ROS parameters (publish rate, URDF file)
    if (!readParameters()) {
        ROS_ERROR("Failed to read parameters");
    }

    // Set up ROS publishers and subscribers
    joint_states_sub_ = nh_.subscribe("/gen3/joint_states", 1, &KIN::jointStatesCallback, this); // Subscribe to joint states
    end_effector_pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/gen3/feedback/pose", 1);        // Publish end-effector pose
    end_effector_twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/gen3/feedback/twist", 1);     // Publish end-effector twist (velocity)
}

// Method to initialize the Pinocchio model by loading the URDF file
bool KIN::init()
{
    try {
        // Load the URDF file and build the Pinocchio model
        pinocchio::urdf::buildModel(urdf_file_name, model_, false);
        // Initialize the data structure for kinematic computations
        data_ = pinocchio::Data(model_);
        // Get the ID of the end-effector joint (e.g., "bracelet_link")
        ee_id_ = model_.getJointId("bracelet_link") - 1;
        // Mark that the model has been successfully loaded
        model_loaded_ = true;
        ROS_INFO("Pinocchio model successfully loaded.");
    }
    catch (const std::exception& e) {
        // Handle errors during URDF loading and log an error message
        ROS_ERROR("Failed to load URDF file: %s", e.what());
        model_loaded_ = false;
        return false;
    }
    return true;
}

// Main update method called in the control loop to compute forward kinematics and publish results
void KIN::update()
{
    // Check if the model was loaded successfully
    if (!model_loaded_) {
        ROS_WARN("Model not loaded. Cannot compute kinematics.");
        return;
    }

    // Compute forward kinematics (joint positions to task space position)
    pinocchio::forwardKinematics(model_, data_, joint_positions_);
    pinocchio::updateGlobalPlacements(model_, data_);  // Update global placements of links

    // Compute the Jacobian at the end-effector
    pinocchio::computeJointJacobians(model_, data_);  // Compute joint Jacobians for the whole robot
    pinocchio::getJointJacobian(model_, data_, ee_id_, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_); // Get Jacobian for end-effector

    // Compute task-space position (translation of the end-effector)
    task_space_pos_ = data_.oMi[ee_id_].translation();

    // Publish the end-effector pose as a ROS message
    geometry_msgs::Pose ee_pose;
    ee_pose.position.x = task_space_pos_(0);  // X-coordinate of end-effector
    ee_pose.position.y = task_space_pos_(1);  // Y-coordinate of end-effector
    ee_pose.position.z = task_space_pos_(2);  // Z-coordinate of end-effector
    end_effector_pose_pub_.publish(ee_pose);  // Publish pose

    // Compute task-space velocity (twist of the end-effector) using the Jacobian and joint velocities
    task_space_vel_ = jacobian_.topRows(3) * joint_velocities_; // Linear velocity part of the twist

    // Publish the end-effector twist as a ROS message
    geometry_msgs::Twist ee_twist;
    ee_twist.linear.x = task_space_vel_(0);  // Linear velocity in X direction
    ee_twist.linear.y = task_space_vel_(1);  // Linear velocity in Y direction
    ee_twist.linear.z = task_space_vel_(2);  // Linear velocity in Z direction
    end_effector_twist_pub_.publish(ee_twist);  // Publish twist
}

// Callback method that receives joint state feedback (joint positions and velocities)
void KIN::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Map the received joint positions and velocities into Eigen vectors for internal use
    joint_positions_ = Eigen::VectorXd::Map(msg->position.data(), num_joints_);
    joint_velocities_ = Eigen::VectorXd::Map(msg->velocity.data(), num_joints_);
}

// Helper method to read parameters from the ROS parameter server
bool KIN::readParameters()
{
    // Retrieve the URDF file name from the parameter server
    if (!nh_.getParam("/gen3/urdf_file_name", urdf_file_name)) {
        ROS_ERROR("URDF file name not found.");
        return false;
    }

    // Retrieve the publish rate from the parameter server
    if (!nh_.getParam("/publish_rate", publish_rate_)) {
        ROS_WARN("Publish rate not found. Using default value: 500.");
        publish_rate_ = 500;  // Default publish rate if not provided
    }

    return true;
}

// Main entry point for the ROS node
int main(int argc, char **argv)
{
    // Initialize the ROS node with the name "inverse_kinematic_controller"
    ros::init(argc, argv, "inverse_kinematic_controller");
    ros::NodeHandle nh("~");  // Private node handle for parameters

    // Create the kinematic controller object
    KIN kin_controller(nh);

    // Initialize the kinematic controller by loading the Pinocchio model
    if (!kin_controller.init()) {
        ROS_ERROR("Failed to initialize KIN controller");
        return -1;
    }

    // Set the loop rate for the control loop (based on the publish rate)
    ros::Rate loop_rate(kin_controller.publish_rate_);

    // Main control loop
    while (ros::ok())
    {
        // Perform kinematic updates and publish results
        kin_controller.update();

        // Process any ROS callbacks (e.g., joint state updates)
        ros::spinOnce();

        // Sleep to maintain the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}
