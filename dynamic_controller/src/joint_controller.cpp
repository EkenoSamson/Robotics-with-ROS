#include "joint_controller.hpp"

/*
 * JointSpaceDyn - class for inverse dynamics
 * Description: Perform joint space inverse dynamics
 * Author: Ekeno
 ***********************************************
 */

// constructor
JointSpaceDyn::JointSpaceDyn(ros::NodeHandle& nh) : nh_(nh) {
  // Initialise variables
  publish_rate_ = 500;


  // Read Parameters
  if (!readParameters() ) {
    ROS_ERROR("[JointSpaceDyn] Could not read parameters. Shut down");
    ros::requestShutdown();
  }
  // Initiate the model
  init(urdf_file_name_);

  // Set up publishers using the parameter-loaded topic
  joint_effort_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(joint_effort_command_topic_, 10);

  // Set up subscribers using the parameter-loaded topics
  joint_states_sub_ = nh_.subscribe(joint_states_topic_, 10, &JointSpaceDyn::jointStatesCallback, this);
  reference_position_sub_ = nh_.subscribe(reference_position_topic_, 10, &JointSpaceDyn::referencePositionCallback, this);
  reference_velocity_sub_ = nh_.subscribe(reference_velocity_topic_, 10, &JointSpaceDyn::referenceVelocityCallback, this);

  // Debug:
  ROS_INFO("Subscribing to %s", joint_states_topic_.c_str());
  ROS_INFO("Publishing to %s", reference_position_topic_.c_str());
  ROS_INFO("Publishing to %s", reference_velocity_topic_.c_str());

  // Service
  set_pd_service_ = nh_.advertiseService("/joint_controller/set_joint_pd", &JointSpaceDyn::setPDServiceCallback, this);

  // Debug
  ROS_INFO("Joint controller object built");
}

// Reading Parameters
bool JointSpaceDyn::readParameters() {
  // Read stiffness and damping from parameter server
  nh_.param("/joint_stiffness", stiffness_, 0.1);
  nh_.param("/joint_damping", damping_, 0.1);

  // Read the URDF file and publish rate if needed
  if ( !nh_.getParam("/publish_rate", publish_rate_) ) {
    ROS_ERROR("[controller_node] cannot read publish_rate") ;
    return false ;
  }

  if (!nh_.getParam("/gen3/urdf_file_name", urdf_file_name_) ) {
    ROS_ERROR_STREAM("cannot read /gen3/urdf_file_name") ;
    return false ;
  }

  // Get parameters for topic names
  if (!nh_.getParam("/topic_names/fbk_joint_state", joint_states_topic_)) {
    ROS_ERROR("cannot read topic names");
    return false;
  }

  if (!nh_.getParam("/topic_names/ref_joint_pos", reference_position_topic_)) {
    ROS_ERROR("cannot read reference position topic names");
    return false;
  }

  if (!nh_.getParam("/topic_names/ref_joint_vel", reference_velocity_topic_)) {
    ROS_ERROR("cannot read reference velocity topic names");
    return false;
  }

  if (!nh_.getParam("/topic_names/cmd_joint_tau", joint_effort_command_topic_)) {
    ROS_ERROR("cannot read joint effort command topic");
    return false;
  }

  return true ;
}

// Initiate the model
void JointSpaceDyn::init(std::string urdf_file_name_) {
  // Build the Pinocchio model from the URDF file
  pinocchio::urdf::buildModel(urdf_file_name_, model_, false);

  // Initialize the data structure for Pinocchio
  data_ = pinocchio::Data(model_);

  // Get the joint ID for the end-effector (bracelet_link in this case)
  hand_id_ = model_.getJointId("bracelet_link") - 1;

  // Set the number of joints (degrees of freedom)
  dim_joints_ = model_.nq;

  joint_positions_ = Eigen::VectorXd::Zero(dim_joints_) ;
  joint_velocities_ = Eigen::VectorXd::Zero(dim_joints_) ;
  reference_positions_ = Eigen::VectorXd::Zero(dim_joints_);
  reference_velocities_ = Eigen::VectorXd::Zero(dim_joints_);
  jacobian_ = Eigen::MatrixXd::Zero(6, dim_joints_) ;
  jacobian_dot_ = Eigen::MatrixXd::Zero(6, dim_joints_) ;
  M_ = Eigen::MatrixXd::Identity(dim_joints_,dim_joints_) ;
  h_ = Eigen::VectorXd::Zero(dim_joints_) ;

}

void JointSpaceDyn::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Process joint state feedback
  joint_positions_ = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
  joint_velocities_ = Eigen::VectorXd::Map(msg->velocity.data(), msg->velocity.size());
}

void JointSpaceDyn::referencePositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Process reference joint position
  reference_positions_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
}

void JointSpaceDyn::referenceVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Process reference joint velocity
  reference_velocities_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
}

// Service
bool JointSpaceDyn::setPDServiceCallback(highlevel_msgs::SetKD::Request &req, highlevel_msgs::SetKD::Response &res) {
  // Receive the user stiffness_ and damping_
  stiffness_ = req.k;
  damping_ = req.d;
  return true ;
}

// Compute Joint Space Inverse Dynamic Controller
void JointSpaceDyn::computeDynamics() {
  // compute general terms
  pinocchio::computeAllTerms(model_, data_, joint_positions_, joint_velocities_);
  M_ = data_.M ;       	// mass matrix
  h_ = data_.nle ;	    // coriolis, centrifugal, and gravity

  // Desired accelleration
  Eigen::VectorXd reference_accel_ = Eigen::VectorXd::Zero(dim_joints_);       // Set to Zero
  Eigen::VectorXd accel_command_ = Eigen::VectorXd::Zero(dim_joints_);

  // velocity error and position error
  Eigen::VectorXd veloc_error_ = reference_velocities_ - joint_velocities_;
  Eigen::VectorXd pos_error_ = reference_positions_ - joint_positions_;

  // Calculate acceleration command based on errors
  accel_command_ = reference_accel_ + damping_ * veloc_error_ + stiffness_ * pos_error_ ;

  // Joint space inverse dynamics controller
  Eigen::VectorXd joint_torque_ = M_ * accel_command_ + h_;

  // Message
  joint_torque_msg_.data.resize(dim_joints_);
  for (int i = 0; i < dim_joints_; i++) {
    joint_torque_msg_.data[i] = joint_torque_(i) ;
  }

  joint_effort_pub_.publish(joint_torque_msg_);
}

