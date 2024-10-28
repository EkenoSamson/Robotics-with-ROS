#include "pose_controller.hpp"

// Constructor
TaskSpaceDyn::TaskSpaceDyn(ros::NodeHandle& nh) : nh_(nh) {
  // Read Parameters
  if (!readParameters()) {
    ROS_ERROR("Failed to read URDF files.");
    ros::requestShutdown();
  }

  // Build the model
  init(urdf_file_name_);

  // Set up publishers using the parameter-loaded topic
  joint_effort_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(joint_effort_command_topic_, 10);
  end_effector_pose_pub_ = nh_.advertise<geometry_msgs::Pose>(feedback_pose_topic_, 10);
  end_effector_twist_pub_ = nh_.advertise<geometry_msgs::Twist>(feedback_twist_topic_, 10);

  // Set up subscribers using the parameter-loaded topics
  joint_states_sub_ = nh_.subscribe(joint_states_topic_, 10, &TaskSpaceDyn::jointStatesCallback, this);
  reference_position_sub_ = nh_.subscribe(reference_position_topic_, 10, &TaskSpaceDyn::referencePositionCallback, this);
  reference_velocity_sub_ = nh_.subscribe(reference_velocity_topic_, 10, &TaskSpaceDyn::referenceVelocityCallback, this);
  reference_pose_sub_ = nh_.subscribe(reference_pose_topic_, 10, &TaskSpaceDyn::referencePoseCallback, this);
  reference_twist_sub_ = nh_.subscribe(reference_twist_topic_, 10, &TaskSpaceDyn::referenceTwistCallback, this);

  // Set up services
  set_linear_pd_service_ = nh_.advertiseService("/pose_controller/set_linear_pd", &TaskSpaceDyn::setLinearPDServiceCallback, this);
  set_joint_pd_service_ = nh_.advertiseService("/pose_controller/set_joint_pd", &TaskSpaceDyn::setJointPDServiceCallback, this);
}

// Read the parameter
bool TaskSpaceDyn::readParameters() {
  // Load URDF file
  if (!nh_.getParam("/gen3/urdf_file_name", urdf_file_name_)) {
    ROS_ERROR("Could not read urdf file name!");
    return false;
  }

  // Load the publish rate
  if (!nh_.getParam("/publish_rate", publish_rate_)) {
    ROS_ERROR("Could not read publish_rate!");
    publish_rate_ = 500;
  }

  // Load redundancy parameter
  if (!nh_.getParam("controller/with_redundancy", with_redundancy_)) {
    ROS_ERROR("Could not read redundancy flag!");
    with_redundancy_ = false;
  }

  // Load end-effector stiffness
  if (!nh_.getParam("/K_E", end_stiffness_)) {
    ROS_ERROR("Could not read end-effector stiffness!");
    end_stiffness_ = 0.1;
  }

  // Load end-effector damping
  if (!nh_.getParam("/D_E", end_damping_)) {
    ROS_ERROR("Could not read end-effector damping!");
    end_damping_ = 0.1;
  }

  // Load joints stiffness
  if (!nh_.getParam("/K_J", joints_stiffness_)) {
    ROS_ERROR("Could not read joints stiffness!");
    joints_stiffness_ = 0.1;
  }

  // Load joints damping
  if (!nh_.getParam("/D_J", joints_damping_)) {
    ROS_ERROR("Could not read joints damping!");
    joints_damping_ = 0.1;
  }

  // Load subscibers topics
  if (!nh_.getParam("/joint_states_topic", joint_states_topic_)) {
    ROS_ERROR("Could get the joint states topic!");
    return false;
  }
  if (!nh_.getParam("/reference_position_topic", reference_position_topic_)) {
    ROS_ERROR("Could get the reference position topic!");
    return false;
  }
  if (!nh_.getParam("/reference_velocity_topic", reference_velocity_topic_)) {
    ROS_ERROR("Could get the reference velocity topic!");
    return false;
  }
  if (!nh_.getParam("/reference_pose_topic", reference_pose_topic_)) {
    ROS_ERROR("Could get the reference pose topic!");
    return false;
  }
  if (!nh_.getParam("/reference_twist_topic", reference_twist_topic_)) {
    ROS_ERROR("Could get the reference twist topic!");
    return false;
  }

  // Load joint torque publisher topic
  if (!nh_.getParam("/joint_group_effort_controller_command", joint_effort_command_topic_)) {
    ROS_ERROR("Could get the reference position topic!");
    return false;
  }

  // Load end effector feedback pose topic
  if (!nh_.getParam("/feedback_pose_topic", feedback_pose_topic_)) {
    ROS_ERROR("Could get the reference pose topic!");
    return false;
  }

  // Load end effector feedback twist topic
  if (!nh_.getParam("/feedback_twist_topic", feedback_twist_topic_)) {
    ROS_ERROR("Could get the reference twist topic!");
    return false;
  }

  return true;
}

// Build the model
void TaskSpaceDyn::init(std::string urdf_file_name) {
  // Build the model
  pinocchio::urdf::buildModel(urdf_file_name_, model_, false);

  // data structure
  data_ = pinocchio::Data(model_);

  // the end_effector ID
  hand_id_ = model_.getJointId("bracelet_link") - 1;

  // Numnber of configuration variables (DoF)
  num_joints_ = model_.nq;

  // Initialise the joints' variables
  joint_positions_.setZero();
  joint_velocities_.setZero();
  reference_positions_.setZero();
  reference_velocities_.setZero();

  // Initialise the end-effector's variables
  end_effector_pose_.setZero();
  end_effector_twist_.setZero();
  reference_pose_.setZero();
  reference_twist_.setZero();

  // Jacobians
  jacobian_ = Eigen::MatrixXd::Zero(6, num_joints_);
  jacobian_dot_ = Eigen::MatrixXd::Zero(6, num_joints_);
  jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(6, num_joints_);

  // Mass Matrix and Coriolis, Centrifugal and gravity
  M_ = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
  h_ = Eigen::VectorXd::Zero(num_joints_);
}

// Callbacks
// Receive the joint state (current positions and velocities)
void TaskSpaceDyn::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Process joint state feedback
  joint_positions_ = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
  joint_velocities_ = Eigen::VectorXd::Map(msg->velocity.data(), msg->velocity.size());
}

// Receive the reference positions of the joints
void TaskSpaceDyn::referencePositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Process reference joints' positions
  reference_positions_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
}

// Receive the reference velocities for the joints
void TaskSpaceDyn::referenceVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Process reference joints' velocities
  reference_velocities_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());
}

// Receive the reference pose of the end_effector
void TaskSpaceDyn::referencePoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  // Process reference end-effector pose
  reference_pose_(0) = msg->position.x;
  reference_pose_(1) = msg->position.y;
  reference_pose_(2) = msg->position.z;
}

// Receive the reference twist of the end_effector
void TaskSpaceDyn::referenceTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Process reference end-effector twist
  reference_twist_(0) = msg->linear.x;
  reference_twist_(1) = msg->linear.y;
  reference_twist_(2) = msg->linear.z;
}

// Setting the Linear K and D
bool TaskSpaceDyn::setLinearPDServiceCallback(highlevel_msgs::SetKD::Request &req, highlevel_msgs::SetKD::Response &res) {
  // Tune the K and D
  end_stiffness_ = req.k;
  end_damping_ = req.d;
  return true;
}

// Setting the Joint K and D
bool TaskSpaceDyn::setJointPDServiceCallback(highlevel_msgs::SetKD::Request &req, highlevel_msgs::SetKD::Response &res) {
  // Tune the joint K and D
  joints_stiffness_ = req.k;
  joints_damping_ = req.d;
  return true;
}

// compute task space dynamics
void TaskSpaceDyn::computeDynamics() {
  // Compute general dynamics
  pinocchio::computeAllTerms(model_, data_, joint_positions_, joint_velocities_);
  M_ = data_.M;  // Mass Matrix
  h_ = data_.nle;  // Nonlinear Effector Vector

  // Get the Jacobian and its time derivative
  pinocchio::getJointJacobian(model_, data_, hand_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);
  pinocchio::getJointJacobianTimeVariation(model_, data_, hand_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot_);

  // End-effector pose and twist
  end_effector_pose_ = data_.oMi[hand_id_].translation();
  end_effector_twist_ = jacobian_.topRows(3) * joint_velocities_;

  pubEndFeedback();

  // Log Jacobian dimensions
  ROS_INFO_STREAM("Jacobian dimensions: " << jacobian_.rows() << " x " << jacobian_.cols());

  // Compute the pseudo-inverse of the Jacobian matrix
  ROS_INFO("Computing Jacobian pseudo-inverse");
  ROS_INFO_STREAM("Jacobian dimensions for pseudo-inverse calculation: " << jacobian_.rows() << " x " << jacobian_.cols());
  ROS_INFO_STREAM("M_ dimensions: " << M_.rows() << " x " << M_.cols());

  jacobian_pseudo_inverse_ = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse();
  ROS_INFO("Jacobian pseudo-inverse computed successfully");

  // Compute task-space mass matrix (Lambda)
  ROS_INFO("Computing Lambda (task-space mass matrix)");
  Eigen::MatrixXd lambda_ = jacobian_pseudo_inverse_.transpose() * M_ * jacobian_pseudo_inverse_;
  ROS_INFO_STREAM("Lambda dimensions: " << lambda_.rows() << " x " << lambda_.cols());

  // Compute task-space bias forces (eta)
  ROS_INFO("Computing eta (task-space bias forces)");
  ROS_INFO_STREAM("Jacobian_dot dimensions: " << jacobian_dot_.rows() << " x " << jacobian_dot_.cols());
  Eigen::VectorXd eta_ = jacobian_pseudo_inverse_.transpose() * h_ - lambda_ * jacobian_dot_ * joint_velocities_;
  ROS_INFO_STREAM("Eta dimensions: " << eta_.rows() << " x " << eta_.cols());

  // Compute desired task-space acceleration
  ROS_INFO("Computing task-space acceleration command");
  Eigen::VectorXd ee_ref_acc_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd twist_error_ = reference_twist_ - end_effector_twist_;
  Eigen::VectorXd pose_error_ = reference_pose_ - end_effector_pose_;
  Eigen::VectorXd ee_acc_cmd_ = ee_ref_acc_ + (end_stiffness_ * pose_error_) + (end_damping_ * twist_error_);
  ROS_INFO_STREAM("ee_acc_cmd_ dimensions: " << ee_acc_cmd_.rows() << " x " << ee_acc_cmd_.cols());

  // Compute the task-space wrench
  ROS_INFO("Computing wrench");
  ROS_INFO_STREAM("Lambda dimensions: " << lambda_.rows() << " x " << lambda_.cols());
  Eigen::VectorXd wrench_ = lambda_.topRows(3).transpose() * ee_acc_cmd_ + eta_;
  ROS_INFO_STREAM("Wrench dimensions: " << wrench_.rows() << " x " << wrench_.cols());

  // Compute the joint torques
  ROS_INFO("Computing joint torques");
  Eigen::VectorXd tau_ = jacobian_.transpose() * wrench_;
  ROS_INFO_STREAM("Tau dimensions: " << tau_.rows() << " x " << tau_.cols());

  // Populate Message
  joint_torque_msg_.data.resize(num_joints_);
  for (int i = 0; i < num_joints_; i++) {
    joint_torque_msg_.data[i] = tau_(i);
  }

  // Publish the joint torques
  joint_effort_pub_.publish(joint_torque_msg_);
}


//
void TaskSpaceDyn::pubEndFeedback() {
  // Publish the end-effector pose as a ROS message
  geometry_msgs::Pose ee_pose;
  ee_pose.position.x = end_effector_pose_(0);  // X-coordinate of end-effector
  ee_pose.position.y = end_effector_pose_(1);  // Y-coordinate of end-effector
  ee_pose.position.z = end_effector_pose_(2);  // Z-coordinate of end-effector
  ee_pose.orientation.x = 0;
  ee_pose.orientation.y = 0;
  ee_pose.orientation.z = 0;
  ee_pose.orientation.w = 1;

  end_effector_pose_pub_.publish(ee_pose);  // Publish pose


  // Publish the end-effector twist as a geometry_msgs::Twist message
  geometry_msgs::Twist ee_twist;
  ee_twist.linear.x = end_effector_twist_(0);  // Linear velocity in X direction
  ee_twist.linear.y = end_effector_twist_(1);  // Linear velocity in Y direction
  ee_twist.linear.z = end_effector_twist_(2);  // Linear velocity in Z direction
  ee_twist.angular.x = 0;
  ee_twist.angular.y = 0;
  ee_twist.angular.z = 0;

  end_effector_twist_pub_.publish(ee_twist);  // Publish twist
}