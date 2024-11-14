#include "pose_controller.hpp"

/*
 * TaskSpaceDyn - class for inverse dynamics
 * Description: Perform task space inverse dynamics with redundancy
 * Author: Ekeno
 ***********************************************
 */

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
    return false;
  }

  // Load redundancy parameter
  if (!nh_.getParam("/controller/with_redundancy", with_redundancy_)) {
    ROS_ERROR("Could not read redundancy flag!");
    return false;
  }

  // Load orientation controll parameter
  if (!nh_.getParam("/controller/with_orientation", with_orientation_)) {
    ROS_ERROR("Could not read orientation flag!");
    return false;
  }

  // Load end-effector stiffness
  if (!nh_.getParam("/end_effector_stiffness", end_stiffness_)) {
    ROS_ERROR("Could not read end-effector stiffness!");
    return false;
  }

  // Load end-effector damping
  if (!nh_.getParam("/end_effector_damping", end_damping_)) {
    ROS_ERROR("Could not read end-effector damping!");
    return false;
  }

  // Load joints stiffness
  if (!nh_.getParam("/joint_stiffness", joints_stiffness_)) {
    ROS_ERROR("Could not read joints stiffness!");
    return false;
  }

  // Load joints damping
  if (!nh_.getParam("/joint_damping", joints_damping_)) {
    ROS_ERROR("Could not read joints damping!");
    return false;
  }

  // Load subscibers topics
  if (!nh_.getParam("/topic_names/fbk_joint_state", joint_states_topic_)) {
    ROS_ERROR("Could get the joint states topic!");
    return false;
  }
  if (!nh_.getParam("/topic_names/ref_joint_pos", reference_position_topic_)) {
    ROS_ERROR("Could get the reference position topic!");
    return false;
  }
  if (!nh_.getParam("/topic_names/ref_joint_vel", reference_velocity_topic_)) {
    ROS_ERROR("Could get the reference velocity topic!");
    return false;
  }
  if (!nh_.getParam("/topic_names/ref_hand_pose", reference_pose_topic_)) {
    ROS_ERROR("Could get the reference pose topic!");
    return false;
  }
  if (!nh_.getParam("/topic_names/ref_hand_twist", reference_twist_topic_)) {
    ROS_ERROR("Could get the reference twist topic!");
    return false;
  }

  // Load joint torque publisher topic
  if (!nh_.getParam("/topic_names/cmd_joint_tau", joint_effort_command_topic_)) {
    ROS_ERROR("Could get the reference position topic!");
    return false;
  }

  // Load end effector feedback pose topic
  if (!nh_.getParam("/topic_names/fbk_hand_pose", feedback_pose_topic_)) {
    ROS_ERROR("Could get the reference pose topic!");
    return false;
  }

  // Load end effector feedback twist topic
  if (!nh_.getParam("/topic_names/fbk_hand_twist", feedback_twist_topic_)) {
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
  jts_fbk_positions_.setZero();
  jts_fbk_velocities_.setZero();
  joint_acceleration_.setZero();
  jts_ref_positions_.setZero();
  jts_ref_velocities_.setZero();

  // Initialise the end-effector's variables
  ee_fbk_position_.setZero();
  ee_fbk_orientation_.setIdentity();
  ee_fbk_twist_.setZero();

  ee_ref_position_.setZero();
  ee_ref_orientation_.setIdentity();
  ee_ref_twist_.setZero();

  ee_acc_cmd_.setZero();
  ee_ref_acc_.setZero();

  twist_error_.setZero();
  twist_error_trans_.setZero();
  pose_error_.setZero();
  pose_error_trans_.setZero();
  position_error_.setZero();
  rotation_error_.setZero();
  orientation_error_.setIdentity();


  // Jacobians
  jacobian_.setZero();
  jacobian_dot_.setZero();
  jacobian_pseudo_inverse_.setZero();
  jacobian_trans_.setZero();
  jacobian_dot_trans_.setZero();
  jacobian_pseudo_inverse_trans_.setZero();


  // Mass Matrix and Coriolis, Centrifugal and gravity
  M_.setZero();
  h_.setZero();
  lambda_.setZero();
  lambda_trans_.setZero();
  eta_.setZero();
  eta_trans_.setZero();
  wrench_.setZero();
  wrench_trans_.setZero();
  tau_task_.setZero();
  tau_joint_.setZero();
  tau_total_.setZero();

  // Null Space
  P_.setZero();
  I_.setIdentity();

  // Message
  joint_torque_msg_.data.resize(num_joints_);
}

// Callbacks
// Receive the joint state (current positions and velocities)
void TaskSpaceDyn::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  // Process joint state feedback
  jts_fbk_positions_ = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
  jts_fbk_velocities_ = Eigen::VectorXd::Map(msg->velocity.data(), msg->velocity.size());

  //ROS_INFO(" Joint states received");
}

// Receive the reference positions of the joints
void TaskSpaceDyn::referencePositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Process reference joints' positions
  jts_ref_positions_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());

}

// Receive the reference velocities for the joints
void TaskSpaceDyn::referenceVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  // Process reference joints' velocities
  jts_ref_velocities_ = Eigen::VectorXd::Map(msg->data.data(), msg->data.size());

}

// Receive the reference pose of the end_effector
void TaskSpaceDyn::referencePoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
  // Process reference end-effector pose
  ee_ref_position_(0) = msg->position.x;
  ee_ref_position_(1) = msg->position.y;
  ee_ref_position_(2) = msg->position.z;

  ee_ref_orientation_.x() = msg->orientation.x;
  ee_ref_orientation_.y() = msg->orientation.y;
  ee_ref_orientation_.z() = msg->orientation.z;
  ee_ref_orientation_.w() = msg->orientation.w;

  // Debugging: Print the reference pose received
//  ROS_DEBUG("Reference Position: [%f, %f, %f]", ee_ref_position_(0), ee_ref_position_(1), ee_ref_position_(2);
//  ROS_DEBUG("Reference Orientation: [%f, %f, %f, %f]", ee_ref_orientation_.x(), ee_ref_orientation_.y(),
//            ee_ref_orientation_.z(), ee_ref_orientation_.w());

}

// Receive the reference twist of the end_effector
void TaskSpaceDyn::referenceTwistCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Process reference end-effector twist
  ee_ref_twist_(0) = msg->linear.x;
  ee_ref_twist_(1) = msg->linear.y;
  ee_ref_twist_(2) = msg->linear.z;
  ee_ref_twist_(3) = msg->angular.x;
  ee_ref_twist_(4) = msg->angular.y;
  ee_ref_twist_(5) = msg->angular.z;


  // Debugging: Print the reference twist received
//  ROS_DEBUG("Reference twist: [%f, %f, %f, %f, %f, %f]", ee_ref_twist_.linear.x(),
//            ee_ref_twist_.linear.y(), ee_ref_twist_.linear.z(), ee_ref_twist_.angular.x(),
//            ee_ref_twist_.angular.y(), ee_ref_twist_.angular.z())

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
  ROS_INFO("Computing Task Space Dynamics");
  pinocchio::computeAllTerms(model_, data_, jts_fbk_positions_, jts_fbk_velocities_);
  M_ = data_.M;  // Mass Matrix
  h_ = data_.nle;  // Nonlinear Effector Vector

  // Get the Jacobian and its time derivative
  pinocchio::getJointJacobian(model_, data_, hand_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_);
  pinocchio::getJointJacobianTimeVariation(model_, data_, hand_id_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot_);
  jacobian_trans_ = jacobian_.topRows(3);
  jacobian_dot_trans_ = jacobian_dot_.topRows(3);

  // End-effector pose
  ee_fbk_position_ = data_.oMi[hand_id_].translation();
  pinocchio::quaternion::assignQuaternion(ee_fbk_orientation_, data_.oMi[hand_id_].rotation());

  // End-effector twist
  ee_fbk_twist_ = jacobian_ * jts_fbk_velocities_;

  pubEndFeedback();

  //Performing dynamic controller
  if (with_orientation_) {
    // Compute the pseudo-inverse of the Jacobian matrix
    jacobian_pseudo_inverse_ = jacobian_.transpose() * (jacobian_ * jacobian_.transpose()).inverse();

    // Compute task-space mass matrix (Lambda)
    lambda_ = jacobian_pseudo_inverse_.transpose() * M_ * jacobian_pseudo_inverse_;

    // Compute task-space bias forces (eta)
    eta_ = jacobian_pseudo_inverse_.transpose() * h_ - lambda_ * jacobian_dot_ * jts_fbk_velocities_;

    twist_error_ = ee_ref_twist_ - ee_fbk_twist_;										// twist error
    position_error_ = ee_ref_position_ - ee_fbk_position_;							    //position error
    orientation_error_ = ee_ref_orientation_ * ee_fbk_orientation_.inverse();			// relative quaternion
    angle_axis_error_ = Eigen::AngleAxisd(orientation_error_);
    rotation_error_ = angle_axis_error_.axis() * angle_axis_error_.angle();
    pose_error_.head<3>() = position_error_;
    pose_error_.tail<3>() = rotation_error_;

    //ROS_INFO_STREAM("Stiffness: " << end_stiffness_ << " Damping: " << end_damping_);
    ee_acc_cmd_ = ee_ref_acc_ + (end_stiffness_ * pose_error_) + (end_damping_ * twist_error_);

    // Compute the task-space wrench
    wrench_ = lambda_ * ee_acc_cmd_ + eta_;

    // Compute the joint torques
    tau_task_ = jacobian_.transpose() * wrench_;
  } else {
    // Compute the pseudo-inverse of the Jacobian matrix
    jacobian_pseudo_inverse_trans_ = jacobian_trans_.transpose() * (jacobian_trans_ * jacobian_trans_.transpose()).inverse();

    // Compute task-space mass matrix (Lambda)
    lambda_trans_ = jacobian_pseudo_inverse_trans_.transpose() * M_ * jacobian_pseudo_inverse_trans_;

    // Compute task-space bias forces (eta)
    eta_trans_ = jacobian_pseudo_inverse_trans_.transpose() * h_ - lambda_trans_ * jacobian_dot_trans_ * jts_fbk_velocities_;

    // Compute desired task-space acceleration
    twist_error_trans_ = ee_ref_twist_.topRows(3) - ee_fbk_twist_.topRows(3);
    pose_error_trans_ = ee_ref_position_ - ee_fbk_position_;
    ee_acc_cmd_trans_ = ee_ref_acc_trans_ + (end_stiffness_ * pose_error_trans_) + (end_damping_ * twist_error_trans_);

    // Compute the task-space wrench
    wrench_trans_ = lambda_trans_ * ee_acc_cmd_trans_ + eta_trans_;

    // Compute the joint torques
    tau_task_ = jacobian_trans_.transpose() * wrench_trans_;
  }
  if (with_redundancy_) {
    // Compute Projection Matrix
    if (with_orientation_)
       P_ = I_ - jacobian_.transpose() * (jacobian_ * M_.inverse() * jacobian_.transpose()).inverse() * jacobian_ * M_.inverse();
    else
       P_ = I_ - jacobian_trans_.transpose() * (jacobian_trans_ * M_.inverse() * jacobian_trans_.transpose()).inverse() * jacobian_trans_ * M_.inverse();

    // Compute joint acceleration
    joint_acceleration_ = joints_stiffness_ * (jts_ref_positions_ - jts_fbk_positions_) + joints_damping_ * (jts_ref_velocities_ - jts_fbk_velocities_);

    // Compute Joint Torque
    tau_joint_ = M_ * joint_acceleration_ + h_;

    // Compute Null Torque
    tau_null_ = P_ * tau_joint_;

    // Compute total torque
    tau_total_ = tau_task_ + tau_null_;

    // Populate Message
    for (int i = 0; i < num_joints_; i++)
      joint_torque_msg_.data[i] = tau_total_(i);
  } else {
    // Populate message with task-space torque
    for (int i = 0; i < num_joints_; i++)
      joint_torque_msg_.data[i] = tau_task_(i);
  }
  // Publish the joint torques
  joint_effort_pub_.publish(joint_torque_msg_);
}


// Publish the end effector feedback
void TaskSpaceDyn::pubEndFeedback() {
  // Populate the ROS Messages
  ee_fbk_pose.position.x = ee_fbk_position_(0);  			// X-coordinate of end-effector
  ee_fbk_pose.position.y = ee_fbk_position_(1);  			// Y-coordinate of end-effector
  ee_fbk_pose.position.z = ee_fbk_position_(2);  			// Z-coordinate of end-effector
  ee_fbk_pose.orientation.x = ee_fbk_orientation_.x();
  ee_fbk_pose.orientation.y = ee_fbk_orientation_.y();
  ee_fbk_pose.orientation.z = ee_fbk_orientation_.z();
  ee_fbk_pose.orientation.w = ee_fbk_orientation_.w();


  end_effector_pose_pub_.publish(ee_fbk_pose);  // Publish pose


  // Publish the end-effector twist as a geometry_msgs::Twist message
  ee_fbk_twist.linear.x = ee_fbk_twist_(0);  // Linear velocity in X direction
  ee_fbk_twist.linear.y = ee_fbk_twist_(1);  // Linear velocity in Y direction
  ee_fbk_twist.linear.z = ee_fbk_twist_(2);  // Linear velocity in Z direction
  ee_fbk_twist.angular.x = ee_fbk_twist_(3);
  ee_fbk_twist.angular.y = ee_fbk_twist_(4);
  ee_fbk_twist.angular.z = ee_fbk_twist_(5);


  end_effector_twist_pub_.publish(ee_fbk_twist);  // Publish twist
}