#include <ros/ros.h>
#include "potential_field_planner.hpp"


// constructor:
PotF::PotF(ros::NodeHandle& nh) : nh_(nh)
{
  // Initialise the variables
  default_.setZero();                // Initializes the 7-element vector to all zeros
  delta_position_.setZero();
  k_att_ = 0;
  max_velocity_ = 0;
  publish_rate_ = 500;
  joint_positions_.setZero();       // Initialize joint positions to zeros
  joint_velocities_.setZero();      // Initialize joint velocities to zeros

  // read parameters
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }


  // Initialise subscriber for joint states
  joint_states_sub_ = nh_.subscribe(joint_states_sub_topic_, 10, &PotF::jointStatesCallBack, this);

  // Initialize the publishers for reference joint positions and velocities
  reference_position_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gen3/reference/position", 10);
  reference_velocity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gen3/reference/velocity", 10);

  //service

}

// Reading the parameters
bool PotF::readParameters()
{
  // Load publish rate
  if (!nh_.getParam("/publish_rate", publish_rate_)) {
    ROS_ERROR("Publish rate not set");
    return false;
  }

  // Load K attractive (potential gains) as a vector and map to Eigen::Matrix
  if (!nh_.getParam("/gen3/joint/k_att", k_att_)) {
    ROS_ERROR("K att not set, setting to default 20 for all joints.");
    return false;
  }

  // Load maximum velocity
  if (!nh_.getParam("/gen3/joint/max_velocity", max_velocity_)) {
    ROS_ERROR("Max velocity not set, setting to default 1.2 for all joints.");
    return false;
  }

  // Load default joint positions
  std::vector<double> default_vec;
  if (!nh_.getParam("/gen3/joint/default", default_vec)) {
    ROS_ERROR("Default joint positions not set, setting to default values.");
    return false;
  } else {
    default_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(default_vec.data());
  }

  // read the joint_states_
  if (!nh_.getParam("/topic_names/fbk_joint_state", joint_states_sub_topic_)) {
    ROS_ERROR("Joint state topic not set");
    return false;
  }

  if (!nh_.getParam("/topic_names/ref_joint_vel", reference_velocity_topic_)) {
    ROS_ERROR("Reference velocity topic not set");
    return false;
  }

  if (!nh_.getParam("/topic_names/ref_joint_pos", reference_position_topic_)) {
    ROS_ERROR("Reference position topic not set.");
    return false;
  }
  return true;
}


// Joint states callback
void PotF::jointStatesCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
  // get the joints positions and velocities
  if (msg->position.size() == 7 && msg->velocity.size() == 7) {
    for (int i = 0; i < 7; i++) {
      joint_positions_(i) = msg->position[i];
      joint_velocities_(i) = msg->velocity[i];
    }

    received_joint_states_ = true;

    // Debug
    ROS_INFO_STREAM("Joint Positions: " << joint_positions_.transpose());
    ROS_INFO_STREAM("Joint Velocities: " << joint_velocities_.transpose());
   }
}

void PotF::computePotentialField()
{
    // Compute the difference between the target and current joint positions
    delta_position_ = default_ - joint_positions_;

    // Compute the attractive potential field
    reference_velocities_ = k_att_ * delta_position_;

    // Normalize the velocities if needed, based on max_velocity_ limits
    for (int i = 0; i < 7; ++i) {
        velocity_magnitude = std::abs(reference_velocities_(i));

        // Normalize the velocity if it exceeds the maximum allowed velocity
        if (velocity_magnitude > max_velocity_) {
            reference_velocities_(i) *= (max_velocity_ / velocity_magnitude);
        }
    }

    // Update the reference joint positions based on current positions and velocities
    reference_positions_ = joint_positions_ + reference_velocities_ * 0.002;

    publishJointReferences();
}

// Publish reference joint positions and velocities
void PotF::publishJointReferences()
{
  // Prepare reference position message
  std_msgs::Float64MultiArray position_msg;
  position_msg.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    position_msg.data[i] = reference_positions_(i);  // Populate with reference joint positions
  }
  reference_position_pub_.publish(position_msg);

  // Prepare reference velocity message
  std_msgs::Float64MultiArray velocity_msg;
  velocity_msg.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    velocity_msg.data[i] = reference_velocities_(i);  // Populate with reference joint velocities
  }
  reference_velocity_pub_.publish(velocity_msg);

  // Debugging
  ROS_INFO("Published reference joint positions and velocities");
  ROS_INFO_STREAM("Reference Positions: " << reference_positions_.transpose());
  ROS_INFO_STREAM("Reference Velocities: " << reference_velocities_.transpose());
}

