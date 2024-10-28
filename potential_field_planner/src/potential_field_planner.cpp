#include <ros/ros.h>
#include "potential_field_planner.hpp"


// constructor:
PotF::PotF(ros::NodeHandle& nh) : nh_(nh)
{
  // Initialise the variables
  default_.setZero();                // Initializes the 7-element vector to all zeros
  k_att_.setZero();
  max_velocity_.setZero();
  joint_positions_.setZero();       // Initialize joint positions to zeros
  joint_velocities_.setZero();      // Initialize joint velocities to zeros

  // read parameters
  readParameters();


  // Initialise subscriber for joint states
  joint_states_sub_ = nh_.subscribe(joint_states_sub_topic_, 10, &PotF::jointStatesCallBack, this);

  // Initialize the publishers for reference joint positions and velocities
  reference_position_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gen3/reference/position", 10);
  reference_velocity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gen3/reference/velocity", 10);

  //service
}

// Reading the parameters
void PotF::readParameters()
{
  // Load publish rate
  if (!nh_.getParam("/publish_rate", publish_rate_)) {
    ROS_WARN("Publish rate not set");
    publish_rate_ = 500.0;
  }

  // Load K attractive (potential gains) as a vector and map to Eigen::Matrix
  std::vector<double> k_att_vec;
  if (!nh_.getParam("/gen3/joint/k_att", k_att_vec)) {
    ROS_WARN("K att not set, setting to default 20 for all joints.");
    k_att_.setConstant(20.0);
  } else {
    k_att_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(k_att_vec.data());
  }

  // Load maximum velocity
  std::vector<double> max_velocity_vec;
  if (!nh_.getParam("/gen3/joint/max_velocity", max_velocity_vec)) {
    ROS_WARN("Max velocity not set, setting to default 1.2 for all joints.");
    max_velocity_.setConstant(1.2);
  } else {
    max_velocity_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(max_velocity_vec.data());
  }

  // Load default joint positions
  std::vector<double> default_vec;
  if (!nh_.getParam("/gen3/joint/default", default_vec)) {
    ROS_WARN("Default joint positions not set, setting to default values.");
    default_ << 1.57, 0.35, 1.57, -2.0, 0.0, -1.0, 1.57;
  } else {
    default_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(default_vec.data());
  }

  // read the joint_states_
  if (!nh_.getParam("/joint_states_topic", joint_states_sub_topic_)) {
    ROS_WARN("Subscriber topic not set");
    joint_states_sub_topic_ = "/gen3/joint_states";
  }

  if (!nh_.getParam("/gen3/reference/velocity_topic", reference_velocity_topic_)) {
    reference_velocity_topic_ = "/gen3/reference/velocity";
    ROS_WARN("Reference velocity topic not set. Using default: /gen3/reference/velocity");
  }

  if (!nh_.getParam("/gen3/reference/position_topic", reference_position_topic_)) {
    reference_position_topic_ = "/gen3/reference/position";
    ROS_WARN("Reference position topic not set. Using default: /gen3/reference/position");
  }
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
    Eigen::Matrix<double, 7, 1> delta_position = default_ - joint_positions_;

    // Compute the attractive potential field
    reference_velocities_ = k_att_.cwiseProduct(delta_position);

    // Normalize the velocities if needed, based on max_velocity_ limits
    for (int i = 0; i < 7; ++i) {
        double velocity_magnitude = std::abs(reference_velocities_(i));

        // Normalize the velocity if it exceeds the maximum allowed velocity
        if (velocity_magnitude > max_velocity_(i)) {
            reference_velocities_(i) *= (max_velocity_(i) / velocity_magnitude);
        }
    }

    // Update the reference joint positions based on current positions and velocities
    double delta_t = 1.0 / publish_rate_; // Time step (1/publish_rate)
    reference_positions_ = joint_positions_ + reference_velocities_ * delta_t;

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

  ROS_INFO("Published reference joint positions and velocities");
}

