#include "cubic_polynomial_planner.hpp"
#include <geometry_msgs/Pose.h>
#include <highlevel_msgs/MoveTo.h>
#include <cmath>

/*
 * CUBIC - class for cubic polynomial trajectory
 * Description:
 * Author: Ekeno
 ***********************************************
 */

// constructor
CUBIC::CUBIC(ros::NodeHandle& nh) : nh_(nh) {
    // Initialise the variables

    // Read the parameters
    readParameters();

    // Subscrive to "/firefly/ground_truth/pose"
    current_pose_sub_ = nh_.subscribe(feedback_pose_topic_, 10, &CUBIC::poseCallback, this);

    // Advertise the move_to service
    move_to_srv_ = nh_.advertiseService("/pose_planner/move_to", &CUBIC::moveToCallback, this);

    // Publishing
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>(reference_pose_topic_, 1);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(reference_twist_topic_, 1);

    // Publish the default translation
    pubDefaultTranslation();

    ROS_INFO("CUBIC_POLYNOMIAL_PLANNER NODE INITIALIZED");   // for debugging
}

// Ground_truth subscriber callback : Getting the ground truth/ current_position
void CUBIC::poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // Store the received information to current_pose_ (only position || ignore orientation)
    current_pose_ << msg->position.x, msg->position.y, msg->position.z;
    current_orient_ = msg->orientation;
}

// MoveTo Service : Getting user request then move the robot
bool CUBIC::moveToCallback(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res) {
    // Receive the request
    // Check if z >= zero
    if (req.z <= 0)
        {
            ROS_WARN("IT WILL CRUSH!!");
            res.success = false;
            return false;
        }
    target_pose_ << req.x, req.y, req.z;
    duration_ = req.T;

    starting_pose_ = current_pose_;
    starting_time_ = ros::Time::now().toSec();

    res.success = true;
    return true;
}

// Plan the trajectory
void CUBIC::update() {
    // Perform the trajectory plan
    current_time_ = ros::Time::now().toSec() - starting_time_;				// elapsed time since we got the request

    if (current_time_ > duration_) {
      	current_time_ = duration_;
        duration_ -= current_time_;
        ROS_INFO("Target reached. Stopping updates.");
    }
	else {
    	pose_scaling_ = ((3 * pow(current_time_, 2)) / pow(duration_, 2)) - ((2 * pow(current_time_, 3)) / pow(duration_, 3));
    	move_to_ = starting_pose_ + pose_scaling_ * (target_pose_ - starting_pose_);

    	twist_scaling_ = ((6 * current_time_) / pow(duration_, 2)) - ((6 * pow(current_time_, 2)) / pow(duration_, 3));
    	moving_vel_ = twist_scaling_ * (target_pose_ - starting_pose_);

    	pose_.position.x = move_to_(0);
    	pose_.position.y = move_to_(1);
    	pose_.position.z = move_to_(2);
    	pose_.orientation = current_orient_;

    	pose_pub_.publish(pose_);

    	twist_.linear.x = moving_vel_(0);
    	twist_.linear.y = moving_vel_(1);
    	twist_.linear.z = moving_vel_(2);
    	twist_.angular.x = 0.0;
    	twist_.angular.y = 0.0;
    	twist_.angular.z = 0.0;

    	twist_pub_.publish(twist_);
    }
}

// Function to read the ROS parameters
void CUBIC::readParameters() {
    // Retrieve the publish rate from the parameter server
    if (!nh_.getParam("/publish_rate", publish_rate_)) {
        ROS_ERROR("Parameter publish_rate not set");
        publish_rate_ = 500;                                // Set to 500Hz if not set
    }

    // feedback_pose_ subscriber
    if (!nh_.getParam("/feedback_pose_topic", feedback_pose_topic_)) {
             ROS_ERROR("Parameter feedback_pose_topic not set");
             feedback_pose_topic_ = "/gen3/feedback/pose";
    }

    // reference_pose publisher
    if (!nh_.getParam("/reference_pose_topic", reference_pose_topic_)) {
             ROS_ERROR("Parameter reference_pose_topic not set");
             reference_pose_topic_ = "/gen3/reference/pose";
    }

    // reference_twist_ publisher
    if (!nh_.getParam("/reference_twist_topic", reference_twist_topic_)) {
             ROS_ERROR("Parameter reference_twist_topic not set");
             reference_twist_topic_ = "/gen3/reference/twist";
    }

    // default_translation for linear
    std::vector<double> linear_default_;
    if (!nh_.getParam("/gen3/linear/default", linear_default_) || linear_default_.size() != 3) {
      		ROS_WARN("Default translation not set or incorrect size, setting to [0, 0, 0].");
      		default_translation_.setZero();
	} else {
      		default_translation_ = Eigen::Vector3d(linear_default_.data());
    }
}

// Handle default translation
void CUBIC::pubDefaultTranslation() {
  // geometry message
  geometry_msgs::Pose default_pose_;

  // Translation
  default_pose_.position.x = default_translation_(0);
  default_pose_.position.y = default_translation_(1);
  default_pose_.position.z = default_translation_(2);

  // Set orientation to identity quaternion (no rotation)
  default_pose_.orientation.x = 0.0;
  default_pose_.orientation.y = 0.0;
  default_pose_.orientation.z = 0.0;
  default_pose_.orientation.w = 1.0;

  pose_pub_.publish(default_pose_);
}


// destructor
CUBIC::~CUBIC() {
  //
}
