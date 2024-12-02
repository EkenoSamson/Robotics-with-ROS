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
    if (!readParameters()) {
      ROS_ERROR("Failed to read parameters.");
      ros::requestShutdown();
    }

    // Subscrive to "/firefly/ground_truth/pose"
    current_pose_sub_ = nh_.subscribe(feedback_pose_topic_, 10, &CUBIC::poseCallback, this);

    // Advertise the move_to service
    move_to_srv_ = nh_.advertiseService("/pose_planner/move_to", &CUBIC::moveToCallback, this);
    move_ori_srv_ = nh_.advertiseService("/pose_planner/move_ori", &CUBIC::moveOriCallback, this);

    // Publishing
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>(reference_pose_topic_, 1);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>(reference_twist_topic_, 1);

    // Publish the default translation
    // pubDefaultTranslation();

    ROS_INFO("CUBIC_POLYNOMIAL_PLANNER NODE INITIALIZED");   // for debugging
}

// Ground_truth subscriber callback : Getting the ground truth/ current_position
void CUBIC::poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    // Store the received information to current_pose_ (only position || ignore orientation)
    current_pose_ << msg->position.x, msg->position.y, msg->position.z;
    tf2::fromMsg(msg->orientation, current_orient_);
}

// MoveOri Service : Getting user request then move the robot
bool CUBIC::moveOriCallback(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res) {
    // Receive the request
    target_orient_.setRPY(req.x, req.y, req.z);
    target_orient_.normalize();
    total_time_ = req.T;
	ROS_DEBUG("Received move_ori request: x=%f, y=%f, z=%f, T=%f", req.x, req.y, req.z, req.T);


    starting_orient_ = current_orient_;
    starting_orient_.normalize();
	const_position_ = current_pose_;
    starting_orient_.normalize();
    start_time_ = ros::Time::now().toSec();

    res.success = true;
    target_orient_received_ = true;
    return true;
}

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
    target_position_received_ = true;
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
    } else {
    	pose_scaling_ = ((3 * pow(current_time_, 2)) / pow(duration_, 2)) - ((2 * pow(current_time_, 3)) / pow(duration_, 3));
    	move_to_ = starting_pose_ + pose_scaling_ * (target_pose_ - starting_pose_);

    	twist_scaling_ = ((6 * current_time_) / pow(duration_, 2)) - ((6 * pow(current_time_, 2)) / pow(duration_, 3));
    	moving_vel_ = twist_scaling_ * (target_pose_ - starting_pose_);


    	pose_.position.x = move_to_(0);
    	pose_.position.y = move_to_(1);
    	pose_.position.z = move_to_(2);
    	pose_.orientation = tf2::toMsg(default_quat_);


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

void CUBIC::computeOrientation() {
    curr_time_ = ros::Time::now().toSec() - start_time_;

	if (curr_time_ > total_time_) {
		curr_time_ = total_time_;
        total_time_ -= curr_time_;
	} else {
    s_ = ((3 * pow(curr_time_, 2)) / pow(total_time_, 2)) - ((2 * pow(curr_time_, 3)) / pow(total_time_, 3));
	angle_ = target_orient_.angleShortestPath(starting_orient_);
    inter_orient_ = starting_orient_.slerp(target_orient_, s_);

	pose_.position.x = default_pose_.position.x;
    pose_.position.y = default_pose_.position.y;
    pose_.position.z = default_pose_.position.z;
    pose_.orientation = tf2::toMsg(inter_orient_);
    pose_pub_.publish(pose_);

    twist_.linear.x = 0;
    twist_.linear.y = 0;
    twist_.linear.z = 0;
    twist_.angular.x = 0;
    twist_.angular.y = 0;
    twist_.angular.z = 0;

    twist_pub_.publish(twist_);
	}
}


// Function to read the ROS parameters
bool CUBIC::readParameters() {
    // Retrieve the publish rate from the parameter server
    if (!nh_.getParam("/publish_rate", publish_rate_)) {
        ROS_ERROR("Parameter publish_rate not set");
		return false;
    }

    // feedback_pose_ subscriber
    if (!nh_.getParam("/topic_names/fbk_hand_pose", feedback_pose_topic_)) {
         ROS_ERROR("Parameter feedback_pose_topic not set");
         return false;
    }

    // reference_pose publisher
    if (!nh_.getParam("/topic_names/ref_hand_pose", reference_pose_topic_)) {
         ROS_ERROR("Parameter reference_pose_topic not set");
         return false;
    }

    // reference_twist_ publisher
    if (!nh_.getParam("/topic_names/ref_hand_twist", reference_twist_topic_)) {
         ROS_ERROR("Parameter reference_twist_topic not set");
         return false;
    }

    // default_translation for linear
    std::vector<double> linear_default_;
    if (!nh_.getParam("/gen3/linear/default", linear_default_)) {
      	ROS_ERROR("Default translation not set.");
      	return false;
	} else {
      	default_translation_ = Eigen::Vector3d(linear_default_.data());
    }

    //default_orientation
    std::vector<double> angular_default_;
    if (!nh_.getParam("/gen3/angular/default", angular_default_)) {
      ROS_ERROR("Default rotation not set.");
      return false;
    } else {
      default_orientation_ = Eigen::Vector3d(angular_default_.data());
    }
    return true;
}

// Handle default translation
void CUBIC::pubDefaultTransformation() {


  // Translation
  default_pose_.position.x = default_translation_(0);
  default_pose_.position.y = default_translation_(1);
  default_pose_.position.z = default_translation_(2);

  // convert from Euler to Quaternion
  default_quat_.setRPY(default_orientation_(0), default_orientation_(1), default_orientation_(2));

  // Set orientation to quaternion
  default_pose_.orientation = tf2::toMsg(default_quat_);


  pose_pub_.publish(default_pose_);

  // Debugging: Print the pose being published
//  ROS_INFO("Pose: Position [x: %f, y: %f, z: %f], Orientation [x: %f, y: %f, z: %f, w: %f]",
//              default_pose_.position.x, default_pose_.position.y, default_pose_.position.z,
//              default_pose_.orientation.x, default_pose_.orientation.y, default_pose_.orientation.z, default_pose_.orientation.w);


    default_twist_.linear.x = 0.0;
    default_twist_.linear.y = 0.0;
    default_twist_.linear.z = 0.0;
    default_twist_.angular.x = 0.0;
    default_twist_.angular.y = 0.0;
    default_twist_.angular.z = 0.0;

    twist_pub_.publish(default_twist_);
}
