#include "pose_command_server.hpp"

// constructor
ActionServer::ActionServer(ros::NodeHandle& nh):
    node_handle_(nh),
    move_to_action_server_(node_handle_, "/gen3/action_planner/pose", boost::bind(&ActionServer::move_to_callback, this, _1), false),
    loop_rate_(500)
{
  // setting the logging to Debug : allow debug_level messages to be printed
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    ros::console::notifyLoggerLevelsChanged();

  // reading parameters
  if (!read_parameters()) {
    ROS_ERROR("Failed to read parameters.");
    ros::requestShutdown();
  }

  // subsscribers
  feedback_pose_subscriber_ = node_handle_.subscribe(feedback_pose_sub_topic_, 1, &ActionServer::feedback_pose_callback, this);

  // publishers
  reference_pose_publisher_ = node_handle_.advertise<geometry_msgs::Pose>(reference_pose_pub_topic_, 10);
  reference_twist_publisher_ = node_handle_.advertise<geometry_msgs::Twist>(reference_twist_pub_topic_, 10);

  // start the server
  move_to_action_server_.start();
  ROS_INFO("[ActionServer::ActionServer] action server is ready!.");

}

// reading parameter
bool ActionServer::read_parameters(){
  // feedback pose subscriber
  if (!node_handle_.getParam("/topic_names/fbk_hand_pose", feedback_pose_sub_topic_)) {
    ROS_ERROR("Parameter feedback_pose_topic not set");
    return false;
  }

  // reference pose publisher
  if (!node_handle_.getParam("/topic_names/ref_hand_pose", reference_pose_pub_topic_)) {
    ROS_ERROR("Parameter reference_pose_topic not set");
    return false;
  }

  // reference twist publisher
  if (!node_handle_.getParam("/topic_names/ref_hand_twist", reference_twist_pub_topic_)) {
    ROS_ERROR("Parameter reference_twist_topic not set");
    return false;
  }

  return true;
}

// Getting the current pose for the robot
void ActionServer::feedback_pose_callback(const geometry_msgs::Pose::ConstPtr& feedback_pose_msg) {
  if (move_to_action_server_.isActive()) {
    tf2::fromMsg(feedback_pose_msg->position, feedback_position_);
    tf2::fromMsg(feedback_pose_msg->orientation, feedback_orientation_);

    // Debug
    ROS_INFO_STREAM("[ActionServer::feedback_pose_callback] position: " << feedback_position_);
    ROS_INFO_STREAM("orientation: " << feedback_orientation_);
  }
}

// interpolation
void ActionServer::move_to_callback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) {
  ROS_INFO_STREAM("[ActionServer::move_to_Callback] Goal :x= " << goal->x << ", y=" << goal->y << ", z=" <<goal->z <<
                  ", roll=" << goal->roll << ", pitch=" << goal->pitch << ", yaw=" << goal->yaw <<", T=" << goal->T);

  starting_time_ = ros::Time::now().toSec();
  starting_position_ = feedback_position_;
  starting_orientation_ = feedback_orientation_;
  target_position_ = tf2::Vector3(goal->x, goal->y, goal->z);
  target_orientation_.setRPY(goal->roll, goal->pitch, goal->yaw);
  target_duration_ = goal->T;

  move_to_feedback_.distance_translation = (target_position_ - starting_position_).length();
  move_to_feedback_.distance_orientation = target_orientation_.angleShortestPath(starting_orientation_);
  move_to_feedback_.time_elapsed = 0.0;

  while (move_to_feedback_.distance_translation > 0.01 && move_to_feedback_.distance_orientation > 0.2) {
    // publish feedback for the client
    move_to_action_server_.publishFeedback(move_to_feedback_);
    ROS_DEBUG_STREAM("[ActionServer::move_to_callback] feedback=" << move_to_feedback_.distance_translation) ;

    compute_transformation();  // transformation interpolation

    loop_rate_.sleep();
  }
  move_to_action_server_.setSucceeded();
  ROS_INFO("[ActionServer::move_to_callback] Action Succeeded");
}

// compute the interpolation of the transformation
void ActionServer::compute_transformation() {
  current_time_ = ros::Time::now().toSec() - starting_time_;

  while (target_duration_ > 0.0) {
    // scaling factors
    transform_scaling_factor_ = ((3 * pow(current_time_, 2)) / pow(target_duration_, 2)) - ((2 * pow(current_time_, 3)) / pow(target_duration_, 3));
    velocity_scaling_factor_ = ((6 * current_time_) / pow(target_duration_, 2)) - ((6 * pow(current_time_, 2)) / pow(target_duration_, 3));

    // interpolation
    planned_position_ = starting_position_ + transform_scaling_factor_ * (target_position_ - starting_position_);
    planned_orientation_ = starting_orientation_.slerp(target_orientation_, transform_scaling_factor_);
    planned_velocity_ = velocity_scaling_factor_ * (target_position_ - starting_position_);

    // feedback
    move_to_feedback_.distance_translation = (planned_position_ - planned_position_).length();
    move_to_feedback_.distance_orientation = target_orientation_.angleShortestPath(planned_orientation_);
    move_to_feedback_.time_elapsed = current_time_;

    // publication
    reference_pose_msg_.position.x = planned_position_.x();
    reference_pose_msg_.position.y = planned_position_.y();
    reference_pose_msg_.position.z = planned_position_.z();
    reference_pose_msg_.orientation = tf2::toMsg(planned_orientation_);
    reference_twist_msg_.linear = tf2::toMsg(planned_velocity_);
    //reference_twist_msg_.angular = 0.0;


    reference_pose_publisher_.publish(reference_pose_msg_);
    reference_twist_publisher_.publish(reference_twist_msg_);

    target_duration_ -= current_time_;
  }
}

// updating
void ActionServer::update() {
  // This is the function that runs in the loop
  // Add your logic here
  ROS_INFO("Action server update called");
}

// destructor
ActionServer::~ActionServer() {
  // Add your cleanup code here
  ROS_INFO("ActionServer destructor called");
}
