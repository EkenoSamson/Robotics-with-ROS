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
    ROS_ERROR("[ActionServer::ActionServer] Failed to read parameters.");
    ros::requestShutdown();
  }

  // fallback values for debugging
  feedback_position_ = tf2::Vector3(0.0, 0.0, 0.0);
  feedback_orientation_ = tf2::Quaternion(0.0, 0.0, 0.0, 1.0);

  // subsscribers
  feedback_pose_subscriber_ = node_handle_.subscribe(feedback_pose_sub_topic_, 1, &ActionServer::feedback_pose_callback, this);

  // publishers
  reference_pose_publisher_ = node_handle_.advertise<geometry_msgs::Pose>(reference_pose_pub_topic_, 10);
  reference_twist_publisher_ = node_handle_.advertise<geometry_msgs::Twist>(reference_twist_pub_topic_, 10);

  // start the server
  ROS_INFO("[ActionServer::ActionServer] Starting action server...");
  move_to_action_server_.start();
  ROS_INFO("[ActionServer::ActionServer] Action server started.");
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
  //ROS_INFO_STREAM("[ActionServer::feedback_pose_callback] Action Server active: " << move_to_action_server_.isActive());
  if (!move_to_action_server_.isActive()) {
    tf2::fromMsg(feedback_pose_msg->position, feedback_position_);
    tf2::fromMsg(feedback_pose_msg->orientation, feedback_orientation_);

    // Debug
    //ROS_DEBUG_STREAM("[ActionServer::move_to_callback] starting position: " << feedback_position_.x() << ", " << feedback_position_.y() << ", " << feedback_position_.z());
    //ROS_DEBUG_STREAM("[ActionServer::move_to_callback] starting Orientation: " << feedback_orientation_.x() << ", " << feedback_orientation_.y() << ", " << feedback_orientation_.z() << ", " << feedback_orientation_.w());

  }
}

// interpolation
void ActionServer::move_to_callback(const highlevel_msgs::PoseCommandGoalConstPtr& goal) {
  /*ROS_INFO_STREAM("[ActionServer::move_to_Callback] Goal :x= " << goal->x << ", y=" << goal->y << ", z=" <<goal->z <<
                  ", roll=" << goal->roll << ", pitch=" << goal->pitch << ", yaw=" << goal->yaw <<", T=" << goal->T);*/

  starting_time_ = ros::Time::now().toSec();
  starting_position_ = feedback_position_;
  starting_orientation_ = feedback_orientation_;

  // Debug log the feedback pose data
  /*ROS_DEBUG_STREAM("[ActionServer::move_to_callback] starting position: " << starting_position_.x() << ", " << starting_position_.y() << ", " << starting_position_.z());
  ROS_DEBUG_STREAM("[ActionServer::move_to_callback] starting Orientation: " << starting_orientation_.x() << ", " << starting_orientation_.y() << ", " << starting_orientation_.z() << ", " << starting_orientation_.w());
  ROS_DEBUG_STREAM("[ActionServer::move_to_callback] starting time: " << starting_time_);*/

  target_position_ = tf2::Vector3(goal->x, goal->y, goal->z);
  target_orientation_.setRPY(goal->roll, goal->pitch, goal->yaw);
  target_duration_ = goal->T;

  // Debug log the reference data
 /* ROS_DEBUG_STREAM("[ActionServer::move_to_callback] target Translation: " << target_position_.x() << ", " << target_position_.y() << ", " << target_position_.z());
  ROS_DEBUG_STREAM("[ActionServer::move_to_callback] target Orientation: " << target_orientation_.x() << ", " << target_orientation_.y() << ", " << target_orientation_.z() << ", " << target_orientation_.w());
  ROS_DEBUG_STREAM("[ActionServer::move_to_callback] target duration: " << target_duration_);*/

  move_to_feedback_.distance_translation = (target_position_ - starting_position_).length();
  move_to_feedback_.distance_orientation = target_orientation_.angleShortestPath(starting_orientation_);
  move_to_feedback_.time_elapsed = 0.0;

  // Debug log the feedback data
  /*ROS_DEBUG_STREAM("[ActionServer::move_to_callback] Distance Translation: " << move_to_feedback_.distance_translation);
  ROS_DEBUG_STREAM("[ActionServer::move_to_callback] Distance Orientation: " << move_to_feedback_.distance_orientation);
  ROS_DEBUG_STREAM("[ActionServer::move_to_callback] Time Elapsed: " << move_to_feedback_.time_elapsed);*/

  while (move_to_feedback_.time_elapsed < target_duration_) {
    //ROS_INFO_STREAM("[ActionServer::move_to_callback] While loop executing...");
//    // Perform feedback calculation inside the loop
//    move_to_feedback_.distance_translation = (target_position_ - feedback_position_).length();
//    move_to_feedback_.distance_orientation = target_orientation_.angleShortestPath(feedback_orientation_);
//    move_to_feedback_.time_elapsed = ros::Time::now().toSec() - starting_time_;

    // Log updated values for debugging
   /* ROS_DEBUG_STREAM("[ActionServer::move_to_callback] Updated feedback: distance_translation=" << move_to_feedback_.distance_translation);
    ROS_DEBUG_STREAM("[ActionServer::move_to_callback] Updated feedback: distance_orientation=" << move_to_feedback_.distance_orientation);
    ROS_DEBUG_STREAM("[ActionServer::move_to_callback] Updated time: " << move_to_feedback_.time_elapsed);
    ROS_DEBUG_STREAM("[ActionServer::compute_transformation] target duration: " << target_duration_);*/

    // publish feedback for the client
    move_to_action_server_.publishFeedback(move_to_feedback_);

    compute_transformation();  // transformation interpolation

    loop_rate_.sleep();
  }
  move_to_action_server_.setSucceeded();
  //ROS_INFO("[ActionServer::move_to_callback] Action Succeeded");
}

// compute the interpolation of the transformation
void ActionServer::compute_transformation() {
  //ROS_INFO_STREAM("[ActionServer::compute_transformation]");
  current_time_ = ros::Time::now().toSec() - starting_time_;

  if (current_time_ > target_duration_)
    current_time_ = target_duration_;
//    target_duration_ = current_time_;
//  } else {
    // scaling factors
    transform_scaling_factor_ = ((3 * pow(current_time_, 2)) / pow(target_duration_, 2)) - ((2 * pow(current_time_, 3)) / pow(target_duration_, 3));
    velocity_scaling_factor_ = ((6 * current_time_) / pow(target_duration_, 2)) - ((6 * pow(current_time_, 2)) / pow(target_duration_, 3));

    // scaling factor debuggin
    //ROS_INFO_STREAM("[ActionServer::compute_transformation] transform_scaling_factor_: " << transform_scaling_factor_);

    // interpolation
    planned_position_ = starting_position_ + transform_scaling_factor_ * (target_position_ - starting_position_);
    planned_orientation_ = starting_orientation_.slerp(target_orientation_, transform_scaling_factor_);
    planned_velocity_ = velocity_scaling_factor_ * (target_position_ - starting_position_);

    // Debugging th calculation (interpolation)
    /*ROS_INFO_STREAM("[ActionServer::compute_transformation] Interpolation Calculation: Planned_Position [x: "
        << planned_position_.x() << ", y: "
        << planned_position_.y() << ", z: "
        << planned_position_.z() << "], Orientation [x: "
        << planned_orientation_.x() << ", y: "
        << planned_orientation_.y() << ", z: "
        << planned_orientation_.z() << ", w: "
        << planned_orientation_.w() << "]");*/

    // feedback
    move_to_feedback_.distance_translation = (target_position_ - planned_position_).length();
    move_to_feedback_.distance_orientation = target_orientation_.angleShortestPath(planned_orientation_);
    move_to_feedback_.time_elapsed = current_time_;

    // Log updated values for debugging
    /*ROS_DEBUG_STREAM("[ActionServer::compute_transformaton] Updated feedback: distance_translation=" << move_to_feedback_.distance_translation);
    ROS_DEBUG_STREAM("[ActionServer::compute_transformation] Updated feedback: distance_orientation=" << move_to_feedback_.distance_orientation);
    ROS_DEBUG_STREAM("[ActionServer::compute_transformation] updated feedback: time_elapsed=" << move_to_feedback_.time_elapsed);
    ROS_DEBUG_STREAM("[ActionServer::compute_transformation] target duration: " << target_duration_);*/

    // publication
    reference_pose_msg_.position.x = planned_position_.x();
    reference_pose_msg_.position.y = planned_position_.y();
    reference_pose_msg_.position.z = planned_position_.z();
    reference_pose_msg_.orientation = tf2::toMsg(planned_orientation_);
    reference_twist_msg_.linear = tf2::toMsg(planned_velocity_);
    //reference_twist_msg_.angular = 0.0;

    // messages to be published
    /*ROS_INFO_STREAM("[ActionServer::compute_transformation] Reference_Pose_Message: Position [x: "
        << reference_pose_msg_.position.x << ", y: "
        << reference_pose_msg_.position.y << ", z: "
        << reference_pose_msg_.position.z << "], Orientation [x: "
        << reference_pose_msg_.orientation.x << ", y: "
        << reference_pose_msg_.orientation.y << ", z: "
        << reference_pose_msg_.orientation.z << ", w: "
        << reference_pose_msg_.orientation.w << "]");*/

    reference_pose_publisher_.publish(reference_pose_msg_);
    reference_twist_publisher_.publish(reference_twist_msg_);
 // }
}

// updating
//void ActionServer::update() {
//  // This is the function that runs in the loop
//  // Add your logic here
//  //if (move_to_action_server_.isActive()) {
//  //ROS_INFO("Action server update called");
//  //compute_transformation();
//  //}
//}

// destructor
ActionServer::~ActionServer() {
  // Add your cleanup code here
  ROS_INFO("ActionServer destructor called");
}
