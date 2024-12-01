#include "action_client.hpp"

// constructor
ActionClient::ActionClient(ros::NodeHandle& nh) : node_handle_(nh), move_to_action_client_("/gen3/action_planner/pose", true) {
    // reading the parameters
  if (!read_parameters()) {
    ROS_ERROR("Failed to read parameters.");
    ros::requestShutdown();
  }

  ROS_INFO_STREAM("Client initialized.");
}

// read the parameters
bool ActionClient::read_parameters() {
  // number of actions
  if (!node_handle_.getParam("/action_list/number_of_targets", NUM_TARGETS_)) {
    ROS_ERROR("Failed to read number_of_targets.");
    return false;
  }

  // Allocate space for translation and orientation
  target_translation_ = Eigen::MatrixXd(NUM_TARGETS_, 3);
  target_orientation_ = Eigen::MatrixXd(NUM_TARGETS_, 3);
  target_duration_ = Eigen::VectorXd(NUM_TARGETS_);

  for (int i = 0; i < NUM_TARGETS_; i++) {
    std::string target_name_ = "action_list_" + std::to_string(i);

    // read the translation
    std::vector<double> translation_;
    if (!node_handle_.getParam(target_name_ + "/translation", translation_)) {
        ROS_ERROR("Failed to read translation.");
        return false;
    }

    // store translation
    for (int j = 0; j < 3; j++)
        target_translation_(i, j) = translation_[j];

    // read the orientation
    std::vector<double> orientation_;
    if (!node_handle_.getParam(target_name_ + "/orientation", orientation_)) {
      ROS_ERROR("Failed to read orientation.");
      return false;
    }

    // store orientation
    for (int k = 0; k < 3; k++)
      target_orientation_(i, k) = orientation_[k];

    // read time
    double duration_;
    if (!node_handle_.getParam(target_name_ + "/duration", duration_)) {
      ROS_ERROR("Failed to read duration.");
      return false;
    }
    // store the duration
    target_duration_(i) = duration_;
  }
  ROS_INFO_STREAM("Parameters loaded: " << NUM_TARGETS_ << " targets.");
  return true;
}

// sending the goal
void ActionClient::update() {
  ROS_INFO_STREAM("[ActionClient::update] Action client is ready. Waiting for the server...");

  for (int counter = 0; counter < NUM_TARGETS_; counter++) {
    // populate the action goal
    move_to_goal_.x = target_translation_(counter, 0);
    move_to_goal_.y = target_translation_(counter, 1);
    move_to_goal_.z = target_translation_(counter, 2);
    move_to_goal_.roll = target_orientation_(counter, 0);
    move_to_goal_.pitch = target_orientation_(counter, 1);
    move_to_goal_.yaw = target_orientation_(counter, 2);
    move_to_goal_.T = target_duration_(counter);

    // wait for the server
    move_to_action_client_.waitForServer();

    // send the goal to the server
    move_to_action_client_.sendGoal(move_to_goal_,
                 boost::bind(&ActionClient::done_callback, this, _1, _2),
                 boost::bind(&ActionClient::active_callback, this),
                 boost::bind(&ActionClient::feedback_callback, this, _1));

    // wait for the results
    ROS_INFO_STREAM("[ActionClient::update] Goal sent. Waiting for the results...");
    move_to_action_client_.waitForResult(ros::Duration(30));

    // completed the goal
    if (move_to_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO_STREAM("[ActionClient::update] Goal Reached successfully.");
  }

  // completed all task
  ROS_INFO_STREAM("[ActionClient::update] All Goals completed.");
  ros::shutdown();
}

// callbacks
void ActionClient::done_callback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) {
  ROS_INFO("[ActionClient::doneCallback] Finished in state [%s]", state.toString().c_str());
}

void ActionClient::active_callback() {
  ROS_INFO_STREAM("[ActionClient::activeCallback] Action has become active");
}


void ActionClient::feedback_callback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) {
  ROS_DEBUG_STREAM("[ActionClient::feedbackCallback] distance_translation:"<< feedback->distance_translation);
  ROS_DEBUG_STREAM("[ActionClient::feedbackCallback] distance_orientation:"<< feedback->distance_orientation);
  ROS_DEBUG_STREAM("[ActionClient::feedbackCallback] elapsed_time:"<< feedback->time_elapsed);
}

// Destructor
ActionClient::~ActionClient() {
  // Destructor implementation (even if it's empty, it should be defined)
}