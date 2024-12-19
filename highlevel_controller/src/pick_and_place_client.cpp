#include <pick_and_place_client.hpp>


/* Constructor */
PickAndPlaceClient::PickAndPlaceClient(ros::NodeHandle& nh) : nh_(nh), pose_action_client_("/gen3/action_planner/pose", true),
                                                              gripper_action_client_("/gen3/finger_group_action_controller/gripper_cmd", true) {
    //reading the parameters
    if (!read_parameters()) {
      ROS_ERROR("Failed to read parameters.");
      ros::requestShutdown();
    }

    ROS_INFO_STREAM("Client Initialized.");
}

/*Reading the parameters*/
bool PickAndPlaceClient::read_parameters() {
  // read the number of targets
  if (!nh_.getParam("/action_list/number_of_target", num_of_targets_)) {
    ROS_ERROR("Failed to capture number of targets.");
    return false;
  }
  actions_.resize(num_of_targets_);

  for (int i = 0; i < num_of_targets_; i++) {
    std::string action_name = "/action_list/action_" + std::to_string(i);
    std::string action_type_;  // temporary holder

    if (!nh_.getParam(action_name + "/action_type", action_type_)) {
     ROS_ERROR("Action type missing.");
     return false;
    }

    if (action_type_ == "pose_action") {
      pose_action_data pose_data_;
      std::vector<double> translation_;
      std::vector<double> orientation_;

      if (!nh_.getParam(action_name + "/translation", translation_)) {
        ROS_ERROR("Failed to pose action translation.");
        return false;
      }
      if (!nh_.getParam(action_name + "/orientation", orientation_)) {
        ROS_ERROR("Failed to pose action orientation.");
        return false;
      }
      if (!nh_.getParam(action_name + "/duration", pose_data_.duration)) {
        ROS_ERROR("Failed to pose action duration.");
        return false;
      }
      for (int j = 0; j < 3; j++) {
        pose_data_.translation[j] = translation_[j];
        pose_data_.orientation[j] = orientation_[j];
      }

      actions_[i].emplace<pose_action_data>(pose_data_);
    } else {
      gripper_action_data gripper_data_;

      if (!nh_.getParam(action_name + "/position", gripper_data_.position)) {
        ROS_ERROR("Failed to gripper action position.");
        return false;
      }
      if (!nh_.getParam(action_name + "/max_effort", gripper_data_.max_effort)) {
        ROS_ERROR("Failed to gripper action max_effort.");
        return false;
      }

      actions_[i].emplace<gripper_action_data>(gripper_data_);
    }
  }
  for(int i=0; i < num_of_targets_; i++){
    if (actions_[i].index() == 0) {
      ROS_INFO_STREAM("Action " << i << " action_type: pose_action");
      ROS_INFO_STREAM("Action " << i << " translation: " << std::get<pose_action_data>(actions_[i]).translation);
      ROS_INFO_STREAM("Action " << i << " orientation: " << std::get<pose_action_data>(actions_[i]).orientation);
      ROS_INFO_STREAM("Action " << i << " duration: " << std::get<pose_action_data>(actions_[i]).duration);
    } else if (actions_[i].index() == 1) {
      ROS_INFO_STREAM("Action " << i << " type: gripper_action");
      ROS_INFO_STREAM("Action " << i << " position: " << std::get<gripper_action_data>(actions_[i]).position);
      ROS_INFO_STREAM("Action " << i << " max_effort: " << std::get<gripper_action_data>(actions_[i]).max_effort);
    }
  }

  return true;
}

/* update */
void PickAndPlaceClient::pick_and_place() {
  //ROS_INFO_STREAM("[PickAndPlaceClient::pick_and_place]Waiting for pose and gripper action servers to start.");

  for (const auto& action_variant : actions_) {
    if (action_variant.index() == 0) {
      const auto& action_data = std::get<pose_action_data>(action_variant);
      pose_action_goal_.x = action_data.translation[0];
      pose_action_goal_.y = action_data.translation[1];
      pose_action_goal_.z = action_data.translation[2];
      pose_action_goal_.roll = action_data.orientation[0];
      pose_action_goal_.pitch = action_data.orientation[1];
      pose_action_goal_.yaw = action_data.orientation[2];
      pose_action_goal_.T = action_data.duration;

      //ROS_INFO_STREAM("Waiting for pose action server...");
      pose_action_client_.waitForServer();
      //ROS_INFO_STREAM("Connected to pose action server.");

      pose_action_client_.sendGoal(pose_action_goal_,
                    boost::bind(&PickAndPlaceClient::pose_done_callback, this, _1, _2),
                    boost::bind(&PickAndPlaceClient::pose_active_callback, this),
                    boost::bind(&PickAndPlaceClient::pose_feedback_callback, this, _1));

      ROS_INFO_STREAM("[PickAndPlace::pick_and_place] Pose Goal sent. Waiting for the results...");
      pose_action_client_.waitForResult(ros::Duration(30));

      if (pose_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {}
        //ROS_INFO_STREAM("[PickAndPlaceClient::pick_and_place] Pose target Reached successfully.");
    } else {
      const auto& action_data = std::get<gripper_action_data>(action_variant);
      gripper_action_goal_.command.position = action_data.position;
      gripper_action_goal_.command.max_effort = action_data.max_effort;

      //ROS_INFO_STREAM("Waiting for gripper action server...");
      gripper_action_client_.waitForServer();
      //ROS_INFO_STREAM("Connected to gripper action server.");

      gripper_action_client_.sendGoal(gripper_action_goal_,
                    boost::bind(&PickAndPlaceClient::gripper_done_callback, this, _1, _2),
                    boost::bind(&PickAndPlaceClient::gripper_active_callback, this),
                    boost::bind(&PickAndPlaceClient::gripper_feedback_callback, this, _1));

      ROS_INFO_STREAM("[PickAndPlace::pick_and_place] Gripper Goal sent. Waiting for the results...");
      gripper_action_client_.waitForResult(ros::Duration(30));

      if (gripper_action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {}
        //ROS_INFO_STREAM("[PickAndPlaceClient::pick_and_place] Gripper open/close successfully.");
    }
  }
  //ROS_INFO_STREAM("[PickAndPlaceClient] Pick and place completed.");
  ros::shutdown();
}

void PickAndPlaceClient::pose_done_callback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result) {
  //ROS_INFO("[PickAndPlaceClient::pose_done_Callback] Finished in state [%s]", state.toString().c_str());
}

void PickAndPlaceClient::gripper_done_callback(const actionlib::SimpleClientGoalState& state, const control_msgs::GripperCommandResultConstPtr& result) {
  //ROS_INFO("[PickAndPlaceClient::gripper_done_Callback] Finished in state [%s]", state.toString().c_str());
}

void PickAndPlaceClient::pose_active_callback() {
  //ROS_INFO_STREAM("[PoseAndPlaceClient::pose_active_callback] Pose action has become active");
}

void PickAndPlaceClient::gripper_active_callback() {
  //ROS_INFO_STREAM("[PoseAndPlaceClient::active_callback] Gripper action has become active");
}


void PickAndPlaceClient::pose_feedback_callback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback) {
/*  ROS_DEBUG_STREAM("[PickAndPlaceClient::pose_feedback_Callback] distance_translation:"<< feedback->distance_translation);
  ROS_DEBUG_STREAM("[PickAndPlaceClient::pose_feedback_Callback] distance_orientation:"<< feedback->distance_orientation);
  ROS_DEBUG_STREAM("[PickAndPlaceClient::pose_feedback_Callback] elapsed_time:"<< feedback->time_elapsed); */
}

void PickAndPlaceClient::gripper_feedback_callback(const control_msgs::GripperCommandFeedbackConstPtr& feedback) {
  //
}

/* Destructor */
PickAndPlaceClient::~PickAndPlaceClient() {}
