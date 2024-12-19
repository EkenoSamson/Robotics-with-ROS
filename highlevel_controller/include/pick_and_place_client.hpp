#ifndef PICK_AND_PLACE_CLIENT_HPP
#define PICK_AND_PLACE_CLIENT_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <Eigen/Dense>
#include <variant>


class PickAndPlaceClient {

  public:
    PickAndPlaceClient();
    PickAndPlaceClient(ros::NodeHandle& nh);
    virtual ~PickAndPlaceClient();
    void pick_and_place();

  protected:
    void pose_active_callback();
    void pose_feedback_callback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback);
    void pose_done_callback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result);

    void gripper_active_callback();
    void gripper_feedback_callback(const control_msgs::GripperCommandFeedbackConstPtr& feedback);
    void gripper_done_callback(const actionlib::SimpleClientGoalState& state, const control_msgs::GripperCommandResultConstPtr& result);

  private:
    ros::NodeHandle nh_;

    actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> pose_action_client_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_action_client_;

    highlevel_msgs::PoseCommandGoal pose_action_goal_;
    control_msgs::GripperCommandGoal gripper_action_goal_;

    bool read_parameters();
    int num_of_targets_;

    struct pose_action_data {
      std::string action_type;
      Eigen::Vector3d translation;
      Eigen::Vector3d orientation;
      double duration;
    };

    struct gripper_action_data {
      std::string action_type;
      double position;
      double max_effort;
    };

    using action_variant = std::variant<pose_action_data, gripper_action_data>;
    std::vector<action_variant> actions_;
};

#endif