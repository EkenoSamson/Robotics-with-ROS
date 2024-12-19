#ifndef ACTION_CLIENT_HPP
#define ACTION_CLIENT_HPP

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <Eigen/Dense>

class ActionClient {
  public:
    ActionClient(ros::NodeHandle& node_handle);
    virtual ~ActionClient();

    // Helper functions
    void update();

  protected:
    void active_callback();
    void feedback_callback(const highlevel_msgs::PoseCommandFeedbackConstPtr& feedback);
    void done_callback(const actionlib::SimpleClientGoalState& state, const highlevel_msgs::PoseCommandResultConstPtr& result);

  private:
    ros::NodeHandle node_handle_;

    // client
    actionlib::SimpleActionClient<highlevel_msgs::PoseCommandAction> move_to_action_client_;
    highlevel_msgs::PoseCommandGoal move_to_goal_;

    // helper functions
    bool read_parameters();

    // variables for target
    int NUM_TARGETS_;
    Eigen::MatrixXd target_translation_;
    Eigen::MatrixXd target_orientation_;
    Eigen::VectorXd target_duration_;
    std::string action_param_name_;

};


#endif