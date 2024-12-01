#ifndef POSE_COMMAND_SERVER_H
#define POSE_COMMAND_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <highlevel_msgs/PoseCommandAction.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

class ActionServer {
  public:
    ActionServer(ros::NodeHandle& nh);        // constructor
    virtual ~ActionServer();                  // destructor

    // Helper functions
    bool read_parameters();
    void update();



  private:
    ros::NodeHandle node_handle_;
    actionlib::SimpleActionServer<highlevel_msgs::PoseCommandAction> move_to_action_server_;      // instance of SimpleActionServer
    //std::string move_to_planner_;                                              // Name of the action
    highlevel_msgs::PoseCommandGoal move_to_goal_;                   // goal of the action
    highlevel_msgs::PoseCommandFeedback move_to_feedback_;             // feedback of the action
    highlevel_msgs::PoseCommandResult move_to_result_;                 // result of the action

    // Callback function
    void move_to_callback(const highlevel_msgs::PoseCommandGoalConstPtr& goal);
    void feedback_pose_callback(const geometry_msgs::Pose::ConstPtr& feedback_pose_msg);

    // subscribers
    ros::Subscriber feedback_pose_subscriber_;

    // Publisher
    ros::Publisher reference_pose_publisher_;
    ros::Publisher reference_twist_publisher_;

    // Messages to publish
    geometry_msgs::Twist reference_twist_msg_;
    geometry_msgs::Pose reference_pose_msg_;

    // for reading topic parameters
    std::string feedback_pose_sub_topic_;
    std::string reference_pose_pub_topic_;
    std::string reference_twist_pub_topic_;

    // Loop rate
    ros::Rate loop_rate_;

    // current feedback variables
    tf2::Vector3 feedback_position_;
    tf2::Quaternion feedback_orientation_;

    // target variables
    tf2::Vector3 target_position_;
    tf2::Quaternion target_orientation_;
    double target_duration_;
    double starting_time_;

    // interpolation calculation
    tf2::Vector3 starting_position_;
    tf2::Quaternion starting_orientation_;
    tf2::Vector3 planned_position_;
    tf2::Quaternion planned_orientation_;
    tf2::Vector3 planned_velocity_;
    double current_time_;
    double transform_scaling_factor_;
    double velocity_scaling_factor_;

    // Helper functions
    void compute_transformation();

};


#endif