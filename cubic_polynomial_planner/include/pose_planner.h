// Created by ekeno on 25/09/24.
// pose_planner.h

#ifndef POSE_PLANNER_H
#define POSE_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/MoveTo.h>
#include <Eigen/Dense>


class PosePlanner
{
public:
    // Constructor
    PosePlanner();

    // Callback function for pose subscription
    void pose_callback(const geometry_msgs::Pose::ConstPtr &current_pose_);

    // Callback function for the move_to service
    bool move_callback(highlevel_msgs::MoveTo::Request &req, highlevel_msgs::MoveTo::Response &res);

    // Function to update and publish the pose
    void update_pose();

private:
    // NodeHandle for ROS
    ros::NodeHandle nh_;

    // ROS publishers, subscribers, and service server
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::Subscriber subscriber_;
    ros::ServiceServer service_;

    // Time variables
    double start_time_;
    double current_time_;
    double terminal_time_;
    double position_scaling_factor_;
    double velocity_scaling_factor_;

    // Position vectors (frame)
    Eigen::Vector3d start_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d move_to_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d terminal_pos_ = Eigen::Vector3d::Zero();

    // Orientation
    Eigen::Quaterniond orient_quat_;

    // Velocities
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_velocity;

    // Publishers
    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;
};

#endif // POSE_PLANNER_H

