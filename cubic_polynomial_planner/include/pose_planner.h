// Created by ekeno on 25/09/24.
// pose_planner.h

#ifndef POSE_PLANNER_H
#define POSE_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/MoveTo.h>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <highlevel_msgs/MoveTo.h>

class PosePlanner
{
public:
    // Constructor
    PosePlanner();

    // Callback function for pose subscription
    void pose_callback(const geometry_msgs::Pose::ConstPtr &initial_pose);

    // Callback function for the move_to service
    bool move_callback(highlevel_msgs::MoveTo::Request &req,
                       highlevel_msgs::MoveTo::Response &res);

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

    // Variables for storing time and positions
    double start_time_;
    double current_time_;
    double target_time_;

    Eigen::Vector3d init_pos_;
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d target_pos_;

    Eigen::Quaterniond orient_quat_;
};

#endif // POSE_PLANNER_H

