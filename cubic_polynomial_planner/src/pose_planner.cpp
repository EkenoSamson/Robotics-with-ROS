// pose_planner.cpp
// created by Ekeno

#include "pose_planner.hpp"
using namespace std;

PosePlanner::PosePlanner()
{
    // Node service, publisers and subscribers
    service_ = nh_.advertiseService("pose_planner/move_to", &PosePlanner::move_callback, this);

    // Update publisher topics to Gen3 reference topics
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/gen3/reference/pose", 2);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/gen3/reference/twist", 2);

    // Subscribe to feedback from Gen3
    subscriber_ = nh_.subscribe("/gen3/feedback/pose", 2, &PosePlanner::pose_callback, this);

    ROS_INFO_STREAM("Node initialised.");
}

bool PosePlanner::move_callback(highlevel_msgs::MoveTo::Request  &req, highlevel_msgs::MoveTo::Response &res)
{
    // Prevent the robot from crashing into the ground
    if (req.z <= 0)
    {
        res.success = false;
        return false;
    }
    else
    {
        terminal_pos_ << req.x, req.y, req.z;
        start_time_ = ros::Time::now().toSec();
        start_pos_ = current_pos_;
        terminal_time_ = req.T;
        res.success = true;
        return true;
    }
}

void PosePlanner::pose_callback(const geometry_msgs::Pose::ConstPtr &current_pose)
{
    current_time_ = ros::Time::now().toSec() - start_time_;
    current_time_ = clamp(current_time_, 0.0, terminal_time_);
    current_pos_ << current_pose->position.x,
                    current_pose->position.y,
                    current_pose->position.z;

    orient_quat_ = Eigen::Quaterniond(    current_pose->orientation.w,
                                          current_pose->orientation.x,
                                          current_pose->orientation.y,
                                          current_pose->orientation.z);
}

void PosePlanner::update_pose()
{
    // Calculate the position scaling factor and the translation
    position_scaling_factor_ = ((3 * pow(current_time_, 2)) / pow(terminal_time_, 2))
                               - ((2 * pow(current_time_, 3)) / pow(terminal_time_, 3));
    move_to_ = start_pos_ + (position_scaling_factor_ * (terminal_pos_ - start_pos_));

    // Calculate the velocity scaling factor and linear velocity
    velocity_scaling_factor_ = ((6 * current_time_) / pow(terminal_time_, 2))
                               - ((6 * pow(current_time_, 2)) / pow(terminal_time_, 3));
    linear_velocity = velocity_scaling_factor_ * (terminal_pos_ - start_pos_);

    // Assign linear velocity to twist message
    twist.linear.x = linear_velocity.x();
    twist.linear.y = linear_velocity.y();
    twist.linear.z = linear_velocity.z();

    // Assign angular velocity to twist message (keeping it constant)
    twist.angular.x = angular_velocity.x();
    twist.angular.y = angular_velocity.y();
    twist.angular.z = angular_velocity.z();

    // Assign position to pose message
    pose.position.x = move_to_.x();
    pose.position.y = move_to_.y();
    pose.position.z = move_to_.z();

    // Assign orientation to pose message
    pose.orientation.x = orient_quat_.x();
    pose.orientation.y = orient_quat_.y();
    pose.orientation.z = orient_quat_.z();
    pose.orientation.w = orient_quat_.w();

    // Publish the messages
    twist_pub_.publish(twist);
    pose_pub_.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_planner");        // Initializing pose_planner
    PosePlanner planner_node;                    // Instance of the node object

    int publish_rate;
    planner_node.nh_.getParam("/publish_rate", publish_rate);
    ros::Rate loop_rate(publish_rate);             // Loop rate from ROS param

    while (ros::ok())
    {
        ros::spinOnce();
        planner_node.update_pose();
        loop_rate.sleep();
    }

    return 0;
}
