// pose_planner.cpp
// created by Ekeno
#include "pose_planner.h"
using namespace std;

PosePlanner::PosePlanner()
{
    // Node service, publisers and subscribers
    service_ = nh_.advertiseService("pose_planner/move_to", &PosePlanner::move_callback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/firefly/command/pose", 2);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/firefly/command/twist", 2);
    subscriber_ = nh_.subscribe("/firefly/ground_truth/pose", 2, &PosePlanner::pose_callback, this);

    ROS_INFO_STREAM("Node initialised.");
}

bool PosePlanner::move_callback(highlevel_msgs::MoveTo::Request  &req, highlevel_msgs::MoveTo::Response &res)
{
	// ensuring the drone don't crush
    if (req.z <= 0) {
        res.success = false;
        return false;
    } else {
      	terminal_pos_ << req.x, req.y, req.z;
        start_time_ = ros::Time::now().toSec();
        terminal_time_ = req.T;
        res.success = true;
        return true;
    }
}

void PosePlanner::pose_callback(const geometry_msgs::Pose::ConstPtr &current_pose)
{
    current_time_ = ros::Time::now().toSec() - start_time_;
    current_time_ = clamp(current_time_, 0.0, terminal_time_);
    start_pos_ << current_pose->position.x,
        			current_pose->position.y,
        			current_pose->position.z;

	orient_quat_ = Eigen::Quaterniond(	current_pose->orientation.w,
    									current_pose->orientation.x,
    									current_pose->orientation.y,
    									current_pose->orientation.z);
}

void PosePlanner::update_pose()
{
    // calculating the position scaling factor and the translation
    position_scaling_factor_ = ((3 * pow(current_time_, 2)) / pow(terminal_time_, 2))
                             - ((2 * pow(current_time_, 3)) / pow(terminal_time_, 3));
    move_to_ = start_pos_ + (position_scaling_factor_ * (terminal_pos_ - start_pos_));

    //calculating the velocity scaling factor and linear_velocity
    velocity_scaling_factor_ = ((6 * current_time_) / pow(terminal_time_, 2))
                             - ((6 * pow(current_time_, 2)) / pow(terminal_time_, 3));
    linear_velocity = velocity_scaling_factor_ * (terminal_pos_ - start_pos_);

    // assigning linear velocity to twist message
    twist.linear.x = linear_velocity.x();
    twist.linear.y = linear_velocity.y();
    twist.linear.z = linear_velocity.z();

    // assigning angular velocity to twist message
    twist.angular.x = angular_velocity.x();
    twist.angular.y = angular_velocity.y();
    twist.angular.z = angular_velocity.z();

	// assigning position to pose message
    pose.position.x = move_to_.x();
    pose.position.y = move_to_.y();
    pose.position.z = move_to_.z();

    // assigning orientation to pose message
    pose.orientation.x = orient_quat_.x();
    pose.orientation.y = orient_quat_.y();
    pose.orientation.z = orient_quat_.z();
    pose.orientation.w = orient_quat_.w();

    // publishing the messages
    twist_pub_.publish(twist);
    pose_pub_.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_planner");		// Initializing pose_planner
    PosePlanner planner_node;					// Instance of the node object
    ros::Rate loop_rate(500); 					// 500 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        planner_node.update_pose();
        loop_rate.sleep();
    }

    return 0;
}
