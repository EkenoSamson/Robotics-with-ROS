// pose_planner.cpp
// created by Ekeno

#include "pose_planner.h"
using namespace std;

PosePlanner::PosePlanner()
{
    target_time_ = 1.0; 								// Set to a valid initial value
    target_pos_ = Eigen::Vector3d::Zero();
    init_pos_ = Eigen::Vector3d::Zero();
    current_pos_ = Eigen::Vector3d::Zero();
    orient_quat_ = Eigen::Quaterniond::Identity();

    service_ = nh_.advertiseService("pose_planner/move_to", &PosePlanner::move_callback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/firefly/command/pose", 2);
    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/firefly/command/twist", 2);
    subscriber_ = nh_.subscribe("/firefly/ground_truth/pose", 2, &PosePlanner::pose_callback, this);

    ROS_INFO_STREAM("Node initialised.");
}

bool PosePlanner::move_callback(highlevel_msgs::MoveTo::Request  &req, highlevel_msgs::MoveTo::Response &res)
{
    if (req.z <= 0) {
        res.success = false;
        ROS_INFO_STREAM("Cannot move to zero, drone will crash :/");
        return false;
    } else {
        target_pos_(0) = req.x;
        target_pos_(1) = req.y;
        target_pos_(2) = req.z;
        start_time_ = ros::Time::now().toSec();  // Save the start time in seconds
        ROS_INFO("Success.");
        res.success = true;
        return true;
    }
}

void PosePlanner::pose_callback(
    const geometry_msgs::Pose::ConstPtr &initial_pose)
{
    current_time_ = ros::Time::now().toSec() - start_time_;
    current_time_ = clamp(current_time_, 0.0, target_time_);

    init_pos_(0) = initial_pose->position.x;
    init_pos_(1) = initial_pose->position.y;
    init_pos_(2) = initial_pose->position.z;

    orient_quat_.coeffs()(0) = initial_pose->orientation.x;
    orient_quat_.coeffs()(1) = initial_pose->orientation.y;
    orient_quat_.coeffs()(2) = initial_pose->orientation.z;
    orient_quat_.coeffs()(3) = initial_pose->orientation.w;
}

void PosePlanner::update_pose()
{
    double time_deriv;
    double time_step;
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d linear_velocity;
    geometry_msgs::Twist twist;
    geometry_msgs::Pose pose;

    time_step = ((3 * pow(current_time_, 2)) / pow(target_time_, 2)) -
                ((2 * pow(current_time_, 3)) / pow(target_time_, 3));

    current_pos_ = init_pos_ + (time_step * (target_pos_ - init_pos_));

    time_deriv = ((6 * current_time_) / pow(target_time_, 2)) -
                 ((6 * pow(current_time_, 2)) / pow(target_time_, 3));
    linear_velocity = time_deriv * (target_pos_ - init_pos_);

    twist.linear.x = linear_velocity(0);
    twist.linear.y = linear_velocity(1);
    twist.linear.z = linear_velocity(2);
    twist.angular.x = angular_velocity(0);
    twist.angular.y = angular_velocity(1);
    twist.angular.z = angular_velocity(2);

    pose.position.x = current_pos_(0);
    pose.position.y = current_pos_(1);
    pose.position.z = current_pos_(2);
    pose.orientation.x = orient_quat_.coeffs()(0);
    pose.orientation.y = orient_quat_.coeffs()(1);
    pose.orientation.z = orient_quat_.coeffs()(2);
    pose.orientation.w = orient_quat_.coeffs()(3);

    twist_pub_.publish(twist);
    pose_pub_.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_planner");
    PosePlanner planner_node;
    ros::Rate loop_rate(500); // 500 Hz

    while (ros::ok())
    {
        ros::spinOnce();
        planner_node.update_pose();
        loop_rate.sleep();
    }

    return 0;
}
