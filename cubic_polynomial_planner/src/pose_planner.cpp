#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <highlevel_msgs/MoveTo.h>
#include <Eigen/Dense>

class PosePlanner
{
public:
    PosePlanner(ros::NodeHandle& nh) : nh_(nh), rate_(500.0)
    {
        // Subscribe to the current pose of the robot
        pose_sub_ = nh_.subscribe("/firefly/ground_truth/pose", 10, &PosePlanner::poseCallback, this);
        
        // Advertise the service
        service_ = nh_.advertiseService("pose_planner/move_to", &PosePlanner::moveToCallback, this);
        
        // Publisher for the pose and twist commands
        pose_pub_ = nh_.advertise<geometry_msgs::Pose>("/firefly/command/pose", 10);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/firefly/command/twist", 10);
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::ServiceServer service_;
    ros::Rate rate_;

    geometry_msgs::PoseStamped current_pose_;
    double start_time_;

    // Callback to update the current robot pose
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        current_pose_ = *msg;
    }

    // Service callback for moving the robot to a specified target using cubic polynomial
    bool moveToCallback(highlevel_msgs::MoveTo::Request& req, highlevel_msgs::MoveTo::Response& res)
    {
        // Save the current time and pose
        start_time_ = ros::Time::now().toSec();
        Eigen::Vector3d start_pos(current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
        Eigen::Vector3d target_pos(req.x, req.y, req.z);
        double target_time = req.T;

        // Check for invalid target (e.g., z=0 is invalid)
        if (req.z <= 0)
        {
            ROS_ERROR("Cannot move to target position with z=0 or less.");
            res.success = false;
            return false;
        }

        // Loop to calculate the trajectory and publish at 500Hz
        while (ros::Time::now().toSec() - start_time_ < target_time)
        {
            double t = ros::Time::now().toSec() - start_time_;
            double tau = t / target_time;

            // Cubic polynomial interpolation for translation
            Eigen::Vector3d current_pos = (2 * std::pow(tau, 3) - 3 * std::pow(tau, 2) + 1) * start_pos
                                        + (-2 * std::pow(tau, 3) + 3 * std::pow(tau, 2)) * target_pos;

            // Fill the Pose message
            geometry_msgs::Pose pose_msg;
            pose_msg.position.x = current_pos.x();
            pose_msg.position.y = current_pos.y();
            pose_msg.position.z = current_pos.z();

            // Keep orientation constant (use current orientation)
            pose_msg.orientation = current_pose_.pose.orientation;

            // Publish the Pose message
            pose_pub_.publish(pose_msg);

            // Fill and publish the Twist message (simplified to zero velocity here)
            geometry_msgs::Twist twist_msg;
            twist_pub_.publish(twist_msg);

            ros::spinOnce();
            rate_.sleep();
        }

        // Successfully reached the target
        res.success = true;
        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_planner");
    ros::NodeHandle nh;

    PosePlanner planner(nh);
    planner.spin();

    return 0;
}

