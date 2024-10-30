#ifndef CUBIC_POLYNOMIAL_PLANNER_HPP
#define CUBIC_POLYNOMIAL_PLANNER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <highlevel_msgs/MoveTo.h>
#include <Eigen/Dense>
#include <string>



class CUBIC
{
    public:
        // Constructor:Initialise or/and Destructor: destroy the class
        CUBIC(ros::NodeHandle& nh);                                //  initializes ROS NodeHandle and any necessary setup
        ~CUBIC();                                                  //  cleans up resources

        // Callback functions
        void poseCallback(const geometry_msgs::Pose::ConstPtr& msg);    // subscribe to current pose message
        bool moveToCallback(highlevel_msgs::MoveTo::Request &req,
                            highlevel_msgs::MoveTo::Response &res);       // service for moving

        // Helper functions
        void update();                                           // Do the calculation for trajectory
        bool readParameters();                                   // Handle Paramaters
        void pubDefaultTranslation();							 // Handle Default pose

        // Global variables

        double duration_ = 0.0;                                 // How long should the flight/moving take?
        double publish_rate_;                                   // Publish_rate
        Eigen::Matrix<double, 3, 1> default_translation_;		// Linear default translation
        bool target_received_ = false;


    private:
        ros::NodeHandle nh_;                             // ROS NodeHandle

        // Subscribers, Services, Publisher
        ros::Subscriber current_pose_sub_;                //Subscriber ground_truth pose (current pose)
        ros::ServiceServer move_to_srv_;                  // Advertises service of moving
        ros::Publisher pose_pub_;						  // Publish the pose command
        ros::Publisher twist_pub_;						  // Publish the twist command


        // Varible for storing received messages(current position, target position, starting position)
        Eigen::Vector3d current_pose_;                  // store the current pose from ground truth
        Eigen::Vector3d target_pose_;                   // store the target pose from the user
        Eigen::Vector3d starting_pose_;                  // starting position
        geometry_msgs::Quaternion current_orient_;

        Eigen::Vector3d move_to_;
        Eigen::Vector3d moving_vel_;

        // Time variables
        double starting_time_;                           // What is the starting time ?
        double current_time_;                            // What is the current time ?


        // Scaling factors
        double pose_scaling_;
        double twist_scaling_;



        // Storage after planning and before publishing
        geometry_msgs::Twist twist_;
        geometry_msgs::Pose pose_;

        // Topic names for parameter server
        std::string feedback_pose_topic_;                 // Topic for feedback_pose
        std::string reference_pose_topic_;                // Topic for reference pose
        std::string reference_twist_topic_;               // Topic for reference twist

};

#endif /*CUBIC_POLYNOMIAL_PLANNER_HPP*/