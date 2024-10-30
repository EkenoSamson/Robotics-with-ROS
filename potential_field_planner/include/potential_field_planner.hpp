#ifndef POTENTIAL_FIELD_PLANNER_HPP
#define POTENTIAL_FIELD_PLANNER_HPP

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <string>



class PotF
{
  public:
    // constructor and destructor
    PotF(ros::NodeHandle& nh);



    // CallBacks
    void jointStatesCallBack(const sensor_msgs::JointState::ConstPtr& msg);

    // Helper functions
    bool readParameters();
    void computePotentialField();
    void publishJointReferences();


    // Public variables
    double publish_rate_;
    Eigen::Matrix<double, 7, 1> default_;        // Default joint positions (7 joints)
    double k_att_;                              // Attractive potential gains (7 joints)
    double max_velocity_;                       // Maximum joint velocities (7 joints)


    // flags
    bool received_joint_states_ = false;


  private:
    ros::NodeHandle nh_;

    // Subscribers
    ros::Subscriber joint_states_sub_;

    // Publishers
    ros::Publisher reference_position_pub_;
    ros::Publisher reference_velocity_pub_;

    // Joint positions and velocities
    Eigen::Matrix<double, 7, 1> joint_positions_;
    Eigen::Matrix<double, 7, 1> joint_velocities_;
    Eigen::Matrix<double, 7, 1> delta_position_;

    // reference positions and velocities (to be published)
    Eigen::Matrix<double, 7, 1> reference_positions_;
    Eigen::Matrix<double, 7, 1> reference_velocities_;
    double velocity_magnitude;

    // Subscribers and Publishers topics
    std::string joint_states_sub_topic_;
    std::string reference_position_topic_;
    std::string reference_velocity_topic_;

};

#endif /* POTENTIAL_FIELD_PLANNER_HPP */