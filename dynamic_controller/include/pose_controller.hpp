#ifndef POSE_CONTROLLER_HPP
#define POSE_CONTROLLER_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <pinocchio/math/rpy.hpp>

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <highlevel_msgs/SetKD.h>

class TaskSpaceDyn {
  public:
    // constructor and initiator
    TaskSpaceDyn(ros::NodeHandle& nh);
    void init(std::string urdf_file_name);

    // Helper functions
    bool readParameters();				// reading ROS parameters from the server
    void computeDynamics();				// compute task space dynamics
    void pubEndFeedback();				// publish the pose and twist of the end-effector

    // Parameters
    double publish_rate_;				// publish_rate
    std::string urdf_file_name_;		// URDF file
    bool with_redundancy_;				// whether redundancy or not
  	double end_stiffness_;				// end-effector stiffness
    double end_damping_;				// end-effector damping
    double joints_stiffness_;			// joints stiffness value
  	double joints_damping_;				// joints damping value

    // Callback functions for pose_controller
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void referencePositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void referenceVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void referencePoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void referenceTwistCallback(const geometry_msgs::Twist::ConstPtr& msg);

    // Service Callbacks
    bool setLinearPDServiceCallback(highlevel_msgs::SetKD::Request &req, highlevel_msgs::SetKD::Response &res);
    bool setJointPDServiceCallback(highlevel_msgs::SetKD::Request &req, highlevel_msgs::SetKD::Response &res);

  private:
    ros::NodeHandle nh_;

    // pinocchio
    pinocchio::Model model_;
    pinocchio::Data data_;
    int hand_id_;
    int num_joints_;

    // ROS Subscirbers
    ros::Subscriber joint_states_sub_;
    ros::Subscriber reference_position_sub_;
    ros::Subscriber reference_velocity_sub_;
    ros::Subscriber reference_pose_sub_;
    ros::Subscriber reference_twist_sub_;

    // ROS publisher
    ros::Publisher joint_effort_pub_;
    ros::Publisher end_effector_pose_pub_;
    ros::Publisher end_effector_twist_pub_;

    // ROS Services
    ros::ServiceServer set_linear_pd_service_;
    ros::ServiceServer set_joint_pd_service_;

    // ROS parameters for topics
  	std::string joint_states_topic_;
  	std::string reference_position_topic_;
  	std::string reference_velocity_topic_;
    std::string reference_pose_topic_;
    std::string reference_twist_topic_;
  	std::string joint_effort_command_topic_;
    std::string feedback_pose_topic_;
    std::string feedback_twist_topic_;

    // ROS Message
    std_msgs::Float64MultiArray joint_torque_msg_;

    // Joints
    Eigen::Matrix<double, 7, 1> joint_positions_;
    Eigen::Matrix<double, 7, 1> joint_velocities_;
    Eigen::Matrix<double, 7, 1> reference_positions_;
    Eigen::Matrix<double, 7, 1> reference_velocities_;

    // End_effector
    Eigen::Matrix<double, 3, 1> reference_pose_;
    Eigen::Matrix<double, 3, 1> reference_twist_;
    Eigen::Matrix<double, 3, 1> end_effector_pose_;
    Eigen::Matrix<double, 3, 1> end_effector_twist_;

    // Jacobian
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_dot_;
    Eigen::MatrixXd jacobian_pseudo_inverse_;

    // Mass Matrix, (Coriolis and Centrifugal force and gravity vector)
    Eigen::MatrixXd M_;
    Eigen::VectorXd h_;

};


#endif /* POSE_CONTROLLER_HPP */