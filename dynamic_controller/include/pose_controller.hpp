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
    bool with_orientation_;				// whether to control orientation

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
    geometry_msgs::Pose ee_fbk_pose;
    geometry_msgs::Twist ee_fbk_twist;
    std_msgs::Float64MultiArray joint_torque_msg_;

    // Joints
    Eigen::Matrix<double, 7, 1> jts_fbk_positions_;
    Eigen::Matrix<double, 7, 1> jts_fbk_velocities_;
    Eigen::Matrix<double, 7, 1> joint_acceleration_;
    Eigen::Matrix<double, 7, 1> jts_ref_positions_;
    Eigen::Matrix<double, 7, 1> jts_ref_velocities_;

    // End_Effector reference pose and twist
    Eigen::Matrix<double, 3, 1> ee_ref_position_;
    Eigen::Quaternion<double> ee_ref_orientation_;
    Eigen::Matrix<double, 6, 1> ee_ref_twist_;

    //End-effector feedback pose and twist
    Eigen::Matrix<double, 3, 1> ee_fbk_position_;
    Eigen::Quaternion<double> ee_fbk_orientation_;
    Eigen::Matrix<double, 6, 1> ee_fbk_twist_;

    // End-effector accelerations
    Eigen::Matrix<double, 6, 1> ee_ref_acc_;
    Eigen::Matrix<double, 3, 1> ee_ref_acc_trans_;
    Eigen::Matrix<double, 6, 1> ee_acc_cmd_;
    Eigen::Matrix<double, 3, 1> ee_acc_cmd_trans_;

    // End-effector errors
    Eigen::Matrix<double, 6, 1> twist_error_;
    Eigen::Matrix<double, 3, 1> twist_error_trans_;
    Eigen::Matrix<double, 6, 1> pose_error_;
    Eigen::Matrix<double, 3, 1> pose_error_trans_;
    Eigen::Matrix<double, 3, 1> position_error_;
    Eigen::Matrix<double, 3, 1> rotation_error_;
    Eigen::Quaternion<double> orientation_error_;
    Eigen::AngleAxisd angle_axis_error_;



    // Jacobian
    Eigen::Matrix<double, 6, 7> jacobian_;
    Eigen::Matrix<double, 6, 7> jacobian_dot_;
    Eigen::Matrix<double, 7, 6> jacobian_pseudo_inverse_;
    Eigen::Matrix<double, 3, 7> jacobian_trans_;
    Eigen::Matrix<double, 3, 7> jacobian_dot_trans_;
    Eigen::Matrix<double, 7, 3> jacobian_pseudo_inverse_trans_;


    // Forces for Task space
    Eigen::Matrix<double, 6, 6> lambda_;			// Task-space Mass Matrix (for both position and orientation)
    Eigen::Matrix<double, 3, 3> lambda_trans_;      // for translation only
    Eigen::Matrix<double, 6, 1> eta_;				// coriolis, centrifugal and gravity vector in TaskSpace
    Eigen::Matrix<double, 3, 1> eta_trans_;          // for translation only
    Eigen::Matrix<double, 6, 1> wrench_;			// Task-space force
    Eigen::Matrix<double, 3, 1> wrench_trans_;      // translation only
    Eigen::Matrix<double, 7, 1> tau_task_;			// Task Joint Torque
    Eigen::Matrix<double, 7, 1> tau_joint_;
    Eigen::Matrix<double, 7, 1> tau_null_;
    Eigen::Matrix<double, 7, 1> tau_total_;

    // Mass Matrix, (Coriolis and Centrifugal force and gravity vector)
    Eigen::Matrix<double, 7, 7> M_;
    Eigen::Matrix<double, 7, 1> h_;

    // Null space Redundancy
    Eigen::Matrix<double, 7, 7> P_;				// Projection Matrix
    Eigen::Matrix<double, 7, 7> I_;				// Identity Matrix

};


#endif /* POSE_CONTROLLER_HPP */