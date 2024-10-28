#ifndef DYNAMIC_CONTROLLER_HPP
#define DYNAMIC_CONTROLLER_HPP

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>
#include <string>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <highlevel_msgs/SetKD.h>

/************* VERY IMPORTANT: DO NOT ADD ROS BEFORE PINOCCHIO *****************/
#include <ros/ros.h>


class DYN {
public:
  // Constructor and Initiator
  DYN(ros::NodeHandle& nh);
  void init(std::string urdf_file_name_) ;

  // Helper functions
  bool readParameters();
  void computeDynamics();

  // Parameters
  double stiffness_;
  double damping_;
  double publish_rate_;
  std::string urdf_file_name_;

  // Callback functions for joint_controller
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void referencePositionCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void referenceVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  bool setPDServiceCallback(highlevel_msgs::SetKD::Request &req, highlevel_msgs::SetKD::Response &res);


private:
  // Node Handler
  ros::NodeHandle nh_;

  // Publishers
  ros::Publisher joint_effort_pub_;

  // Subscribers
  ros::Subscriber joint_states_sub_;
  ros::Subscriber reference_position_sub_;
  ros::Subscriber reference_velocity_sub_;

  // Service
  ros::ServiceServer set_pd_service_;


  // Parameters
  pinocchio::Model model_;                 // Pinocchio model
  pinocchio::Data data_;                   // Pinocchio data structure
  unsigned int hand_id_;                   // Joint ID for the end-effector
  int dim_joints_;                         // Number of joints

  // Data storage
  Eigen::VectorXd joint_positions_;
  Eigen::VectorXd joint_velocities_;
  Eigen::VectorXd reference_positions_;
  Eigen::VectorXd reference_velocities_;

  // ROS parameters for topics
  std::string joint_states_topic_;
  std::string reference_position_topic_;
  std::string reference_velocity_topic_;
  std::string joint_effort_command_topic_;

  // Jacobian
  Eigen::MatrixXd jacobian_ ;
  Eigen::MatrixXd jacobian_dot_ ;

  // Equation of Motion
  Eigen::MatrixXd M_;
  Eigen::VectorXd h_;

  // Message
  std_msgs::Float64MultiArray joint_torque_msg_;

};

#endif
