#include "joint_controller.hpp"

int main(int argc, char **argv)
{
  // intiate the node
  ros::init(argc, argv, "joint_controller");
  ros::NodeHandle nh;

  //Instance of class DYN
  JointSpaceDyn controller(nh);

  ros::Rate loop_rate(controller.publish_rate_);

  // Loop
  while (ros::ok()) {
    ros::spinOnce();
    controller.computeDynamics();
    loop_rate.sleep();
  }

  return 0;
}