#include <action_client.hpp>

/**
* main - entry point for the action client node
* @argc: command line arguments size
* @argv: command line arguments array
* Return: 0
*/


int main(int argc, char **argv) {
  ros::init(argc, argv, "action_client");
  ros::NodeHandle nh;

  ActionClient action_client(nh);

  ros::Rate loop_rate(500);
  while (ros::ok()) {
    ros::spinOnce();
    action_client.update();
    loop_rate.sleep();
  }
  return 0;
}