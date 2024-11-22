#include "pose_controller.hpp"

/*
 * main - Entry Point for pose_controller
 * Description: Task-space controller
 * Author: Ekeno
 ***********************************************
 * @argc: number of arguments passed to the node
 * @argv: pointer to the arguments' array.
 * Return: 0 on success
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle nh;

    TaskSpaceDyn controller(nh);
    ros::Rate loop_rate(controller.publish_rate_);

    while (ros::ok()) {
      ros::spinOnce();
      controller.update();
      loop_rate.sleep();
    }

    return 0;
}