#include "cubic_polynomial_planner.hpp"

/*
 * main - Entry Point
 * Description: Node that spins at 500Hz
 * Author: Ekeno
 ***********************************************
 * @argc: number of arguments passed to the node
 * @argv: pointer to the arguments' array.
 * Return: 0 on success
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_planner");      // Initialize the ROS node with a name, e.g "pose_planner"
    ros::NodeHandle nh;                         // Create a NodeHandle; it is the main access point to communicate with ROS
    CUBIC planner(nh);                          // Instiance of the the class CUBIC

    ros::Rate loop_rate(planner.publish_rate_);                   // Set the loop rate to 500Hz (500 times per second)

    while (ros::ok())
    {
        ros::spinOnce();                        // Processes/tiggers callbacks
        if (planner.duration_ > 0.0) {
            planner.update();
        }
        loop_rate.sleep();                      // Sleep to maintain the loop rate of 500Hz
    }

    return 0;                                   // Exit the program gracefully
}