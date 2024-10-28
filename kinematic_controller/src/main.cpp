#include "kinematic_controller.hpp"

/*
 * main - Entry Point
 * Description: kinematic_controller Node
 * Author: Ekeno
 ***********************************************
 * @argc: number of arguments passed to the node
 * @argv: pointer to the arguments' array.
 * Return: 0 on success
 */

int main(int argc, char **argv)
{
    // Initialize the ROS node with the name "inverse_kinematic_controller"
    ros::init(argc, argv, "inverse_kinematic_controller");
    ros::NodeHandle nh("~");                              // Private node handle for parameters/ subscribers/publishers...

    // Create the kinematic controller object
    KIN kin_controller(nh);

    // Initialize the kinematic controller by loading the Pinocchio model
    if (!kin_controller.init()) {
        ROS_ERROR("Failed to initialize KIN controller");
        return -1;
    }

    // Set the loop rate for the control loop (based on the publish rate)
    ros::Rate loop_rate(kin_controller.publish_rate_);

    // Main control loop
    while (ros::ok())
    {

        // Process any ROS callbacks (e.g., joint state updates)
        ros::spinOnce();

        // Perform kinematic updates and publish results
        kin_controller.update();

        // Sleep to maintain the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}