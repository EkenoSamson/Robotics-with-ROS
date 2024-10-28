#include <ros/ros.h>
#include "potential_field_planner.hpp"

int main(int argc, char *argv[])
{
    // Initialize the ROS node
    ros::init(argc, argv, "joint_planner");
    ros::NodeHandle nh;

    // Create an instance of the PotF class
    PotF joint_planner(nh);

    // Set up the loop rate based on the publish_rate_ parameter
    ros::Rate loop_rate(joint_planner.publish_rate_);

    // Main control loop
    while (ros::ok())
    {
        // Spin once to process incoming messages (callbacks)
        ros::spinOnce();

        // If we haven't received the first joint states, skip this iteration
        if (!joint_planner.received_joint_states_)
        {
            loop_rate.sleep();
            continue;
        }

        // Compute the potential field and update reference positions and velocities
        joint_planner.computePotentialField();

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
