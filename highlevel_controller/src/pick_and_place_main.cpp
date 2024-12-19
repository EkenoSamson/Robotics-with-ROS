#include <pick_and_place_client.hpp>

/**
* main - entry point for the pick and place action client node
* @argc: command line arguments size
* @argv: command line arguments array
* Return: 0
*/


int main(int argc, char **argv) {
    ros::init(argc, argv, "pick_and_place_client");
    ros::NodeHandle nh;

    PickAndPlaceClient client(nh);

    ros::Rate loop_rate(500);
    while (ros::ok()) {
        ros::spinOnce();
        client.pick_and_place();
        loop_rate.sleep();
    }
    return 0;
}