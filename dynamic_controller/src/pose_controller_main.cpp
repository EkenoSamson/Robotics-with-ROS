#include "pose_controller.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle nh;

    TaskSpaceDyn controller(nh);
    ros::Rate loop_rate(controller.publish_rate_);

    while (ros::ok()) {
      ros::spinOnce();
      controller.computeDynamics();
      loop_rate.sleep();
    }

    return 0;
}