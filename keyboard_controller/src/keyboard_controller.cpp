#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// Callback function for keyboard input
void keyboardCallback(const std_msgs::String::ConstPtr& msg, ros::Publisher pub)
{
    geometry_msgs::Twist twist;
    char key = msg->data[0];  // Get the first character of the message

    // Determine the movement based on the key
    switch (key) {
        case 'i':  // Move forward
            twist.linear.x = 0.5;
            twist.angular.z = 0.0;
            break;
        case 'u':  // Turn left
            twist.linear.x = 0.5;
            twist.angular.z = 0.5;
            break;
        case 'o':  // Turn right
            twist.linear.x = 0.5;
            twist.angular.z = -0.5;
            break;
        default:  // Stop
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            break;
    }

    // Publish the twist message to control the robot
    pub.publish(twist);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "keyboard_controller");
    ros::NodeHandle nh;

    // Publisher for twist messages to control the Husky robot
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    // Subscribe to the keyboard_reader topic
    ros::Subscriber sub = nh.subscribe<std_msgs::String>("/keyboard_reader/cmd", 10, boost::bind(keyboardCallback, _1, twist_pub));

    // Keep the node running
    ros::spin();

    return 0;
}

