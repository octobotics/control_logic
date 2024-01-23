#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_pose_publisher_node");
    ros::NodeHandle nh;

    // Publish the value 250 to the /servo_pose topic
    ros::Publisher servoPosePub = nh.advertise<std_msgs::Int32>("/servo_pose", 1);
    std_msgs::Int32 servoPoseMsg;
    servoPoseMsg.data = 255;

    
    ros::Duration(1.0).sleep();

    // Print a debug message
    ROS_INFO("Publishing value %d to /servo_pose", servoPoseMsg.data);

    // Publish the message
    servoPosePub.publish(servoPoseMsg);

    // Print another debug message
    ROS_INFO("Message published");

    // Add a brief delay to allow time for the message to be published
    ros::Duration(1.0).sleep();  // Adjust the duration as needed

    return 0;
}