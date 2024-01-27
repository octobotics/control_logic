#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>

ros::Publisher servoPosePub;
ros::ServiceClient motorTriggerClient;
std_msgs::Int32 servoPoseMsg;
bool reductionOccurred = false;

ros::Publisher wireValuePub;
std_msgs::Float32 wireValueMsg;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double z = msg->orientation.z;

    if (z >= 60.0 && z <= 70.0 && !reductionOccurred) {
        
        servoPoseMsg.data -= 25;
        
        servoPosePub.publish(servoPoseMsg);

        
        reductionOccurred = true;

        
        std_srvs::Trigger motorTriggerSrv;
        if (motorTriggerClient.call(motorTriggerSrv)) {
            ROS_INFO("Motor turned on");
        } else {
            ROS_ERROR("Failed to call motor_trig service");
        }
    } else {
        ROS_INFO("False");
    }
}

void wireValueCallback(const std_msgs::Float32::ConstPtr& msg) {
    float wireValue = msg->data;

    /
    if (wireValue >= 0.024 && wireValue <= 0.026) {
       
        std_srvs::Trigger motorTriggerSrv;
        if (motorTriggerClient.call(motorTriggerSrv)) {
            ROS_INFO("Motor turned off");

            
            servoPoseMsg.data += 25;
            
            servoPosePub.publish(servoPoseMsg);
        } else {
            ROS_ERROR("Failed to call motor_trig service");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_subscriber_node");
    ros::NodeHandle nh;

    
    servoPosePub = nh.advertise<std_msgs::Int32>("servo_pose", 1);
    servoPoseMsg.data = 250;

    
    motorTriggerClient = nh.serviceClient<std_srvs::Trigger>("motor_trig");

    
    wireValuePub = nh.advertise<std_msgs::Float32>("wire_value", 1);

    
    ros::Duration(1.0).sleep();

    
    servoPosePub.publish(servoPoseMsg);

    
    ros::Subscriber imu_sub = nh.subscribe("imu_topic", 1, imuCallback);

    
    ros::Subscriber wireValue_sub = nh.subscribe("wire_value", 1, wireValueCallback);

    
    ros::Rate loop_rate(10);  // 10 Hz loop
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
