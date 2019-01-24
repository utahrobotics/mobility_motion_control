#include <stdio.h>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "motion_control/Mobility.h"

void UpdateSteer (const motion_control::Mobility::ConstPtr& msg) {
    printf("Steering: FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n", msg->front_left, msg->front_right, msg->rear_left, msg->rear_right);
}
void UpdateDrive (const motion_control::Mobility::ConstPtr& msg) {
    printf("Drive: FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n", msg->front_left, msg->front_right, msg->rear_left, msg->rear_right);
}
ros::Rate loop_rate(100);

int main(int argc, char **argv) {
    ros::NodeHandle n;
    ros::init(argc, argv, "M5Test");

    ros::Subscriber steersub = n.subscribe("steering", 1000, UpdateSteer);
    ros::Subscriber drivesub = n.subscribe("odrive_vel", 1000, UpdateDrive);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    for (float x = -1; x <= 1; x+=0.1) {
        for (float y = -1; y <= 1; y+= 0.1) {
            geometry_msgs::Twist msg;
            msg.linear.x = x;
            msg.angular.z = y;
            cmd_pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    ros::Duration(2).sleep();
    ros::shutdown();
}