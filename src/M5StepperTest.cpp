
#include <stdio.h>
#include <ros/ros.h>

#include <queue>

#include "geometry_msgs/Twist.h"
#include "motion_control/Mobility.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "M5StepperTest");
    ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    usleep(1000*1000);
    while (ros::ok()) {
        for (float y = -1; y <= 1; y+= 0.125) {
            printf("%f\n", y);
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = y;
            cmd_pub.publish(msg);
            ros::spinOnce();
            usleep(100*1000);
        }
        for (float y = 1; y >= -1; y-= 0.125) {
            printf("%f\n", y);
            geometry_msgs::Twist msg;
            msg.linear.x = 0;
            msg.angular.z = y;
            cmd_pub.publish(msg);
            ros::spinOnce();
            usleep(100*1000);
        }
    }
}