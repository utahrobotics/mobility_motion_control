
#include <stdio.h>
#include <ros/ros.h>

#include <queue>

#include "geometry_msgs/Twist.h"
#include "motion_control/Mobility.h"

std::queue<std::string> inputs;
std::queue<std::string> steers;
std::queue<std::string> drives;
void QueueMessages(std::string message, int index) {
    switch (index)
    {
        case 0:
            inputs.push(message);
            break;
        case 1:
            steers.push(message);
            break;
        case 2:
            drives.push(message);
            break;
    
        default:
            break;
    }
    if (inputs.size() > 0 && steers.size() > 0 && drives.size() > 0) {
        printf("%s%s%s\n", inputs.front().c_str(), steers.front().c_str(), drives.front().c_str());
        inputs.pop();
        steers.pop();
        drives.pop();
    }
}
void UpdateSteer (const motion_control::Mobility::ConstPtr& msg) {
    char buffer[100];
    sprintf(buffer, "Steering: \tFL: %.2f, \tFR: %.2f, \tBL: %.2f, \tBR: %.2f\n", msg->front_left, msg->front_right, msg->rear_left, msg->rear_right);
    QueueMessages(buffer, 1);
}
void UpdateDrive (const motion_control::Mobility::ConstPtr& msg) {
    char buffer[100];
    sprintf(buffer,"Drive: \t\tFL: %.2f, \tFR: %.2f, \tBL: %.2f, \tBR: %.2f\n", msg->front_left, msg->front_right, msg->rear_left, msg->rear_right);
    QueueMessages(buffer, 2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "M5Test");
    ros::NodeHandle n;
    //ros::Time::init();
    ros::Subscriber steersub = n.subscribe("steering", 1000, UpdateSteer);
    ros::Subscriber drivesub = n.subscribe("odrive_vel", 1000, UpdateDrive);
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    usleep(1000*1000);
    for (float x = -1; x <= 1.05; x+=0.1) {
        for (float y = -1; y <= 1.05; y+= 0.1) {
            char buffer[100];
            sprintf(buffer,"Inputs: \tX: %.1f, \tY: %.1f\n",x,y);
            QueueMessages(buffer, 0);
            geometry_msgs::Twist msg;
            msg.linear.x = x;
            msg.angular.z = y;
            cmd_pub.publish(msg);
            ros::spinOnce();
        }
    }
    ros::spin();
}