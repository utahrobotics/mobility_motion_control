#include <math.h>
#include <stdio.h>
#include <ros/ros.h>

#include "motion_control/RoboclawCommand.h"
#include "std_msgs/Float32.h"

ros::Publisher commandpub;

float m1 = 0;
float m2 = 0;
void SendMessage () {
    motion_control::RoboclawCommand cmd;
    cmd.M1 = m1;
    cmd.M2 = m2;
    commandpub.publish(cmd);
}
void UpdateLinear (const std_msgs::Float32::ConstPtr& msg) {
    m1 = msg->data;
    SendMessage();
}

void UpdateDigger (const std_msgs::Float32::ConstPtr& msg) {
    m2 = msg->data;
    SendMessage();
}

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "DiggerControl");
    ros::NodeHandle n;

    ros::Subscriber sub1 = n.subscribe("linear_rc", 1000, UpdateLinear);
    ros::Subscriber sub2 = n.subscribe("digger_rc", 1000, UpdateDigger);

    commandpub = n.advertise<motion_control::RoboclawCommand>("cmd_roboclaw", 1000);

    ros::spin();
}