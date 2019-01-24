#include <math.h>
#include <stdio.h>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "motion_control/Mobility.h"

struct DriveValues{
    double front_left;
    double front_right;
    double back_left;
    double back_right;
};
DriveValues ConstructDriveValues(double fl, double fr, double bl, double br) {
    DriveValues d;
    d.front_left = fl;
    d.front_right = fr;
    d.back_left = bl;
    d.back_right = br;
    return d;
}    
DriveValues MultiplyDriveValues(DriveValues dv, double multiplier) {
        dv.front_left *= multiplier;
        dv.front_right *= multiplier;
        dv.back_left *= multiplier;
        dv.back_right *= multiplier;
        return dv;
}
DriveValues ClampDriveValues (DriveValues dv, double cap){
    //Get the max out of the four values
    double max = dv.front_left;
    if (dv.front_right > max) {
        max = dv.front_right;
    }
    if (dv.back_left > max) {
        max = dv.back_left;
    }
    if (dv.back_right > max) {
        max = dv.back_right;
    }
    //Reduce all values if they exceed the cap
    if (max > cap) {
        double ratio = cap / max;
        dv = MultiplyDriveValues(dv, ratio);
    }
    return dv;
}
motion_control::Mobility DriveValuesToMsg (DriveValues dv) {
    motion_control::Mobility msg;
    msg.front_left = dv.front_left;
    msg.front_right = dv.front_right;
    msg.rear_left = dv.back_left;
    msg.rear_right = dv.back_right;
    return msg;
}
struct DriveCommand {
    DriveValues WheelPower;
    DriveValues WheelAngles;
};

class Wheel {
    private:
    public:
    //+X is forward 
    //+Y is left
    double XPosition, YPosition;
    Wheel(double x, double y) {
        XPosition = x;
        YPosition = y;
    }
    Wheel() {
        Wheel(0,0);
    }
    //Gets the angle the wheel needs to be deviated 
    //from zero position, positive values correspond to 
    //counter-clockwise roatation when robot is viewed 
    //from above
    double getAngle(double centerOffset) {
        double deg = atan2(XPosition, centerOffset-YPosition)*180/M_PI;
        if (centerOffset < 0) {
            deg -= 180;
        }
        return deg;
    }
    //Gets an adjustment to be applied to the throttle on 
    //the wheel to prevent slippage 
    double getRadiusOfTurn(double centerOffset) {
        return sqrt(pow(XPosition,2) + pow(centerOffset-YPosition,2));
    }

};
class Wheelbase {
    private:
    Wheel front_left, front_right, back_left, back_right;
    public:
    Wheelbase() {
        front_left = Wheel(0.5,0.35);
        front_right = Wheel(0.5,-0.35);
        back_left = Wheel(-0.5,0.35);
        back_right = Wheel(-0.5,-0.35);
    }
    
    DriveValues CalculateRatios(double centerOffset){
        DriveValues ratios;

        //Calculating radius on each wheel
        ratios.front_left   = front_left.getRadiusOfTurn(centerOffset);
        ratios.front_right  = front_right.getRadiusOfTurn(centerOffset);
        ratios.back_left    = back_left.getRadiusOfTurn(centerOffset);
        ratios.back_right   = back_right.getRadiusOfTurn(centerOffset);

        //Summing all the radii for wheel power distribution
        double total = (ratios.front_left + ratios.front_right + ratios.back_left + ratios.back_right);

        //Converting all radii into power ratios
        ratios = MultiplyDriveValues(ratios, 4/total);

        return ratios;
    }
    //Calculates the angles to set wheels to
    DriveValues CalculateAngles(double centerOffset) {
        return ConstructDriveValues(front_left.getAngle(centerOffset),
                                    front_right.getAngle(centerOffset), 
                                    back_left.getAngle(centerOffset), 
                                    back_right.getAngle(centerOffset)
                                    );
    }
    DriveCommand TwistToMotor (double throttle, double steer, double minRadius) {
        DriveCommand command;
        if (fabs(steer) <= 0.005) {
            command.WheelAngles = ConstructDriveValues(0,0,0,0);
            command.WheelPower = MultiplyDriveValues(ClampDriveValues(ConstructDriveValues(throttle,throttle,throttle,throttle),1), throttle);
            return command;
        } 
        double radius = minRadius / steer;
        command.WheelAngles = CalculateAngles(radius);
        command.WheelPower = CalculateRatios(radius);
        return command;
    }

};

/*
void PrintDriveValues (DriveValues dv) {
    printf("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n", dv.front_left, dv.front_right, dv.back_left, dv.back_right);
}
*/
Wheelbase wb;
ros::NodeHandle n;
ros::Publisher steerpub;
ros::Publisher drivepub;
void UpdateDrive (const geometry_msgs::Twist::ConstPtr& msg) {
    DriveCommand command = wb.TwistToMotor(msg->linear.x, msg->angular.z, 1);
    steerpub.publish(DriveValuesToMsg(command.WheelAngles));
    drivepub.publish(DriveValuesToMsg(command.WheelPower));
}
int main(int argc, char **argv) {
    wb = Wheelbase();
    ros::init(argc, argv, "M5");

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, UpdateDrive);
    steerpub = n.advertise<motion_control::Mobility>("steering", 1000);
    drivepub = n.advertise<motion_control::Mobility>("odrive_vel", 1000);
    ros::spin();
}