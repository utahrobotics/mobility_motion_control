#include <math.h>
#include <stdio.h>
#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "motion_control/Mobility.h"

//Used to contain 4 doubles for internal methods
struct DriveValues{
    double front_left;
    double front_right;
    double back_left;
    double back_right;
    DriveValues(double fl, double fr, double bl, double br) : front_left(fl), front_right(fr), back_left(bl), back_right(br) {}
    DriveValues() {}
    DriveValues operator* (double mult) {
        return {front_left*mult, front_right*mult, back_left*mult, back_right*mult};
    };
    DriveValues scaleToMaximum (double cap){
        //Get the max out of the four values
        double max = front_left;
        if (front_right > max) {
            max = front_right;
        }
        if (back_left > max) {
            max = back_left;
        }
        if (back_right > max) {
            max = back_right;
        }
        if (max > cap) {
            double ratio = cap / max;
            return *this * ratio;
        }
        return *this;
    }
    motion_control::Mobility toMessage() {
        motion_control::Mobility msg;
        msg.front_left = front_left;
        msg.front_right = front_right;
        msg.rear_left = back_left;
        msg.rear_right = back_right;
        return msg;
    }
};

struct DriveCommand {
    DriveValues WheelAngles;
    DriveValues WheelPower;
    DriveCommand(){}
    DriveCommand(DriveValues a, DriveValues b) : WheelAngles(a), WheelPower(b){}
};
//Represetitive of a wheel on the chassis, defined by its 
//position in relation to the center of the chassis
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
        
        if (deg < -90){
            deg += 180;
        }
        if (deg > 90){
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
//Used to contain 4 wheel objects and provide methods to calculate 
//the speed and angle each wheel should be set to to perform turns 
//properly.
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
    //Calculates the power ratios between wheels, outer values will be above 1 and inner below
    //Use ClampDriveValues to make outer ratios equal to 1
    DriveValues CalculateRatios(double centerOffset){

        //Calculating radius about the centerpoint of the turn for each wheel
        DriveValues ratios(
            front_left.getRadiusOfTurn(centerOffset),
            front_right.getRadiusOfTurn(centerOffset),
            back_left.getRadiusOfTurn(centerOffset),
            back_right.getRadiusOfTurn(centerOffset)
        );

        //Summing all the radii 
        double total = (ratios.front_left + ratios.front_right + ratios.back_left + ratios.back_right);

        //Converting all radii into power ratios
        ratios = ratios * (4/total);

        return ratios;
    }
    //Calculates the angles to set wheels to
    DriveValues CalculateAngles(double centerOffset) {
        return DriveValues(     front_left.getAngle(centerOffset),
                                front_right.getAngle(centerOffset), 
                                back_left.getAngle(centerOffset), 
                                back_right.getAngle(centerOffset)
                            );
    }
    //Converts throttle and steering into a DriveCommand, minimum 
    //turning radius defined by minRadius
    DriveCommand TwistToMotor (double throttle, double steer, double minRadius) {
        if (fabs(steer) <= 0.005) {
            return DriveCommand(DriveValues(0,0,0,0), DriveValues(throttle,throttle,throttle,throttle));;
        } 
        DriveCommand command;
        double radius = minRadius / steer;
        command.WheelAngles = CalculateAngles(radius);
        command.WheelPower = CalculateRatios(radius).scaleToMaximum(1) * throttle;
        return command;
    }

};

/*
void PrintDriveValues (DriveValues dv) {
    printf("FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f\n", dv.front_left, dv.front_right, dv.back_left, dv.back_right);
}
*/
Wheelbase wb;
ros::Publisher steerpub;
ros::Publisher drivepub;

void UpdateDrive (const geometry_msgs::Twist::ConstPtr& msg) {
    DriveCommand command = wb.TwistToMotor(msg->linear.x, msg->angular.z, 1);
    steerpub.publish(command.WheelAngles.toMessage());
    drivepub.publish(command.WheelPower.toMessage());
}

int main(int argc, char **argv) {
    wb = Wheelbase();
    
    ros::init(argc, argv, "M5");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, UpdateDrive);

    steerpub = n.advertise<motion_control::Mobility>("steering", 1000);
    drivepub = n.advertise<motion_control::Mobility>("odrive_vel", 1000);

    ros::spin();
}