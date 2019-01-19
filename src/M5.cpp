#include <math.h>
#include <map>
#include <stdio.h>
#include <tuple>

struct DriveValues{
    double front_left;
    double front_right;
    double back_left;
    double back_right;
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
        Wheel front_left(0.5,0.5);
        Wheel front_right(0.5,-0.5);
        Wheel back_left(-0.5,0.5);
        Wheel back_right(-0.5,-0.5);
    }
    
    DriveValues CalculateRatios(double centerOffset){
        DriveValues ratios;

        //Calculating radius on each wheel
        ratios.front_left   = front_left.getRadiusOfTurn(centerOffset);
        ratios.front_right  = front_right.getRadiusOfTurn(centerOffset);
        ratios.back_left    = back_left.getRadiusOfTurn(centerOffset);
        ratios.back_right   = back_right.getRadiusOfTurn(centerOffset);

        //Summing all the radii for wheel power distribution
        double total = ratios.front_left + ratios.front_right + ratios.back_left + ratios.back_right;

        //Converting all radii into power ratios
        ratios.front_left /= total;
        ratios.front_right /= total;
        ratios.back_left /= total;
        ratios.back_right /= total;

        return ratios;
    }

};

int main() {
    Wheelbase wb();

    //Wheel w(1.0,0.0);
    //for (double i = 10; i >= -10; i--) {
    //    printf("%.1f %.3f %.3f\n", i/10, w.getAngle(i/10), w.getRadiusOfTurn(i/10));
    //}
}