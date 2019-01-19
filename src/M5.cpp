#include <math.h>
#include <map>
#include <stdio.h>

class Wheel {
    private:
    //+X is forward 
    //+Y is left
    double XPosition, YPosition;
    public:
    Wheel(double x, double y) {
        XPosition = x;
        YPosition = y;
    }
    //Gets the angle the wheel needs to be deviated 
    //from zero position, positive values correspond to 
    //counter-clockwise roatation when robot is viewed 
    //from above
    double getAngle(double centerOffset) {
        return atan2(XPosition, centerOffset-YPosition);
    }
    //Gets an adjustment to be applied to the throttle on 
    //the wheel to prevent slippage 
    double getVelocityMultiplier(double centerOffset) {
        return sqrt(pow(XPosition,2) + pow(centerOffset-YPosition,2))/centerOffset;
    }
};
class Wheelbase {
    private:
    std::map<std::string,Wheel>wheels;
    public:
    Wheelbase() {
    }
    //Adds a wheel to the system
    void AddWheel(std::string name, Wheel wheel) {
        wheels.insert(std::pair<std::string, Wheel>(name, wheel));
    }
    void AddWheel(std::string name, double x, double y) {
        Wheel wheel(x,y);
        wheels.insert(std::pair<std::string, Wheel>(name, wheel));
    }

};

int main() {
    Wheel w(1.0,1.0);
    printf("%f\n", w.getAngle(2));
}