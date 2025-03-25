#include "dragonlib/api.hpp"

void Chassis::moveToPoint(double x, double y, double cutoff, double maxSpeed, double minSpeed, bool reverse) {


    Point target(x, y, 0 /* figure out this value thru atan2 or something */);

    double timer = 0;

    // reset integral and derivative values at the beginning of iterations

    this->angularController.reset();
    this->linearController.reset();
    double linearError;
    double angularError;

    while(timer < cutoff){
        linearError = this->position.getDistance(target);
        angularError = this->position.getDirectionTo(target)-this->inertial.get_heading();

        // determine whether the point is behind, in which case moving backwards is optimal

        if(abs(angularError) > 90){
            angularError = remainder(angularError-180, 360);
            linearError *= -1;
        }

        double throttle = this->linearController.PIDUpdate(linearError, 10);
        double turn = this->angularController.PIDUpdate(angularError, 10);

        // bias towards turning

        throttle *= cos(this->position.getDirectionTo(target));

        // convert throttle and turn into left and right motorspots

        double leftVoltage = throttle + turn;
        double rightVoltage = throttle - turn;


        // normalize values so we don't oversaturate and kill our motors (big nono)

        if(leftVoltage > 12){
            leftVoltage = 12;
            rightVoltage *= 12/leftVoltage;
        } else if (rightVoltage > 12){
            rightVoltage = 12;
            leftVoltage *= 12/rightVoltage;
        }

        this->leftMotorGroup.move_voltage(leftVoltage);
        this->rightMotorGroup.move_voltage(rightVoltage);

        timer += 10;
        pros::delay(10);
    }


}