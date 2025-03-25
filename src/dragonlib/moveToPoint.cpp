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
        angularError = this->position.getAngularError(target)-this->inertial.get_heading();

        // determine whether the point is behind, in which case moving backwards is optimal

        if(abs(angularError) > 90){
            angularError = remainder(angularError-180, 360);
            linearError *= -1;
        }

        


        timer += 10;
        pros::delay(10);
    }


}