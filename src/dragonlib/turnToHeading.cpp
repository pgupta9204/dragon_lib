#include "dragonlib/api.hpp"

void Chassis::turnToHeading(double theta, double cutoff, double maxSpeed, double minSpeed, double tolerance){
    double timer = 0;
    double angularError;

    while(timer < cutoff){
        angularError = theta - this->position.theta;


        
        double turn = this->angularController.PIDUpdate(angularError, 10);

        if(turn > maxSpeed*12){
            turn = maxSpeed*12;
        } else if (turn < -maxSpeed*12) {
            turn = -maxSpeed*12;
        } else if (turn > 0){
            if (turn > minSpeed*12) {
                turn = minSpeed*12;
            }
        } else if (turn < 0){
            if(turn < -minSpeed*12){
                turn = -minSpeed*12;
            }
        }

        this->leftMotorGroup.move_voltage(turn*1000);
        this->rightMotorGroup.move_voltage(-turn*1000);

        timer += 10;
        pros::delay(10);
    }
}