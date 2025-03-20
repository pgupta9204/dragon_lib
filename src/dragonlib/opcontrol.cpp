#include "dragonlib/api.hpp"


void Chassis::arcadeDrive(float throttle, float turn){
    float lm = throttle + turn;
    float rm = throttle - turn;
    this->leftMotorGroup.move_velocity(600*lm);
    this->rightMotorGroup.move_velocity(600*rm);

};

