#include "dragonlib/api.hpp"


double PIDController::PIDUpdate(double error, int milli){
    double proportional = this->kP*error;
    this->integral += error*this->kI;
    
    if(this->prevError = NULL) {
        this->prevError = error;
    }
    
    double derivative = this->kD*(error - prevError)/milli;

    return proportional + this->integral + derivative;

}

void PIDController::resetParams(double kP, double kI, double kD){
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->integral = 0;
    this->prevError = 0;
}

void PIDController::reset() {
    this->integral = 0;
    this->prevError = 0;
}