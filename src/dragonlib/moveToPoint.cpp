#include "main.h"
#include <cmath>
#include "dragonlib/api.hpp"



void Chassis::moveToPoint(double x, double y, double cutoff, double maxSpeed, double minSpeed, bool reverse) {


    Point target(x, y, 0 /* figure out this value thru atan2 or something */);

    double tolerance = 1;

    double timer = 0;

    // reset integral and derivative values at the beginning of iterations

    this->angularController.reset();
    this->linearController.reset();
    double linearError;
    double angularError;
    if(!reverse){
        while(timer < cutoff){
            linearError = this->position.getDistance(target);

            if(linearError < tolerance){
                break;
            }

            angularError = this->position.getDirectionTo(target)-this->inertial.get_heading();

            if(angularError > 180) {
                angularError -= 360;
            } else if (angularError < -180) {
                angularError += 360;
            }
    
            // determine whether the point is behind, in which case moving backwards is optimal
    
            if(abs(angularError) > 90){
                angularError = remainder(angularError-180, 360);
                linearError *= -1;
            }
    
            double throttle = this->linearController.PIDUpdate(linearError, 10);
            double turn = this->angularController.PIDUpdate(angularError, 10);
    
            // bias towards turning
    
            throttle *= cos(degreesToRadians(angularError));
    
            // convert throttle and turn into left and right motorspots
    
            double leftVoltage = throttle + turn;
            double rightVoltage = throttle - turn;
    
    
            // normalize values so we don't oversaturate and not follow the path
    
            if(leftVoltage > 12){
                leftVoltage = 12;
                rightVoltage *= 12/leftVoltage;
            } else if (rightVoltage > 12){
                rightVoltage = 12;
                leftVoltage *= 12/rightVoltage;
            }
    
            this->leftMotorGroup.move_voltage(leftVoltage*1000);
            this->rightMotorGroup.move_voltage(rightVoltage*1000);
    
            timer += 10;
            pros::delay(10);
        }
    } else {
        while(timer < cutoff){
            linearError = this->position.getDistance(target);
            
            if(linearError < tolerance){
                break;
            }

            angularError = this->position.getDirectionTo(target)-(this->inertial.get_heading()-180);

            if(angularError > 180) {
                angularError -= 360;
            } else if (angularError < -180) {
                angularError += 360;
            }
    
            // determine whether the point is behind, in which case moving backwards is optimal
    
            if(abs(angularError) > 90){
                angularError = remainder(angularError-180, 360);
                linearError *= -1;
            }
    
            double throttle = this->linearController.PIDUpdate(linearError, 10);
            double turn = this->angularController.PIDUpdate(angularError, 10);
    
            // bias towards turning
    
            throttle *= cos(degreesToRadians(angularError));
    
            // convert throttle and turn into left and right motorspots
    
            double leftVoltage = throttle + turn;
            double rightVoltage = throttle - turn;
    
    
            // normalize values so we don't oversaturate and not follow the path
    
            if(leftVoltage > 12){
                leftVoltage = 12;
                rightVoltage *= 12/leftVoltage;
            } else if (rightVoltage > 12){
                rightVoltage = 12;
                leftVoltage *= 12/rightVoltage;
            }
    
            this->leftMotorGroup.move_voltage(-leftVoltage*1000);
            this->rightMotorGroup.move_voltage(-rightVoltage*1000);
    
            timer += 10;
            pros::delay(10);
        }
    }
   


}