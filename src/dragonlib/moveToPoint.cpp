#include "main.h"
#include <cmath>
#include "dragonlib/api.hpp"



void Chassis::moveToPoint(double x, double y, double cutoff, double maxSpeed, double minSpeed, bool reverse, double tolerance) {

    // create point object to make calculations easier

    Point target(x, y, 0);

    target.theta = this->position.getDirectionTo(target);

    double timer = 0;

    // reset integral and derivative values at the beginning of iterations

    this->angularController.reset();
    this->linearController.reset();
    double linearError;
    double angularError;
    if(!reverse){
        while(timer < cutoff){
            timer += 10;

            // determine linear error - distance to the target

            linearError = this->position.getDistance(target);

            // if linear error is lower than our tolerance then we are done with the movement and can exit the function

            if(linearError < tolerance){
                break;
            }

            // determine angular error

            angularError = this->position.getDirectionTo(target)-this->inertial.get_heading();

            // keep angular error within -180 and 180

            if(angularError > 180) {
                angularError -= 360;
            } else if (angularError < -180) {
                angularError += 360;
            }
    
            // determine whether the point is behind, in which case moving backwards is optimal
    
            if(fabs(angularError) > 90){
                angularError = remainder(angularError-180, 360);
                linearError *= -1;
            }
            
            double throttle = this->linearController.PIDUpdate(linearError, 10);    // throttle should have a value from -12 to 12


            double turn = this->angularController.PIDUpdate(angularError, 10);      // turn should have a value from -12 to 12
    
            // make throttle conform to minspeed and maxspeed

            if(throttle > 0){
                if(throttle > 12 * maxSpeed){
                    throttle = 12 * maxSpeed;
                } else if (throttle < 12 * minSpeed){
                    throttle = 12 * minSpeed;
                }
            } else if (throttle < 0) {
                if(throttle < -12 * maxSpeed){
                    throttle = -12 * maxSpeed;
                } else if (throttle > -12 * minSpeed){
                    throttle = -12 * minSpeed;
                }
            }

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
    
            
            pros::delay(10);
        }
    } else {
        while(timer < cutoff){
            timer += 10;

            linearError = this->position.getDistance(target);

            if(linearError < tolerance){
                break;
            }

            // determine angular error

            angularError = this->position.getDirectionTo(target)-this->inertial.get_heading()-180;

            // keep angular error within -180 and 180

            if(angularError > 180) {
                angularError -= 360;
            } else if (angularError < -180) {
                angularError += 360;
            }
    
            // determine whether the point is behind, in which case moving backwards is optimal
    
            if(fabs(angularError) > 90){
                angularError = remainder(angularError-180, 360);
                linearError *= -1;
            }
            
            double throttle = this->linearController.PIDUpdate(linearError, 10);    // throttle should have a value from -12 to 12


            double turn = this->angularController.PIDUpdate(angularError, 10);      // turn should have a value from -12 to 12
    
            // make throttle conform to minspeed and maxspeed

            if(throttle > 0){
                if(throttle > 12 * maxSpeed){
                    throttle = 12 * maxSpeed;
                } else if (throttle < 12 * minSpeed){
                    throttle = 12 * minSpeed;
                }
            } else if (throttle < 0) {
                if(throttle < -12 * maxSpeed){
                    throttle = -12 * maxSpeed;
                } else if (throttle > -12 * minSpeed){
                    throttle = -12 * minSpeed;
                }
            }

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

            
            // finally move the motors
            
    
            this->leftMotorGroup.move_voltage(-leftVoltage*1000);
            this->rightMotorGroup.move_voltage(-rightVoltage*1000);
    
            // add delay to not fry the brain :D
            
            pros::delay(10);
        }
    }
   


}