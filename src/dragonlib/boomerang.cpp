#include "dragonlib/api.hpp"
#include <cmath>

void Chassis::boomerang(double x, double y, double theta, double dlead, double cutoff, double maxSpeed, double minSpeed, bool reverse, double linearTolerance, double angularTolerance){
    Point target(x, y, theta);
    double timer = 0;
    double linearError;
    double angularError;
    while(timer < cutoff){

        timer += 10;


        Point carrot(target.x - this->position.getDistance(target)*dlead*sin(degreesToRadians(theta)), 
                    target.y - this->position.getDistance(target)*dlead*cos(degreesToRadians(theta)),
        0);


        
        
        linearError = this->position.getDistance(carrot);
        
        angularError = theta - this->position.theta;




        
        if(angularError > 180) {
            angularError -= 360;
        } else if (angularError < -180) {
            angularError += 360;
        }


        double throttle = this->linearController.PIDUpdate(linearError, 1);
        double turn = this->angularController.PIDUpdate(angularError, 1);

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
}