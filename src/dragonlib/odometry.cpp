#include "main.h"
#include "dragonlib/api.hpp"

Point Chassis::getPosition(){
    return this->position;
};

void Chassis::setPose(double x, double y, double theta){
    position.x = x;
    position.y = y;
    position.theta = theta;
};



