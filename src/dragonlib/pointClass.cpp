#include "dragonlib/api.hpp"
#include <cmath>

double Point::getDistance(Point& other){
    float dx = this->x + other.x;
    float dy = this->y + other.y;
    return sqrt(dx*dx+dy*dy);
};

double Point::getAngularError(Point& other){
    return this->theta - other.theta;
};



