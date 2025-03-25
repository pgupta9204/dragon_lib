#include "dragonlib/api.hpp"
#include <cmath>


double radiansToDegrees(double radians){
    return radians*180/M_PI;
}

double degreesToRadians(double degrees){
    return degrees*M_PI/180;
}

double Point::getDistance(Point& other){
    double dx = this->x - other.x;
    double dy = this->y - other.y;
    return sqrt(dx*dx+dy*dy);
};

double Point::getAngularError(Point& other){
    return other.theta - this->theta;
};


double Point::getDirectionTo(Point& other){
    double dx = other.x - this->x;
    double dy = other.y - this->y;
    return radiansToDegrees(atan2(dx, dy));
}