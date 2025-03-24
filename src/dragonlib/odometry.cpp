#include "main.h"
#include "dragonlib/api.hpp"
#include <cmath>

Point Chassis::getPosition(){
    return this->position;
};

void Chassis::setPose(double x, double y, double theta){
    position.x = x;
    position.y = y;
    position.theta = theta;
};


void Chassis::odomUpdate() {

    

    // determines change in theta

    double inertial_in_radians = this->inertial.get_heading() * M_PI / 180;
    
    double delta_theta = -position.theta + this->inertial.get_heading();
    // normalizes delta theta between -180 deg and 180 deg

    if(delta_theta < -180){
        delta_theta += 360;

    } else if (delta_theta > 180) {
        delta_theta -= 360;

    }
    
    double delta_theta_in_radians = delta_theta * M_PI / 180;

    // figures out change in tracking wheels

    double delta_vert_tracker = this->verticalWheel.getTotalDistanceTravelled() - this->previousVertTracker;
    double delta_horiz_tracker = this->horizontalWheel.getTotalDistanceTravelled() - this->previousHorizTracker;


    // find local position

    double arc_radius;
    double horiz_arc_radius;
    double local_delta_y;
    double local_delta_x;


    if(delta_theta == 0){
        local_delta_y = delta_vert_tracker;
        local_delta_x = delta_horiz_tracker;
    } else {
        arc_radius = verticalTracking + delta_vert_tracker/delta_theta_in_radians;
        horiz_arc_radius = horizontalTracking + delta_horiz_tracker/delta_theta_in_radians;
        local_delta_y = sin(delta_theta_in_radians/2)*arc_radius;
        local_delta_x = sin(delta_theta_in_radians/2)*horiz_arc_radius;
    }

    


    double transformation_angle = inertial_in_radians + delta_theta_in_radians/2;
    
    // switches to global coordinate system by multiplying by a transformation matrix

    double global_delta_x = local_delta_y*sin(transformation_angle) + local_delta_x*cos(transformation_angle);
    double global_delta_y = local_delta_y*cos(transformation_angle) - local_delta_x*sin(transformation_angle);
    
    // updates position variable

    this->position.x += global_delta_x;
    this->position.y += global_delta_y;
    this->position.theta = this->inertial.get_heading();

}
