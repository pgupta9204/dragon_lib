#include "dragonlib/api.hpp"


/*
Point Constructors
*/

// constructor for 2DMP



Point::Point(
    double x,
    double y, 
    double theta, 
    double linvel,
    double angvel) :
x(x),
y(y),
theta(theta),
linvel(linvel),
angvel(angvel)

{}

// constructor for regular movements

Point::Point(double x, double y, double theta) : x(x), y(y), theta(theta) {}



// default constructor

Point::Point() : x(0), y(0), theta(0) {}


/*
Tracking Wheel Constructors
*/

TrackingWheel::TrackingWheel(pros::Rotation& rotationSensor, float wheelDiameter) : rotationSensor(rotationSensor), wheelDiameter(wheelDiameter) {}



/*
Chassis Constructors
*/

// constructor with odom + imu

Chassis::Chassis(
    pros::MotorGroup& l, 
    pros::MotorGroup& r, 
    TrackingWheel& vertical, 
    TrackingWheel& horizontal, 
    pros::Imu& inertial,
    float verticalTracking, 
    float horizontalTracking


) : 

// initializes motor groups
leftMotorGroup(l), 
rightMotorGroup(r), 

// initializes tracking wheels (optional)
verticalWheel(vertical), 
horizontalWheel(horizontal), 

// initializes inertial sensor
inertial(inertial), 

// initalizes vertical tracking width (optional)
verticalTracking(verticalTracking), 

// initializes horizontal tracking width (required)
horizontalTracking(horizontalTracking),

previousHorizTracker(0), 
previousVertTracker(0),

config(FULL)

{}









