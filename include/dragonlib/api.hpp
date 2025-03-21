#include "main.h"
#ifndef CLASSES_HPP
#define CLASSES_HPP



// configurations for chassis

enum ChassisConfig{
    FULL
};

class Chassis {
    public:
        // constructor
        Chassis(
            pros::MotorGroup& l, 
            pros::MotorGroup& r, 
            TrackingWheel& vertical, 
            TrackingWheel& horizontal, 
            pros::Imu& inertial,
            float verticalTracking, 
            float horizontalTracking
        
        );        


        // destructor (probably not going to be used)
        ~Chassis();


        // opcontrol functions

        void arcadeDrive(float throttle, float turn);

        // odometry functions

        void setPose(double x, double y, double theta);

        Point getPosition();
        
        void odomUpdate();

        // motion functions (PID)

        // 2DMP
        
        
        

    private:

    // defined by user

        // motors
        pros::MotorGroup& rightMotorGroup;
        pros::MotorGroup& leftMotorGroup;
        // inertial + tracking wheels
        pros::Imu& inertial;
        TrackingWheel& verticalWheel;
        TrackingWheel& horizontalWheel;
        
        // tracking widths
        float horizontalTracking;
        float verticalTracking;

    // not defined by user
        
        // odometry 

        ChassisConfig config;
        Point position;
        double previousVertTracker;
        double previousHorizTracker;


};

class TrackingWheel {
    public:
    TrackingWheel(pros::Rotation& rotationSensor, float wheelDiameter);

    double getTotalDistanceTravelled();
    void reverse();

    private:
    pros::Rotation& rotationSensor;
    float wheelDiameter;

};

class PIDController {
    public: 
};

class Point {
    public:

    // constructors

    Point(double x, double y, double theta, double linvel, double angvel);
    Point(double x, double y, double theta);
    Point();

    // destructors

    ~Point();

    // methods

    double getDistance(Point& other);
    double getAngularError(Point& other);
    double x = 0;
    double y = 0;
    double theta = 0;
    double linvel = 0;
    double angvel = 0;

    
    
    

};

#endif