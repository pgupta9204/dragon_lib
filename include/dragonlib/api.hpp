#include "main.h"
#ifndef CLASSES_HPP
#define CLASSES_HPP

extern const double pi = 3.141592653589793;

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
        std::optional<TrackingWheel&> verticalWheel;
        std::optional<TrackingWheel&> horizontalWheel;
        
        // tracking widths
        float horizontalTracking;
        std::optional<float> verticalTracking;

    // not defined by user
        
    // odometry 
        Point position;


};

class TrackingWheel {
    public:
    TrackingWheel(pros::Rotation& rotationSensor, float wheelDiameter);

    double getTotalDistanceTravelled();

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
    std::optional<double> linvel = 0;
    std::optional<double> angvel = 0;

    
    
    

};

#endif