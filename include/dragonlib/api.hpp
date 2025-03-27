#include "main.h"
#ifndef CLASSES_HPP
#define CLASSES_HPP



// configurations for chassis

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
        PIDController(double kP, double kI, double kD);

        double PIDUpdate(double error, int milli);
        
        void resetParams(double kP, double kI, double kD);

        void reset();

    private:
        double kP;
        double kI;
        double kD;
        double prevError;
        double integral;
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
    double getDirectionTo(Point& other);
    
    double x = 0;
    double y = 0;
    double theta = 0;
    double linvel = 0;
    double angvel = 0;

    
    
    

};

class Chassis {
    public:
        // full constructor
        Chassis(
            pros::MotorGroup& l, 
            pros::MotorGroup& r, 
            PIDController& angularController,
            PIDController& linearController,
            TrackingWheel& vertical, 
            TrackingWheel& horizontal, 
            pros::Imu& inertial,
            float verticalTracking, 
            float horizontalTracking
        
        );        
        
        Chassis(
            pros::MotorGroup& l, 
            pros::MotorGroup& r, 
            PIDController& angularController,
            PIDController& linearController, 
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

        void turnToHeading(double theta, double cutoff, double maxSpeed, double minSpeed, double tolerance);

        void moveToPoint(double x, double y, double cutoff, double maxSpeed, double minSpeed, bool reverse, double tolerance);

        // 2DMP
        
        
        

    private:

    // defined by user

        // motors
        pros::MotorGroup& rightMotorGroup;
        pros::MotorGroup& leftMotorGroup;

        // PID Controllers
        PIDController& angularController;
        PIDController& linearController;

        // inertial + tracking wheels
        pros::Imu& inertial;
        TrackingWheel& verticalWheel;
        TrackingWheel& horizontalWheel;
        
        // tracking widths
        float horizontalTracking;
        float verticalTracking;

        // gear ratio

        float gearRatio;

    // not defined by user
        
        // odometry 

        float config;
        Point position;
        double previousVertTracker;
        double previousHorizTracker;


};





extern TrackingWheel empty_tracker;

extern double degreesToRadians(double degrees);

extern double radiansToDegrees(double radians);



#endif



// Coded by Team 9204A Dragons