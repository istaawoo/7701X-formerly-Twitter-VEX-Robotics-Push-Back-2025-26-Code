#pragma once

#include <vector>
#include <memory>
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "mcl/pose.hpp"
#include "mcl/mcl.hpp"
#include "mcl/pid.hpp"

/*
class Pose { //Variable that stores and x, y and theta position
public:
    float x;     //x-cord
    float y;     //y-cord
    float theta; //angle of heading

    Pose(float x_=0, float y_=0, float theta_ = 0) : x(x_), y(y_), theta(theta_) {}

    Pose operator=(const Pose &other) { //sets the pose cords to the cords of a Pose
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
    }

    Pose operator=(const particle &other) { //sets the pose cords to the cords of a particle
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
    }
};

*/

enum class LatPID {
    lat_one = 1,
    lat_two = 2,
    lat_three = 3
};

enum class TurnPID {
    turn_one = 1,
    turn_two = 2,
    turn_three = 3
};

class Robot {
private:
    Pose robotPose;

public:
    particleFilter robotFilter = particleFilter(50, gaussian(3), gaussian(0.15), &this->imus, &this->mclDistances);

    double trackWidth; //distance between left and right wheels
    double trackLength;
    double wheelRatio; //Gear ratio of motors to wheels
    double wheelSize; //Wheel radius in inches
    double rotCenterDistance;

    pros::MotorGroup* right_motors;
    pros::MotorGroup* left_motors;

    LatPID latteral_PID;
    TurnPID turning_PID;

    std::vector<std::unique_ptr<pros::Imu>> imus;
    std::vector<std::unique_ptr<pros::Rotation>> rotations;
    std::vector<std::unique_ptr<pros::Distance>> mclDistances;
    std::vector<std::unique_ptr<pros::Distance>> distances;
    std::vector<std::unique_ptr<pros::adi::AnalogIn>> adiIns;
    std::vector<std::unique_ptr<pros::adi::DigitalOut>> adiOuts;
    std::vector<std::unique_ptr<pros::Optical>> opticals;

    Robot(double trackWidth, double trackLength, double wheelRatio, double wheelSize, double rotCenterDistance,
          pros::MotorGroup* right_motors, pros::MotorGroup* left_motors, LatPID latPID, TurnPID turnPID);

    Pose getPose();

    void setPID(LatPID latPDI, TurnPID turnPID);

    void waitUntil(double threshold);

    void place(float x, float y, float theta, gaussian errorLat, gaussian errorRot); //Put the robot at a starting position. initializes particle filter
    
    void checkStart(); //uses MCL to check and update starting position

    void odometer(); //updates robot position based on wheel movements

    void move(float distance, float theta, int timeout, float maxSpeed,                               // move a relative distance along a target heading
              float earlyExitDelta, gaussian errorLat, gaussian errorRot);

    void moveToPoint(float x, float y, int timeout, float earlyExitDelta,                             // move to global point with final heading along movement
                     gaussian errorLat, gaussian errorRot);                            
    
    void moveToPose(float x, float y, float theta, int timeout, float maxSpeed,                       // move to global point with a target heading
                    float earlyExitDelta, float lead, float horizontalDrift,                                         
                    gaussian errorLat, gaussian errorRot);

    void turn(float thetaRelative, int timeout, float earlyExitDelta,                                 // turn a relative angle
              gaussian errorLat, gaussian errorRot);

    void turnTo(float thetaAbsolute, int timeout, float earlyExitDelta,                               // turn to an absolute heading
                gaussian errorLat, gaussian errorRot);

    void turnToPoint(float x, float y, int timeout, float earlyExitDelta,                             // turn to face a global point
                     gaussian errorLat, gaussian errorRot);
};

//Declare the robot object once, allows use in multiple files.
extern Robot robot;

extern PID latteral_high_qual;
extern PID latteral_med_qual;
extern PID latteral_low_qual;

extern PID turning_high_qual;
extern PID turning_med_qual;
extern PID turning_low_qual;