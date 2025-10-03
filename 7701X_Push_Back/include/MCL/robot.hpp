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
#include "mcl/mcl.hpp"

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
    particleFilter robotFilter = particleFilter(50, gaussian(3), gaussian(0.15));
public:
    double trackWidth;
    double trackLength;
    double wheelRatio;
    double wheelSize;
    double rotCenterDistance;

    pros::MotorGroup* right_motors;
    pros::MotorGroup* left_motors;

    LatPID latteral_PID;
    TurnPID turning_PID;

    std::vector<std::unique_ptr<pros::Imu>> imus;
    std::vector<std::unique_ptr<pros::Rotation>> rotations;
    std::vector<std::unique_ptr<pros::Distance>> distances;
    std::vector<std::unique_ptr<pros::ADIAnalogIn>> adiIns;
    std::vector<std::unique_ptr<pros::ADIDigitalOut>> adiOuts;
    std::vector<std::unique_ptr<pros::Optical>> opticals;

    Robot(double trackWidth, double trackLength, double wheelRatio, double wheelSize, double rotCenterDistance,
          pros::MotorGroup* right_motors, pros::MotorGroup* left_motors, LatPID, TurnPID);

    Pose getPose();

    void setPID(LatPID, TurnPID);

    void waitUntil(double threshold);

    void place(float x, float y, float theta, gaussian errorLat, gaussian errorRot); //Put the robot at a starting position. initializes particle filter
    
    void checkStart(); //uses MCL to check and update starting position

    void move(float distance, float theta, int timeout, float maxSpeed,                               // move a relative distance along a target heading
              float earlyExitDelta, gaussian errorLat, gaussian errorRot);

    void moveToPoint(float x, float y, int timeout, float earlyExitDelta);                            // move to global point with final heading along movement
    
    void moveToPose(float x, float y, float theta, int timeout, float maxSpeed,                       // move to global point with a target heading
                    float earlyExitDelta, float lead, float horizontalDrift);                                         
    
    void turn(float thetaRelative, int timeout, float earlyExitDelta);                                // turn a relative angle
    
    void turnTo(float thetaAbsolute, int timeout, float earlyExitDelta);                              // turn to an absolute heading
    
    void turnToPoint(float x, float y, int timeout, float earlyExitDelta);                            // turn to face a global point
    
};