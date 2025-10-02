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

    Pose(float x_, float y_, float theta_ = 0) : x(x_), y(y_), theta(theta_) {}

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
    Pose robotPose();
public:
    double trackWidth;
    double trackLength;
    double wheelRatio;
    double wheelSize;
    double rotCenterDistance;

    LatPID latteral_PID;
    TurnPID turning_PID;

    std::vector<std::unique_ptr<pros::Imu>> imus;
    std::vector<std::unique_ptr<pros::Rotation>> rotations;
    std::vector<std::unique_ptr<pros::Distance>> distances;
    std::vector<std::unique_ptr<pros::ADIAnalogIn>> adiIns;
    std::vector<std::unique_ptr<pros::ADIDigitalOut>> adiOuts;
    std::vector<std::unique_ptr<pros::Optical>> opticals;

    Robot(double trackWidth, double trackLength, double wheelRatio, double wheelSize, double rotCenterDistance, LatPID, TurnPID);

    Pose getPose();

    void move(float distance, float theta, int timeout);        // move relative distance and heading ofset
    void moveToPoint(float x, float y, int timeout);            // move to global point keeping current heading
    void moveToPose(float x, float y, float theta, int timeout);// move to global point while rotating
    void turn(float thetaRelative, int timeout);                // relative turn
    void turnTo(float thetaAbsolute, int timeout);              // turn to absolute heading
    void turnToPoint(float x, float y, int timeout);

};