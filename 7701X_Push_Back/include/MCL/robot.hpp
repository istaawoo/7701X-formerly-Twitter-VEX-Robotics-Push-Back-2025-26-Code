#pragma once

#include <vector>
#include <memory>
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
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

class Robot {
public:
    float trackWidth;
    float trackLength;
    float wheelRatio;
    float wheelSize;
    int rotCenterDistance;

    std::vector<std::unique_ptr<pros::Imu>> imus;
    std::vector<std::unique_ptr<pros::Rotation>> rotations;
    std::vector<std::unique_ptr<pros::Distance>> distances;
    std::vector<std::unique_ptr<pros::ADIAnalogIn>> adiIns;
    std::vector<std::unique_ptr<pros::ADIDigitalOut>> adiOuts;
    std::vector<std::unique_ptr<pros::Optical>> opticals;

    Robot(float trackWidth, float trackLength, float wheelRatio, float wheelSize, int rotCenterDistance);

    void IMU(int port);
    void Rotation(int port);
    void Distance(int port);
    void Optical(int port);
    void ADI_In(char port);
    void ADI_Out(char port);

    void move(float distance, float theta, int timeout);        // move relative distance and heading ofset
    void moveTo(float x, float y, int timeout);                 // move to global point keeping current heading
    void moveTo(float x, float y, float theta, int timeout);    // move to global point while rotating
    void turn(float thetaRelative, int timeout);                // relative turn
    void turnTo(float thetaAbsolute, int timeout);              // turn to absolute heading

};