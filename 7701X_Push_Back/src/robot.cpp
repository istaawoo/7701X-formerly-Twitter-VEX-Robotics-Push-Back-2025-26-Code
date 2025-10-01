#include "robot.hpp"

Robot::Robot(float trackWidth, float trackLength, float wheelRatio, float wheelSize, int rotCenterDistance) {
    this->trackWidth = trackWidth;
    this->trackLength = trackLength;
    this->wheelRatio = wheelRatio;
    this->wheelSize = wheelSize;
    this->rotCenterDistance = rotCenterDistance;
}

void Robot::move(float distance, float theta, int timeout) {
    int u = 0;
}

// Add sensors dynamically
void Robot::IMU(int port) {
    imus.push_back(std::make_unique<pros::Imu>(port));
}

void Robot::Rotation(int port) {
    rotations.push_back(std::make_unique<pros::Rotation>(port));
}

void Robot::Distance(int port) {
    distances.push_back(std::make_unique<pros::Distance>(port));
}

void Robot::Optical(int port) {
    opticals.push_back(std::make_unique<pros::Optical>(port));
}

void Robot::ADI_In(char port) {
    adiIns.push_back(std::make_unique<pros::ADIAnalogIn>(port));
}

void Robot::ADI_Out(char port) {
    adiOuts.push_back(std::make_unique<pros::ADIDigitalOut>(port));
}
