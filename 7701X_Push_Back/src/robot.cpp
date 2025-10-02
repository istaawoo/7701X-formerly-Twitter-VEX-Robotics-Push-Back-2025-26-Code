#include "robot.hpp"
#include "pid.cpp"
#include <cmath>

PID latteral_high_qual(1, 0, 1);  // lat_one
PID latteral_med_qual(5, 1, 5);   // lat_two
PID latteral_low_qual(10, 5, 10); // lat_three

PID turning_high_qual(1, 0, 1);   // turn_one
PID turning_med_qual(5, 1, 5);	  // turn_two
PID turning_low_qual(10, 5, 10);  // turn_three

Robot::Robot(double trackWidth, double trackLength, double wheelRatio, double wheelSize, double rotCenterDistance, 
             pros::MotorGroup* right_motors, pros::MotorGroup* left_motors, LatPID latPID, TurnPID turnPID) {

    this->trackWidth = trackWidth;
    this->trackLength = trackLength;
    this->wheelRatio = wheelRatio;
    this->wheelSize = wheelSize;
    this->rotCenterDistance = rotCenterDistance;

    this->right_motors = right_motors;
    this->left_motors = left_motors;
    this->latteral_PID = latPID;
    this->turning_PID = turnPID;
}

void Robot::move(float distance, float theta, int timeout) {
    targetPose.x = getPose().x + distance * cos(theta);
    targetPose.y = getPose().y + distance * sin(theta);
    targetPose.theta = theta;

    PID* lat_pid = nullptr;
    PID* turn_pid = nullptr;
    switch (latteral_PID) {
    case LatPID::lat_one:
        lat_pid = &latteral_high_qual;
        break;
    case LatPID::lat_two:
        lat_pid = &latteral_med_qual;
        break;
    case LatPID::lat_three:
        lat_pid = &latteral_low_qual;
        break;
    }
    switch (turning_PID) {
    case TurnPID::turn_one:
        turn_pid = &turning_high_qual;
        break;
    case TurnPID::turn_two:
        turn_pid = &turning_med_qual;
        break;
    case TurnPID::turn_three:
        turn_pid = &turning_low_qual;
        break;
    }
    if (!lat_pid || !turn_pid) return;

    pros::Task moveTask([this, lat_pid, turn_pid, distance, timeout] {
        int startTime = pros::millis();
        while (true) {
            double dx = targetPose.x - getPose().x;
            double dy = targetPose.y - getPose().y;
            double difference = sqrt(dx * dx + dy * dy);
            double angleError = atan2(dy, dx) - getPose().theta;
            // Normalize angleError to [-pi, pi]
            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError < -M_PI) angleError += 2 * M_PI;

            // PID outputs
            double lateralPower = lat_pid->calculate(difference) * cos(angleError);
            double turningPower = turn_pid->calculate(angleError) * sin(angleError);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(lateralPower - turningPower);
                right_motors->move(lateralPower + turningPower);
            }

            // Check if PID is settled or timeout
            if ((lat_pid->isSettled(distance - difference) 
                && turn_pid->isSettled(angleError)) 
                || (pros::millis() - startTime > timeout)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                break;
            }
            pros::delay(20);
        }
    });
}

Pose Robot::getPose() {
    return robotPose();
}

Pose targetPose(0, 0, 0);

/*
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
*/