#pragma once

#include "main.h"
#include "pros/rtos.hpp"
#include "pros/motors.hpp"
#include <functional>
#include <cmath>

class PID {
private:
    double prevError;       // previous error
    double integral;        // accumulated error
    double target;          // setpoint
    double integralLimit;   // max integral

    double errorThreshold;  // acceptable error
    int settleTimeMs;       // required stable time
    int timeoutMs;          // maximum run time

    int startTime;          // start timestamp in ms
    int stableTime;         // last time error went out of bounds

    pros::Task* task;       // background PID task
    bool running;           // is task running
    double output;          // last PID output

public:
    double Kp;              // proportional constant
    double Ki;              // integral constant
    double Kd;              // derivative constant

    // constructor
    PID(double Kp, double Ki, double Kd, double integralLimit = 1000.0);

    // set target
    void setTarget(double newTarget);

    // calculate PID output
    double calculate(double current);

    // reset PID
    void reset();

    // set exit condition parameters
    void setExitConditions(double errorThreshold, int settleTimeMs, int timeoutMs);

    // check if PID is settled
    bool isSettled(double current);

    // start PID loop in background task
    void start(std::function<double()> sensor, std::function<void(double)> actuator);

    // stop PID task
    void stop();

    // get last output
    double getOutput();
};
