//include main
#include "main.h" 

//include pros files
#include "liblvgl/llemu.h" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp" // IWYU pragma: keep
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.h"  // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/screen.hpp" // IWYU pragma: keep

//general C++ utilities 
#include <cmath>
#include <iostream> // IWYU pragma: keep
#include <stdio.h> // IWYU pragma: keep

//include PID/movement utilities
#include "mcl.hpp"
#include "auton.hpp"


class PID {
public:
    double Kp; // proportional constant
    double Ki; // integral constant
    double Kd; // derivative constant

    double prevError; // previous error
    double integral;  // error sum
    double target;    // setpoint

    double integralLimit; // max integral

    // constructor
    PID(double Kp, double Ki, double Kd, double integralLimit = 1000.0) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->integralLimit = integralLimit;

        prevError = 0.0;
        integral = 0.0;
        target = 0.0;
    }

    // set target
    void setTarget(double newTarget) {
        target = newTarget;
        integral = 0.0;
        prevError = 0.0;
    }

    // calculate PID output
    double calculate(double current) {
        double error = target - current; // error
        integral += error; // accumulate

        // clamp integral
        if (integral > integralLimit) integral = integralLimit;
        if (integral < -integralLimit) integral = -integralLimit;

        double derivative = error - prevError; // change
        prevError = error;

        return (Kp * error) + (Ki * integral) + (Kd * derivative);
    }

    // reset PID
    void reset() {
        integral = 0.0;
        prevError = 0.0;
    }
};
