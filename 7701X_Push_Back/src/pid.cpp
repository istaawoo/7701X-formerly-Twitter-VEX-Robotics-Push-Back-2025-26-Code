#include "pid.hpp"

    // constructor
    PID::PID(double Kp, double Ki, double Kd, double integralLimit = 1000.0) {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->integralLimit = integralLimit;

        prevError = 0.0;
        integral = 0.0;
        target = 0.0;
    }

    // set target
    void PID::setTarget(double newTarget) {
        target = newTarget;
        integral = 0.0;
        prevError = 0.0;
    }

    // calculate PID output
    double PID::calculate(double current) {
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
    void PID::reset() {
        integral = 0.0;
        prevError = 0.0;
    }