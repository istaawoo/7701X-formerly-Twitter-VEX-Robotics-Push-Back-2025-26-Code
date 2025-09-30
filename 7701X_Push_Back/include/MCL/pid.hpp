#pragma once

class PID {
private:
    double prevError; // previous error
    double integral;  // error sum
    double target;    // setpoint
    double integralLimit; // max integral
public:
    double Kp; // proportional constant
    double Ki; // integral constant
    double Kd; // derivative constant

    // constructor
    PID(double Kp, double Ki, double Kd, double integralLimit = 1000.0);

    // set target
    void setTarget(double newTarget);

    // calculate PID output
    double calculate(double current);

    // reset PID
    void reset() {
        integral = 0.0;
        prevError = 0.0;
    }
};
