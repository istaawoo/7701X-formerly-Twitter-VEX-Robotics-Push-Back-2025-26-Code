#include "mcl/pid.hpp"
#include <cmath>

// constructor
PID::PID(double Kp, double Ki, double Kd, double integralLimit) {
    this->Kp = Kp; 
    this->Ki = Ki; 
    this->Kd = Kd; 
    this->integralLimit = integralLimit;

    prevError = 0.0;
    integral = 0.0;

    errorThreshold = 5.0;
    settleTimeMs = 250;
    timeoutMs = 2000;

    task = nullptr;
    running = false;
    output = 0.0;
    stableTime = pros::millis();
    startTime = pros::millis();
}

// calculate PID output
double PID::calculate(double error) {
    integral += error; // accumulate

    // limit integral
    if (integral > integralLimit) integral = integralLimit;
    if (integral < -integralLimit) integral = -integralLimit;

    double derivative = error - prevError; // change
    prevError = error;

    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    return output;
}

// set exit conditions
void PID::setExitConditions(double errorThreshold, int settleTimeMs, int timeoutMs) {
    this->errorThreshold = errorThreshold;
    this->settleTimeMs = settleTimeMs;
    this->timeoutMs = timeoutMs;
}

// check if PID has settled
bool PID::isSettled(double error) {
    auto now = pros::millis();

    if (fabs(error) < errorThreshold) {
        if (now - stableTime >= settleTimeMs) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 10 , "Settled in: %0.2f", float(now-startTime));
            return true;
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 9 , "Entered range: %0.2f", float(now-startTime));
    } else {
        stableTime = now;
    }

    return false;
}

// reset PID
void PID::reset() {
    integral = 0.0;
    prevError = 0.0;
    output = 0.0;
    stableTime = pros::millis();
    startTime = pros::millis();
}

// get last output
double PID::getOutput() {
    return output;
}
