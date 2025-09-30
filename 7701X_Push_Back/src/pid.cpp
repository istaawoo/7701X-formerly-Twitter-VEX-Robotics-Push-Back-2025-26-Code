#include "pid.hpp"
#include <cmath>

// constructor
PID::PID(double Kp, double Ki, double Kd, double integralLimit) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->integralLimit = integralLimit;

    prevError = 0.0;
    integral = 0.0;
    target = 0.0;

    errorThreshold = 5.0;   // default tolerance
    settleTimeMs = 250;     // must stay within error for 250ms
    timeoutMs = 2000;       // hard stop after 2 seconds

    task = nullptr;
    running = false;
    output = 0.0;
    stableTime = pros::millis();
    startTime = pros::millis();
}

// set target
void PID::setTarget(double newTarget) {
    target = newTarget;
    integral = 0.0;
    prevError = 0.0;
    stableTime = pros::millis();
    startTime = pros::millis();
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
bool PID::isSettled(double current) {
    double error = std::fabs(target - current);
    auto now = pros::millis();

    if (error < errorThreshold) {
        if (now - stableTime >= settleTimeMs) return true;
    } else {
        stableTime = now;
    }

    if (now - startTime >= timeoutMs) return true; // timeout
    return false;
}

// start PID in a background PROS task
void PID::start(std::function<double()> sensor, std::function<void(double)> actuator) {
    if (running) return; // already running

    running = true;
    task = new pros::Task([this, sensor, actuator] {
        startTime = pros::millis();
        stableTime = pros::millis();

        while (running) {
            double current = sensor();
            double power = calculate(current);
            actuator(power);

            if (isSettled(current)) break;

            pros::delay(20); // loop every 20ms
        }

        running = false;
    });
}

// stop PID and clean up task
void PID::stop() {
    running = false;
    if (task != nullptr) {
        task->remove();
        delete task;
        task = nullptr;
    }
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
