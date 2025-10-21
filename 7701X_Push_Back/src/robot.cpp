#include "mcl/robot.hpp"
#include "pid.cpp"
#include <cmath>

PID latteral_high_qual(1, 0, 1);  // lat_one
PID latteral_med_qual(5, 1, 5);   // lat_two
PID latteral_low_qual(10, 5, 10); // lat_three

PID turning_high_qual(1, 0, 1);   // turn_one
PID turning_med_qual(5, 1, 5);	  // turn_two
PID turning_low_qual(10, 5, 10);  // turn_three

bool finished = true;

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

Pose targetPose(0, 0, 0);

void Robot::waitUntil(double threshold) {
    while (true) {
        double dx = targetPose.x - getPose().x;
        double dy = targetPose.y - getPose().y;
        double distance = sqrt(dx * dx + dy * dy);
        if (abs(distance <= threshold)) {
            break;
        }
        pros::delay(20);
    }
    finished = true;
}

void Robot::place(float x, float y, float theta, gaussian errorLat, gaussian errorRot) {
    robotPose.x = x;
    robotPose.y = y;
    robotPose.theta = theta;
    
    //robotFilter.initializeParticles(x,y,theta,errorLat,errorLat,errorRot);
}

void Robot::setPID(LatPID latPID, TurnPID turnPID) {
    this->latteral_PID = latPID;
    this->turning_PID = turnPID;
}

void waitUntilFinished() {
    if (!finished) {
        pros::delay(20);
    }
}

void Robot::checkStart() {
    //robotFilter.senseUpdate();
    //robotPose = robotFilter.predictPosition();
}

//Move the robot a certian distance along a certian heading.
void Robot::move(float distance, float theta, int timeout, float maxSpeed, float earlyExitDelta, gaussian errorLat, gaussian errorRot) { 
    waitUntilFinished();
    finished = false;
    targetPose.x = getPose().x + distance * cos(theta); //gets the target x position by adding x component of movement vector with current position
    targetPose.y = getPose().y + distance * sin(theta); //gets the target y position by adding y component of movement vector with current position
    targetPose.theta = theta; //gets target theta as heading of movement

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

    pros::Task moveTask([this, lat_pid, turn_pid, distance, timeout, earlyExitDelta] {
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
            if ((lat_pid->isSettled(difference) 
                && turn_pid->isSettled(angleError)) 
                || (pros::millis() - startTime > timeout)
                || (difference < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                //robotFilter.senseUpdate();
                //robotPose = robotFilter.predictPosition();
                finished = true;
                break;
            }
            pros::delay(20);
        }
    });
    

    pros::Task moveTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        //robotFilter.resample();
        //robotFilter.moveUpdate(targetPose.x, targetPose.y, targetPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
}

//Move the robot to a point with a heading along path of travel
void Robot::moveToPoint(float x, float y, int timeout, float earlyExitDelta) {
    waitUntilFinished();
    finished = false;
    targetPose.x = x;
    targetPose.y = y;
    targetPose.theta = atan2(y,x);
    float distance = sqrt(x*x+y*y);

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

    pros::Task moveToPointTask([this, lat_pid, turn_pid, distance, &timeout, &earlyExitDelta] {
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
                || (pros::millis() - startTime > timeout)
                || (difference < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
    });
}

//Move the robot to a point with a target heading
void Robot::moveToPose(float x, float y, float theta, int timeout, float maxSpeed, float earlyExitDelta, float lead, float horizontalDrift) {
    waitUntilFinished();
    finished = false;
    targetPose.x = x;
    targetPose.y = y;
    targetPose.theta = theta;
    float distance = sqrt(x*x+y*y);

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

    pros::Task moveToPose([this, lat_pid, turn_pid, distance, &lead, &timeout, &earlyExitDelta]{
        int startTime = pros::millis();
        while (true) {
            double dx = targetPose.x - getPose().x;
            double dy = targetPose.y - getPose().y;
            double difference = sqrt(dx * dx + dy * dy);
            double angleError = atan2(dy, dx) - getPose().theta;
            // Normalize carrot angleError to [-pi, pi]
            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError < -M_PI) angleError += 2 * M_PI;

            double carrotX = targetPose.x - difference*sin(targetPose.theta)*lead;
            double carrotY = targetPose.y - difference*cos(targetPose.theta)*lead;
            double carrotDx = carrotX - getPose().x;
            double carrotDy = carrotY - getPose().y;
            double carrotDifference = sqrt(carrotDx*carrotDx + carrotDy*carrotDy);
            double carrotAngleError = atan2(dy, dx) - getPose().theta;

            // Normalize carrot angleError to [-pi, pi]
            while (carrotAngleError > M_PI) carrotAngleError -= 2 * M_PI;
            while (carrotAngleError < -M_PI) carrotAngleError += 2 * M_PI;

            // PID outputs
            double lateralPower = lat_pid->calculate(carrotDifference) * cos(carrotAngleError);
            double turningPower = turn_pid->calculate(carrotAngleError) * sin(carrotAngleError);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(lateralPower - turningPower);
                right_motors->move(lateralPower + turningPower);
            }

            // Check if PID is settled or timeout
            if ((lat_pid->isSettled(distance - difference) 
                && turn_pid->isSettled(angleError)) 
                || (pros::millis() - startTime > timeout)
                || (difference < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
    });
}

//Turn the robot an ammount relative to current heading
void Robot::turn(float thetaRelative, int timeout, float earlyExitDelta) {
    waitUntilFinished();
    finished = false;
    targetPose.x = getPose().x;
    targetPose.y = getPose().y;
    targetPose.theta = getPose().theta + thetaRelative;

    PID* turn_pid = nullptr;
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
    if (!turn_pid) return;

    pros::Task turnTask([this, turn_pid, &timeout, &earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            double dtheta = targetPose.theta - getPose().theta;

            // PID outputs
            double turningPower = turn_pid->calculate(dtheta);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(-turningPower);
                right_motors->move(turningPower);
            }

            // Check if PID is settled or timeout
            if ((turn_pid->isSettled(dtheta)) || (pros::millis() - startTime > timeout) || (dtheta < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
    });
}

//Turn the robot to a global heading
void Robot::turnTo(float thetaAbsolute, int timeout, float earlyExitDelta) {
    waitUntilFinished();
    finished = false;
    targetPose.x = getPose().x;
    targetPose.y = getPose().y;
    targetPose.theta = thetaAbsolute;

    PID* turn_pid = nullptr;
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
    if (!turn_pid) return;

    pros::Task turnTask([this, turn_pid, &timeout, &earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            double dtheta = targetPose.theta - getPose().theta;

            // PID outputs
            double turningPower = turn_pid->calculate(dtheta);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(-turningPower);
                right_motors->move(turningPower);
            }

            // Check if PID is settled or timeout
            if ((turn_pid->isSettled(dtheta)) || (pros::millis() - startTime > timeout) || (dtheta < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
    });
}

//Turn the robot to face a global point
void Robot::turnToPoint(float x, float y, int timeout, float earlyExitDelta) {
    waitUntilFinished();
    finished = false;
    float dx = x - getPose().x;
    float dy = y - getPose().y;
    targetPose.theta = atan2(dy,dx);

    PID* turn_pid = nullptr;
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
    if (!turn_pid) return;

    pros::Task turnTask([this, turn_pid, &timeout, &earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            double dtheta = targetPose.theta - getPose().theta;

            // PID outputs
            double turningPower = turn_pid->calculate(dtheta);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(-turningPower);
                right_motors->move(turningPower);
            }

            // Check if PID is settled or timeout
            if ((turn_pid->isSettled(dtheta)) || (pros::millis() - startTime > timeout) || (dtheta < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->move(0);
                    right_motors->move(0);
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
    });
}

Pose Robot::getPose() {
    return robotPose;
}

