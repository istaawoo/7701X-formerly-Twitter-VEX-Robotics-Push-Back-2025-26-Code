#include "mcl/robot.hpp"
#include "mcl/pid.hpp"
#include "mcl/mcl.hpp"
#include <cmath>

PID latteral_high_qual(6, 1, 0);  // lat_one
PID latteral_med_qual(5, 1, 5);   // lat_two
PID latteral_low_qual(10, 5, 10); // lat_three

PID turning_high_qual(170, 1, 650);   // turn_one
PID turning_med_qual(100, 30, 50);	  // turn_two
PID turning_low_qual(160, 120, 100);  // turn_three

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
        double dx = targetPose.x - robotPose.x;
        double dy = targetPose.y - robotPose.y;
        double distance = sqrt(dx * dx + dy * dy);
        if (abs(distance) <= threshold) {
            break;
        }
        pros::delay(20);
    }
    finished = true;
}

void Robot::place(float x, float y, float theta, gaussian errorLat, gaussian errorRot) {
    theta *= M_PI/180;
    errorRot *= M_PI/180;

    robotPose.x = x;
    robotPose.y = y;
    robotPose.theta = theta;
    
    //robotFilter.initializeParticles(x,y,theta,errorLat,errorLat,errorRot);
    //robotFilter.senseUpdate();
    //robotPose = robotFilter.predictPosition();

    imus[0]->set_heading(robotPose.theta * 180/M_PI);
}

void Robot::setPID(LatPID latPID, TurnPID turnPID) {
    this->latteral_PID = latPID;
    this->turning_PID = turnPID;
}

void waitUntilFinished() {
    while (!finished) {
        pros::delay(20);
    }
}

void Robot::odometer() {
    static float leftWheelDist = 0;
    static float rightWheelDist = 0;

    //Sets the initial distance to the previous distances from last run. Before update
    float initialLeft = leftWheelDist;
    float initialRight = rightWheelDist;

    //Updates the wheel distances based on motor positions
    left_motors->set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_ROTATIONS);
    right_motors->set_encoder_units(pros::motor_encoder_units_e::E_MOTOR_ENCODER_ROTATIONS);

    leftWheelDist = left_motors->get_position() * wheelRatio * M_PI * wheelSize; //Convert motor rotations to linear distance
    rightWheelDist = right_motors->get_position() * wheelRatio * M_PI * wheelSize; //Convert motor rotations to linear distance

    float dL = leftWheelDist - initialLeft; //Change in left wheel distance
    float dR = rightWheelDist - initialRight; //Change in right wheel distance

    float dTheta = (dL - dR) / trackWidth; //Change in heading

    if (fabs(dTheta) < 1e-6) {
        double dS = (dR + dL)/2;
        robotPose.x += dS * sin(robotPose.theta);
        robotPose.y += dS * cos(robotPose.theta);
    } else {
        double trackRadius = (dR / dTheta) + (trackWidth/2);
        double trackChord = 2 * trackRadius * sin(dTheta/2);
        robotPose.x += trackChord * sin(robotPose.theta + dTheta/2);
        robotPose.y += trackChord * cos(robotPose.theta + dTheta/2);
    }
    robotPose.theta = (imus[0]->get_heading() * M_PI/180);
}

//Move the robot a certian distance along a certian heading.
void Robot::move(float distance, float theta, int timeout, float maxSpeed, float earlyExitDelta, gaussian errorLat, gaussian errorRot) { 
    waitUntilFinished();
    finished = false;
    
    theta *= M_PI/180;
    errorRot *= M_PI/180;

    targetPose.x = robotPose.x + distance * sin(theta); //gets the target x position by adding x component of movement vector with current position
    targetPose.y = robotPose.y + distance * cos(theta); //gets the target y position by adding y component of movement vector with current position
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

    //Reset PID at start of new move
    lat_pid->reset();
    turn_pid->reset();

    pros::Task moveTask([this, lat_pid, turn_pid, distance, timeout, earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            odometer();

            double dx = targetPose.x - robotPose.x;
            double dy = targetPose.y - robotPose.y;
            double angleError = targetPose.theta - robotPose.theta;
            double difference = sqrt(dx * dx + dy * dy) * cos(atan2(dx,dy) - robotPose.theta);

            pros::screen::print(pros::E_TEXT_MEDIUM, 1 , "Angle Error: %.2f",angleError*180/M_PI);

            // Normalize angleError to [-pi, pi]
            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError < -M_PI) angleError += 2 * M_PI;

            pros::screen::print(pros::E_TEXT_MEDIUM, 6 , "Angle Error: %.2f",angleError*180/M_PI);

            // PID outputs
            double lateralPower = lat_pid->calculate(difference);
            double turningPower = turn_pid->calculate(angleError); // * sin(angleToTarget);
            
            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(lateralPower - turningPower);
                right_motors->move(lateralPower + turningPower);
            }

            // Check if PID is settled or timeout
            if ((lat_pid->isSettled(difference) && turn_pid->isSettled(angleError)) 
                || (pros::millis() - startTime > timeout)
                || (fabs(difference) < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->brake();
                    right_motors->brake();
                }

                //robotFilter.senseUpdate();
                //robotPose = robotFilter.predictPosition();
                finished = true;
                pros::screen::print(pros::E_TEXT_MEDIUM, 10 , "Move Done");
                break;
            }
            pros::delay(20);
        }
        pros::Task::current().remove();
    });
    /*
    pros::Task moveTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        robotFilter.resample();
        robotFilter.moveUpdate(targetPose.x-robotPose.x, targetPose.y-robotPose.y, targetPose.theta-robotPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
    */
}

//Move the robot to a point with a heading along path of travel
void Robot::moveToPoint(float x, float y, int timeout, float earlyExitDelta, gaussian errorLat, gaussian errorRot) {
    waitUntilFinished();
    finished = false;

    errorRot *= M_PI/180;

    targetPose.x = x;
    targetPose.y = y;
    targetPose.theta = -atan2(x,y);
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

    //Reset PID at start of new move
    lat_pid->reset();
    turn_pid->reset();

    pros::Task moveToPointTask([this, lat_pid, turn_pid, distance, timeout, earlyExitDelta] {
        int startTime = pros::millis();
        
        while (true) {

            double dx = targetPose.x - robotPose.x;
            double dy = targetPose.y - robotPose.y;
            double difference = sqrt(dx * dx + dy * dy);
            double angleError = -atan2(dx, dy) - robotPose.theta;
            // Normalize angleError to [-pi, pi]
            while (angleError > M_PI) angleError -= 2 * M_PI;
            while (angleError <= -M_PI) angleError += 2 * M_PI;

            // PID outputs
            double lateralPower = lat_pid->calculate(difference) * cos(angleError);
            double turningPower = turn_pid->calculate(angleError) * sin(angleError);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(lateralPower - turningPower);
                right_motors->move(lateralPower + turningPower);
            }

            // Check if PID is settled or timeout
            if ((lat_pid->isSettled(difference) && turn_pid->isSettled(angleError)) 
                || (pros::millis() - startTime > timeout)
                || (fabs(difference) < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->brake();
                    right_motors->brake();
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
        pros::Task::current().remove();
    });

    /*
    pros::Task moveToPointTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        robotFilter.resample();
        robotFilter.moveUpdate(targetPose.x-robotPose.x, targetPose.y-robotPose.y, targetPose.theta-robotPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
    */
}

//Move the robot to a point with a target heading
void Robot::moveToPose(float x, float y, float theta, int timeout, float maxSpeed, float earlyExitDelta, 
                       float lead, float horizontalDrift, gaussian errorLat, gaussian errorRot) {
    waitUntilFinished();
    finished = false;

    theta *= M_PI/180;
    errorRot*= M_PI/180;

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

    //Reset PID at start of new move
    lat_pid->reset();
    turn_pid->reset();

    pros::Task moveToPose([this, lat_pid, turn_pid, distance, lead, timeout, earlyExitDelta]{
        int startTime = pros::millis();
        while (true) {
            odometer(); //Uses motor encoders to update robot position
            robotPose.theta = (imus[0]->get_heading() * M_PI/180);
            
            double dx = targetPose.x - robotPose.x;
            double dy = targetPose.y - robotPose.y;
            double difference = sqrt(dx * dx + dy * dy);
            double angleError = -atan2(dx, dy) - robotPose.theta;
            // Normalize carrot angleError to [-pi, pi]
            //while (angleError > M_PI) angleError -= 2 * M_PI;
            //while (angleError < -M_PI) angleError += 2 * M_PI;

            double carrotX = targetPose.x - difference*sin(targetPose.theta)*lead;
            double carrotY = targetPose.y - difference*cos(targetPose.theta)*lead;
            double carrotDx = carrotX - robotPose.x;
            double carrotDy = carrotY - robotPose.y;
            double carrotDifference = sqrt(carrotDx*carrotDx + carrotDy*carrotDy);
            double carrotAngleError = -atan2(carrotDx, carrotDy) - robotPose.theta;

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
            if ((lat_pid->isSettled(difference) && turn_pid->isSettled(angleError)) 
                || (pros::millis() - startTime > timeout)
                || (fabs(difference) < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->brake();
                    right_motors->brake();
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
        pros::Task::current().remove();
    });

    /*
    pros::Task moveToPoseTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        robotFilter.resample();
        robotFilter.moveUpdate(targetPose.x-robotPose.x, targetPose.y-robotPose.y, targetPose.theta-robotPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
    */
}

//Turn the robot an ammount relative to current heading
void Robot::turn(float thetaRelative, int timeout, float earlyExitDelta, gaussian errorLat, gaussian errorRot) {
    waitUntilFinished();
    finished = false;

    thetaRelative *= M_PI/180;
    earlyExitDelta *= M_PI/180;
    errorRot *= M_PI/180;

    targetPose.x = robotPose.x;
    targetPose.y = robotPose.y;
    targetPose.theta = robotPose.theta + thetaRelative;

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

    //Reset PID at start of new move
    turn_pid->reset();

    pros::Task turnTask([this, turn_pid, timeout, earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            odometer();

            double dtheta = targetPose.theta - robotPose.theta;

            // PID outputs
            double turningPower = turn_pid->calculate(dtheta);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(turningPower);
                right_motors->move(-turningPower);
            }

            // Check if PID is settled or timeout
            if ((turn_pid->isSettled(dtheta)) || (pros::millis() - startTime > timeout) || (fabs(dtheta) < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->brake();
                    right_motors->brake();
                }
                finished = true;
                pros::screen::print(pros::E_TEXT_MEDIUM, 11 , "Turn Done");
                break;
            }
            pros::delay(20);
        }
        pros::Task::current().remove();
    });
    /*
    pros::Task turnTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        robotFilter.resample();
        robotFilter.moveUpdate(targetPose.x-robotPose.x, targetPose.y-robotPose.y, targetPose.theta-robotPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
    */
}

//Turn the robot to a global heading
void Robot::turnTo(float thetaAbsolute, int timeout, float earlyExitDelta, gaussian errorLat, gaussian errorRot) {
    waitUntilFinished();
    finished = false;
    
    thetaAbsolute *= M_PI/180;
    earlyExitDelta *= M_PI/180;
    errorRot *= M_PI/180;

    targetPose.x = robotPose.x;
    targetPose.y = robotPose.y;
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

    //Reset PID at start of new move
    turn_pid->reset();

    pros::Task turnToTask([this, turn_pid, timeout, earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            odometer();

            double dtheta = targetPose.theta - robotPose.theta;

            // PID outputs
            double turningPower = turn_pid->calculate(dtheta);

            if(fabs(turningPower) < 4) turningPower = 0;

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(turningPower);
                right_motors->move(-turningPower);
            }

            // Check if PID is settled or timeout
            if ((turn_pid->isSettled(dtheta)) || (pros::millis() - startTime > timeout) || (fabs(dtheta) < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->brake();
                    right_motors->brake();
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
        pros::Task::current().remove();
    });

    /*
    pros::Task turnToTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        robotFilter.resample();
        robotFilter.moveUpdate(targetPose.x-robotPose.x, targetPose.y-robotPose.y, targetPose.theta-robotPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
    */
}

//Turn the robot to face a global point
void Robot::turnToPoint(float x, float y, int timeout, float earlyExitDelta, gaussian errorLat, gaussian errorRot) {
    waitUntilFinished();
    finished = false;

    earlyExitDelta += M_PI/180;
    errorRot *= M_PI/180;

    float dx = x - robotPose.x;
    float dy = y - robotPose.y;
    targetPose.theta = -atan2(dx,dy);

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

    //Reset PID at start of new move
    turn_pid->reset();

    pros::Task turnToPointTask([this, turn_pid, timeout, earlyExitDelta] {
        int startTime = pros::millis();
        while (true) {
            odometer(); //Uses motor encoders to update robot position
            robotPose.theta = (imus[0]->get_heading() * M_PI/180);

            double dtheta = targetPose.theta - robotPose.theta;

            // PID outputs
            double turningPower = turn_pid->calculate(dtheta);

            // Set motor power
            if (left_motors && right_motors) {
                left_motors->move(turningPower);
                right_motors->move(-turningPower);
            }

            // Check if PID is settled or timeout
            if ((turn_pid->isSettled(dtheta)) || (pros::millis() - startTime > timeout) || (fabs(dtheta) < earlyExitDelta)) {
                // Stop motors
                if (left_motors && right_motors) {
                    left_motors->brake();
                    right_motors->brake();
                }
                finished = true;
                break;
            }
            pros::delay(20);
        }
        pros::Task::current().remove();
    });

    /*
    pros::Task turnToPointTaskMCL([this, errorLat, errorRot] {
        int startTime = pros::millis();
        robotFilter.resample();
        robotFilter.moveUpdate(targetPose.x-robotPose.x, targetPose.y-robotPose.y, targetPose.theta-robotPose.theta, errorLat, errorLat, errorRot);
        pros::Task::current().remove();
    });
    */
}

Pose Robot::getPose() {
    return robotPose;
}