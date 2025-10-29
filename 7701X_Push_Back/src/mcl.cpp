#include "pros/abstract_motor.hpp"
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.h"  // IWYU pragma: keep
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "mcl/mcl.hpp"

#include <cmath>
#include <iostream> // IWYU pragma: keep
#include <stdio.h>
#include <vector>
#include <variant>
#include <random>

//primes randomization:
std::random_device rd; //uses device randomizer for seed
std::mt19937 gen(7701); //fixed seed for now, will use rd() later

/*
void updateSenseData(float rightSensor, float frontSensor, float leftSensor, float backSensor, float imu1, float imu2) {
    sensors[0].reading = rightSensor;
    sensors[1].reading = frontSensor;
    sensors[2].reading = leftSensor;
    sensors[3].reading = backSensor;
    sensors[4].reading = imu1;
    sensors[5].reading = imu2;
}
*/

sensor mclRight(0,0,1.5*M_PI,1);
//sensor mclFront(0,0,0,1);
sensor mclLeft(0,0,.5*M_PI,1);
//sensor mclBack(0,0,M_PI,1);
sensor mclImu1(0,0,0,5);
//sensor mclImu2(0,0,0,5);

sensor sensors[3] = {mclRight,mclLeft,mclImu1};

float randError(gaussian error) { //Uses a gaussian distribution of error to output a random value in that distribution
    std::normal_distribution<float> dist(error.mean,error.stanDev); 
    return dist(gen); 
}

float rayCastWalls(float orginX, float orginY, float rayAngle) { //Input the ray origin x, y and direction angle
    int xMin = -1828; //Min X bound of the field (bottom left corner)
    int yMin = -1828; //Min Y bound of the field (bottom left corner)
    int xMax = 1828;  //Max X bound of the field (top right corner)
    int yMax = 1828;  //Max Y bound of the field (top right corner)
    //These values make the center of the field (0,0)

    float dx = sin(rayAngle); //The rate of change in x of the ray
    float dy = cos(rayAngle); //The rate of change in y of the ray

    float tX;
    float tY;

    if(dx>0) { //Checks if the x is going in positive direction and thus towards right wall
        tX = (xMax-(orginX*25.4))/dx; //at what t it intersects the right wall line (not segment), (25.4 converts from inches to mm)
    } else if(dx<0) { //Checks if the x is going in negative direction and thus towards left wall
        tX = (xMin-(orginX*25.4))/dx; //at what t it intersects the left wall line (not segment), (25.4 converts from inches to mm)
    } else {
        tX = std::numeric_limits<float>::infinity();
    }

    if(dy>0) { //Checks if the y is going in the positive direction and thus towards the far wall
        tY = (yMax-(orginY*25.4))/dy; //at what t it intersects the far wall line (not segment), (25.4 converts from inches to mm)
    } else if(dy<0) { //Only option left is going towards the near wall
        tY = (yMin-(orginY*25.4))/dy; //at what t it intersects the near wall line (not segment), (25.4 converts from inches to mm)
    } else {
        tY = std::numeric_limits<float>::infinity();
    }

    if (std::isinf(tX) || std::isnan(tX)) tX = std::numeric_limits<float>::max();
    if (std::isinf(tY) || std::isnan(tY)) tY = std::numeric_limits<float>::max();

    float tMin = std::min(tX,tY); //Finds the smallest time, which will always intersect a wall from within the field
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Sensor value distance: %f", floor(tMin));
    pros::delay(2);
    return tMin;
}

    void particleFilter::predictDistance(particle* p) {
        //useSense(p); //Function depricated for now
        uint32_t start = pros::millis();
        for(int k = 0; k < 2; k++) {
            p->expSense[k] = rayCastWalls(p->x+sensors[k].offX,p->y+sensors[k].offY,p->theta+sensors[k].face);
            pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Sensor %i", k);
            pros::delay(2);
        }
        uint32_t end = pros::millis();
        predictSenseTime += end-start;
    }

    //Adds particles at a starting location with random setup error;
    void particleFilter::initializeParticles(float initialX, float initialY, float initialTheta, 
                                            gaussian errorX, gaussian errorY, gaussian errorTheta) {
        u_int32_t start = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 0 , "initialize start: %lu", start);
        for(int p = 0; p < maxParticles; p++) {
            float x_ = initialX + randError(errorX);
            float y_ = initialY + randError(errorY);
            heading theta_ = initialTheta + randError(errorTheta);

            particle* newParticle = new particle(x_,y_,theta_);
            particles.push_back(newParticle);
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Working on particle %i", p);
            pros::delay(2);

            predictDistance(newParticle);
            newParticle->expSense[2] = 0; 
            pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Initialized particle %i", p);
            pros::delay(2);
        }
        u_int32_t end = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 0 , "initialize start: %lu done: %lu", start, end);
    }

    //moves the particles to a positions randomly offset from target position.
    void particleFilter::moveUpdate(float dx, float dy, float dtheta, gaussian errorX, gaussian errorY, gaussian errorTheta) {
        uint32_t start = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 2 , "move update start: %lu", start);
        for(particle* p : particles) {
            p->x += dx + randError(errorX);
            p->y += dy + randError(errorY);
            p->theta += dtheta + randError(errorTheta);
            p->expSense[2] = p->theta;
            predictDistance(p);
        }
        uint32_t end = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 2 , "move update start: %lu done: %lu", start, end);
        moveTime = end - start - predictSenseTime;
    }

    //updates weights of particles based on how likely they are to recieve the sensor data recieved
    void particleFilter::senseUpdate() {
        uint32_t start = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 3 , "sense update start: %lu", start);
        for(int i = 0; i<2; i++) { //Set readings for each distance sensor
            float reading = (*robotDistances)[i]->get();
            if (reading != PROS_ERR) {
                sensors[i].reading = reading;
                sensors[i].use = true;
            } else {
                sensors[i].use = false;
            }
        }
        for(int i = 0; i<1; i++) { //Set readings for each inertial sensor
            float reading = (*robotImus)[i]->get_heading();
            if(reading != PROS_ERR) { //checks reading is not an error.
                sensors[i+2].reading = reading;
                sensors[i+2].use = true;
            } else {
                sensors[i+2].use = false;
            }
        }
        
        float totalWeight = 0; //defines a variable for the sum of all particle weights. Used in normalization 
        for(particle* p : particles) { //loops through each particle
            p->weight = 1; //sets weight to 1 initially so the *= can be used for sensors afterwards
            for(int k = 0; k<3; k++) { //calulates the sensor weights by looping and taking the product of gaussian probability that the sensor readings would be what they are at each particle
                if(sensors[k].use) {
                    //Save stanDev^2 in the sensor variable
                    p->weight *= exp(-1*(sensors[k].reading-p->expSense[k])*(sensors[k].reading-p->expSense[k])/(2*sensors[k].stanDev*sensors[k].stanDev));
                }
            }
            totalWeight += p->weight; //adds unNormalized weight of each particle to total weight
        }
        for(particle* p : particles) { //loops through each particle to divide calulated weight by total weight to normalize the weights (Dividing weights by total weights so they are the chance the robot is at that particle out of every other particle)
            p->weight /= totalWeight;
        }

        uint32_t end = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 3 , "sense update start: %lu done: %lu", start, end);
        uint32_t senseTime = end - start;
    }

    Pose particleFilter::predictPosition() { //take the wighted sum of all particles positions to determin the predicted position
        u_int32_t start = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 4 , "predict position start: %lu", start);
        
        Pose position(0,0,0); //Initializes position to 0.
       
        for(particle* p : particles) {
            position.x += p->x*p->weight;
            position.y += p->y*p->weight;
            position.theta += p->theta*p->weight;
        }
        u_int32_t end = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 4 , "predict position start: %lu done: %lu", start, end);
        predictPosTime = end - start;
        return position;
    }

    //resamples the particles, favoring those with high weights and adding some noise. Converges particles on likely robot position.
    void particleFilter::resample() {
        u_int32_t startTime = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 5 , "resample start: %lu", startTime);

        std::vector<particle*> resampledParticles;
        float step = 1.0/particles.size(); //Regular step ammount
        std::uniform_real_distribution<float> dist(0,step); //defines a random number range from 0 to the step. 
        float start = dist(gen); //Random starting point in predefined range ^
        float cumulativeWeight = particles[0]->weight; //sets first cumulative weight to the first particles weight
        int index = 0;

        for(int p = 0; p < particles.size(); p++) { 
            float target = start + p*step; //sets a target cumilative weight that it wants to add a particle at
            while (target > cumulativeWeight) { //skips over particle weights that are bellow target (they are likely small chances)
                index++; //moves down the list of particles
                cumulativeWeight += particles[index]->weight; //adds next particles weight to get the cumulative weight for that particle to check again
            }
            //adds the resampled particle to the feild and adds some noise
            resampledParticles.push_back(new particle(
                particles[index]->x + randError(statNoiseLinear),
                particles[index]->y + randError(statNoiseLinear),
                particles[index]->theta + randError(statNoiseRot)
            ));
        }

        for (particle* p : particles) delete p;
        particles = std::move(resampledParticles);

        uint32_t end = pros::millis();
        pros::screen::print(pros::E_TEXT_MEDIUM, 5 , "resample start: %lu done: %lu", startTime, end);
        resampleTime = end - startTime;
    }
    
    /*
    MOVE UPDATES THAT UPDATE DURING THE MOVEMENT

    void particleFilter::turnMoveUpdate(float targetTheta, float initialUpdateTime, gaussian errorTheta) {
        float deltaTheta = closerTarget(position.theta,targetTheta)-position.theta;
        uint32_t startTime = pros::millis(); //sets the start time for the first update to be the start of the move command
        uint32_t endTime;
        uint32_t lastUpdateTime = initialUpdateTime; //for first update it is a preset value. This will be used to estimate the time the current update will take to make the move update
        float rotSpeed; //robot speed. Will likely be calculated by some function when actually implemented in a move command
        float dTheta = rotSpeed*initialUpdateTime;
        float totalTurned = 0;

        while(deltaTheta-t/otalTurned<dTheta) {
            //Will likely set robot speed here.
            float updateFraction = dTheta/deltaTheta; //what percent of the total move is this update
            gaussian errorThetaPer = {errorTheta.mean,errorTheta.stanDev*updateFraction}; //Splits up error for whole move by the distance turned in this update

            //SIM ROBOT, Testing only
            
            robot.theta = restrainAngle(robot.theta + dTheta + randError(errorThetaPer));
            simSenses();
            
            
            //Move update portion
            for(int p=0;p<particles.size();p++) { //loops through all particles
                particles[p].theta = restrainAngle(particles[p].theta + dTheta + randError(errorThetaPer)); //adds change in theta and random error to partilce angle
                predictSenses(p);
            }

            //The sensor data at this time will be used to guess the robots position.

            endTime = pros::millis(); //stores the end time for the computation of robots position. Will be compared to the start time set in the last update.
            lastUpdateTime = endTime - startTime; //How long the sense updates took. Will be used to make next move update.
            startTime = pros::millis(); //stores the time that the robots position is being predicted for (start of sense update computation). Will be used to caluclate last update time in next update
            senseUpdate();
            predictPosition();
            resample();

            dTheta = rotSpeed*lastUpdateTime; //uses last update time * by robot speed to predict the distance that has been turned since last robot position estimate.
            //defiens dDist for next move, will be used to check while loop
        }
        dTheta = targetTheta-totalTurned;
        float updateFraction = dTheta/deltaTheta; //what percent of the total move is this update
        gausian errorThetaPer = {errorTheta.mean,errorTheta.stanDev*updateFraction}; //Splits up error for whole move by the distance turned in this update

        for(int p=0;p<particles.size();p++) { //loops through all particles
            particles[p].theta = restrainAngle(particles[p].theta + dTheta + randError(errorThetaPer)); //adds change in theta and random error to partilce angle
            predictSenses(p);
        }

        senseUpdate();
        predictPosition();
        resample();
        move++; //used for printing to brain screen, testing.
    }

    //updates particles for a linear movement
    void particleFilter::driveDistMoveUpdate(float targetDistance, float initialUpdateTime, gausian errorDistance, gausian errorDrift, gausian errorTheta) {
        uint32_t startTime = pros::millis(); //sets the start time for the first update to be the start of the move command
        uint32_t endTime;
        uint32_t lastUpdateTime = initialUpdateTime; //for first update it is a preset value. This will be used to estimate the time the current update will take to make the move update
        float lineSpeed; //robot speed. Will likely be calculated by some function when actually implemented in a move command
        float dDist = lineSpeed*initialUpdateTime;
        float totalDist = 0;

        while(targetDistance-totalDist<dDist) {
            float updateFraction = dDist/targetDistance;
            gausian errorDistPer = {errorDistance.mean,errorDistance.stanDev*updateFraction};
            gausian errorDriftPer = {errorDrift.mean,errorDrift.stanDev*updateFraction};
            gausian errorThetaPer = {errorTheta.mean,errorTheta.stanDev*updateFraction};

            //SIM ROBOT, testing only
            
            float dist = dDist + randError(errorDistPer); //predicted inches moved in this update plus random error
            float drift = randError(errorDriftPer);
            robot.x += cos(robot.theta-atan2(drift,dDist))*sqrt(dDist*dDist+drift*drift);
            robot.y += sin(robot.theta-atan2(drift,dDist))*sqrt(dDist*dDist+drift*drift);
            robot.theta += randError(errorThetaPer);
            simSenses();
            
            
            //Move update portion
            for(int p=0;p<particles.size();p++) {
                float dist = dDist + randError(errorDistPer); //predicted inches moved in this update plus random error
                float drift = randError(errorDriftPer); //calculates random drift error
                float angleShift = atan2(drift,dist); //how much the actual heading of the move shifted during the move
                float actualDist = sqrt(dist*dist+drift*drift); //the actual distance traveled based on move error and drift
                particles[p].x += cos(particles[p].theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
                particles[p].y += sin(particles[p].theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
                particles[p].theta += randError(errorThetaPer); //adds random error to particle angle.
                predictSenses(p);
            }

            endTime = pros::millis(); //stores the end time for the computation of robots position. Will be compared to the start time set in the last update.
            lastUpdateTime = endTime - startTime; //How long the sense updates took. Will be used to make next move update.
            startTime = pros::millis(); //stores the time that the robots position is being predicted for (start of sense update computation). Will be used to caluclate last update time in next update
            senseUpdate();
            predictPosition();
            resample();


            totalDist += dDist;
            dDist = lineSpeed*lastUpdateTime; //defiens dDist for next move, will be used to check while loop
        }
        dDist = targetDistance-totalDist;
        float updateFraction = dDist/targetDistance;
        gausian errorDistPer = {errorDistance.mean,errorDistance.stanDev*updateFraction};
        gausian errorDriftPer = {errorDrift.mean,errorDrift.stanDev*updateFraction};
        gausian errorThetaPer = {errorTheta.mean,errorTheta.stanDev*updateFraction};

        for(int p=0;p<particles.size();p++) {
            float dist = dDist + randError(errorDistPer); //predicted inches moved in this update plus random error
            float drift = randError(errorDriftPer); //calculates random drift error
            float angleShift = atan2(drift,dist); //how much the actual heading of the move shifted during the move
            float actualDist = sqrt(dist*dist+drift*drift); //the actual distance traveled based on move error and drift
            particles[p].x += cos(particles[p].theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
            particles[p].y += sin(particles[p].theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
            particles[p].theta += randError(errorThetaPer); //adds random error to particle angle.
            predictSenses(p);
        }

        senseUpdate();
        predictPosition();
        resample();
        move++; //used to print to brain screen, for testing
    }
    */

    /*    void particleFilter::liveMoveUpdate(float dR, float dL, gaussian errorR, gaussian errorL, float* trackWidth) {
            for(particle* p : particles) {
                float dR = dR + randError(errorR);
                float dL = dL + randError(errorL);

                float dtheta = (dL - dR) / *trackWidth; //Change in heading
                float trackRadius = (dR/dtheta) + (*trackWidth/2); //Radius of the turn based on right wheel
                float trackChord = 2 * trackRadius * sin(dtheta/2); //Chord length of the turn
    
                p->x += trackChord * sin(p->theta + (dtheta/2)); //Change in x position
                p->y += trackChord * cos(p->theta + (dtheta/2)); //Change in y position
                p->theta += dtheta;
                p->expSense[2] += dtheta;
                predictDistance(p);
            }
        }

    //Probably not worth the extra math, more accurate.
    void particleFilter::turnMoveUpdate(float targetTheta, gaussian errorX, gaussian errorY, gaussian errorTheta) {
        uint32_t start = pros::millis();
        for(particle* p : particles) {
            p->x += randError(errorX);
            p->y += randError(errorY);
            p->theta = targetTheta + randError(errorTheta);
            p->expSense[2] += p->theta;
            predictDistance(p);
        }
        uint32_t end = pros::millis();
        moveTime = end - start - predictSenseTime;
    }

    //Probably not worth the extra math, more accurate.
    void particleFilter::driveDistMoveUpdate(float targetDistance, gaussian errorDistance, gaussian errorDrift, gaussian errorTheta) {
        uint32_t start = pros::millis(); //move update start time
        for(particle* p : particles) {
            float dist = targetDistance + randError(errorDistance); //predicted inches moved in this update plus random error
            float drift = randError(errorDrift); //calculates random drift error
            float angleShift = atan2(drift,dist); //how much the actual heading of the move shifted during the move
            float actualDist = sqrt(dist*dist+drift*drift); //the actual distance traveled based on move error and drift
            p->x += sin((p->theta)-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
            p->y += cos((p->theta)-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
            p->theta += randError(errorTheta); //adds random error to particle angle.
            predictDistance(p);
            p->expSense[2] += p->theta;
        }
        u_int32_t end = pros::millis(); //move update end time
        moveTime = end - start - predictSenseTime; //total time for mcl
    }*/