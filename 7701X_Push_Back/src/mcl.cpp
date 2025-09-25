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
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "MCL/mcl.hpp"
#include ""
#include <cmath>
#include <iostream> // IWYU pragma: keep
#include <stdio.h>
#include <vector>
#include <variant>
#include <random>
#define M_PI 3.14159265358979323846

//primes randomization:
std::random_device rd; //uses device randomizer for seed
std::mt19937 gen(7701); //fixed seed for now, will use rd() later

sensor right(0,0,90,1);
sensor front(0,0,0,1);
sensor left(0,0,270,1);
sensor back(0,0,180,1);
sensor imu(0,0,0,1);

sensor sensors[5] = {right,front,left,back,imu};

void updateSenseData(float rightSensor, float frontSensor, float leftSensor, float backSensor, float rotSensor) {
    sensors[0].reading = rightSensor;
    sensors[1].reading = frontSensor;
    sensors[2].reading = leftSensor;
    sensors[3].reading = backSensor;
    sensors[4].reading = rotSensor;
}

float randError(gaussian error) { //Uses a gaussian distribution of error to output a random value in that distribution
    std::normal_distribution<float> dist(error.mean,error.stanDev); 
    return dist(gen); 
}

float rayCastWalls(float orginX, float orginY, float rayAngle) { //Input the ray origin x, y and direction angle
    int xMin = -72; //Min X bound of the field (bottom left corner)
    int yMin = -72; //Min Y bound of the field (bottom left corner)
    int xMax = 72;  //Max X bound of the field (top right corner)
    int yMax = 72;  //Max Y bound of the field (top right corner)
    //These values make the center of the field (0,0)

    float dx = cos(rayAngle*M_PI/180); //The rate of change in x of the ray
    float dy = sin(rayAngle*M_PI/180); //The rate of change in y of the ray

    float tX;
    float tY;

    if(dx>0) { //Checks if the x is going in positive direction and thus towards right wall
        tX = (xMax-orginX)/dx; //at what t it intersects the right wall line (not segment)
    } else if(dx<0) { //Checks if the x is going in negative direction and thus towards left wall
        tX = (xMin-orginX)/dx; //at what t it intersects the left wall line (not segment)
    } else {
        tX = std::numeric_limits<float>::infinity();
    }

    if(dy>0) { //Checks if the y is going in the positive direction and thus towards the far wall
        tY = (yMax-orginY)/dy; //at what t it intersects the far wall line (not segment)
    } else if(dy<0) { //Only option left is going towards the near wall
        tY = (yMin-orginY)/dy; //at what t it intersects the near wall line (not segment)
    } else {
        tY = std::numeric_limits<float>::infinity();
    }

    float tMin = std::min(tX,tY);//Finds the smallest time, which will always intersect a wall from within the field
    
    return tMin;
}

float restrainAngle(float theta) { //Confines an angle to between -360 and 360
    return theta-360*floor(theta/360);
}

float closerTarget(float theta, float targetTheta) { //Returns a target theta that is greater than theta if the difference is shorter that way
    if(abs(targetTheta+360-theta) < abs(targetTheta-theta)) {
        return targetTheta+360;
    } else {
        return targetTheta;
    }
}

//particle class functions
void particle::avg(particle p1, particle p2) {
    float totalWeight = p1.weight + p2.weight;
    float comboWeight = p1.weight*p2.weight;
    p1.weight /= totalWeight;
    p2.weight /= totalWeight;
    particle avg(p1.x*p1.weight+p2.x*p2.weight,p1.y*p1.weight+p2.y*p2.weight,p1.theta*p1.weight+p2.theta*p2.weight,comboWeight);
}

void particle::moveUpdate(float dx, float dy, float dtheta) {
    x+=dx;
    y+=dy;
    theta+=dtheta;
}

particle particle::operator=(const lemlib::Pose &other) {
    this->x = other.x;
    this->y = other.y;
    this->theta = other.theta;
    return *this;
}

//Sim robot for testing functions
void particleFilter::simSenses() {
    for(int k; k < 4; k++) {        
        sensors[k].reading = rayCastWalls(robot.x+sensors[k].offX,robot.y+sensors[k].offY,robot.theta+sensors[k].face) + randError(sensors[k].stanDev);
    }
}

//particleFilter class functions
    void particleFilter::useSense(particle* p) { //Check this function
        uint32_t start = pros::millis();
        if(p->x>0 & p->y>0) { //Quadrant 1
            if(45 <= p->theta < 225) {
                sensors[2].use = true;
                sensors[0].use = false;
            } else {
                sensors[2].use = false;
                sensors[0].use = true;
            }
            if(135 <= p->theta < 315) {
                sensors[3].use = true;
                sensors[1].use = false;
            } else {
                sensors[3].use = false;
                sensors[1].use = true;
            }
        }
        if(p->x>0 & p->y<0) { //Quadrant 2
            if(45 < p->theta < 225) {
                sensors[1].use = true;
                sensors[3].use = false;
            } else {
                sensors[1].use = false;
                sensors[3].use = true;
            }
            if(135 < p->theta < 315) {
                sensors[2].use = true;
                sensors[0].use = false;
            } else {
                sensors[2].use = false;
                sensors[0].use = true;
            }
        }
        if(p->x<0 & p->y<0) { //Quadrant 3
            if(45 < p->theta < 225) {
                sensors[0].use = true;
                sensors[2].use = false;
            } else {
                sensors[0].use = false;
                sensors[2].use = true;
            }
            if(135 < p->theta < 315) {
                sensors[1].use = true;
                sensors[3].use = false;
            } else {
                sensors[1].use = false;
                sensors[3].use = true;
            }
        }
        if(p->x>0 & p->y>0) { //Quadrant 2
            if(45 < p->theta < 225) {
                sensors[3].use = true;
                sensors[1].use = false;
            } else {
                sensors[3].use = false;
                sensors[1].use = true;
            }
            if(135 < p->theta < 315) {
                sensors[0].use = true;
                sensors[2].use = false;
            } else {
                sensors[0].use = false;
                sensors[2].use = true;
            }
        }
        for(int k; k < 4; k++) {
            if(sensors[k].reading > 72) {
                sensors[k].use = false;
            }
        }
        uint32_t end = pros::millis();
        useSenseTime += end - start;
    }

    void particleFilter::predictDistance(particle* p) {
        useSense(p);
        uint32_t start = pros::millis();
        for(int k; k < 4; k++) {
            if(sensors[k].use = true) {
                float distance = rayCastWalls(p->x+sensors[k].offX,p->y+sensors[k].offY,p->theta+sensors[k].face);
                if(distance < 72) {
                    p->expSense[k] = distance;
                } else {
                    sensors[k].use = false;
                }
            }
        }
        uint32_t end = pros::millis();
        predictSenseTime += end-start;
    }

    //Adds particles at a starting location with random setup error;
    void particleFilter::initializeParticles(float initialX, float initialY, float initialTheta, 
                                            gaussian errorX, gaussian errorY, gaussian errorTheta) {
        //SIM ROBOT, testing only
        robot.x = initialX + randError(errorX);
        robot.y = initialY + randError(errorY);
        robot.theta = initialTheta + randError(errorTheta);
        simSenses();
        sensors[4].reading = 0;
        
        for(int p; p < maxParticles; p++) {
            float x_ = initialX + randError(errorX);
            float y_ = initialY + randError(errorY);
            float theta_ = initialTheta + randError(errorTheta);

            particles.push_back(new particle(x_,y_,theta_));

            predictDistance(particles[p]);
            particles[p]->expSense[4] = 0; 
        }
    }

    void particleFilter::turnMoveUpdate(float targetTheta, gaussian errorX, gaussian errorY, gaussian errorTheta) {
        //Sim robot, testing only
        robot.x += randError(errorX);
        robot.y += randError(errorY);
        robot.theta = targetTheta + randError(errorTheta);
        simSenses();
        
        uint32_t start = pros::millis();
        for(particle* p : particles) {
            p->expSense[5] = targetTheta - p->theta;
            p->x += randError(errorX);
            p->y += randError(errorY);
            p->theta = targetTheta + randError(errorTheta);
            predictDistance(p);
        }
        uint32_t end = pros::millis();
        moveTime = end - start - predictSenseTime - useSenseTime;
    }

    void particleFilter::driveDistMoveUpdate(float targetDistance, gaussian errorDistance, gaussian errorDrift, gaussian errorTheta) {
        //Sim robot, testing only
        float initialTheta = robot.theta;
        float dist = targetDistance + randError(errorDistance); //predicted inches moved in this update plus random error
        float drift = randError(errorDrift); //calculates random drift error
        float angleShift = atan2(drift,dist); //how much the actual heading of the move shifted during the move
        float actualDist = sqrt(dist*dist+drift*drift); //the actual distance traveled based on move error and drift
        robot.x += cos(robot.theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
        robot.y += sin(robot.theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
        robot.theta += randError(errorTheta); //adds random error to particle angle.
        simSenses();
        sensors[4].reading = robot.theta - initialTheta + randError(sensors[4].stanDev);

        
        uint32_t start = pros::millis();
        for(particle* p : particles) {
            float initialTheta = p->theta;
            float dist = targetDistance + randError(errorDistance); //predicted inches moved in this update plus random error
            float drift = randError(errorDrift); //calculates random drift error
            float angleShift = atan2(drift,dist); //how much the actual heading of the move shifted during the move
            float actualDist = sqrt(dist*dist+drift*drift); //the actual distance traveled based on move error and drift
            p->x += cos(p->theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
            p->y += sin(p->theta-angleShift)*actualDist; //Uses linar distance and drift to caluclate x position
            p->theta += randError(errorTheta); //adds random error to particle angle.
            predictDistance(p);
            p->expSense[4] = p->theta - initialTheta;
        }
        u_int32_t end = pros::millis();
        moveTime = end - start - predictSenseTime - useSenseTime;
    }

    //updates weights of particles based on how likely they are to recieve the sensor data recieved
    void particleFilter::senseUpdate() {
        uint32_t start = pros::millis();
        float totalWeight = 0; //defines a variable for the sum of all particle weights. Used in normalization 
        for(particle* p : particles) { //loops through each particle
            p->weight = 1; //sets weight to 1 initially so the *= can be used for sensors afterwards
            for(int k = 0;k<5; k++) { //calulates the sensor weights by looping and taking the product of gaussian probability that the sensor readings would be what they are at each particle
                if(sensors[k].use = true) {
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
        uint32_t senseTime = end - start;
    }

    void particleFilter::predictPosition() { //take the wighted sum of all particles positions to determin the predicted position
        u_int32_t start = pros::millis();
        position.x = 0;
        position.y = 0;
        position.theta = 0;
        for(particle* p : particles) {
            position.x += p->x*p->weight;
            position.y += p->y*p->weight;
            position.theta += p->theta*p->weight;
        }
        u_int32_t end = pros::millis();
        predictPosTime = end - start;
    }

    //resamples the particles, favoring those with high weights and adding some noise. Converges particles on likely robot position.
    void particleFilter::resample() {
        u_int32_t startTime = pros::millis();
        //std::vector<particle*> resampledParticles; //defines a vector of resampled particles

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
            particles[p]->x = particles[index]->x +randError(statNoiseLinear);
            particles[p]->y= particles[index]->y +randError(statNoiseLinear);
            particles[p]->theta = particles[index]->theta +randError(statNoiseRot);
            //particle* resampledPart = new particle(*particles[index]);
            //resampledParticles.push_back(resampledPart);

            //Adds some noise to the resampled particles
            //resampledParticles[p]->x += randError(statNoiseLinear);
            //resampledParticles[p]->y += randError(statNoiseLinear);
            //resampledParticles[p]->theta += randError(statNoiseRot);
        }
        //for (particle* p : particles) delete p; //clears the old particles
        //particles = resampledParticles; //redifines the particle vector as the resampled vector

        uint32_t end = pros::millis();
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