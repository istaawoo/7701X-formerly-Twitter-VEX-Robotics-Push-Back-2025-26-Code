#pragma once

#include <vector>
#include <variant>

struct sensor { //defines a sensor position with offset from center of roation in Y and X and angle facing from front of robot. Stores if we are using the sensor
    //std::variant<pros::Distance, pros::Imu, pros::Rotation> theSensor; not needed in testing
    float offX, offY, face, reading, stanDev;
    bool use = true;

    sensor(float offX_, float offY_, float face_, float stanDev_) : offX(offX_), offY(offY_), face(face_), stanDev(stanDev_) {}
};

struct gaussian { //defines a gaussian with a mean and standard deviation
    float mean, stanDev;
    gaussian(float stanDev_, float mean_ = 0) : stanDev(stanDev_), mean(mean_) {}
};

void updateSenseData(double rightSensor, double frontSensor, double leftSensor, double backSensor, double rotSensor);

class particle { //defines a particle. (A guess of where the robot is)  
public:
    //Inheirites x, y and theta from lemlib Pose class.
    float x;
    float y;
    float theta;
    float weight; //how likely the robot is at that particle

    particle(float x_, float y_, float theta_, float weight_ = 1) : x(x_), y(y_), theta(theta_), weight(weight_) {}

    float expSense[5]; //array of all the expected sensor values. Used to loop through them.

    particle operator+(const particle &other) const {
        return particle(x+other.x,y+other.y,theta+other.theta);
    }

    particle operator-(const particle &other) const {
        return particle(x-other.x,y-other.y,theta-other.theta);
    }

    particle operator*(const float& other) const {
        return particle(x*other,y*other,theta*other);
    }

    particle operator/(const float& other) const {
        return particle(x/other,y/other,theta/other);
    }

    particle operator=(const particle &other) {
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
        this->weight = 1;
        return *this;
    }

    void avg(particle p1, particle p2);

    void moveUpdate(float dx, float dy, float dtheta);
    
    /*
    expSense[0]: right distance sensor: expected right distance sensor value at this particle
    expSense[1]: front distance sensor: expected front distance sensor value at this particle
    expSense[2]: left distance sensor: expected left distance sensor value at this particle
    expSense[3]: back distance sensor: expected back distance sensor value at this particle
    expSense[4]: rotation sensor IMU: expected IMU sensor rotation value at this particle
    */    
};

class particleFilter {
private:
    const int maxParticles; //Number of particles in the filter
    std::vector<particle*> particles; //create a vector of all the particles
    particle position;
    particle robot;
public:
    int move = 0;

    //Time variables, for testing:
    
    uint32_t useSenseTime = 0; //Time to decide what senses to use 
    uint32_t predictSenseTime = 0; //Time to predict senses (not use sense?)
    uint32_t moveTime = 0; //Move update time (not sense prediction?)
    uint32_t senseTime = 0; //Sense update time
    uint32_t predictPosTime = 0; //Predict Position time
    uint32_t resampleTime = 0; //Resampling time
    uint32_t executionTime = 0; //Total run time

    float sensorSim(sensor sensor);

    void simSenses();

    void useSense(particle* p);

    void predictDistance(particle* p);

    gaussian statNoiseLinear;
    gaussian statNoiseRot;

    particleFilter(int total, gaussian statNoiseL, gaussian statNoiseR) : statNoiseLinear(statNoiseL), statNoiseRot(statNoiseR), 
    maxParticles(total), position(0,0,0), robot(0,0,0) {}

    void initializeParticles(float initialX, float initialY, float initialTheta, 
                            gaussian errorX, gaussian errorY, gaussian errorTheta);


    //Default Error Values

    void moveUpdate(float targetX, float targetY, float targetTheta, float errorX, float errorY, float errorTheta);

    void turnMoveUpdate(float targetTheta, gaussian errorX, gaussian errorY, gaussian errorTheta);

    void driveDistMoveUpdate(float targetDistance, gaussian errorDistance, gaussian errorDrift, gaussian errorTheta);

    void senseUpdate();

    void predictPosition();

    void resample();
};
