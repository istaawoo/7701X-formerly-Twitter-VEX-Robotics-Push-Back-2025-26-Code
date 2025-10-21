#pragma once

class Pose { //Variable that stores and x, y and theta position
public:
    float x;     //x-cord
    float y;     //y-cord
    float theta; //angle of heading

    Pose(float x_=0, float y_=0, float theta_ = 0) : x(x_), y(y_), theta(theta_) {}

    Pose& operator=(const Pose &other) { //sets the pose cords to the cords of a Pose
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
        return *this;
    }
};

struct gaussian { //defines a gaussian with a mean and standard deviation
    float mean, stanDev;
    gaussian(float stanDev_, float mean_ = 0) : stanDev(stanDev_), mean(mean_) {}
};