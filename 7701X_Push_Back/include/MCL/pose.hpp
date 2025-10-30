#pragma once
#include <cmath>
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

struct heading { //defines a float that automatically normalizes to [0,360)
    float theta;

    heading(float theta_) : theta(normAngle(theta_)) {}

    static float normAngle(float angle) { //Normalizes an angle to [0,360)
        while(angle >= 2*M_PI) {angle -= 2*M_PI;}
        while(angle < 0) {angle += 2*M_PI;}
        return angle;
    }

    heading operator+(const float other) const {return(heading(theta+other));}

    heading operator-(const float other) const {return(heading(theta-other));}

    heading& operator=(const float other) {
        this->theta = normAngle(other);
        return *this;
    }

    heading& operator+=(const float other) {
        this->theta = normAngle(theta+other);
        return *this;
    }

    heading& operator-=(const float other) {
        this->theta = normAngle(theta-other);
        return *this;
    }

    float operator-(const heading other) const {
        pros::screen::print(pros::E_TEXT_MEDIUM, 2 , "StuffL %f, %f",theta, other.theta);
        float diff = theta - other.theta;
        while(diff > M_PI) {diff-=2*M_PI;}
        while(diff <= -M_PI) {diff+=2*M_PI;}
        return diff;
    }

    operator float() const {return theta;}
};

class Pose { //Variable that stores and x, y and theta position
public:
    float x;     //x-cord
    float y;     //y-cord
    heading theta; //angle of heading

    Pose(float x_=0, float y_=0, float theta_ = 0) : x(x_), y(y_), theta(theta_) {}

    Pose operator+(const Pose &other) const {
        return Pose(x+other.x,y+other.y);
    }

    Pose operator-(const Pose &other) const {
        return Pose(x-other.x,y-other.y);
    }

    Pose& operator=(const Pose &other) { //sets the pose cords to the cords of a Pose
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
        return *this;
    }

    Pose& operator+=(const Pose &other) {
        this->x += other.x;
        this->y += other.y;
        return *this;
    }

    Pose& operator-=(const Pose &other) {
        this->x -= other.x;
        this->y -= other.y;
        return *this;
    }
};

struct gaussian { //defines a gaussian with a mean and standard deviation
    float mean, stanDev;
    gaussian(float stanDev_, float mean_ = 0) : stanDev(stanDev_), mean(mean_) {}

    gaussian operator*(const float other) {
        return gaussian(stanDev*other,mean*other);
    }

    gaussian operator/(const float other) {
        return gaussian(stanDev/other,mean/other);
    }

    gaussian& operator*=(const float other) {
        this->mean *= other;
        this->stanDev *= other;
        return *this;
    }

    gaussian& operator/=(const float other) {
        this->mean /= other;
        this->stanDev /= other;
        return *this;
    }
};