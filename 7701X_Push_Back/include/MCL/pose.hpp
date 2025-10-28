#pragma once

struct heading { //defines a float that automatically normalizes to [0,360)
    float theta;

    heading(float theta_) : theta(normAngle(theta_)) {}

    static float normAngle(float angle) { //Normalizes an angle to [0,360)
        while(angle >= 360) {angle -= 360;}
        while(angle < 0) {angle += 360;}
        return angle;
    }

    heading operator+(const float other) const {return(normAngle(theta+other));}

    heading operator-(const float other) const {return(normAngle(theta-other));}

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

    heading operator-(const heading other) const {
        float diff = theta - other.theta;
        while(diff >= 180) {diff-=360;}
        while(diff < -180) {diff+=360;}
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
};