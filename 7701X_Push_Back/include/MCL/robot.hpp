#pragma once
//movements: forwards distance, backwards distance, to-point, turn to-point, turn to-heading

class pose { //Variable that stores and x, y and theta position
public:
    float x; //x-cord
    float y; //y-cord
    float theta; //angle of heading

    pose(float x_, float y_, float theta_ = 0) : x(x_), y(y_), theta(theta_) {}

    pose operator=(const particle &other) { //sets the pose cords to the cords of a particle
        this->x = other.x;
        this->y = other.y;
        this->theta = other.theta;
    }
};

class robot {
public:
    float trackWidth;
    float trackLength;
    float wheelRatio;
    float wheelSize;
    float rotCenterDistance;

    pose position;

    void moveTurnHeading(float targetTheta, gaussian errorX, gaussian errorY, gaussian errorTheta);

    void moveTurnPoint(float targetTheta, gaussian errorX, gaussian errorY, gaussian errorTheta);

    void moveForwards(float targetDistance, gaussian errorDistance, gaussian errorDrift, gaussian errorTheta);

    void moveBackwards(float targetDistance, gaussian errorDistance, gaussian errorDrift, gaussian errorTheta);

    void move() {
        
    }
};