#pragma once
#include "mcl/pose.hpp"
#include <vector>
#include <string>
#include <functional>

struct Auton {
    std::string name;
    std::function<void()> func;
};

void auton1();
void auton2();
void auton3();
void auton4();
void auton5();
void auton6();
void auton7();
void auton8();
void auton9();
void auton10();
void auton11();
void auton12();
void noAuton();
void skillsAuton();

void displayAutons();
void on_center_button();
void on_left_button();
void on_right_button();

// global storage
extern std::vector<Auton> autons;
extern int selectedAuton;
extern Pose autonPose[14];

