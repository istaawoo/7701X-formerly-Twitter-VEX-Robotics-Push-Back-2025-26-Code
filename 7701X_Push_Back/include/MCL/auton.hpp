#pragma once
#include <vector>
#include <string>
#include <functional>

struct Auton {
    std::string name;
    std::function<void()> func;
};

void autonLeft();
void autonRight();
void autonSkills();

void displayAutons();
void on_center_button();
void on_left_button();
void on_right_button();

// global storage
extern std::vector<Auton> autons;
extern int selectedAuton;
