#include "main.h"
#include "screen.hpp"
#include "auton.hpp"
#include "rtos.hpp"
#include "misc.hpp"

void displayAutons() {
    pros::lcd::clear();
    for (int i = 0; i < autons.size(); i++) {
        if (i == selectedAuton) {
            pros::lcd::print(i, "> %s", autons[i].name.c_str());
        } else {
            pros::lcd::print(i, "  %s", autons[i].name.c_str());
        }
    }
}

