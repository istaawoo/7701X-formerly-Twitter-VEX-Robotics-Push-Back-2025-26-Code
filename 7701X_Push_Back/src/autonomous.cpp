#include "mcl/auton.hpp"
#include "mcl/robot.hpp"
#include "pros/screen.hpp"

std::vector<Auton> autons = {
    {"Right Solo AWP", auton1},
    {"Right Center Goal Rush", auton2},
    {"Right Long Goal Rush", auton3},
    {"", auton4},
    {"", auton5},
    {"", auton6},
    {"", auton7},
    {"", auton8},
    {"", auton9},
    {"", auton10},
    {"", auton11},
    {"", auton12},
    {"NO AUTON", noAuton},
    {"SKILLS AUTON", skillsAuton},
};

int selectedAuton = 2 - 1;

Pose autonPose[14];

void auton1() {
    autonPose[0] = {0,0,0};
    //pros::screen::print(pros::E_TEXT_MEDIUM, 9 , "Running");
    //pros::delay(500);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 9 , "Place Done");
    //robot.checkStart();
    //pros::delay(500);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 9 , "Check Start Done");
    //robot.move(24, 0, 2000, 1, 0, {0,1}, {0,5});
    //pros::screen::print(pros::E_TEXT_MEDIUM, 5 , "Turn Start");

    robot.move(24,0,4000,128,0);
    robot.move(-24,0,8000,128,0);

    pros::delay(500);
    
    //pros::screen::print(pros::E_TEXT_MEDIUM, 1 , "predictSense Time: %lu", robot.robotFilter.predictSenseTime);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 2 , "move update time Time: %lu", robot.robotFilter.moveTime);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 3 , "sense update Time: %lu", robot.robotFilter.senseTime);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 4 , "predict position Time: %lu", robot.robotFilter.predictPosTime);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 5 , "resample Time: %lu", robot.robotFilter.resampleTime);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 6 , "total Time: %lu", robot.robotFilter.executionTime);
    //pros::screen::print(pros::E_TEXT_MEDIUM, 7 , "Position Guess X: %.2f Y: %.2f Theta: %.2f", robot.getPose().x, robot.getPose().y, robot.getPose().theta);
}

void auton2() {
    autonPose[1] = {0,0,0};
    robot.intake1->move_voltage(12000);
    robot.move(36, 0, 2000, 127, 0);
    pros::delay(500);
    robot.turnTo(225, 1000, 0);
    robot.move(-12,45,2000,60,0);
    robot.intake2->move_voltage(-12000);
    pros::delay(2000);
    robot.intake1->brake();
    robot.intake2->brake();
}

void auton3() {
    autonPose[2] = {0,0,0};
}

void auton4() {
    autonPose[3] = {0,0,0};
}

void auton5() {
    autonPose[4] = {0,0,0};
}

void auton6() {
    autonPose[5] = {0,0,0};
}

void auton7() {
    autonPose[6] = {0,0,0};
}

void auton8() {
    autonPose[7] = {0,0,0};
}

void auton9() {
    autonPose[8] = {0,0,0};
}

void auton10() {
    autonPose[9] = {0,0,0};
}

void auton11() {
    autonPose[10] = {0,0,0};
}

void auton12() {
    autonPose[11] = {0,0,0};
}

void noAuton() {
    autonPose[12] = {0,0,0};
}

void skillsAuton() {
    autonPose[13] = {0,0,0};
}






