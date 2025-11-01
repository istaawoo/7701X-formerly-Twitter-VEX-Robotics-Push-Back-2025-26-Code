#include "mcl/auton.hpp"
#include "mcl/robot.hpp"
#include "pros/screen.hpp"

std::vector<Auton> autons = {
    {"Right Solo AWP", auton1},
    {"Left Center Goal Rush", auton2},
    {"Right Center Goal Rush", auton3},
    {"Left Long Goal Rush", auton4},
    {"Right Long Goal Rush", auton5},
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

int selectedAuton = 3 - 1;

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
    robot.move(29,320,3000,70,0);
    robot.waitUntil(14);
    robot.intake2->move_voltage(12000);
    pros::delay(1200);
    robot.intake2->brake();
    robot.turnTo(225,1000,0);
    robot.move(-13,225,2000,100,0);
    robot.waitUntilFinished();
    robot.intake2->move_voltage(12000);
    robot.intake1->move_voltage(-12000);
    pros::delay(3000);
    robot.intake2->brake();
    robot.intake1->brake();
}

void auton3() {
    robot.move(29,30,3000,70,0);
    robot.waitUntil(14);
    robot.intake2->move_voltage(12000);
    pros::delay(1200);
    robot.intake2->brake();
    robot.turnTo(315,1000,0);
    robot.move(13,315,2000,100,0);
    robot.waitUntil(8);
    robot.intake2->move_voltage(-12000);
    robot.waitUntil(1);
    robot.intake2->brake();
    robot.intake1->brake();
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
    robot.move(29,320,3000,70,0);
    robot.waitUntil(14);
    robot.intake2->move_voltage(12000);
    pros::delay(1200);
    robot.intake2->brake();
    robot.turnTo(225,1000,0);
    robot.move(-13,225,2000,100,0);
    robot.waitUntilFinished();
    robot.intake2->move_voltage(12000);
    robot.intake1->move_voltage(-12000);
    pros::delay(3000);
    robot.intake2->brake();
    robot.intake1->brake();
    robot.move(13,225,2000,100);
    robot.turnTo(90,1000,0);
    robot.move(48,90,3000,100);
    robot.waitUntil(14);
    robot.intake2->move_voltage(12000);
    pros::delay(1200);
    robot.intake2->brake();
    robot.turnTo(315,1000,0);
    robot.move(13,315,2000,100,0);
    robot.waitUntil(8);
    robot.intake2->move_voltage(-12000);
    robot.waitUntil(1);
    robot.intake2->brake();
    robot.intake1->brake();
    robot.move(-13,315,2000,100);
    robot.turnTo(0,1000,0);
    robot.move(-36,0,4000,100);
    robot.turnTo(270,1000,0);
    robot.intake2->move_voltage(-12000);
    robot.move(72,270,4000,127);


}






