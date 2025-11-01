//include main
#include "main.h"

//include pros files
#include "liblvgl/llemu.h" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp" // IWYU pragma: keep 
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.h"  // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/screen.hpp" // IWYU pragma: keep

//general C++ utilities 
#include <cmath>
#include <iostream> // IWYU pragma: keep
#include <stdio.h> // IWYU pragma: keep

//include robot
#include "mcl/robot.hpp"

//inlcude screen 
#include "mcl/screen.hpp"

//include PID/movement utilities
#include "mcl/pid.hpp" 

//include MCL stuff (Only needed in robot, don't need in main)
//#include "mcl/mcl.hpp" 

//include auton
#include "mcl/auton.hpp"

using namespace pros;

//robot definitions
MotorGroup right_motors({18, 19, 20}, MotorCartridge::blue);
MotorGroup left_motors({-11, -12, -13}, MotorCartridge::blue); 

MotorGroup intake1({-1}, MotorCartridge::blue);
MotorGroup intake2({-21}, MotorCartridge::blue);

//Initializes IMUs and Distance sensors
Imu imu1(8);
Imu imu2(2);

Distance right(9);
Distance front(4);
Distance left(7);
Distance back(6);

Robot robot(11.0, 10.0, .75, 3.25, 5.75, &right_motors, &left_motors, &intake1, &intake2, LatPID::lat_one, TurnPID::turn_one);

Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() { // runs initialization; keep execution time under three seconds
    //Adds sensors to the robot sensor vectors
    robot.imus.push_back(std::make_unique<Imu>(8));
    robot.mclDistances.push_back(std::make_unique<Distance>(9));
    robot.mclDistances.push_back(std::make_unique<Distance>(7));
    robot.mclDistances.push_back(std::make_unique<Distance>(5));
    robot.mclDistances.push_back(std::make_unique<Distance>(6));

    left_motors.set_zero_position_all(0);
    right_motors.set_zero_position_all(0);

    latteral_high_qual.setExitConditions(1, 250, 5000);
	latteral_med_qual.setExitConditions(1, 250, 5000);
	latteral_low_qual.setExitConditions(1.5, 250, 5000);

	turning_high_qual.setExitConditions(1*M_PI/180, 250, 5000);
	turning_med_qual.setExitConditions(1*M_PI/180, 250, 5000);
	turning_low_qual.setExitConditions(1*M_PI/180, 250, 5000);

	pros::Task screenTask([] {
		while(true) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 7 , "Robot Pose: %.2f, %.2f, %.2f",robot.getPose().x,robot.getPose().y,float(robot.getPose().theta)*180/M_PI);
            pros::screen::print(pros::E_TEXT_MEDIUM, 8 , "Target Pose: %.2f, %.2f, %.2f",targetPose.x,targetPose.y,float(targetPose.theta)*180/M_PI);
            pros::delay(20);
        }
    });
        
    pros::Task autonSelector(handleTouch);
}

void disabled() { // task exits when robot is re-enabled

}

void competition_initialize() { // pre-auton; ends when auton begins
    //calibrate IMUs
    imu1.reset(true);
    imu2.reset(true);

    robot.place(autonPose[selectedAuton].x, autonPose[selectedAuton].y, autonPose[selectedAuton].theta);
}

void autonomous() {
    autons[selectedAuton].func();
}

void opcontrol() {
	bool shift = false;

	while (true) {
		int rightStickX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		int leftStickY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

        left_motors.move(leftStickY + rightStickX);
		right_motors.move(leftStickY - rightStickX);

		shift = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (!shift) {
                intake1.move_voltage(12000);
            } else {
                intake1.move_voltage(-12000);
            }
        } else {
            if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake1.brake();
            }
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (!shift) {
                intake1.move_voltage(12000); // 
                intake2.move_voltage(12000);
            } else {
                intake1.move_voltage(12000); //
                intake2.move_voltage(-12000);
            }
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake2.brake();
        }

		
		
		
		pros::delay(20);
	}
}