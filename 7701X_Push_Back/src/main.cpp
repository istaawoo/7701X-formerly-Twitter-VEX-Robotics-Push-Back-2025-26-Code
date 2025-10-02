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
#include "robot.cpp"

//inlcude screen 
#include "screen.cpp"

//include PID/movement utilities
#include "src/pid.cpp"
#include "mcl/pid.hpp" 

//include MCL stuff
#include "src/mcl.cpp"
#include "mcl/mcl.hpp"

//robot definitions
pros::MotorGroup right_motors({1,2,3}, pros::MotorCartridge::blue);
pros::MotorGroup left_motors({4,5,6}, pros::MotorCartridge::blue);

PID latteral_one(1, 0, 1);
PID latteral_two(5, 1, 5);
PID latteral_three(10, 5, 10);

PID turning_one(1, 0, 1);
PID turning_two(5, 1, 5);
PID turning_three(10, 5, 10);

void initialize() { // runs initialization; keep execution time under three seconds
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
	latteral_one.setExitConditions(1, 250, 5000);
	latteral_two.setExitConditions(1, 250, 5000);
	latteral_three.setExitConditions(1, 250, 5000);

	turning_one.setExitConditions(1, 250, 1000);
	turning_two.setExitConditions(1, 250, 1000);
	turning_three.setExitConditions(1, 250, 1000);
}

void disabled() {} // task exits when robot is re-enabled

void competition_initialize() {} // pre-auton; ends when auton begins

void autonomous() { // auton; if disconnected the task will restart; not continue where stopped

}

void opcontrol() {
	while (true) {
		static int analogValueRightStickX = pros::E_CONTROLLER_ANALOG_RIGHT_X;
		static int analogValueLeftStickY = pros::E_CONTROLLER_ANALOG_LEFT_Y;
		
		
		pros::delay(20);
	}
}