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
//#include "robot.cpp"

//inlcude screen 
#include "mcl/screen.hpp" 
//#include "screen.cpp"

//include PID/movement utilities
//#include "pid.cpp"
#include "mcl/pid.hpp" 

//include MCL stuff
//#include "mcl.cpp"
#include "mcl/mcl.hpp" 

#include "mcl/auton.hpp"

using namespace pros;

//robot definitions
MotorGroup right_motors({1,2,3}, MotorCartridge::blue);
MotorGroup left_motors({4,5,6}, MotorCartridge::blue); 

Robot robot(11.0, 10.0, .75, 3.25, 5.75, &right_motors, &left_motors, LatPID::lat_one, TurnPID::turn_one);

Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() { // runs initialization; keep execution time under three seconds
/*
	latteral_high_qual.setExitConditions(1, 250, 5000);
	latteral_med_qual.setExitConditions(1, 250, 5000);
	latteral_low_qual.setExitConditions(1, 250, 5000);

	turning_high_qual.setExitConditions(1, 250, 1000);
	turning_med_qual.setExitConditions(1, 250, 1000);
	turning_low_qual.setExitConditions(1, 250, 1000);
*/
}

void disabled() { // task exits when robot is re-enabled

}

void competition_initialize() { // pre-auton; ends when auton begins

}

void autonomous() {
    autons[selectedAuton].func();
}

void opcontrol() {

	while (true) {
		static int rightStickX = E_CONTROLLER_ANALOG_RIGHT_X;
		static int leftStickY = E_CONTROLLER_ANALOG_LEFT_Y;

		
		
		
		pros::delay(20);
	}
}