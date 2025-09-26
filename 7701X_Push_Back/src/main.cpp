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

//include PID/movement utilities
#include "pid.cpp"
#include "pid.hpp"

//include MCL stuff
#include "mcl.cpp"
#include "mcl.hpp"

void on_left_button() {

}

void on_center_button() {
	
}

void on_right_button() {

}

void initialize() { // runs initialization; keep execution time under three seconds
	pros::lcd::initialize();
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn2_cb(on_right_button);
}

void disabled() {} // task exits when robot is re-enabled

void competition_initialize() {} // pre-auton; ends when auton begins

void autonomous() {} // auton; if disconnected the task will restart; not continue where stopped

void opcontrol() {
	while (true) {
		
		pros::delay(20);
	}
}