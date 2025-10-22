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
MotorGroup right_motors({18,19,20}, MotorCartridge::blue);
MotorGroup left_motors({11,12,13}, MotorCartridge::blue); 

MotorGroup intake1({-1}, MotorCartridge::blue);
MotorGroup intake2({-21}, MotorCartridge::blue);

Robot robot(11.0, 10.0, .75, 3.25, 5.75, &right_motors, &left_motors, LatPID::lat_one, TurnPID::turn_one);

Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() { // runs initialization; keep execution time under three seconds
    //Initializes IMUs and Distance sensors and adds them to the robot sensor vectors
    Imu imu1(8);
    robot.imus.push_back(std::make_unique<Imu>(1));
    Imu imu2(2);
    robot.imus.push_back(std::make_unique<Imu>(2));

    Distance right(3);
    robot.distances.push_back(std::make_unique<Distance>(3));
    Distance front(4);
    robot.distances.push_back(std::make_unique<Distance>(4));
    Distance left(5);
    robot.distances.push_back(std::make_unique<Distance>(5));
    Distance back(6);
    robot.distances.push_back(std::make_unique<Distance>(6));

    //calibrate IMUs
    imu1.reset();
    imu2.reset();
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
    //autons[selectedAuton].func();
    robot.move(24, 0, 2000, 1.0, 0, {0,1}, {0,5});
}

void opcontrol() {

	bool shift = false;

	while (true) {
		static int rightStickX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
		static int leftStickY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {shift = true;} else {shift = false;}

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            if (!shift) {
                intake1.move_voltage(-12000); // 
                intake2.move_voltage(12000);
            } else {
                intake1.move_voltage(-12000); //
                intake2.move_voltage(-12000);
            }
        } else if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake1.brake();
                intake2.brake();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (!shift) {
                intake1.move_voltage(12000);
                intake2.move_voltage(12000);
            } else {
                intake1.move_voltage(-12000);
                intake2.move_voltage(12000);
            }
        } else {
            if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake1.brake();
                intake2.brake();
            }
        }

		left_motors.move(rightStickX - leftStickY);
		right_motors.move(rightStickX + leftStickY);
		
		pros::delay(20);
	}
}